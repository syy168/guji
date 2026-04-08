#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 标记检测节点

功能：
- 订阅 RealSense D435 的彩色图像话题
- 使用 OpenCV ArUco 检测算法识别标记
- 通过 PnP 姿态估计计算标记的 6DOF 位姿
- 发布 TF：target_right_camera（检测结果坐标系）
- 提供 ROS2 Service 接口供主控制器调用

检测流程：
  1. 等待下一帧彩色图像
  2. 灰度化 → ArUco 检测（字典: DICT_5X5_1000）
  3. 按 ID 筛选目标标记
  4. PnP 姿态估计（使用相机内参）
  5. 角度稳定性过滤（滑动窗口）
  6. 发布 TF：camera_right → target_right_camera
  7. TF 监听：target_right_camera → base_frame
  8. 返回基坐标系下的 6DOF 位姿

运行前检查：
  ros2 run guji aruco_detector
  ros2 service call /right_arm_controller/detect_aruco guji/srv/DetectAruco "{marker_id: 11, timeout: 5.0}"
  ros2 run tf2_ros tf2_echo right_base target_right_camera

依赖：
  - realsense2_camera 节点已启动
  - camera_bridge 节点已启动（用于诊断）
  - tf_broadcaster 节点已启动（发布 camera_right TF）
  - camera.yaml 和 system.yaml 配置文件
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations as T
import cv2
import cv_bridge
import numpy as np
import yaml
import os
import time


class ArucoDetector(Node):
    """ArUco 标记检测节点"""

    # ArUco 字典名称到 OpenCV 常量的映射
    ARUCO_DICT_MAP = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
        'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
        'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
        'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
        'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
        'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
        'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
    }

    def __init__(self):
        super().__init__('aruco_detector')

        self._load_config()

        # ==========================================
        # ROS 消息桥接
        # ==========================================
        self._bridge = cv_bridge.CvBridge()

        # ==========================================
        # 状态变量
        # ==========================================
        self._latest_image = None       # 最新彩色图像
        self._latest_camera_info = None # 最新相机内参
        self._camera_matrix = None      # 内参矩阵 (3x3)
        self._dist_coeffs = None       # 畸变系数
        self._camera_frame_id = ''      # 相机坐标系名称

        # 角度平滑滑动窗口（稳定性过滤）
        self._angle_history = []        # 历史旋转角度列表

        # ==========================================
        # TF 相关
        # ==========================================
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ==========================================
        # ArUco 检测器初始化
        # ==========================================
        dict_name = self.cfg.get('dict_type', 'DICT_5X5_1000')
        dict_id = self.ARUCO_DICT_MAP.get(dict_name, cv2.aruco.DICT_5X5_1000)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        self.get_logger().info(f'ArUco 检测器初始化完成: 字典={dict_name}, 标记边长={self.marker_size}m')

        # ==========================================
        # 订阅图像话题
        # ==========================================
        self._image_sub = self.create_subscription(
            Image,
            f'{self.topic_prefix}/color/image_raw',
            self._image_callback,
            10
        )

        self._info_sub = self.create_subscription(
            CameraInfo,
            f'{self.topic_prefix}/color/camera_info',
            self._info_callback,
            10
        )

        # ==========================================
        # 发布检测结果可视化（调试用）
        # ==========================================
        self._debug_pub = self.create_publisher(Image, '/right_arm_controller/aruco/debug_image', 10)

        # ==========================================
        # 提供 ArUco 检测 Service
        # ==========================================
        from guji.srv import DetectAruco
        self._detect_srv = self.create_service(
            DetectAruco,
            '/right_arm_controller/detect_aruco',
            self._detect_callback
        )

        self.get_logger().info('ArUco 检测节点已启动')
        self.get_logger().info(f'  监听话题: {self.topic_prefix}/color/image_raw')
        self.get_logger().info(f'  发布 Service: /right_arm_controller/detect_aruco')
        self.get_logger().info(f'  发布 TF: base_frame → target_camera_frame')
        self.get_logger().info(f'  期望帧率: ≥15Hz, 超时: {self.vision_timeout}s')
        self.get_logger().info(
            f'  稳定性参数: 阈值={self.stability_threshold}rad, '
            f'窗口={self.history_size}, 平滑={self.smoothing_factor}'
        )

    # ==========================================
    # 配置加载
    # ==========================================
    def _load_config(self):
        """从 camera.yaml 和 system.yaml 加载配置"""
        node_dir = os.path.dirname(__file__)

        # 相机配置路径
        cam_paths = [
            os.path.join(node_dir, '..', 'config', 'camera.yaml'),
            os.path.join(node_dir, '..', '..', 'config', 'camera.yaml'),
        ]
        cam_config_path = next((p for p in cam_paths if os.path.exists(p)), None)

        # 系统配置路径
        sys_paths = [
            os.path.join(node_dir, '..', 'config', 'system.yaml'),
            os.path.join(node_dir, '..', '..', 'config', 'system.yaml'),
        ]
        sys_config_path = next((p for p in sys_paths if os.path.exists(p)), None)

        # 默认值
        self.topic_prefix = '/camera_right'
        self.marker_size = 0.030
        self.vision_timeout = 5.0
        self.stability_threshold = 0.1
        self.history_size = 10
        self.smoothing_factor = 0.8
        self.depth_radius = 3
        self.base_frame = 'right_base'
        self.camera_frame = 'camera_right'
        self.target_frame = 'target_right_camera'
        self.debug_enabled = False

        if cam_config_path:
            try:
                with open(cam_config_path, 'r', encoding='utf-8') as f:
                    cfg = yaml.safe_load(f)
                cam_cfg = cfg.get('camera', {})

                self.topic_prefix = cam_cfg.get('topic_prefix', self.topic_prefix)
                self.marker_size = cam_cfg.get('aruco', {}).get('marker_size', self.marker_size)
                self.cfg_dict_type = cam_cfg.get('aruco', {}).get('dict_type', 'DICT_5X5_1000')

                frames = cam_cfg.get('frames', {})
                self.base_frame = frames.get('base_frame', self.base_frame)
                self.camera_frame = frames.get('camera_frame', self.camera_frame)
                self.target_frame = frames.get('target_frame', self.target_frame)

                self.get_logger().info(f'已加载相机配置: {cam_config_path}')
            except Exception as e:
                self.get_logger().warn(f'加载 camera.yaml 失败: {e}')

        if sys_config_path:
            try:
                with open(sys_config_path, 'r', encoding='utf-8') as f:
                    cfg = yaml.safe_load(f)
                sys_cfg = cfg.get('system', {}).get('vision', {})

                self.vision_timeout = sys_cfg.get('timeout', self.vision_timeout)
                self.stability_threshold = sys_cfg.get('stability_threshold', self.stability_threshold)
                self.history_size = sys_cfg.get('history_size', self.history_size)
                self.smoothing_factor = sys_cfg.get('smoothing_factor', self.smoothing_factor)
                self.depth_radius = sys_cfg.get('depth_averaging_radius', self.depth_radius)
                self.debug_enabled = cfg.get('system', {}).get('debug', {}).get('enabled', False)

                self.get_logger().info(f'已加载系统配置: {sys_config_path}')
            except Exception as e:
                self.get_logger().warn(f'加载 system.yaml 失败: {e}')

        self.cfg = {'dict_type': self.cfg_dict_type}

    # ==========================================
    # 回调函数
    # ==========================================
    def _image_callback(self, msg: Image):
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return
        if self._camera_matrix is not None:
            self.get_logger().info(
                f'相机内参已就绪: fx={self._camera_matrix[0,0]:.1f}, '
                f'fy={self._camera_matrix[1,1]:.1f}, '
                f'cx={self._camera_matrix[0,2]:.1f}, cy={self._camera_matrix[1,2]:.1f}'
            )

    # ==========================================
    # CameraInfo 回调 — 从运行时话题提取内参矩阵
    # ==========================================
    def _info_callback(self, msg: CameraInfo):
        """从 CameraInfo 话题提取相机内参（K）和畸变系数（D），供 ArUco PnP 使用"""
        if self._camera_matrix is not None:
            return  # 已初始化，无需重复设置
        if len(msg.k) != 9:
            self.get_logger().warn(f'CameraInfo K 数组长度异常: {len(msg.k)}，跳过内参初始化')
            return
        import numpy as np
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self._camera_frame_id = msg.header.frame_id
        self.get_logger().info(
            f'相机内参已就绪: fx={self._camera_matrix[0,0]:.1f}, '
            f'fy={self._camera_matrix[1,1]:.1f}, '
            f'cx={self._camera_matrix[0,2]:.1f}, cy={self._camera_matrix[1,2]:.1f}'
        )

    # ==========================================
    # Service 回调（核心检测逻辑）
    # ==========================================
    def _detect_callback(self, req, resp):
        """
        ArUco 检测 Service 回调。

        检测流程：
        1. 清除历史数据，等待下一帧
        2. 持续检测直到找到目标 ID 或超时
        3. 角度稳定性过滤
        4. TF 变换到基坐标系
        5. 返回结果
        """
        from guji.srv import DetectAruco
        resp.found = False
        resp.pose = Pose()
        resp.header = Header()

        marker_id = req.marker_id
        timeout = req.timeout if req.timeout > 0 else self.vision_timeout

        self.get_logger().info(f'开始检测 ArUco ID={marker_id}, 超时={timeout}s')

        # 清除历史，确保从新帧开始检测
        self._angle_history.clear()
        start_time = time.time()
        wait_start = time.time()
        attempts = 0

        # 等待第一帧图像到达
        while self._latest_image is None and (time.time() - wait_start) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._latest_image is None:
            self.get_logger().error('等待图像帧超时，未收到任何图像')
            return resp
        if self._camera_matrix is None:
            self.get_logger().error('相机内参未就绪，请检查 camera_info 话题')
            return resp

        # 主检测循环
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            attempts += 1

            if self._latest_image is None:
                continue

            # 灰度化
            gray = cv2.cvtColor(self._latest_image, cv2.COLOR_BGR2GRAY)

            # ArUco 检测
            corners, ids, rejected = self._aruco_detector.detectMarkers(gray)

            if ids is None:
                continue

            # 查找目标 ID
            target_indices = np.where(ids.flatten() == marker_id)[0]
            if target_indices.size == 0:
                continue

            idx = target_indices[0]
            corner = corners[idx]

            # PnP 姿态估计：估算标记相对于相机的位姿
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, self.marker_size,
                self._camera_matrix, self._dist_coeffs
            )
            rvec = rvec[0, 0, :]  # 旋转向量
            tvec = tvec[0, 0, :]  # 平移向量

            # 当前帧的旋转角度（旋转向量的模）
            current_angle = np.linalg.norm(rvec)

            # === 稳定性过滤 ===
            self._angle_history.append(current_angle)
            if len(self._angle_history) > self.history_size:
                self._angle_history.pop(0)

            if len(self._angle_history) < self.history_size:
                self.get_logger().debug(
                    f'  [ID={marker_id}] 积累稳定性: {len(self._angle_history)}/{self.history_size}'
                )
                continue

            # 计算与历史最大偏差
            recent = self._angle_history[:-1]
            if recent:
                max_diff = max(abs(current_angle - a) for a in recent)
                is_stable = max_diff < self.stability_threshold
            else:
                is_stable = True

            if not is_stable:
                self.get_logger().debug(
                    f'  [ID={marker_id}] 角度不稳定: diff={max_diff:.4f} > {self.stability_threshold}'
                )
                self._angle_history.clear()
                continue

            # === 角度平滑 ===
            if len(self._angle_history) >= 2:
                smoothed_angle = (
                    self.smoothing_factor * self._angle_history[-2]
                    + (1 - self.smoothing_factor) * current_angle
                )
            else:
                smoothed_angle = current_angle

            # 旋转向量缩放
            if smoothed_angle > 1e-6:
                rvec_smooth = rvec * (smoothed_angle / current_angle)
            else:
                rvec_smooth = rvec.copy()

            self.get_logger().info(
                f'  [ID={marker_id}] 检测成功! pos=({tvec[0]:.4f}, {tvec[1]:.4f}, {tvec[2]:.4f})m, '
                f'angle={current_angle:.4f}rad, attempts={attempts}'
            )

            # === 发布 TF ===
            self._publish_target_tf(rvec_smooth, tvec)

            # === TF 变换到基坐标系 ===
            base_pose = self._lookup_base_pose()
            if base_pose is not None:
                resp.found = True
                resp.pose = base_pose
                resp.header = Header()
                resp.header.stamp = self.get_clock().now().to_msg()
                resp.header.frame_id = self.base_frame
                self.get_logger().info(
                    f'  基坐标系位姿: pos=({resp.pose.position.x:.4f}, '
                    f'{resp.pose.position.y:.4f}, {resp.pose.position.z:.4f})m'
                )
            else:
                self.get_logger().warn('TF 变换到基坐标系失败，使用相机坐标系结果')
                resp.found = True
                resp.pose.position.x = tvec[0]
                resp.pose.position.y = tvec[1]
                resp.pose.position.z = tvec[2]
                resp.pose.orientation = self._rvec_to_quaternion_msg(rvec_smooth)
                resp.header.stamp = self.get_clock().now().now().to_msg()
                resp.header.frame_id = self.camera_frame

            # 发布调试图像
            if self.debug_enabled:
                self._publish_debug_image(gray, corners, ids, rvec, tvec)

            return resp

        self.get_logger().warn(f'检测超时，未找到 ID={marker_id}（尝试 {attempts} 帧）')
        return resp

    # ==========================================
    # TF 发布
    # ==========================================
    def _publish_target_tf(self, rvec, tvec):
        """发布检测结果 TF：camera_right → target_right_camera"""
        q = self._rvec_to_quaternion(rvec)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.target_frame

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self._tf_broadcaster.sendTransform(t)

    def _lookup_base_pose(self):
        """
        监听 TF，将目标位姿从相机坐标系转换到基坐标系。
        返回 geometry_msgs.Pose（基坐标系下）
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            pose = Pose()
            pose.position.x = t.x
            pose.position.y = t.y
            pose.position.z = t.z
            pose.orientation.x = q.x
            pose.orientation.y = q.y
            pose.orientation.z = q.z
            pose.orientation.w = q.w

            return pose

        except Exception as e:
            self.get_logger().warn(f'TF 监听失败: {e}')
            return None

    # ==========================================
    # 辅助方法
    # ==========================================
    def _rvec_to_quaternion(self, rvec):
        """旋转向量 → 四元数 (x,y,z,w)"""
        angle = np.linalg.norm(rvec)
        if angle < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])
        axis = rvec / angle
        return T.quaternion_from_euler(
            axis[0] * angle,
            axis[1] * angle,
            axis[2] * angle
        )

    def _rvec_to_quaternion_msg(self, rvec):
        """旋转向量 → geometry_msgs.Quaternion"""
        q = self._rvec_to_quaternion(rvec)
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def _publish_debug_image(self, gray, corners, ids, rvec, tvec):
        """发布调试可视化图像（标记检测结果叠加）"""
        output = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(output, corners, ids)
            cv2.drawFrameAxes(
                output, self._camera_matrix, self._dist_coeffs,
                rvec, tvec, 0.05
            )
        try:
            msg = self._bridge.cv2_to_imgmsg(output, encoding='bgr8')
            self._debug_pub.publish(msg)
        except cv_bridge.CvBridgeError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()

    print()
    print("========================================")
    print("  ArUco 标记检测节点")
    print("========================================")
    print()
    print("  Service 调用示例:")
    print("    ros2 service call /right_arm_controller/detect_aruco guji/srv/DetectAruco \\")
    print("      \"{marker_id: 11, timeout: 5.0}\"")
    print()
    print("  TF 验证:")
    print("    ros2 run tf2_ros tf2_echo right_base target_right_camera")
    print("    ros2 run tf2_ros view_frames")
    print()
    print("  调试话题:")
    print("    /right_arm_controller/aruco/debug_image")
    print()
    print("========================================")
    print()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
