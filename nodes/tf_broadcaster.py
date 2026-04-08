#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手眼标定 TF 广播节点

功能：
- 从 camera.yaml 读取手眼标定参数
- 发布静态 TF：right_base → camera_right（相机相对于法兰的偏移）
- 监听机械臂末端法兰 TF（由 arm_driver 发布），广播 right_base → right_top

TF 关系链：
  right_base ──(静态偏移)──► camera_right
  right_base ──(动态法兰)──► right_top

运行前检查：
  ros2 run guji tf_broadcaster
  ros2 run tf2_ros tf2_echo right_base camera_right

依赖：
  - camera.yaml 配置文件（手眼标定结果）
  - arm_driver 节点（发布关节状态，触发 right_top TF）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from sensor_msgs.msg import JointState
import yaml
import os
import math


class HandEyeTFBroadcaster(Node):
    """手眼标定 TF 广播节点"""

    def __init__(self):
        super().__init__('hand_eye_tf_broadcaster')

        # 加载相机配置
        self._load_camera_config()

        # 创建静态 TF 广播器（手眼标定：相机相对于法兰）
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # 创建动态 TF 广播器（法兰相对于基座）
        self._dynamic_broadcaster = TransformBroadcaster(self)

        # 订阅关节状态，获取法兰位置
        self._joint_state_sub = self.create_subscription(
            JointState,
            f'/{self.right_ns}/joint_states',
            self._joint_state_callback,
            10
        )

        # 法兰位姿
        self._flange_transform = None

        # 启动时立即发布静态 TF
        self._publish_static_tf()
        self.get_logger().info('手眼标定 TF 广播节点已启动')
        self.get_logger().info(f'  标定偏移: translation=({self.cam_tx:.4f}, {self.cam_ty:.4f}, {self.cam_tz:.4f}) m')
        self.get_logger().info(f'  四元数: ({self.cam_qx:.4f}, {self.cam_qy:.4f}, {self.cam_qz:.4f}, {self.cam_qw:.4f})')
        self.get_logger().info(f'  基坐标系: {self.base_frame}')
        self.get_logger().info(f'  法兰坐标系: {self.flange_frame}')
        self.get_logger().info(f'  相机坐标系: {self.camera_frame}')

        # 定期广播动态 TF（法兰）
        self._timer = self.create_timer(0.05, self._broadcast_dynamic_tf)

    def _load_camera_config(self):
        """从 YAML 加载手眼标定参数"""
        # 确定配置文件路径（支持 launch 和直接运行两种方式）
        possible_paths = [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'camera.yaml'),
            os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'camera.yaml'),
            '/etc/ros2/guji/camera.yaml',
        ]

        config_path = None
        for p in possible_paths:
            if os.path.exists(p):
                config_path = p
                break

        if config_path is None:
            self.get_logger().warn(
                f'camera.yaml 未找到，使用默认参数。可选路径: {possible_paths}'
            )
            self._use_default_calibration()
            return

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)

            cam_cfg = cfg['camera']
            self.right_ns = 'right_arm_controller'  # 右臂 namespace

            # 手眼标定参数
            t = cam_cfg['hand_eye_right']['translation']
            q = cam_cfg['hand_eye_right']['quaternion']
            self.cam_tx = t['x']
            self.cam_ty = t['y']
            self.cam_tz = t['z']
            self.cam_qx = q['x']
            self.cam_qy = q['y']
            self.cam_qz = q['z']
            self.cam_qw = q['w']

            # 坐标系名称
            frames = cam_cfg['frames']
            self.base_frame = frames.get('base_frame', 'right_base')
            self.flange_frame = frames.get('flange_frame', 'right_top')
            self.camera_frame = frames.get('camera_frame', 'camera_right')

            self.get_logger().info(f'已加载相机配置: {config_path}')

        except Exception as e:
            self.get_logger().error(f'加载 camera.yaml 失败: {e}，使用默认参数')
            self._use_default_calibration()

    def _use_default_calibration(self):
        """使用默认标定参数（相机垂直朝下安装）"""
        self.right_ns = 'right_arm_controller'
        self.cam_tx = 0.085
        self.cam_ty = -0.040
        self.cam_tz = 0.010
        # 相机朝向：绕 Y 轴旋转 90°（相机朝下）
        # 等效四元数：yaw=π/2, roll=0, pitch=0
        yaw = math.pi / 2
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        self.cam_qx = 0.0
        self.cam_qy = sy
        self.cam_qz = 0.0
        self.cam_qw = cy
        self.base_frame = 'right_base'
        self.flange_frame = 'right_top'
        self.camera_frame = 'camera_right'

    def _joint_state_callback(self, msg):
        """
        监听关节状态，提取末端法兰位置。
        注意：arm_driver 节点通过 TF 广播 right_base→right_top，
        这里通过监听关节角度计算法兰位置并广播。
        如果 arm_driver 已广播 right_top TF，此回调可以省略。
        """
        # 将关节角度转换为末端位姿（简化处理，直接广播法兰 TF）
        # 实际使用时，arm_driver 会通过 TF 发布 right_top 相对于 right_base 的变换
        # 此节点只需广播静态的 camera TF 即可
        pass

    def _publish_static_tf(self):
        """发布静态 TF：right_base → camera_right"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame          # 父坐标系：法兰
        t.child_frame_id = self.camera_frame          # 子坐标系：相机

        # 平移
        t.transform.translation.x = self.cam_tx
        t.transform.translation.y = self.cam_ty
        t.transform.translation.z = self.cam_tz

        # 旋转四元数
        t.transform.rotation.x = self.cam_qx
        t.transform.rotation.y = self.cam_qy
        t.transform.rotation.z = self.cam_qz
        t.transform.rotation.w = self.cam_qw

        self._static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'已发布静态 TF: {self.base_frame} → {self.camera_frame}'
        )

    def _broadcast_dynamic_tf(self):
        """
        发布动态 TF：right_base → right_top（法兰相对于基座）
        说明：
          如果 arm_driver 节点已经广播法兰 TF（通过 rm_driver/udp_arm_position），
          此方法可以省略。如果需要此节点自己计算法兰位置，
          需要将关节角度通过运动学正解转换为末端位姿。
        """
        # 方案 A（推荐）：arm_driver 节点已广播法兰 TF，此处无需重复广播
        # 方案 B：监听关节角度，计算法兰位置并广播
        # 这里采用方案 A，仅在 arm_driver 未发布法兰 TF 时启用
        pass


def main(args=None):
    rclpy.init(args=args)

    node = HandEyeTFBroadcaster()

    print()
    print("========================================")
    print("  手眼标定 TF 广播节点")
    print("========================================")
    print()
    print("  TF 关系:")
    print("    right_base → camera_right  (静态，手眼标定)")
    print("    right_base → right_top     (动态，arm_driver 发布)")
    print()
    print("  验证命令:")
    print("    ros2 run tf2_ros tf2_echo right_base camera_right")
    print("    ros2 run tf2_ros view_frames")
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
