#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手眼标定 TF 广播节点

功能：
- 从 camera.yaml 读取手眼标定参数
- 发布静态 TF：right_top → camera_right（相机相对于法兰的偏移）
- 发布动态 TF：right_base → right_top（法兰相对于基座）
- 从 rm_driver 的 udp_arm_position 话题获取法兰实时位姿

TF 关系链（Eye-in-Hand / 相机在手上）：
  right_base ──(动态: 实时法兰位姿)──► right_top ──(静态: 手眼标定)──► camera_right
                                                                          │
                                                                          ▼
                                                               target_right_camera (ArUco检测结果)

运行前检查：
  ros2 run guji tf_broadcaster
  ros2 run tf2_ros tf2_echo right_base right_top        # 法兰 TF
  ros2 run tf2_ros tf2_echo right_top camera_right      # 相机标定 TF

依赖：
  - camera.yaml 配置文件（手眼标定结果）
  - rm_driver 节点已启动（发布 udp_arm_position 话题）
  - 右臂 namespace 与配置一致
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose
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

        # 静态 TF 广播器（手眼标定：right_top → camera_right）
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # 动态 TF 广播器（法兰：right_base → right_top）
        self._dynamic_broadcaster = TransformBroadcaster(self)

        # 订阅关节状态（用于检测连接）
        self._joint_state_sub = self.create_subscription(
            JointState,
            f'/{self.right_ns}/joint_states',
            self._joint_state_callback,
            10
        )

        # 订阅法兰实时位姿（rm_driver 发布的 udp_arm_position）
        self._flange_pose_sub = self.create_subscription(
            Pose,
            f'/{self.right_ns}/rm_driver/udp_arm_position',
            self._flange_pose_callback,
            10
        )

        self._has_flange_pose = False

        # 启动时发布静态 TF（仅需一次）
        self._publish_static_tf()
        self.get_logger().info('手眼标定 TF 广播节点已启动')
        self.get_logger().info(f'  标定偏移: translation=({self.cam_tx:.4f}, {self.cam_ty:.4f}, {self.cam_tz:.4f}) m')
        self.get_logger().info(f'  四元数: ({self.cam_qx:.4f}, {self.cam_qy:.4f}, {self.cam_qz:.4f}, {self.cam_qw:.4f})')
        self.get_logger().info(f'  基坐标系: {self.base_frame}')
        self.get_logger().info(f'  法兰坐标系: {self.flange_frame}')
        self.get_logger().info(f'  相机坐标系: {self.camera_frame}')
        self.get_logger().info(f'  动态 TF 来源: /{self.right_ns}/rm_driver/udp_arm_position')

    def _load_camera_config(self):
        """从 camera.yaml 加载手眼标定参数"""
        node_dir = os.path.dirname(__file__)
        possible_paths = [
            os.path.join(node_dir, '..', 'config', 'camera.yaml'),
            os.path.join(node_dir, '..', '..', 'config', 'camera.yaml'),
        ]

        config_path = None
        for p in possible_paths:
            if os.path.exists(p):
                config_path = p
                break

        if config_path is None:
            self.get_logger().warn(f'camera.yaml 未找到，使用默认参数。可选路径: {possible_paths}')
            self._use_default_calibration()
            return

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)

            cam_cfg = cfg['camera']
            self.right_ns = 'right_arm_controller'

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

    def _joint_state_callback(self, msg: JointState):
        """监听关节状态（仅用于日志确认连接）"""
        pass

    def _flange_pose_callback(self, msg: Pose):
        """从 udp_arm_position 提取法兰实时位姿，发布动态 TF"""
        self._has_flange_pose = True

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame      # right_base
        t.child_frame_id = self.flange_frame     # right_top

        # udp_arm_position 的 position 即法兰在基坐标系下的位置
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        # orientation 即法兰在基坐标系下的姿态
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self._dynamic_broadcaster.sendTransform(t)

    def _publish_static_tf(self):
        """发布静态 TF：right_top → camera_right（手眼标定结果）"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.flange_frame     # 父坐标系：法兰 (right_top)
        t.child_frame_id = self.camera_frame     # 子坐标系：相机 (camera_right)

        # 平移：相机原点相对于法兰坐标系的偏移
        t.transform.translation.x = self.cam_tx
        t.transform.translation.y = self.cam_ty
        t.transform.translation.z = self.cam_tz

        # 旋转四元数：相机坐标系相对于法兰坐标系的姿态
        t.transform.rotation.x = self.cam_qx
        t.transform.rotation.y = self.cam_qy
        t.transform.rotation.z = self.cam_qz
        t.transform.rotation.w = self.cam_qw

        self._static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'已发布静态 TF: {self.flange_frame} → {self.camera_frame} '
            f'(手眼标定，translation=({self.cam_tx:.4f}, {self.cam_ty:.4f}, {self.cam_tz:.4f}))'
        )


def main(args=None):
    rclpy.init(args=args)

    node = HandEyeTFBroadcaster()

    print()
    print("========================================")
    print("  手眼标定 TF 广播节点")
    print("========================================")
    print()
    print("  TF 关系:")
    print("    right_base  → right_top      (动态: rm_driver/udp_arm_position)")
    print("    right_top   → camera_right    (静态: 手眼标定结果)")
    print()
    print("  验证命令:")
    print("    ros2 run tf2_ros tf2_echo right_base right_top")
    print("    ros2 run tf2_ros tf2_echo right_top camera_right")
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
