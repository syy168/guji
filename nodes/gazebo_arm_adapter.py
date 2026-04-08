#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gazebo 机械臂适配器

功能：
- 在 Gazebo 仿真环境中，将真实的 rm_driver 消息接口转换为 Gazebo 的 JointTrajectoryController 接口
- 让使用 rm_driver 接口的上层控制器（如 dual_arm_controller.py、dual_arm_pick_place.py）
  无需修改即可在 Gazebo 仿真中运行

原理：
  真实硬件：
    /left_arm_controller/rm_driver/movej_cmd (rm_ros_interfaces/Movej)
  Gazebo：
    /left_arm_controller/follow_joint_trajectory/goal (trajectory_msgs/JointTrajectory)

本节点桥接两者：
    rm_driver/Movej 消息  ──►  JointTrajectory 消息

使用方法：
  # 终端 1：启动 Gazebo
  ros2 launch dual_rm_gazebo dual_rm_65b_gazebo.launch.py

  # 终端 2：启动 MoveIt2
  ros2 launch dual_rm_65b_moveit_config demo.launch.py

  # 终端 3：启动适配器
  python3 guji/nodes/gazebo_arm_adapter.py

  # 终端 4：使用原有控制器（自动连接到适配后的接口）
  python3 guji/dual_arm_controller.py --use_gazebo

TF 关系链：
  world ──► l_base ──► l_top
  world ──► r_base ──► r_top
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time


class GazeboArmAdapter(Node):
    """
    Gazebo 机械臂适配器

    订阅真实的 rm_driver 消息，转换为 Gazebo JointTrajectoryController 接口
    """

    # 关节名称映射（Gazebo urdf 中定义的名称）
    LEFT_JOINT_NAMES = ['l_joint1', 'l_joint2', 'l_joint3', 'l_joint4', 'l_joint5', 'l_joint6']
    RIGHT_JOINT_NAMES = ['r_joint1', 'r_joint2', 'r_joint3', 'r_joint4', 'r_joint5', 'r_joint6']

    def __init__(self,
                 left_ns: str = "left_arm_controller",
                 right_ns: str = "right_arm_controller",
                 default_speed: int = 30):
        super().__init__('gazebo_arm_adapter')

        self.left_ns = left_ns
        self.right_ns = right_ns
        self.default_speed = default_speed

        self.get_logger().info('Gazebo 机械臂适配器已启动')
        self.get_logger().info(f'  左臂 namespace: {self.left_ns}')
        self.get_logger().info(f'  右臂 namespace: {self.right_ns}')

        # ==========================================
        # 导入消息类型
        # ==========================================
        try:
            from rm_ros_interfaces.msg import Movej, Movel, Gripperset, Gripperpick
            self._Movej_msg = Movej
            self._Movel_msg = Movel
            self._Gripperset_msg = Gripperset
            self._Gripperpick_msg = Gripperpick
        except ImportError:
            self.get_logger().error('无法导入 rm_ros_interfaces 消息类型')
            self._Movej_msg = None

        # ==========================================
        # 订阅真实的 rm_driver 命令
        # ==========================================

        # MoveJ 命令
        self._left_movej_sub = self.create_subscription(
            self._Movej_msg if self._Movej_msg else JointState,
            f'/{self.left_ns}/rm_driver/movej_cmd',
            lambda msg: self._on_left_movej(msg),
            10
        )
        self._right_movej_sub = self.create_subscription(
            self._Movej_msg if self._Movej_msg else JointState,
            f'/{self.right_ns}/rm_driver/movej_cmd',
            lambda msg: self._on_right_movej(msg),
            10
        )

        # ==========================================
        # 发布到 Gazebo JointTrajectoryController
        # ==========================================
        self._left_traj_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.left_ns}/follow_joint_trajectory/goal',
            10
        )
        self._right_traj_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.right_ns}/follow_joint_trajectory/goal',
            10
        )

        # ==========================================
        # 订阅 Gazebo 关节状态（用于反馈）
        # ==========================================
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            10
        )

        self._left_positions = {}
        self._right_positions = {}

        self.get_logger().info('适配器就绪，等待 rm_driver 命令...')

    def _on_joint_state(self, msg: JointState):
        """更新关节位置缓存"""
        for name, pos in zip(msg.name, msg.position):
            if name in self.LEFT_JOINT_NAMES:
                self._left_positions[name] = pos
            elif name in self.RIGHT_JOINT_NAMES:
                self._right_positions[name] = pos

    def _get_current_positions(self, joint_names: list, positions: dict) -> list:
        """获取当前关节位置"""
        result = []
        for name in joint_names:
            if name in positions:
                result.append(positions[name])
            else:
                result.append(0.0)  # 默认值
        return result

    def _compute_duration(self, target_positions: list, current_positions: list,
                         speed: int) -> float:
        """
        根据速度计算运动时长

        参数:
            target_positions: 目标位置（弧度）
            current_positions: 当前关节位置（弧度）
            speed: 速度 1-100

        返回:
            float: 运动时长（秒）
        """
        # 计算最大关节位移
        max_delta = 0.0
        for target, current in zip(target_positions, current_positions):
            delta = abs(target - current)
            if delta > max_delta:
                max_delta = delta

        # 速度 100 -> 0.1s, 速度 1 -> 3.0s
        base_speed = 0.1 + (100 - speed) / 100.0 * 2.9
        duration = max(max_delta * 2.0, base_speed)
        return min(duration, 10.0)  # 最多 10 秒

    def _publish_trajectory(self, pub, joint_names: list,
                            target_positions: list, duration: float):
        """发布 JointTrajectory 消息"""
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()

        traj.points = [point]
        pub.publish(traj)

    def _on_left_movej(self, msg):
        """处理左臂 MoveJ 命令"""
        if self._Movej_msg is None or not isinstance(msg, self._Movej_msg):
            return

        try:
            joint_names = self.LEFT_JOINT_NAMES
            target_positions = list(msg.joint[:6])

            current = self._get_current_positions(joint_names, self._left_positions)
            speed = int(msg.speed) if hasattr(msg, 'speed') and msg.speed else self.default_speed
            duration = self._compute_duration(target_positions, current, speed)

            self._publish_trajectory(self._left_traj_pub, joint_names, target_positions, duration)
            self.get_logger().info(
                f'左臂 MoveJ (Gazebo 适配): joints={[f"{math.degrees(j):.1f}°" for j in target_positions]}, '
                f'speed={speed}, duration={duration:.1f}s'
            )
        except Exception as e:
            self.get_logger().error(f'处理左臂 MoveJ 失败: {e}')

    def _on_right_movej(self, msg):
        """处理右臂 MoveJ 命令"""
        if self._Movej_msg is None or not isinstance(msg, self._Movej_msg):
            return

        try:
            joint_names = self.RIGHT_JOINT_NAMES
            target_positions = list(msg.joint[:6])

            current = self._get_current_positions(joint_names, self._right_positions)
            speed = int(msg.speed) if hasattr(msg, 'speed') and msg.speed else self.default_speed
            duration = self._compute_duration(target_positions, current, speed)

            self._publish_trajectory(self._right_traj_pub, joint_names, target_positions, duration)
            self.get_logger().info(
                f'右臂 MoveJ (Gazebo 适配): joints={[f"{math.degrees(j):.1f}°" for j in target_positions]}, '
                f'speed={speed}, duration={duration:.1f}s'
            )
        except Exception as e:
            self.get_logger().error(f'处理右臂 MoveJ 失败: {e}')


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(description='Gazebo 机械臂适配器')
    parser.add_argument('--left_ns', default='left_arm_controller',
                       help='左臂 namespace')
    parser.add_argument('--right_ns', default='right_arm_controller',
                       help='右臂 namespace')
    parser.add_argument('--speed', type=int, default=30,
                       help='默认速度 1-100')
    args, _ = parser.parse_known_args()

    rclpy.init(args=args)

    adapter = GazeboArmAdapter(
        left_ns=args.left_ns,
        right_ns=args.right_ns,
        default_speed=args.speed
    )

    print()
    print("=" * 60)
    print("         Gazebo 机械臂适配器")
    print("=" * 60)
    print()
    print("  等待 rm_driver 命令...")
    print("  收到命令后自动转换并发布到 JointTrajectoryController")
    print()
    print("  使用方法：")
    print("    # 在 Gazebo 中运行本适配器")
    print("    python3 guji/nodes/gazebo_arm_adapter.py")
    print()
    print("    # 然后使用原有控制器")
    print("    python3 guji/dual_arm_controller.py --use_gazebo")
    print("    python3 guji/dual_arm_pick_place.py --use_gazebo")
    print()
    print("=" * 60)
    print()

    try:
        rclpy.spin(adapter)
    except KeyboardInterrupt:
        print("\n\n适配器已退出")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
