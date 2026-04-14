#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
睿尔曼机械臂 ROS2 连接验证程序（支持双机械臂）

功能：
1. 验证与机械臂的 ROS2 通信连接
2. 实时显示单臂或双臂的关节角度
3. 实时显示末端位姿数据

使用方法：
1. 启动机械臂驱动
2. 运行本程序

  单臂: python3 arm_monitor.py
  双臂: python3 arm_monitor.py l_arm r_arm

依赖：
- ROS2 Humble
- rclpy
- sensor_msgs
- geometry_msgs
- rm_ros_interfaces
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
import sys
from datetime import datetime


class ArmMonitor(Node):
    """机械臂状态监控节点（支持单臂/双臂）"""

    def __init__(self, namespaces: list = None):
        """
        初始化监控节点

        参数:
            namespaces: namespace 列表，如 ['l_arm', 'r_arm']
                       默认 None 表示单臂（无 namespace）
        """
        super().__init__('arm_monitor')

        self.namespaces = namespaces if namespaces else []
        self.is_dual_arm = len(self.namespaces) == 2

        # 存储数据
        self.arm_data = {}  # {namespace: {joints: [], pose: Pose}}

        for ns in self.namespaces:
            self.arm_data[ns] = {
                'joint_positions': [],
                'pose': Pose(),
                'has_joint': False,
                'has_pose': False
            }

        # 单臂数据
        if not self.is_dual_arm:
            self.arm_data['_single_'] = {
                'joint_positions': [],
                'pose': Pose(),
                'has_joint': False,
                'has_pose': False
            }

        # ==========================================
        # 创建订阅者
        # ==========================================
        if self.is_dual_arm:
            # 双臂模式
            for ns in self.namespaces:
                self.create_subscription(
                    JointState,
                    f'/{ns}/joint_states',
                    lambda msg, n=ns: self.joint_callback(msg, n),
                    10
                )
                self.create_subscription(
                    Pose,
                    f'/{ns}/rm_driver/udp_arm_position',
                    lambda msg, n=ns: self.pose_callback(msg, n),
                    10
                )
        else:
            # 单臂模式（默认话题）
            self.joint_sub = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_callback_single,
                10
            )
            self.pose_sub = self.create_subscription(
                Pose,
                '/rm_driver/udp_arm_position',
                self.pose_callback_single,
                10
            )

        # 定时器
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'机械臂监控节点已启动')
        if self.is_dual_arm:
            self.get_logger().info(f'  模式: 双臂')
            self.get_logger().info(f'  Namespace: {self.namespaces}')
        else:
            self.get_logger().info(f'  模式: 单臂')

    # ==========================================
    # 双臂回调
    # ==========================================
    def joint_callback(self, msg: JointState, namespace: str):
        if len(msg.position) > 0:
            self.arm_data[namespace]['joint_positions'] = list(msg.position)
            self.arm_data[namespace]['has_joint'] = True

    def pose_callback(self, msg: Pose, namespace: str):
        self.arm_data[namespace]['pose'] = msg
        self.arm_data[namespace]['has_pose'] = True

    # ==========================================
    # 单臂回调
    # ==========================================
    def joint_callback_single(self, msg: JointState):
        if len(msg.position) > 0:
            self.arm_data['_single_']['joint_positions'] = list(msg.position)
            self.arm_data['_single_']['has_joint'] = True

    def pose_callback_single(self, msg: Pose):
        self.arm_data['_single_']['pose'] = msg
        self.arm_data['_single_']['has_pose'] = True

    # ==========================================
    # 定时回调
    # ==========================================
    def timer_callback(self):
        self.display_data()

    # ==========================================
    # 显示数据
    # ==========================================
    def display_data(self):
        """显示机械臂数据"""
        print("\033[2J\033[H")  # 清屏
        print("=" * 55)
        print("         睿尔曼机械臂 ROS2 连接验证")
        print(f"         时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 55)
        print()

        if self.is_dual_arm:
            self.display_dual_arm()
        else:
            self.display_single_arm()

        print()
        print("-" * 55)
        print("  按 Ctrl+C 退出程序")
        print("-" * 55)

    def display_single_arm(self):
        """显示单臂数据"""
        data = self.arm_data['_single_']

        # 连接状态
        if data['has_joint']:
            print("[✓] 关节状态连接正常 (/joint_states)")
        else:
            print("[✗] 等待关节状态数据... (/joint_states)")

        if data['has_pose']:
            print("[✓] 末端位姿连接正常 (/rm_driver/udp_arm_position)")
        else:
            print("[✗] 等待末端位姿数据... (/rm_driver/udp_arm_position)")

        print()
        print("-" * 55)
        print("实时数据:")
        print("-" * 55)

        # 关节角度
        print()
        print("  关节角度 (度):")
        if data['has_joint'] and len(data['joint_positions']) > 0:
            joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
            for i, pos in enumerate(data['joint_positions'][:6]):
                name = joint_names[i] if i < len(joint_names) else f'Joint{i+1}'
                print(f"    {name}: {math.degrees(pos):8.2f}°")
        else:
            print("    (无数据)")

        # 末端位姿
        print()
        print("  末端位姿:")
        if data['has_pose']:
            p = data['pose']
            print(f"    X: {p.position.x:.4f} m")
            print(f"    Y: {p.position.y:.4f} m")
            print(f"    Z: {p.position.z:.4f} m")
        else:
            print("    (无数据)")

        # 末端姿态
        print()
        print("  末端姿态 (四元数):")
        if data['has_pose']:
            q = data['pose'].orientation
            print(f"    W: {q.w:8.4f}")
            print(f"    X: {q.x:8.4f}")
            print(f"    Y: {q.y:8.4f}")
            print(f"    Z: {q.z:8.4f}")
        else:
            print("    (无数据)")

    def display_dual_arm(self):
        """显示双臂数据"""
        # 连接状态
        for ns in self.namespaces:
            data = self.arm_data[ns]
            status = "[✓]" if data['has_joint'] else "[✗]"
            print(f"{status} {ns}/joint_states")
        print()

        print("-" * 55)
        for idx, ns in enumerate(self.namespaces):
            data = self.arm_data[ns]
            arm_name = "左臂" if ns == self.namespaces[0] else "右臂"

            print(f"  {arm_name} ({ns}):")
            print("-" * 55)

            if data['has_joint'] and len(data['joint_positions']) > 0:
                for i, pos in enumerate(data['joint_positions'][:7]):
                    print(f"    Joint{i+1}: {math.degrees(pos):8.2f}°")
            else:
                print("    (无数据)")

            print()

    def print_startup_message(self):
        """打印启动信息"""
        print()
        print("=" * 55)
        print("         睿尔曼机械臂 ROS2 连接验证")
        print("=" * 55)
        print()
        print("  等待连接...")
        print()
        print("  请确保：")
        print("    1. 已启动机械臂驱动")
        if self.is_dual_arm:
            print(f"       ros2 launch arm_driver dual_arm_75_driver.launch.py")
        else:
            print("       ros2 launch rm_driver rm_65_driver.launch.py")
        print()
        print("    2. 机械臂与计算机网络互通")
        print()
        print("    3. 配置文件中的 IP 设置正确")
        print()
        print("=" * 55)
        print()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    # 解析命令行参数
    namespaces = None
    if len(sys.argv) >= 3:
        namespaces = [sys.argv[1], sys.argv[2]]
    elif len(sys.argv) == 2:
        # 单参数时作为单臂使用
        namespaces = None

    try:
        node = ArmMonitor(namespaces)
        node.print_startup_message()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n程序已退出")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
