#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
睿尔曼双机械臂关节角度控制程序

功能：
1. 控制左/右机械臂运动到指定关节角度
2. 支持同时控制两个机械臂
3. 显示机械臂当前状态

使用方法：
1. 启动两个机械臂驱动（使用不同的 namespace）
2. 运行本程序

前提条件：
- 安装 ROS2 Humble
- 编译 ros2_rm_robot 功能包
- 网络连接到两个机械臂
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
import sys
from datetime import datetime

# Movej 消息类型需要在 rclpy.init() 之后才能导入
# 这里使用延迟导入，在 Node 初始化时导入
Movej_msg = None


class DualArmController(Node):
    """双机械臂控制节点"""

    def __init__(self, left_ns: str = "l_arm", right_ns: str = "r_arm"):
        super().__init__('dual_arm_controller')

        self.left_ns = left_ns
        self.right_ns = right_ns

        # 存储最新的数据
        self.left_joint_positions = []
        self.right_joint_positions = []
        self.left_pose = Pose()
        self.right_pose = Pose()

        self.has_left_joint = False
        self.has_right_joint = False
        self.has_left_pose = False
        self.has_right_pose = False

        # ==========================================
        # 左臂订阅者 (订阅 /<namespace>/joint_states)
        # ==========================================
        self.left_joint_sub = self.create_subscription(
            JointState,
            f'/{self.left_ns}/joint_states',
            self.left_joint_callback,
            10
        )

        self.left_pose_sub = self.create_subscription(
            Pose,
            f'/{self.left_ns}/rm_driver/udp_arm_position',
            self.left_pose_callback,
            10
        )

        # ==========================================
        # 右臂订阅者
        # ==========================================
        self.right_joint_sub = self.create_subscription(
            JointState,
            f'/{self.right_ns}/joint_states',
            self.right_joint_callback,
            10
        )

        self.right_pose_sub = self.create_subscription(
            Pose,
            f'/{self.right_ns}/rm_driver/udp_arm_position',
            self.right_pose_callback,
            10
        )

        # ==========================================
        # 导入 Movej 消息类型（必须在 rclpy.init() 之后）
        # ==========================================
        global Movej_msg
        try:
            from rm_ros_interfaces.msg import Movej
            Movej_msg = Movej
        except ImportError:
            self.get_logger().error('无法导入 rm_ros_interfaces.msg.Movej，请确保已编译 ros2_rm_robot')
            return

        # ==========================================
        # 左臂发布者 (发布控制命令到 /<namespace>/rm_driver/movej_cmd)
        # ==========================================
        self.left_movej_pub = self.create_publisher(
            Movej_msg,
            f'/{self.left_ns}/rm_driver/movej_cmd',
            10
        )

        # ==========================================
        # 右臂发布者
        # ==========================================
        self.right_movej_pub = self.create_publisher(
            Movej_msg,
            f'/{self.right_ns}/rm_driver/movej_cmd',
            10
        )

        # 创建定时器，每秒更新一次显示
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'双机械臂控制节点已启动')
        self.get_logger().info(f'  左臂 namespace: {self.left_ns}')
        self.get_logger().info(f'  右臂 namespace: {self.right_ns}')

    # ==========================================
    # 回调函数
    # ==========================================
    def left_joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.left_joint_positions = list(msg.position)
            self.has_left_joint = True

    def right_joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.right_joint_positions = list(msg.position)
            self.has_right_joint = True

    def left_pose_callback(self, msg: Pose):
        self.left_pose = msg
        self.has_left_pose = True

    def right_pose_callback(self, msg: Pose):
        self.right_pose = msg
        self.has_right_pose = True

    # ==========================================
    # 控制函数
    # ==========================================
    def movej(self, arm: str, joints: list, speed: int = 30, block: bool = True):
        """
        控制机械臂运动到指定关节角度

        参数:
            arm: 'left' 或 'right'，选择要控制的机械臂
            joints: 关节角度列表（单位：度），6个或7个值
            speed: 速度 1-100，默认30
            block: 是否阻塞等待运动完成，默认True
        """
        global Movej_msg
        if Movej_msg is None:
            try:
                from rm_ros_interfaces.msg import Movej
                Movej_msg = Movej
            except ImportError:
                self.get_logger().error('无法导入 rm_ros_interfaces.msg.Movej')
                return

        msg = Movej_msg()
        msg.joint = [math.radians(j) for j in joints]  # 度转弧度
        msg.speed = float(speed)
        msg.block = block

        if arm.lower() == 'left':
            self.left_movej_pub.publish(msg)
            self.get_logger().info(f'左臂 MoveJ: {[f"{j:.1f}°" for j in joints]}, 速度: {speed}%')
        elif arm.lower() == 'right':
            self.right_movej_pub.publish(msg)
            self.get_logger().info(f'右臂 MoveJ: {[f"{j:.1f}°" for j in joints]}, 速度: {speed}%')
        else:
            self.get_logger().error(f'无效的机械臂选择: {arm}，请使用 "left" 或 "right"')

    def movej_both(self, left_joints: list, right_joints: list, speed: int = 30, block: bool = True):
        """
        同时控制两个机械臂运动

        参数:
            left_joints: 左臂关节角度列表（单位：度）
            right_joints: 右臂关节角度列表（单位：度）
            speed: 速度 1-100，默认30
            block: 是否阻塞等待运动完成，默认True
        """
        self.movej('left', left_joints, speed, False)
        self.movej('right', right_joints, speed, False)
        self.get_logger().info(f'双臂同时运动: 左臂={[f"{j:.1f}°" for j in left_joints]}, 右臂={[f"{j:.1f}°" for j in right_joints]}')

    # ==========================================
    # 显示函数
    # ==========================================
    def timer_callback(self):
        self.display_data()

    def display_data(self):
        """显示机械臂数据"""
        print("\033[2J\033[H")  # 清屏
        print("=" * 65)
        print("         睿尔曼双机械臂控制程序")
        print(f"         时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 65)
        print()

        # 连接状态
        print(f"  左臂 ({self.left_ns}):")
        if self.has_left_joint:
            print(f"    [✓] 关节状态正常")
        else:
            print(f"    [✗] 等待数据...")
        print(f"  右臂 ({self.right_ns}):")
        if self.has_right_joint:
            print(f"    [✓] 关节状态正常")
        else:
            print(f"    [✗] 等待数据...")

        print()
        print("-" * 65)
        print("  左臂关节角度:")
        print("-" * 65)
        if self.has_left_joint and len(self.left_joint_positions) > 0:
            for i, pos in enumerate(self.left_joint_positions[:7]):
                print(f"    Joint{i+1}: {math.degrees(pos):8.2f}°")
        else:
            print("    (无数据)")

        print()
        print("-" * 65)
        print("  右臂关节角度:")
        print("-" * 65)
        if self.has_right_joint and len(self.right_joint_positions) > 0:
            for i, pos in enumerate(self.right_joint_positions[:7]):
                print(f"    Joint{i+1}: {math.degrees(pos):8.2f}°")
        else:
            print("    (无数据)")

        print()
        print("-" * 65)
        print("  使用说明:")
        print("    controller.movej('left',  [j1, j2, j3, j4, j5, j6], speed=30)")
        print("    controller.movej('right', [j1, j2, j3, j4, j5, j6], speed=30)")
        print("    controller.movej_both(left_joints, right_joints, speed=30)")
        print("-" * 65)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    # 可以通过参数指定 namespace
    left_ns = 'l_arm'
    right_ns = 'r_arm'

    if len(sys.argv) > 1:
        left_ns = sys.argv[1]
    if len(sys.argv) > 2:
        right_ns = sys.argv[2]

    try:
        # 创建控制器节点
        controller = DualArmController(left_ns, right_ns)

        # 打印使用说明
        print()
        print("=" * 65)
        print("         睿尔曼双机械臂控制程序")
        print("=" * 65)
        print()
        print("  Namespace 配置:")
        print(f"    左臂: /{left_ns}")
        print(f"    右臂: /{right_ns}")
        print()
        print("  使用示例:")
        print("    # 控制左臂运动到指定关节角度")
        print("    controller.movej('left', [0, -45, 45, 0, 90, 0], speed=30)")
        print()
        print("    # 控制右臂运动到指定关节角度")
        print("    controller.movej('right', [0, -45, 45, 0, 90, 0], speed=30)")
        print()
        print("    # 同时控制两个机械臂")
        print("    controller.movej_both([0, -45, 45, 0, 90, 0], [0, -45, 45, 0, 90, 0], speed=30)")
        print()
        print("=" * 65)
        print()

        # 进入主循环
        rclpy.spin(controller)

    except KeyboardInterrupt:
        print("\n\n程序已退出")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
