#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
睿尔曼双机械臂关节角度控制程序

功能：
1. 控制左/右机械臂运动到指定关节角度
2. 支持同时控制两个机械臂
3. 显示机械臂当前状态
4. 支持 Gazebo 仿真模式

使用仿真模式：
  ros2 launch dual_rm_gazebo dual_rm_65b_gazebo.launch.py
  ros2 launch dual_rm_65b_moveit_config demo.launch.py
  python3 dual_arm_controller.py --use_gazebo

使用方法：
1. 启动两个机械臂驱动（使用不同的 namespace）
2. 运行本程序

前提条件：
- 安装 ROS2 Foxy
- 编译 ros2_ws 功能包
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

    def __init__(self, left_ns: str = "left_arm_controller",
                 right_ns: str = "right_arm_controller",
                 use_gazebo: bool = False,
                 use_sim_time: bool = False):
        super().__init__('dual_arm_controller')

        self.left_ns = left_ns
        self.right_ns = right_ns
        self.use_gazebo = use_gazebo
        self.use_sim_time = use_sim_time

        # Gazebo 仿真时使用不同的命名空间和话题
        if self.use_gazebo:
            self._setup_gazebo_mode()
        else:
            self._setup_real_mode()

        # 创建定时器，每秒更新一次显示
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'双机械臂控制节点已启动')
        self.get_logger().info(f'  左臂 namespace: {self.left_ns}')
        self.get_logger().info(f'  右臂 namespace: {self.right_ns}')
        if self.use_gazebo:
            self.get_logger().info(f'  模式: [Gazebo 仿真]')
            self.get_logger().warn('  警告: 运动命令发送到 JointTrajectoryController')

    def _setup_real_mode(self):
        """真实硬件模式：使用 rm_driver 消息接口"""
        self.left_ns = self.left_ns
        self.right_ns = self.right_ns

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
            self.get_logger().error('无法导入 rm_ros_interfaces.msg.Movej，请确保已编译 ros2_ws')
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

    def _setup_gazebo_mode(self):
        """Gazebo 仿真模式：使用 JointTrajectoryController"""
        self.get_logger().info('初始化 Gazebo 仿真模式...')

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
        # Gazebo 关节状态来源：直接订阅 /joint_states（Gazebo 发布）
        # ==========================================
        self.left_joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._gazebo_joint_callback,
            10
        )

        # ==========================================
        # Gazebo 使用 trajectory_msgs/JointTrajectoryGoal
        # ==========================================
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            self._Trajectory_msg = JointTrajectory
            self._TrajectoryPoint_msg = JointTrajectoryPoint
        except ImportError:
            self.get_logger().error('无法导入 trajectory_msgs.msg.JointTrajectory')
            return

        # ==========================================
        # 左臂 JointTrajectory 发布者
        # Gazebo 中的话题：/<namespace>/follow_joint_trajectory/goal
        # ==========================================
        self.left_movej_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.left_ns}/follow_joint_trajectory/goal',
            10
        )

        # ==========================================
        # 右臂 JointTrajectory 发布者
        # ==========================================
        self.right_movej_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.right_ns}/follow_joint_trajectory/goal',
            10
        )

    def _gazebo_joint_callback(self, msg: JointState):
        """
        Gazebo 模式下的关节状态回调
        从 /joint_states 中提取左右臂的关节角度
        """
        # Gazebo 中关节名称格式：l_joint1, l_joint2, ... 和 r_joint1, r_joint2, ...
        left_joints = []
        right_joints = []

        for name, pos in zip(msg.name, msg.position):
            if name.startswith('l_joint'):
                left_joints.append((name, pos))
            elif name.startswith('r_joint'):
                right_joints.append((name, pos))

        # 按关节序号排序
        left_joints.sort(key=lambda x: int(x[0].replace('l_joint', '')))
        right_joints.sort(key=lambda x: int(x[0].replace('r_joint', '')))

        if left_joints:
            self.left_joint_positions = [pos for _, pos in left_joints]
            self.has_left_joint = True

        if right_joints:
            self.right_joint_positions = [pos for _, pos in right_joints]
            self.has_right_joint = True

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
            speed: 速度 1-100，默认30（Gazebo 模式下此参数用于计算运动时间）
            block: 是否阻塞等待运动完成，默认True（Gazebo 模式下此参数被忽略）

        Gazebo 仿真模式：
            使用 JointTrajectory 消息，发布到 /<namespace>/follow_joint_trajectory/goal
        真实硬件模式：
            使用 rm_ros_interfaces/Movej 消息，发布到 /<namespace>/rm_driver/movej_cmd
        """
        if self.use_gazebo:
            self._movej_gazebo(arm, joints, speed)
        else:
            self._movej_real(arm, joints, speed, block)

    def _movej_real(self, arm: str, joints: list, speed: int = 30, block: bool = True):
        """真实硬件模式：使用 Movej 消息"""
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

    def _movej_gazebo(self, arm: str, joints: list, speed: int = 30):
        """Gazebo 仿真模式：使用 JointTrajectory 消息"""
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        except ImportError:
            self.get_logger().error('无法导入 trajectory_msgs.msg.JointTrajectory')
            return

        # 确定关节名列表（Gazebo 中的命名：l_joint1-l_joint6, r_joint1-r_joint6）
        if arm.lower() == 'left':
            joint_names = [f'l_joint{i}' for i in range(1, 7)]
            topic = f'/{self.left_ns}/follow_joint_trajectory/goal'
        elif arm.lower() == 'right':
            joint_names = [f'r_joint{i}' for i in range(1, 7)]
            topic = f'/{self.right_ns}/follow_joint_trajectory/goal'
        else:
            self.get_logger().error(f'无效的机械臂选择: {arm}，请使用 "left" 或 "right"')
            return

        # 计算运动时间（速度 1-100 -> 0.1s-3.0s）
        duration = 0.1 + (100 - speed) / 100.0 * 2.9  # 速度100=0.1s, 速度1=3.0s
        positions_rad = [math.radians(j) for j in joints[:6]]

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points = [JointTrajectoryPoint(
            positions=positions_rad,
            time_from_start=rclpy.duration.Duration(seconds=duration).to_msg()
        )]

        self.left_movej_pub.publish(traj) if arm.lower() == 'left' else self.right_movej_pub.publish(traj)
        self.get_logger().info(
            f'{"左" if arm.lower() == "left" else "右"}臂 MoveJ (Gazebo): '
            f'{[f"{j:.1f}°" for j in joints[:6]]}, 速度: {speed}%, 预计时长: {duration:.1f}s'
        )

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
        print(f"  模式: {'[Gazebo 仿真]' if self.use_gazebo else '[真实硬件]'}")
        print()
        print("-" * 65)
        print("  使用说明:")
        print("    controller.movej('left',  [j1, j2, j3, j4, j5, j6], speed=30)")
        print("    controller.movej('right', [j1, j2, j3, j4, j5, j6], speed=30)")
        print("    controller.movej_both(left_joints, right_joints, speed=30)")
        print("-" * 65)


def main(args=None):
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='睿尔曼双臂控制器')
    parser.add_argument('left_ns', nargs='?', default='left_arm_controller',
                       help='左臂 namespace（默认 left_arm_controller）')
    parser.add_argument('right_ns', nargs='?', default='right_arm_controller',
                       help='右臂 namespace（默认 right_arm_controller）')
    parser.add_argument('--use_gazebo', action='store_true',
                       help='使用 Gazebo 仿真模式')
    parser.add_argument('--use_sim_time', action='store_true',
                       help='使用仿真时间')
    parsed_args, _ = parser.parse_known_args()

    rclpy.init(args=args)

    try:
        # 创建控制器节点
        controller = DualArmController(
            left_ns=parsed_args.left_ns,
            right_ns=parsed_args.right_ns,
            use_gazebo=parsed_args.use_gazebo,
            use_sim_time=parsed_args.use_sim_time
        )

        # 打印使用说明
        print()
        print("=" * 65)
        print("         睿尔曼双机械臂控制程序")
        print("=" * 65)
        print()
        print("  Namespace 配置:")
        print(f"    左臂: /{parsed_args.left_ns}")
        print(f"    右臂: /{parsed_args.right_ns}")
        print(f"  模式: {'[Gazebo 仿真]' if parsed_args.use_gazebo else '[真实硬件]'}")
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
        print("  仿真模式使用:")
        print("    python3 dual_arm_controller.py --use_gazebo")
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
