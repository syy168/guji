#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
睿尔曼机械臂示教记录程序

功能：
1. 实时记录拖动示教过程中的关键数据
2. 支持关节角度、末端位姿、速度、力数据记录
3. 支持记录点位标记（用于标记关键动作如抓取、放置）
4. 支持数据导出为 CSV 格式
5. 支持实时显示记录状态

示教记录关键数据：
- 关节角度 (6-7轴)
- 末端执行器位置 (X, Y, Z)
- 末端执行器姿态 (四元数 + 欧拉角)
- 时间戳 (相对时间，秒)
- 关节速度
- 末端速度
- 六维力数据 (FX, FY, FZ, MX, MY, MZ)
- 点位标记 (用于标记关键动作)

使用方法：
1. 启动机械臂驱动
2. 运行程序: python3 teach_record.py
3. 按空格键标记关键点位 (如抓取点、放置点)
4. 按 S 键保存记录数据
5. 按 Q 键退出程序

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
import csv
import os
from datetime import datetime
from threading import Thread
import time

try:
    from rm_ros_interfaces.msg import Jointspeed, Sixforce
    HAS_RM_MSGS = True
except ImportError:
    HAS_RM_MSGS = False
    print("警告: 无法导入 rm_ros_interfaces，将跳过速度和力数据记录")


class TeachRecordNode(Node):
    """示教记录节点"""

    def __init__(self, namespace: str = ""):
        super().__init__('teach_record')

        self.ns = namespace.strip('/')
        self.prefix = f'/{self.ns}/' if self.ns else '/'

        # ==========================================
        # 数据存储
        # ==========================================
        self.recorded_points = []  # 记录的点位列表
        self.is_recording = False  # 是否正在记录
        self.record_start_time = None  # 记录开始时间
        self.point_index = 0  # 点位索引

        # 最新数据
        self.current_joints = []
        self.current_pose = Pose()
        self.current_joint_speeds = []
        self.current_six_force = None

        # 数据接收标志
        self.has_joint = False
        self.has_pose = False
        self.has_speed = False
        self.has_force = False

        # 夹爪状态 (可选，通过参数设置)
        self.gripper_state = False  # False=打开, True=关闭

        # ==========================================
        # 订阅者
        # ==========================================

        # 关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            f'{self.prefix}joint_states',
            self.joint_callback,
            10
        )

        # 末端位姿
        self.pose_sub = self.create_subscription(
            Pose,
            f'{self.prefix}rm_driver/udp_arm_position',
            self.pose_callback,
            10
        )

        # 关节速度 (可选)
        if HAS_RM_MSGS:
            self.speed_sub = self.create_subscription(
                Jointspeed,
                f'{self.prefix}rm_driver/udp_joint_speed',
                self.speed_callback,
                10
            )

            # 六维力数据 (可选)
            self.force_sub = self.create_subscription(
                Sixforce,
                f'{self.prefix}rm_driver/udp_six_force',
                self.force_callback,
                10
            )

        # ==========================================
        # 定时器 - 记录数据
        # ==========================================
        # 记录频率：50Hz (每20ms记录一次)
        self.record_timer = self.create_timer(0.02, self.record_callback)

        self.get_logger().info(f'示教记录节点已启动 (namespace: {self.ns or "默认"})')
        self.get_logger().info('提示: 按空格键标记关键点位, S键保存, Q键退出')

    # ==========================================
    # 回调函数
    # ==========================================
    def joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.current_joints = list(msg.position)
            self.has_joint = True

    def pose_callback(self, msg: Pose):
        self.current_pose = msg
        self.has_pose = True

    def speed_callback(self, msg):
        if hasattr(msg, 'joint_speed') and len(msg.joint_speed) > 0:
            self.current_joint_speeds = list(msg.joint_speed)
            self.has_speed = True

    def force_callback(self, msg):
        self.current_six_force = msg
        self.has_force = True

    # ==========================================
    # 记录回调
    # ==========================================
    def record_callback(self):
        """定时记录数据"""
        if not self.is_recording:
            return

        if not (self.has_joint and self.has_pose):
            return

        # 计算相对时间
        current_time = time.time()
        relative_time = current_time - self.record_start_time

        # 转换四元数为欧拉角
        euler = self.quaternion_to_euler(
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )

        # 构建点位数据
        point = {
            'index': self.point_index,
            'time': round(relative_time, 4),

            # 关节角度 (度)
            'joint_1': math.degrees(self.current_joints[0]) if len(self.current_joints) > 0 else 0,
            'joint_2': math.degrees(self.current_joints[1]) if len(self.current_joints) > 1 else 0,
            'joint_3': math.degrees(self.current_joints[2]) if len(self.current_joints) > 2 else 0,
            'joint_4': math.degrees(self.current_joints[3]) if len(self.current_joints) > 3 else 0,
            'joint_5': math.degrees(self.current_joints[4]) if len(self.current_joints) > 4 else 0,
            'joint_6': math.degrees(self.current_joints[5]) if len(self.current_joints) > 5 else 0,
            'joint_7': math.degrees(self.current_joints[6]) if len(self.current_joints) > 6 else 0,

            # 末端位置 (米)
            'pos_x': round(self.current_pose.position.x, 6),
            'pos_y': round(self.current_pose.position.y, 6),
            'pos_z': round(self.current_pose.position.z, 6),

            # 末端姿态 - 四元数
            'quat_w': round(self.current_pose.orientation.w, 6),
            'quat_x': round(self.current_pose.orientation.x, 6),
            'quat_y': round(self.current_pose.orientation.y, 6),
            'quat_z': round(self.current_pose.orientation.z, 6),

            # 末端姿态 - 欧拉角 (弧度)
            'roll': round(euler[0], 6),
            'pitch': round(euler[1], 6),
            'yaw': round(euler[2], 6),

            # 关节速度 (度/秒)
            'speed_1': math.degrees(self.current_joint_speeds[0]) if self.has_speed and len(self.current_joint_speeds) > 0 else 0,
            'speed_2': math.degrees(self.current_joint_speeds[1]) if self.has_speed and len(self.current_joint_speeds) > 1 else 0,
            'speed_3': math.degrees(self.current_joint_speeds[2]) if self.has_speed and len(self.current_joint_speeds) > 2 else 0,
            'speed_4': math.degrees(self.current_joint_speeds[3]) if self.has_speed and len(self.current_joint_speeds) > 3 else 0,
            'speed_5': math.degrees(self.current_joint_speeds[4]) if self.has_speed and len(self.current_joint_speeds) > 4 else 0,
            'speed_6': math.degrees(self.current_joint_speeds[5]) if self.has_speed and len(self.current_joint_speeds) > 5 else 0,
            'speed_7': math.degrees(self.current_joint_speeds[6]) if self.has_speed and len(self.current_joint_speeds) > 6 else 0,

            # 六维力数据
            'force_fx': round(self.current_six_force.force_fx, 4) if self.has_force and self.current_six_force else 0,
            'force_fy': round(self.current_six_force.force_fy, 4) if self.has_force and self.current_six_force else 0,
            'force_fz': round(self.current_six_force.force_fz, 4) if self.has_force and self.current_six_force else 0,
            'force_mx': round(self.current_six_force.force_mx, 4) if self.has_force and self.current_six_force else 0,
            'force_my': round(self.current_six_force.force_my, 4) if self.has_force and self.current_six_force else 0,
            'force_mz': round(self.current_six_force.force_mz, 4) if self.has_force and self.current_six_force else 0,

            # 夹爪状态 (0=打开, 1=关闭)
            'gripper': 1 if self.gripper_state else 0,

            # 点位标记 (0=普通点, 1=关键点)
            'marker': 0
        }

        self.recorded_points.append(point)
        self.point_index += 1

    # ==========================================
    # 四元数转欧拉角
    # ==========================================
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """四元数转欧拉角 (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # ==========================================
    # 控制函数
    # ==========================================
    def start_recording(self):
        """开始记录"""
        self.is_recording = True
        self.record_start_time = time.time()
        self.recorded_points = []
        self.point_index = 0
        self.get_logger().info('开始记录示教轨迹...')

    def stop_recording(self):
        """停止记录"""
        self.is_recording = False
        duration = time.time() - self.record_start_time if self.record_start_time else 0
        self.get_logger().info(f'停止记录，共记录 {len(self.recorded_points)} 个点位，耗时 {duration:.2f} 秒')

    def add_marker(self):
        """标记当前点为关键点位"""
        if len(self.recorded_points) > 0:
            self.recorded_points[-1]['marker'] = 1
            self.get_logger().info(f'已标记关键点位 (索引: {len(self.recorded_points)-1})')
        else:
            self.get_logger().warn('当前没有记录点位')

    def toggle_gripper(self):
        """切换夹爪状态"""
        self.gripper_state = not self.gripper_state
        state = "关闭" if self.gripper_state else "打开"
        self.get_logger().info(f'夹爪状态: {state}')
        # 记录夹爪状态变化点
        if len(self.recorded_points) > 0:
            self.recorded_points[-1]['gripper'] = 1 if self.gripper_state else 0

    def save_to_csv(self, filename: str = None):
        """保存记录数据到 CSV 文件"""
        if not self.recorded_points:
            self.get_logger().warn('没有记录数据可保存')
            return False

        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'teach_record_{timestamp}.csv'

        # 确保目录存在
        os.makedirs('records', exist_ok=True)
        filepath = os.path.join('records', filename)

        # CSV 列定义
        fieldnames = [
            # 基本信息
            'index', 'time', 'marker', 'gripper',

            # 关节角度 (度)
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7',

            # 末端位置 (米)
            'pos_x', 'pos_y', 'pos_z',

            # 末端姿态 - 四元数
            'quat_w', 'quat_x', 'quat_y', 'quat_z',

            # 末端姿态 - 欧拉角 (弧度)
            'roll', 'pitch', 'yaw',

            # 关节速度 (度/秒)
            'speed_1', 'speed_2', 'speed_3', 'speed_4', 'speed_5', 'speed_6', 'speed_7',

            # 六维力数据
            'force_fx', 'force_fy', 'force_fz',
            'force_mx', 'force_my', 'force_mz',
        ]

        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.recorded_points)

            self.get_logger().info(f'数据已保存到: {filepath}')
            self.get_logger().info(f'共 {len(self.recorded_points)} 个点位')
            return True
        except Exception as e:
            self.get_logger().error(f'保存失败: {e}')
            return False

    def get_statistics(self):
        """获取记录统计信息"""
        if not self.recorded_points:
            return None

        total_points = len(self.recorded_points)
        duration = self.recorded_points[-1]['time'] if total_points > 0 else 0
        avg_freq = total_points / duration if duration > 0 else 0
        marker_count = sum(1 for p in self.recorded_points if p['marker'] == 1)

        return {
            'total_points': total_points,
            'duration': duration,
            'avg_frequency': avg_freq,
            'marker_count': marker_count
        }


def run_input_thread(node: TeachRecordNode):
    """运行输入监听线程 (非阻塞输入)"""
    import sys
    import select

    print("\n" + "=" * 60)
    print("  示教记录 - 控制面板")
    print("=" * 60)
    print("  空格键: 标记关键点位")
    print("  G 键:   切换夹爪状态")
    print("  S 键:   保存数据到 CSV")
    print("  R 键:   开始/停止记录")
    print("  Q 键:   退出程序")
    print("=" * 60 + "\n")

    while rclpy.ok():
        if sys.platform == 'win32':
            import msvcrt
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key == b' ':
                    node.add_marker()
                elif key in [b'g', b'G']:
                    node.toggle_gripper()
                elif key in [b's', b'S']:
                    node.save_to_csv()
                elif key in [b'r', b'R']:
                    if node.is_recording:
                        node.stop_recording()
                    else:
                        node.start_recording()
                elif key in [b'q', b'Q']:
                    break
        else:
            import termios
            import tty
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == ' ':
                    node.add_marker()
                elif key in ['g', 'G']:
                    node.toggle_gripper()
                elif key in ['s', 'S']:
                    node.save_to_csv()
                elif key in ['r', 'R']:
                    if node.is_recording:
                        node.stop_recording()
                    else:
                        node.start_recording()
                elif key in ['q', 'Q']:
                    break

        time.sleep(0.05)


def display_thread(node: TeachRecordNode):
    """显示线程 - 实时显示记录状态"""
    while rclpy.ok():
        if node.is_recording:
            stats = node.get_statistics()
            if stats:
                print("\033[2J\033[H")  # 清屏
                print("=" * 60)
                print("         示教记录 - 实时显示")
                print("=" * 60)
                print(f"\n  记录状态: [Recording]")
                print(f"  点位数量: {stats['total_points']}")
                print(f"  记录时长: {stats['duration']:.2f} 秒")
                print(f"  记录频率: {stats['avg_frequency']:.1f} Hz")
                print(f"  关键点位: {stats['marker_count']}")
                print(f"  夹爪状态: {'关闭' if node.gripper_state else '打开'}")
                print()
                print("-" * 60)
                print("  当前关节角度 (度):")
                if node.has_joint and node.current_joints:
                    for i, joint in enumerate(node.current_joints[:7]):
                        print(f"    Joint{i+1}: {math.degrees(joint):8.2f}°")
                print()
                print("-" * 60)
                print("  末端位置 (米):")
                if node.has_pose:
                    print(f"    X: {node.current_pose.position.x:.4f}")
                    print(f"    Y: {node.current_pose.position.y:.4f}")
                    print(f"    Z: {node.current_pose.position.z:.4f}")
                print()
                print("-" * 60)
                print("  按 R 停止, 空格标记, S 保存, Q 退出")
                print("=" * 60)

        time.sleep(0.2)


def main(args=None):
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='睿尔曼机械臂示教记录程序')
    parser.add_argument('--namespace', '-n', type=str, default='',
                        help='机械臂 namespace (如 l_arm, r_arm)')
    parser.add_argument('--auto-start', '-a', action='store_true',
                        help='程序启动后自动开始记录')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=args)

    try:
        node = TeachRecordNode(namespace=args.namespace)

        # 如果指定了自动开始
        if args.auto_start:
            node.start_recording()

        # 启动输入线程
        input_thread = Thread(target=run_input_thread, args=(node,))
        input_thread.daemon = True
        input_thread.start()

        # 启动显示线程
        display_t = Thread(target=display_thread, args=(node,))
        display_t.daemon = True
        display_t.start()

        # 主循环
        print("\n" + "=" * 60)
        print("  睿尔曼机械臂示教记录程序")
        print(f"  Namespace: {args.namespace or '默认'}")
        print("=" * 60)
        print("\n按 R 开始记录，Q 退出...\n")

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\n程序已退出")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        # 退出前保存数据
        if 'node' in dir() and node.recorded_points:
            save = input('\n是否保存未保存的记录数据? (Y/n): ')
            if save.lower() != 'n':
                node.save_to_csv()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
