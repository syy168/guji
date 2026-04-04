#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
睿尔曼机械臂关节角度读取程序 (使用 Python SDK RM_API2)

功能：
- 通过 TCP 直接连接机械臂读取关节角度
- 无需 ROS2

使用方法：

方法1 - 创建软链接（推荐）:
    cd ~/Documents/cs/realman/guji
    ln -s ../RM_API2/Python/Robotic_Arm Robotic_Arm
    python3 rm_joint_reader.py

方法2 - 直接运行:
    python3 rm_joint_reader.py
    (需要设置 PYTHONPATH 或在有软链接的情况下运行)

依赖：
- RM_API2 SDK (位于 ../RM_API2/Python/Robotic_Arm/)
- Python 3.8+
"""

import sys
import os
import time
import math

# 获取脚本所在目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# SDK 路径
SDK_PATHS = [
    os.path.join(SCRIPT_DIR, 'Robotic_Arm'),                    # 软链接方式
    os.path.join(SCRIPT_DIR, '..', 'RM_API2', 'Python', 'Robotic_Arm'),  # 原始路径
]

SDK_PATH = None
for p in SDK_PATHS:
    if os.path.exists(p):
        SDK_PATH = os.path.abspath(p)
        break

if SDK_PATH:
    sys.path.insert(0, SDK_PATH)
    print(f"SDK 路径: {SDK_PATH}", file=sys.stderr)
else:
    print(f"错误: SDK 路径不存在", file=sys.stderr)
    print(f"  请创建软链接:", file=sys.stderr)
    print(f"    cd {SCRIPT_DIR}", file=sys.stderr)
    print(f"    ln -s ../RM_API2/Python/Robotic_Arm Robotic_Arm", file=sys.stderr)
    sys.exit(1)


def import_sdk():
    """
    导入 SDK，支持两种方式:
    1. 作为包导入 (需要软链接)
    2. 直接导入 rm_ctypes_wrap
    """
    # 方式1: 尝试作为包导入
    try:
        # 创建 __init__.py 使其成为包
        init_file = os.path.join(SDK_PATH, '__init__.py')
        if not os.path.exists(init_file):
            with open(init_file, 'w') as f:
                f.write('# Package init\n')

        # 导入主模块
        sys.path.insert(0, os.path.dirname(SDK_PATH))
        import Robotic_Arm.rm_robot_interface as rm_interface
        return rm_interface, 'package'
    except (ImportError, ModuleNotFoundError):
        pass

    # 方式2: 直接导入 ctypes 封装版本
    try:
        import rm_ctypes_wrap as rm_interface
        return rm_interface, 'direct'
    except ImportError as e:
        print(f"错误: 无法导入 SDK: {e}", file=sys.stderr)
        sys.exit(1)


# 导入 SDK
rm, import_mode = import_sdk()
print(f"导入方式: {import_mode}", file=sys.stderr)


class RMJointReader:
    """机械臂关节角度读取器"""

    def __init__(self, ip: str = "192.168.1.25", port: int = 8080):
        self.ip = ip
        self.port = port
        self.handle = None
        self.connected = False

    def connect(self) -> bool:
        """连接机械臂"""
        try:
            if import_mode == 'package':
                # RoboticArm 类方式
                self.robot = rm.RoboticArm(rm.rm_thread_mode_e.RM_TRIPLE_MODE_E)
                print(f"正在连接 {self.ip}:{self.port}...", file=sys.stderr)
                self.robot.rm_create_robot_arm(self.ip, self.port)
                # 检查连接状态: handle.contents.id != -1 表示成功
                if self.robot.handle.contents.id != -1:
                    self.connected = True
                    print(f"[✓] 已连接到 {self.ip}:{self.port}", file=sys.stderr)
                    print(f"    机械臂 ID: {self.robot.handle.contents.id}", file=sys.stderr)
                    return True
                else:
                    print(f"[✗] 连接失败: 机械臂 ID = -1", file=sys.stderr)
                    return False
            else:
                # 直接调用 API
                self.robot = rm
                self.handle = rm.rm_create_robot_arm(self.ip, self.port)
                if self.handle is not None and self.handle.contents.id != -1:
                    self.connected = True
                    print(f"[✓] 已连接到 {self.ip}:{self.port}", file=sys.stderr)
                    return True
                else:
                    print(f"[✗] 连接失败: handle 为 None 或 ID = -1", file=sys.stderr)
                    return False

        except Exception as e:
            import traceback
            print(f"[✗] 连接异常: {e}", file=sys.stderr)
            traceback.print_exc()
            return False

    def disconnect(self):
        """断开连接"""
        if self.connected:
            try:
                if import_mode == 'package':
                    self.robot.rm_delete_robot_arm()
                else:
                    rm.rm_delete_robot_arm(self.handle)
            except:
                pass
            self.connected = False
            print("[*] 已断开", file=sys.stderr)

    def get_joint_angles(self) -> list:
        """获取关节角度（度）"""
        if not self.connected:
            return []

        try:
            if import_mode == 'package':
                ret, joints = self.robot.rm_get_joint_degree()
            else:
                ret, joints = rm.rm_get_joint_degree(self.handle)

            if ret == 0:
                return list(joints)
            return []

        except Exception as e:
            print(f"[!] 读取异常: {e}", file=sys.stderr)
            return []

    def get_tcp_position(self) -> list:
        """获取 TCP 位置 [x, y, z, rx, ry, rz]"""
        if not self.connected:
            return []

        try:
            if import_mode == 'package':
                ret, pose = self.robot.rm_get_tcp_position()
            else:
                ret, pose = rm.rm_get_tcp_position(self.handle)

            if ret == 0:
                return list(pose)
            return []

        except Exception as e:
            return []

    def is_moving(self) -> bool:
        """检查机械臂是否在运动"""
        if not self.connected:
            return False

        try:
            if import_mode == 'package':
                ret, state = self.robot.rm_get_arm_all_state()
            else:
                ret, state = rm.rm_get_arm_all_state(self.handle)

            if ret == 0 and isinstance(state, dict):
                return state.get('motion_state', 0) == 1
            return False

        except:
            return False


def print_joints(joints: list):
    """打印关节角度"""
    if not joints:
        print("  关节角度: (无数据)")
        return

    print("  关节角度 (°):")
    names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7']
    for i, angle in enumerate(joints[:7]):
        name = names[i] if i < len(names) else f'J{i+1}'
        print(f"    {name}: {angle:8.2f}°")


def print_pose(pose: list):
    """打印末端位置"""
    if not pose or len(pose) < 6:
        print("  末端位置: (无数据)")
        return

    print("  末端位置:")
    print(f"    X:  {pose[0]:8.4f} m")
    print(f"    Y:  {pose[1]:8.4f} m")
    print(f"    Z:  {pose[2]:8.4f} m")
    print(f"    Rx: {math.degrees(pose[3]):8.2f}°")
    print(f"    Ry: {math.degrees(pose[4]):8.2f}°")
    print(f"    Rz: {math.degrees(pose[5]):8.2f}°")


def main():
    import argparse

    parser = argparse.ArgumentParser(description='读取机械臂关节角度')
    parser.add_argument('--ip', '-i', default='192.168.150.112', help='机械臂 IP')
    parser.add_argument('--port', '-p', type=int, default=8080, help='TCP 端口')
    parser.add_argument('--interval', '-t', type=float, default=0.2, help='读取间隔(秒)')
    parser.add_argument('--once', '-o', action='store_true', help='只读取一次')
    args = parser.parse_args()

    reader = RMJointReader(args.ip, args.port)

    if not reader.connect():
        sys.exit(1)

    try:
        if args.once:
            # 单次读取
            print("\n" + "=" * 50)
            print("  关节角度读取结果")
            print("=" * 50)
            joints = reader.get_joint_angles()
            print_joints(joints)
            print()
            pose = reader.get_tcp_position()
            print_pose(pose)
            print()

        else:
            # 持续读取
            print("\n持续读取中，按 Ctrl+C 退出...\n")
            while True:
                joints = reader.get_joint_angles()
                pose = reader.get_tcp_position()

                # 清屏
                print("\033[2J\033[H", end='')

                print(f"连接: {args.ip}:{args.port}")
                print("-" * 50)
                print_joints(joints)
                print()
                print_pose(pose)
                print("-" * 50)
                print(f"时间: {time.strftime('%H:%M:%S')}")

                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n")
    finally:
        reader.disconnect()


if __name__ == '__main__':
    main()
