#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机械臂轨迹回放程序

功能：
1. 读取轨迹文件 (支持 CSV 和 JSON Lines 两种格式)
   - CSV 格式: teach_record.py 记录的轨迹 (单位: 度数)
   - JSON Lines 格式: 原始编码值 (编码值/1000 = 度数)
2. 选择左臂或右臂
3. 将当前位置分段移动到轨迹的起点
4. 按照记录的轨迹重新发送关节命令

使用方法：
1. 启动机械臂驱动
2. 运行程序: python3 trajectory_player.py
3. 选择臂 (left/right)
4. 输入轨迹文件路径 (相对于 records/ 目录或绝对路径)
5. 按 's' 键逐段确认移动到起点
6. 按 's' 键开始回放轨迹

文件格式支持:
- CSV: 度数格式 (joint_1, joint_2, ..., joint_6 列)
- JSON Lines / TXT: 编码值格式 {"point":[val1,val2,val3,val4,val5,val6]}

依赖：
- ROS2
- rclpy
- sensor_msgs
- rm_ros_interfaces
"""

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import Movej
from sensor_msgs.msg import JointState
import time
import sys
import csv
import os
import math
import json
from pathlib import Path


# ===== 配置参数 =====
# 设置默认机械臂 ('left' 或 'right')，为 None 时进行交互式选择
DEFAULT_ARM = 'right'

# 设置默认轨迹文件路径，为 None 时进行交互式选择
# 使用相对路径或绝对路径都可以
# 例如: DEFAULT_TRAJECTORY_FILE = 'c:/Users/suo/Desktop/realman/new/123888.txt'
# 或:    DEFAULT_TRAJECTORY_FILE = '/path/to/trajectory.txt'
# 或:    DEFAULT_TRAJECTORY_FILE = 'records/trajectory_001.csv'
DEFAULT_TRAJECTORY_FILE = 'guji/123888.txt'
# ==================


class TrajectoryPlayer(Node):
    def __init__(self, arm='right', trajectory_file=None):
        super().__init__('trajectory_player')

        self.arm = arm.lower()  # 'left' or 'right'
        self.trajectory_file = trajectory_file
        self.trajectory_data = []  # 轨迹数据列表
        
        # 创建发布者
        arm_prefix = 'r' if self.arm == 'right' else 'l'
        self.movej_pub = self.create_publisher(
            Movej,
            f'/{self.arm}_arm_controller/rm_driver/movej_cmd',
            10
        )
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # 当前关节角度 (弧度)
        self.current_joints = {}
        
        # 关节前缀（用于查询当前状态）
        self.joint_prefix = f'{arm_prefix}_joint'
        
        self.get_logger().info(f'🤖 轨迹回放程序已启动 (臂: {self.arm})')
        
        # 等待获取初始状态
        self.wait_for_joint_data()
        
        # 加载轨迹文件
        if self.trajectory_file:
            self.load_trajectory(self.trajectory_file)
        else:
            self.interactive_load_trajectory()
        
        if self.trajectory_data:
            self.interactive_playback()

    def joint_callback(self, msg):
        """关节状态回调函数"""
        for name, position in zip(msg.name, msg.position):
            if name.startswith(self.joint_prefix):
                self.current_joints[name] = position

    def wait_for_joint_data(self):
        """等待获取初始关节状态"""
        self.get_logger().info(f'⏳ 正在读取{self.arm}臂的当前姿态...')
        start_time = time.time()
        timeout = 5.0
        expected_joints = 6  # 通常为6轴
        
        while len(self.current_joints) < expected_joints:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⚠️  只获取了 {len(self.current_joints)} 个关节，继续执行...')
                break
        
        self.get_logger().info('✅ 成功获取当前姿态！')

    def get_current_joint_array(self):
        """获取当前关节角度数组 (弧度)"""
        rclpy.spin_once(self, timeout_sec=0.1)
        try:
            joints = []
            for i in range(1, 7):
                joint_name = f'{self.joint_prefix}{i}'
                if joint_name in self.current_joints:
                    joints.append(self.current_joints[joint_name])
                else:
                    self.get_logger().warn(f'⚠️  未找到关节: {joint_name}')
                    return None
            return joints
        except Exception as e:
            self.get_logger().error(f'❌ 获取关节数据失败: {e}')
            return None

    def interactive_load_trajectory(self):
        """交互式加载轨迹文件"""
        print('\n' + '='*60)
        print('🎬 轨迹文件加载')
        print('='*60)
        
        while True:
            # 列出已有的轨迹文件
            records_dir = Path('records')
            if records_dir.exists():
                csv_files = sorted(records_dir.glob('*.csv'))
                if csv_files:
                    print('\n📁 已有的轨迹文件:')
                    for i, f in enumerate(csv_files, 1):
                        file_size = f.stat().st_size / 1024
                        print(f'  {i}. {f.name} ({file_size:.1f} KB)')
            
            print('\n请输入轨迹文件路径:')
            print('  - 可以输入文件编号 (如果上面有列表)')
            print('  - 可以输入相对路径 (如: records/trajectory.csv)')
            print('  - 可以输入绝对路径')
            
            user_input = input('\n轨迹文件: ').strip()
            
            if not user_input:
                print('❌ 输入为空，请重试')
                continue
            
            # 处理编号输入
            try:
                idx = int(user_input)
                if records_dir.exists():
                    csv_files = sorted(records_dir.glob('*.csv'))
                    if 1 <= idx <= len(csv_files):
                        filepath = csv_files[idx - 1]
                    else:
                        print('❌ 编号不在范围内')
                        continue
                else:
                    print('❌ records 目录不存在')
                    continue
            except ValueError:
                # 使用路径输入
                if not user_input.startswith('/'):
                    filepath = Path('records') / user_input
                else:
                    filepath = Path(user_input)
            
            if self.load_trajectory(str(filepath)):
                break
            else:
                print('❌ 加载失败，请重试\n')

    def load_trajectory(self, filepath):
        """加载轨迹文件 (支持 CSV 和 JSON Lines 格式)"""
        filepath = Path(filepath)
        
        if not filepath.exists():
            self.get_logger().error(f'❌ 文件不存在: {filepath}')
            return False
        
        try:
            self.trajectory_data = []
            
            # 根据文件扩展名判断格式
            if filepath.suffix.lower() in ['.json', '.jsonl', '.txt']:
                # JSON Lines 格式
                self._load_json_trajectory(filepath)
            else:
                # CSV 格式
                self._load_csv_trajectory(filepath)
            
            self.get_logger().info(f'✅ 成功加载轨迹文件: {filepath}')
            self.get_logger().info(f'📊 轨迹包含 {len(self.trajectory_data)} 个点位')
            
            if self.trajectory_data:
                # 显示轨迹信息
                first_point = self.trajectory_data[0]
                last_point = self.trajectory_data[-1]
                print(f'\n轨迹信息:')
                print(f'  起点关节: J1={first_point["joint_1"]:.2f}°, '
                      f'J2={first_point["joint_2"]:.2f}°, '
                      f'J3={first_point["joint_3"]:.2f}°')
                print(f'  终点关节: J1={last_point["joint_1"]:.2f}°, '
                      f'J2={last_point["joint_2"]:.2f}°, '
                      f'J3={last_point["joint_3"]:.2f}°')
            
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 加载轨迹文件失败: {e}')
            return False

    def _load_json_trajectory(self, filepath):
        """加载 JSON Lines 格式的轨迹文件
        
        格式: {"point":[encoding1, encoding2, ..., encoding6]}
        转换: angle_degrees = encoding_value / 1000
        """
        with open(filepath, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                try:
                    line = line.strip()
                    if not line:
                        continue
                    
                    data = json.loads(line)
                    
                    if 'point' in data and len(data['point']) >= 6:
                        # 编码值转为度数 (除以1000)
                        angles_deg = [val / 1000.0 for val in data['point'][:6]]
                        
                        # 构造与 CSV 格式兼容的字典
                        point_dict = {
                            'index': line_num - 1,
                            'time': (line_num - 1) * 0.05,  # 假设采样频率 20Hz
                            'joint_1': angles_deg[0],
                            'joint_2': angles_deg[1],
                            'joint_3': angles_deg[2],
                            'joint_4': angles_deg[3],
                            'joint_5': angles_deg[4],
                            'joint_6': angles_deg[5],
                        }
                        
                        self.trajectory_data.append(point_dict)
                except json.JSONDecodeError as e:
                    self.get_logger().warn(f'⚠️  行 {line_num} JSON 解析失败: {e}')
                    continue

    def _load_csv_trajectory(self, filepath):
        """加载 CSV 格式的轨迹文件"""
        with open(filepath, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.trajectory_data.append(row)

    def degrees_to_radians(self, degrees):
        """将度转换为弧度"""
        return [d * math.pi / 180.0 for d in degrees]

    def move_to_joint_angles(self, joint_angles_rad, wait_time=3.0):
        """移动到指定的关节角度"""
        cmd = Movej()
        cmd.joint = joint_angles_rad
        cmd.speed = 20  # 设置速度 (0-100)
        
        self.movej_pub.publish(cmd)
        self.get_logger().info(f'📤 已发送关节命令')
        time.sleep(wait_time)

    def plan_segmented_path(self, current_joints, target_joints, num_segments=5):
        """
        规划分段路径 (使用余弦平滑插值/S型曲线)
        
        Args:
            current_joints: 当前关节角度 (弧度)
            target_joints: 目标关节角度 (弧度)
            num_segments: 分段数
        
        Returns:
            路径点列表，每个点都是一个关节角度列表
        """
        path = []
        for i in range(1, num_segments + 1):
            # 原始的线性进度 (0 到 1)
            t_linear = i / num_segments
            
            # 转换为余弦平滑进度 (S型曲线)
            # 当 t_linear 从 0 变到 1 时，t_smooth 会缓慢启动，中间加速，最后缓慢停止
            t_smooth = (1 - math.cos(t_linear * math.pi)) / 2.0
            
            segment_point = [
                current_joints[j] + (target_joints[j] - current_joints[j]) * t_smooth
                for j in range(len(current_joints))
            ]
            path.append(segment_point)
        return path
    
    
    def interactive_playback(self):
        """交互式轨迹回放"""
        print('\n' + '='*60)
        print('🎬 轨迹回放序列')
        print('='*60 + '\n')
        
        # 第一步：移动到起点
        if not self.trajectory_data:
            self.get_logger().error('❌ 没有轨迹数据')
            return
        
        first_point = self.trajectory_data[0]
        
        # 提取起点的关节角度 (从度转换为弧度)
        first_joints_deg = [
            float(first_point.get('joint_1', 0)),
            float(first_point.get('joint_2', 0)),
            float(first_point.get('joint_3', 0)),
            float(first_point.get('joint_4', 0)),
            float(first_point.get('joint_5', 0)),
            float(first_point.get('joint_6', 0)),
        ]
        first_joints_rad = self.degrees_to_radians(first_joints_deg)
        
        # 获取当前关节角度
        current_joints_rad = self.get_current_joint_array()
        if not current_joints_rad:
            self.get_logger().error('❌ 无法获取当前关节状态')
            return
        
        # 显示对比信息
        print('【第一步：移动到轨迹起点 (平滑移动)】\n')
        print('当前位置 (度) -> 目标起点 (度)')
        print('-' * 70)
        for i in range(6):
            current_deg = current_joints_rad[i] * 180 / math.pi
            target_deg = first_joints_deg[i]
            delta = abs(target_deg - current_deg)
            print(f'J{i+1}: {current_deg:8.2f}° -> {target_deg:8.2f}° (Δ {delta:6.2f}°)')
        print('-' * 70)
        
        # 确认移动
        user_input = input('\n✋ 确认无误请按 s 键执行移动 (其他键取消): ')
        
        if user_input.strip().lower() != 's':
            print('❌ 已取消启动序列，退出程序')
            return
        
        # 直接平滑移动到起点（一条平滑路径，不再分段）
        print(f'\n⏳ 执行平滑移动到轨迹起点...')
        self.move_to_joint_angles(first_joints_rad, wait_time=3.0)
        print(f'\n✅ 已到达轨迹起点！')
        
        # 第二步：回放轨迹
        # 第二步：回放轨迹
        print('\n' + '='*60)
        print('【第二步：开始回放轨迹】\n')
        print(f'🎬 即将回放 {len(self.trajectory_data)} 个点位的轨迹')
        
        user_input = input('若确认开始回放，请按 s 键并回车 (其他任意键取消): ')
        
        if user_input.strip().lower() != 's':
            self.get_logger().warn('⏭️  已取消回放，退出程序')
            return
        
        print('\n🎬 开始分步回放轨迹 (分成 5 段，每段需要确认)...\n')
        
        # 从轨迹中均匀提取 5 个分段点（每段的终点）
        total_points = len(self.trajectory_data)
        segment_size = total_points // 5
        
        # 提取 5 个分段终点
        segment_indices = []
        segment_joint_target_deg = []
        
        for seg_num in range(1, 6):
            if seg_num < 5:
                point_idx = seg_num * segment_size - 1
            else:
                point_idx = total_points - 1
            
            segment_indices.append(point_idx)
            point = self.trajectory_data[point_idx]
            joints_deg = [
                float(point.get('joint_1', 0)),
                float(point.get('joint_2', 0)),
                float(point.get('joint_3', 0)),
                float(point.get('joint_4', 0)),
                float(point.get('joint_5', 0)),
                float(point.get('joint_6', 0)),
            ]
            segment_joint_target_deg.append(joints_deg)
        
        # 显示所有分段终点
        print(f'轨迹分段终点:')
        print('=' * 70)
        for i, idx in enumerate(segment_indices, 1):
            print(f'\n第 {i} 段终点 (点 {idx}):')
            print('  关节 |     目标位置 (°)    ')
            print('  ' + '-'*40)
            for j in range(6):
                print(f'   J{j+1}  | {segment_joint_target_deg[i-1][j]:15.2f}°')
        print('\n' + '=' * 70)
        
        # 一次性确认所有分段
        user_input = input('\n✋ 确认无误请按 s 键，将自动执行全部 5 段回放 (其他键取消): ')
        
        if user_input.strip().lower() != 's':
            print('❌ 已取消回放，退出程序')
            return
        
        # 自动执行所有分段
        for seg_num in range(1, 6):
            target_joints_deg = segment_joint_target_deg[seg_num - 1]
            target_joints_rad = self.degrees_to_radians(target_joints_deg)
            
            print(f'\n⏳ 执行第 {seg_num}/5 段...')
            self.move_to_joint_angles(target_joints_rad, wait_time=2.0)
            print(f'✅ 第 {seg_num} 段完成！')
        
        print(f'\n✅ 轨迹回放完成！共 {len(self.trajectory_data)} 个点位')


def main(args=None):
    rclpy.init(args=args)
    
    # 交互式选择臂 (如果未指定默认值)
    if DEFAULT_ARM is None:
        print('\n' + '='*60)
        print('🤖 机械臂轨迹回放程序')
        print('='*60 + '\n')
        
        while True:
            arm_input = input('请选择机械臂 (left/right): ').strip().lower()
            if arm_input in ['left', 'right', 'l', 'r']:
                arm = 'left' if arm_input in ['left', 'l'] else 'right'
                break
            else:
                print('❌ 输入无效，请输入 left 或 right')
    else:
        arm = DEFAULT_ARM.lower()
        print('\n' + '='*60)
        print('🤖 机械臂轨迹回放程序')
        print('='*60 + '\n')
        print(f'✅ 使用配置的默认机械臂: {arm}臂\n')
    
    try:
        # 如果配置了默认轨迹文件路径，使用它；否则传 None 进行交互式选择
        node = TrajectoryPlayer(arm=arm, trajectory_file=DEFAULT_TRAJECTORY_FILE)
    except KeyboardInterrupt:
        print('\n\n⏹️  用户中断程序')
    except Exception as e:
        print(f'\n❌ 程序出错: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
