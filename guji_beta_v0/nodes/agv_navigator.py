#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AGV 导航控制器

功能：
- 封装 Woosh AGV 底盘的导航通信接口
- 提供点位导航（_navigation_plan）和等待完成（_navigation_wait）接口
- 订阅底盘状态话题（task_state）监听任务完成事件
- 支持充电导航、货架导航等多种导航类型
- 线程安全，支持与机械臂操作并行执行

设计参考：Warehouse_handling_robot/body_handling_demo/scripts/body_handling_action.py

TF 关系链：
  world ──► agv_base ──► right_base / left_base ──► right_top / left_top

使用方法：
  # 方式1：组合调用（规划+等待）
  nav = AGVNavigator()
  nav.navigate_wait('110')  # 导航到点位并等待完成

  # 方式2：分离调用（支持中间逻辑）
  carry = nav.navigate('110')
  # ... 中间处理逻辑（如调整升降机高度）...
  nav.wait_until_reached(carry)

  # 方式3：在主控制器中集成
  from guji.nodes.agv_navigator import AGVNavigator
  class MyController(Node):
      def __init__(self):
          super().__init__()
          self.nav = AGVNavigator()

      def execute(self):
          if not self.nav.navigate_wait('110'):
              self.get_logger().error('导航失败')
              return
          self.pick_workpiece()
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import time
from datetime import datetime

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class AGVNavigator(Node):
    """
    AGV 导航控制器

    封装 Woosh AGV 底盘的导航通信接口，提供：
    - navigate(mark_no, task_type) / navigate_wait(mark_no, task_type)：点位导航
    - cancel_navigation()：取消当前导航任务
    - wait_until_reached(carry, timeout)：阻塞等待到达
    - set_speed(speed)：设置底盘移动速度
    """

    # Woosh 底盘 task_state 状态值
    TASK_STATE_IDLE = 0          # 空闲
    TASK_STATE_RUNNING = 1       # 任务执行中
    TASK_STATE_PAUSED = 2        # 任务暂停
    TASK_STATE_COMPLETED = 7     # 任务完成
    TASK_STATE_FAILED = 8         # 任务失败

    def __init__(self,
                 agv_base_frame: str = 'agv_base',
                 mark_topic: str = 'goto_mark/go',
                 result_topic: str = 'goto_mark/result',
                 status_topic: str = 'robot_status',
                 speed_topic: str = 'cmd_vel',
                 default_timeout: float = 120.0):
        """
        初始化 AGV 导航控制器

        参数:
            agv_base_frame: AGV 基座坐标系名称
            mark_topic: 导航目标点发布话题
            result_topic: 导航结果订阅话题
            status_topic: 底盘状态订阅话题（task_state）
            speed_topic: 底盘速度控制话题
            default_timeout: 默认导航超时时间（秒）
        """
        super().__init__('agv_navigator')

        self.agv_base_frame = agv_base_frame
        self.mark_topic = mark_topic
        self.result_topic = result_topic
        self.status_topic = status_topic
        self.speed_topic = speed_topic
        self.default_timeout = default_timeout

        # ==========================================
        # 线程同步事件
        # ==========================================
        self._wait_navigation_event = threading.Event()
        self._navigation_succeeded = False
        self._current_mark_no = ''

        # ==========================================
        # 导航状态
        # ==========================================
        self._is_navigating = False
        self._task_state = self.TASK_STATE_IDLE
        self._robot_mode = 0        # 0=未知, 1=自动, 2=手动
        self._robot_state = 0      # 0=未定义, 1=未初始化, 2=空闲, 8=已初始化
        self._current_pose_x = 0.0
        self._current_pose_y = 0.0
        self._current_pose_theta = 0.0

        # ==========================================
        # 发布者
        # ==========================================
        # 导航目标点发布（String 类型，发布目标点位名称）
        self._mark_pub = self.create_publisher(String, self.mark_topic, 10)

        # 底盘速度控制（用于手动微调或停止）
        self._speed_pub = self.create_publisher(Twist, self.speed_topic, 10)

        # ==========================================
        # 订阅者
        # ==========================================
        # 导航结果订阅（String 类型，收到消息表示到达）
        qos_result = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._result_sub = self.create_subscription(
            String, self.result_topic, self._result_callback, qos_result
        )

        # 底盘状态订阅（Int32 类型，task_state）
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._status_sub = self.create_subscription(
            Int32, self.status_topic, self._status_callback, qos_status
        )

        # 底盘里程计订阅（JointState 包含 x, y, theta）
        self._odom_sub = self.create_subscription(
            JointState, '/odom', self._odom_callback, 10
        )

        # ==========================================
        # 定时器：定期检查导航状态
        # ==========================================
        self._check_timer = self.create_timer(0.5, self._check_navigation_status)

        self.get_logger().info('AGV 导航控制器已启动')
        self.get_logger().info(f'  目标话题: {self.mark_topic}')
        self.get_logger().info(f'  结果话题: {self.result_topic}')
        self.get_logger().info(f'  状态话题: {self.status_topic}')
        self.get_logger().info(f'  默认超时: {self.default_timeout}s')

    # ==========================================
    # 核心导航方法
    # ==========================================
    def navigate(self, mark_no: str, task_type: int = 1) -> bool:
        """
        发送导航目标点请求（非阻塞）

        参数:
            mark_no: 目标点位名称，如 '11', '110', '120'
            task_type: 任务类型
                1 = 导航到普通点位
                3 = 导航到充电点位
                5 = 导航到取货点位

        返回:
            bool: 服务调用是否成功

        注意：需要配合 wait_until_reached() 或 navigate_wait() 使用
        """
        if not mark_no or len(mark_no) == 0:
            self.get_logger().error('导航目标点不能为空')
            return False

        msg = String()
        msg.data = mark_no

        self._wait_navigation_event.clear()
        self._navigation_succeeded = False
        self._current_mark_no = mark_no
        self._is_navigating = True

        self._mark_pub.publish(msg)
        self.get_logger().info(f'发送导航目标: mark_no={mark_no}, task_type={task_type}')
        return True

    def wait_until_reached(self, carry: bool, timeout: float = None) -> bool:
        """
        阻塞等待导航任务完成

        工作机制：
        1. 清除事件标志
        2. 启动定时检查，每 0.5s 检查一次 _task_state
        3. 当 task_state == 7 (TASK_STATE_COMPLETED) 或收到 result_topic 消息时，停止等待
        4. 超时则报错并返回

        参数:
            carry: navigate() 的返回值（True 表示需要等待）
            timeout: 超时时间（秒），None 则使用 default_timeout

        返回:
            bool: True=导航成功到达，False=导航失败或超时
        """
        if not carry:
            return True  # 不需要等待

        if timeout is None:
            timeout = self.default_timeout

        self.get_logger().info(f'等待导航到达: mark_no={self._current_mark_no}, timeout={timeout}s')

        start_time = time.time()
        while rclpy.ok():
            # 检查超时
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().error(f'导航超时！已等待 {elapsed:.1f}s，未到达目标点')
                self._is_navigating = False
                return False

            # 检查是否到达
            if self._navigation_succeeded:
                self.get_logger().info(f'导航成功到达: mark_no={self._current_mark_no}')
                self._is_navigating = False
                return True

            # 检查 task_state
            if self._task_state == self.TASK_STATE_COMPLETED:
                self._navigation_succeeded = True
                self.get_logger().info(f'导航成功: task_state={self._task_state}')
                self._is_navigating = False
                return True

            if self._task_state == self.TASK_STATE_FAILED:
                self.get_logger().error(f'导航失败: task_state={self._task_state}')
                self._is_navigating = False
                return False

            # 短暂休眠，避免 CPU 占用过高
            rclpy.spin_once(self, timeout_sec=0.1)

        self._is_navigating = False
        return False

    def navigate_wait(self, mark_no: str, task_type: int = 1, timeout: float = None) -> bool:
        """
        组合调用：导航到目标点并等待到达（推荐使用）

        等价于：
          carry = self.navigate(mark_no, task_type)
          return self.wait_until_reached(carry, timeout)

        参数:
            mark_no: 目标点位名称
            task_type: 任务类型
            timeout: 超时时间（秒）

        返回:
            bool: True=导航成功到达，False=导航失败或超时
        """
        carry = self.navigate(mark_no, task_type)
        return self.wait_until_reached(carry, timeout)

    def cancel_navigation(self) -> None:
        """
        取消当前导航任务

        发送空消息或特殊取消指令到 mark_topic
        """
        self._is_navigating = False
        self._wait_navigation_event.clear()
        # 发送空消息取消导航
        msg = String()
        msg.data = ''
        self._mark_pub.publish(msg)
        self.get_logger().info('导航任务已取消')

    def stop(self) -> None:
        """
        立即停止 AGV 移动（发送零速度）
        """
        twist = Twist()
        self._speed_pub.publish(twist)
        self.get_logger().info('AGV 已停止')

    def set_speed(self, linear: float = 0.0, angular: float = 0.0) -> None:
        """
        设置底盘移动速度（手动控制）

        参数:
            linear: 线性速度 (m/s)
            angular: 角速度 (rad/s)
        """
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._speed_pub.publish(twist)

    # ==========================================
    # 查询方法
    # ==========================================
    def is_navigating(self) -> bool:
        """返回当前是否正在执行导航任务"""
        return self._is_navigating

    def get_current_pose(self) -> tuple:
        """
        获取 AGV 当前位姿

        返回:
            tuple: (x, y, theta) 单位：米，弧度
        """
        return (self._current_pose_x, self._current_pose_y, self._current_pose_theta)

    def get_task_state(self) -> int:
        """返回 Woosh 底盘当前 task_state"""
        return self._task_state

    def get_robot_state(self) -> int:
        """返回机器人状态 (robot_state)"""
        return self._robot_state

    def get_robot_mode(self) -> int:
        """返回机器人模式 (robot_mode): 0=未知, 1=自动, 2=手动"""
        return self._robot_mode

    def is_auto_mode(self) -> bool:
        """返回是否为自动模式"""
        return self._robot_mode == 1

    def is_idle(self) -> bool:
        """返回机器人是否处于空闲状态"""
        return self._robot_state in [2, 8]  # 空闲或已初始化

    # ==========================================
    # 回调函数
    # ==========================================
    def _result_callback(self, msg: String):
        """
        导航结果回调

        当收到 result_topic 消息时，表示 AGV 已到达目标点
        """
        if not self._is_navigating:
            return

        result = msg.data.strip()
        self.get_logger().info(f'收到导航结果: {result}')

        # 如果结果中包含当前目标点，认为到达成功
        if result and self._current_mark_no in result:
            self._navigation_succeeded = True
            self._is_navigating = False
            self._wait_navigation_event.set()
            self.get_logger().info(f'导航到达成功: mark_no={self._current_mark_no}')

    def _status_callback(self, msg: Int32):
        """
        底盘状态回调（task_state）

        task_state 值参考 Woosh 官方文档：
        0 = 空闲
        1 = 任务执行中
        2 = 任务暂停
        7 = 任务完成
        8 = 任务失败
        """
        self._task_state = msg.data

        # 任务完成
        if msg.data == self.TASK_STATE_COMPLETED:
            if self._is_navigating:
                self._navigation_succeeded = True
                self._is_navigating = False
                self._wait_navigation_event.set()
                self.get_logger().info('task_state=7: 导航任务完成')

        # 任务失败
        elif msg.data == self.TASK_STATE_FAILED:
            if self._is_navigating:
                self._is_navigating = False
                self._wait_navigation_event.set()
                self.get_logger().error('task_state=8: 导航任务失败')

    def _odom_callback(self, msg: JointState):
        """
        里程计回调

        JointState.position[0] = x (m)
        JointState.position[1] = y (m)
        JointState.position[2] = theta (rad)
        """
        if len(msg.position) >= 3:
            self._current_pose_x = msg.position[0]
            self._current_pose_y = msg.position[1]
            self._current_pose_theta = msg.position[2]

    def _check_navigation_status(self):
        """
        定时检查导航状态（0.5s 一次）

        用于在没有收到回调的情况下，检测 task_state 变化
        """
        if not self._is_navigating:
            return

        if self._task_state == self.TASK_STATE_COMPLETED:
            self._navigation_succeeded = True
            self._is_navigating = False
            self._wait_navigation_event.set()
            self.get_logger().info(f'定时检查: task_state=7, 导航到达成功')

        elif self._task_state == self.TASK_STATE_FAILED:
            self._is_navigating = False
            self._wait_navigation_event.set()
            self.get_logger().error(f'定时检查: task_state=8, 导航到达失败')

        elif self._task_state == self.TASK_STATE_IDLE and self._current_mark_no:
            # 从运行变为空闲（未到达），可能是异常中断
            self.get_logger().warn(
                f'定时检查: task_state=0 (空闲), 导航可能被中断, mark_no={self._current_mark_no}'
            )

    # ==========================================
    # 显示方法
    # ==========================================
    def display_status(self):
        """显示 AGV 状态信息"""
        print("\033[2J\033[H")  # 清屏
        print("=" * 60)
        print("         AGV 导航控制器")
        print(f"         时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 60)
        print()
        print(f"  导航状态: {'[导航中]' if self._is_navigating else '[空闲]'}")
        print(f"  当前目标: {self._current_mark_no if self._current_mark_no else '(无)'}")
        print()
        print("-" * 60)
        print("  底盘状态:")
        print("-" * 60)
        print(f"    task_state: {self._task_state} ({self._get_task_state_name()})")
        print(f"    robot_mode: {self._robot_mode} ({self._get_robot_mode_name()})")
        print(f"    robot_state: {self._robot_state} ({self._get_robot_state_name()})")
        print()
        print(f"    当前位置: x={self._current_pose_x:.3f}m, "
              f"y={self._current_pose_y:.3f}m, "
              f"theta={math.degrees(self._current_pose_theta):.1f}deg")
        print()
        print("-" * 60)
        print("  使用说明:")
        print("    nav.navigate_wait('110')  # 导航到点位110并等待")
        print("    nav.cancel_navigation()    # 取消当前导航")
        print("    nav.stop()                # 立即停止移动")
        print("    nav.set_speed(0.5, 0.0)  # 手动设置速度")
        print("-" * 60)

    def _get_task_state_name(self) -> str:
        names = {
            0: '空闲', 1: '运行中', 2: '暂停',
            7: '完成', 8: '失败'
        }
        return names.get(self._task_state, '未知')

    def _get_robot_mode_name(self) -> str:
        names = {0: '未知', 1: '自动', 2: '手动'}
        return names.get(self._robot_mode, '未知')

    def _get_robot_state_name(self) -> str:
        names = {
            0: '未定义', 1: '未初始化', 2: '空闲', 8: '已初始化'
        }
        return names.get(self._robot_state, '未知')


class AGVNavigatorSimulated(Node):
    """
    AGV 导航模拟器（用于仿真环境）

    当没有真实 Woosh AGV 底盘时，使用此模拟器进行开发和测试。
    模拟器会自动模拟导航过程，无需实际硬件。
    """

    def __init__(self,
                 agv_base_frame: str = 'agv_base',
                 mark_topic: str = 'goto_mark/go',
                 result_topic: str = 'goto_mark/result',
                 default_timeout: float = 5.0):
        super().__init__('agv_navigator_simulated')

        self.agv_base_frame = agv_base_frame
        self.mark_topic = mark_topic
        self.result_topic = result_topic
        self.default_timeout = default_timeout

        # 模拟状态
        self._is_navigating = False
        self._current_mark_no = ''
        self._navigation_succeeded = False
        self._wait_navigation_event = threading.Event()
        self._sim_speed = 2.0  # 模拟速度（秒）

        # 模拟位置
        self._current_pose_x = 0.0
        self._current_pose_y = 0.0
        self._current_pose_theta = 0.0
        self._target_pose_x = 0.0
        self._target_pose_y = 0.0

        # 发布者
        self._result_pub = self.create_publisher(String, self.result_topic, 10)
        self._status_pub = self.create_publisher(Int32, 'robot_status', 10)

        # 订阅者
        self._mark_sub = self.create_subscription(
            String, self.mark_topic, self._mark_callback, 10
        )

        # 模拟定时器（模拟 AGV 移动过程）
        self._sim_timer = self.create_timer(0.1, self._sim_step)

        self.get_logger().info('AGV 导航模拟器已启动（仿真模式）')
        self.get_logger().warn('  警告：正在使用模拟器，无真实 AGV 硬件！')

    def navigate(self, mark_no: str, task_type: int = 1) -> bool:
        """模拟导航请求"""
        if not mark_no:
            return False

        self._wait_navigation_event.clear()
        self._navigation_succeeded = False
        self._current_mark_no = mark_no
        self._is_navigating = True

        # 模拟目标位置（基于 mark_no 生成简单坐标）
        self._target_pose_x = float(hash(mark_no) % 100) / 100.0 * 5.0 - 2.5
        self._target_pose_y = float(hash(mark_no + 'y') % 100) / 100.0 * 5.0 - 2.5

        self.get_logger().info(f'模拟导航: mark_no={mark_no} -> ({self._target_pose_x:.2f}, {self._target_pose_y:.2f})')
        return True

    def wait_until_reached(self, carry: bool, timeout: float = None) -> bool:
        """模拟等待到达"""
        if not carry:
            return True

        timeout = timeout or self.default_timeout
        self.get_logger().info(f'模拟等待导航到达: mark_no={self._current_mark_no}, timeout={timeout}s')

        # 等待模拟移动完成或超时
        elapsed = 0.0
        while rclpy.ok() and elapsed < timeout:
            if not self._is_navigating:
                return self._navigation_succeeded
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1

        self._is_navigating = False
        if elapsed >= timeout:
            self.get_logger().warn(f'模拟导航超时: mark_no={self._current_mark_no}')
            return False
        return self._navigation_succeeded

    def navigate_wait(self, mark_no: str, task_type: int = 1, timeout: float = None) -> bool:
        """组合调用"""
        carry = self.navigate(mark_no, task_type)
        return self.wait_until_reached(carry, timeout)

    def cancel_navigation(self) -> None:
        """模拟取消导航"""
        self._is_navigating = False
        self._wait_navigation_event.clear()
        self.get_logger().info('模拟导航已取消')

    def stop(self) -> None:
        """模拟停止"""
        self._is_navigating = False
        self.get_logger().info('模拟 AGV 已停止')

    def set_speed(self, linear: float = 0.0, angular: float = 0.0) -> None:
        """模拟速度设置"""
        pass

    def is_navigating(self) -> bool:
        return self._is_navigating

    def get_current_pose(self) -> tuple:
        return (self._current_pose_x, self._current_pose_y, self._current_pose_theta)

    def get_task_state(self) -> int:
        return 7 if not self._is_navigating and self._navigation_succeeded else 1

    def get_robot_state(self) -> int:
        return 2  # 空闲

    def get_robot_mode(self) -> int:
        return 1  # 自动

    def is_auto_mode(self) -> bool:
        return True

    def is_idle(self) -> bool:
        return True

    def _mark_callback(self, msg: String):
        """接收导航目标点"""
        mark_no = msg.data.strip()
        if mark_no:
            self.navigate_wait(mark_no)

    def _sim_step(self):
        """模拟 AGV 移动（每隔 0.1s 调用一次）"""
        if not self._is_navigating:
            return

        # 简单模拟：线性插值接近目标
        dx = self._target_pose_x - self._current_pose_x
        dy = self._target_pose_y - self._current_pose_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.05:  # 到达阈值
            self._current_pose_x = self._target_pose_x
            self._current_pose_y = self._target_pose_y
            self._is_navigating = False
            self._navigation_succeeded = True

            # 发布到达结果
            result_msg = String()
            result_msg.data = f'arrive {self._current_mark_no}'
            self._result_pub.publish(result_msg)

            # 发布 task_state=7
            status_msg = Int32()
            status_msg.data = 7
            self._status_pub.publish(status_msg)

            self._wait_navigation_event.set()
            self.get_logger().info(f'模拟到达: mark_no={self._current_mark_no}')
        else:
            # 移动一小步
            step = min(self._sim_speed * 0.1, dist)
            self._current_pose_x += dx / dist * step
            self._current_pose_y += dy / dist * step


def main():
    """演示模式：运行导航控制器并显示状态"""
    import argparse

    parser = argparse.ArgumentParser(description='AGV 导航控制器')
    parser.add_argument('--sim', action='store_true',
                       help='使用模拟器模式（无真实 AGV）')
    parser.add_argument('--mark', type=str, default='110',
                       help='导航目标点（如 110）')
    parser.add_argument('--timeout', type=float, default=120.0,
                       help='导航超时时间（秒）')
    args, _ = parser.parse_known_args()

    rclpy.init(args=None)

    if args.sim:
        nav = AGVNavigatorSimulated()
    else:
        nav = AGVNavigator()

    print()
    print("=" * 60)
    print("         AGV 导航控制器")
    print("=" * 60)
    print()
    print(f"  模式: {'[模拟器]' if args.sim else '[真实 AGV]'}")
    print(f"  导航目标: {args.mark}")
    print(f"  超时时间: {args.timeout}s")
    print()
    print("  使用说明:")
    print("    nav.navigate_wait('110')  # 导航到点位110并等待")
    print("    nav.cancel_navigation()    # 取消当前导航")
    print("    nav.stop()                # 立即停止移动")
    print("    nav.display_status()      # 显示状态")
    print()
    print("=" * 60)
    print()

    try:
        # 执行导航
        success = nav.navigate_wait(args.mark, timeout=args.timeout)
        if success:
            print(f"\n  导航成功到达目标点: {args.mark}")
        else:
            print(f"\n  导航失败或超时: {args.mark}")

        # 持续显示状态
        while rclpy.ok():
            nav.display_status()
            rclpy.spin_once(nav, timeout_sec=0.5)

    except KeyboardInterrupt:
        print("\n\n程序已退出")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
