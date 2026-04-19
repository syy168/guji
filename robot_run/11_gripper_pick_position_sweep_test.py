#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
11 - 力控参数扫描 + 位置往复开合测试

对每一组 (speed, force) 先发 set_gripper_pick_cmd，再在 position_lo / position_hi 之间往复
set_gripper_position_cmd。

依赖：rm_driver 已运行；已 source 含 rm_ros_interfaces 的工作空间。

示例：
  python3 11_gripper_pick_position_sweep_test.py --arm-side right
  python3 11_gripper_pick_position_sweep_test.py --arm-ns right_arm_controller --full-matrix
"""

from __future__ import annotations

import argparse
import itertools
import sys
import time
from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node

try:
    from rm_ros_interfaces.msg import Gripperpick, Gripperset
except ImportError as e:
    print(f'需要 rm_ros_interfaces: {e}', file=sys.stderr)
    sys.exit(1)


def _pairs(sweep_values: list[int], full_matrix: bool) -> Iterable[Tuple[int, int]]:
    if full_matrix:
        return itertools.product(sweep_values, repeat=2)
    return zip(sweep_values, sweep_values)


class GripperPickPositionSweepNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('gripper_pick_position_sweep_11')
        self._arm_ns = args.arm_ns.strip('/')
        self._pos_lo = max(1, min(1000, int(args.position_lo)))
        self._pos_hi = max(1, min(1000, int(args.position_hi)))
        if self._pos_lo >= self._pos_hi:
            raise ValueError('position_lo 必须小于 position_hi')
        self._osc = max(1, int(args.oscillations_per_group))
        self._pause = max(0.0, float(args.pause_sec))
        self._after_pick = max(0.0, float(args.settle_after_pick_sec))
        self._block = bool(args.block)
        self._timeout = max(0, int(args.timeout_ms))
        self._sweep = [max(1, min(1000, int(x))) for x in args.sweep_values]

        base = f'/{self._arm_ns}/rm_driver'
        self._pub_pick = self.create_publisher(Gripperpick, f'{base}/set_gripper_pick_cmd', 10)
        self._pub_pos = self.create_publisher(Gripperset, f'{base}/set_gripper_position_cmd', 10)
        self.get_logger().info(
            f'话题: {base}/set_gripper_pick_cmd , {base}/set_gripper_position_cmd'
        )

    def _spin_short(self) -> None:
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_pick(self, speed: int, force: int) -> None:
        m = Gripperpick()
        m.speed = max(1, min(1000, int(speed)))
        m.force = max(50, min(1000, int(force)))
        m.block = self._block
        m.timeout = self._timeout
        self._pub_pick.publish(m)
        self.get_logger().info(f'pick: speed={m.speed} force={m.force} block={m.block} timeout={m.timeout}')
        self._spin_short()

    def send_position(self, position: int) -> None:
        m = Gripperset()
        m.position = int(max(1, min(1000, position)))
        m.block = self._block
        m.timeout = self._timeout
        self._pub_pos.publish(m)
        self.get_logger().info(f'position: {m.position} block={m.block} timeout={m.timeout}')
        self._spin_short()
        time.sleep(self._pause)

    def run_sweep(self, full_matrix: bool) -> None:
        time.sleep(0.3)
        pairs = list(_pairs(self._sweep, full_matrix))
        self.get_logger().info(
            f'共 {len(pairs)} 组 (speed,force)；每组往复 {self._osc} 次 [{self._pos_lo}<->{self._pos_hi}]'
        )
        for speed, force in pairs:
            self.get_logger().warn(f'=== 新一组 speed={speed} force={force} ===')
            self.send_pick(speed, force)
            time.sleep(self._after_pick)
            for i in range(self._osc):
                self.get_logger().info(f'往复 {i + 1}/{self._osc}: -> {self._pos_hi}')
                self.send_position(self._pos_hi)
                self.get_logger().info(f'往复 {i + 1}/{self._osc}: -> {self._pos_lo}')
                self.send_position(self._pos_lo)
        self.get_logger().info('全部组完成')


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='11 力控参数 + 位置往复测试')
    p.add_argument('--arm-side', choices=('left', 'right'), default='right')
    p.add_argument('--arm-ns', default=None)
    p.add_argument(
        '--sweep-values',
        type=int,
        nargs='+',
        default=[50, 200, 400, 600],
        help='用于扫描的 speed/force 取值（默认 50 200 400 600）',
    )
    p.add_argument(
        '--full-matrix',
        action='store_true',
        help='若指定，则遍历 speed×force 全部组合（默认仅 (50,50)(200,200)... 对角）',
    )
    p.add_argument('--position-lo', type=int, default=50, help='往复较小开口度')
    p.add_argument('--position-hi', type=int, default=900, help='往复较大开口度')
    p.add_argument('--oscillations-per-group', type=int, default=3, help='每组内开合往复次数')
    p.add_argument('--settle-after-pick-sec', type=float, default=0.2, help='发 pick 后等待再开始位置往复')
    p.add_argument('--pause-sec', type=float, default=0.35, help='每次 position 指令后的额外休眠')
    p.add_argument('--block', action='store_true', help='pick/position 均使用阻塞')
    p.add_argument('--timeout-ms', type=int, default=0, help='阻塞超时 ms，0 为驱动默认')
    return p.parse_args()


def main() -> None:
    args = parse_args()
    side_cfg = {'left': 'left_arm_controller', 'right': 'right_arm_controller'}
    if args.arm_ns is None:
        args.arm_ns = side_cfg[args.arm_side]

    rclpy.init()
    node = GripperPickPositionSweepNode(args)
    try:
        node.run_sweep(full_matrix=bool(args.full_matrix))
    except KeyboardInterrupt:
        node.get_logger().info('中断退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
