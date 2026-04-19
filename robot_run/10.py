      
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
10 - 键盘测试夹爪（力控夹取 / 张开）

依赖：先启动 dual_rm_65b real_moveit_demo（内含 dual_rm_65_driver），与 08 脚本相同 namespace。

终端交互：
  c — 闭合：由 --close-mode 决定（默认 position：与 o 对称，发 set_gripper_position_cmd
       到较小开口度；pick=仅力控；both=先位置再力控）
  o — 张开：set_gripper_position_cmd（可选分段渐开）
  t — 可选：灵巧手预存手势 set_hand_posture_cmd（需 --hand-posture）
  q / Esc — 退出

说明：若系统存在 udp_hand_status / set_hand_* 话题，末端可能是灵巧手；平行夹爪一般用 position 开合即可。

用法示例：
  python3 10_gripper_keyboard_test.py --arm-side right
  python3 10_gripper_keyboard_test.py --arm-ns right_arm_controller
"""

from __future__ import annotations

import argparse
import select
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from rm_ros_interfaces.msg import Gripperpick, Gripperset
except ImportError as _e:
    Gripperpick = None  # type: ignore
    Gripperset = None  # type: ignore
    _IMPORT_ERR = _e
else:
    _IMPORT_ERR = None

try:
    from rm_ros_interfaces.msg import Handposture
except ImportError:
    Handposture = None  # type: ignore


def _script_dir() -> Path:
    return Path(__file__).resolve().parent


class GripperKeyboardTestNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('gripper_keyboard_test_10')
        self._arm_ns = args.arm_ns.strip('/')
        self._pick_speed = int(args.pick_speed)
        self._pick_force = max(50, min(1000, int(args.pick_force)))
        if self._pick_force != int(args.pick_force):
            self.get_logger().warn(
                f'力控阈值已钳位到 50~1000: {int(args.pick_force)} -> {self._pick_force}'
            )
        self._open_position = int(args.open_position)
        self._block = bool(args.block)
        self._timeout = int(args.timeout_ms)
        self._input_mode = args.input_mode
        self._pick_mode = args.gripper_pick_mode.strip().lower()
        self._close_mode = args.close_mode.strip().lower()
        self._close_position = max(1, min(1000, int(args.close_position)))
        self._open_ramp_step = max(0, int(args.open_ramp_step))
        self._open_ramp_delay = float(args.open_ramp_delay)
        self._open_ramp_start_if_unknown = int(args.open_ramp_start_if_unknown)
        # 最近一次「位置指令」估计，用于分段张开起点（Gripperset 无速度字段）
        self._last_cmd_position: Optional[int] = None
        self._hand_posture_num: Optional[int] = None
        self._pub_hand_posture = None
        if args.hand_posture is not None:
            self._hand_posture_num = max(1, min(40, int(args.hand_posture)))

        if self._close_position >= self._open_position:
            self.get_logger().warn(
                f'闭合目标 close_position={self._close_position} 应小于张开 open_position={self._open_position}，已钳位'
            )
            self._close_position = max(1, self._open_position - 50)

        base = f'/{self._arm_ns}/rm_driver'
        self._pub_pick = self.create_publisher(Gripperpick, f'{base}/set_gripper_pick_cmd', 10)
        self._pub_pick_on = self.create_publisher(Gripperpick, f'{base}/set_gripper_pick_on_cmd', 10)
        self._pub_position = self.create_publisher(Gripperset, f'{base}/set_gripper_position_cmd', 10)
        if self._hand_posture_num is not None and Handposture is not None:
            self._pub_hand_posture = self.create_publisher(Handposture, f'{base}/set_hand_posture_cmd', 10)
            self.create_subscription(
                Bool,
                f'{base}/set_hand_posture_result',
                self._on_hand_posture_result,
                10,
            )

        self.create_subscription(
            Bool,
            f'{base}/set_gripper_pick_result',
            self._on_pick_result,
            10,
        )
        self.create_subscription(
            Bool,
            f'{base}/set_gripper_pick_on_result',
            self._on_pick_on_result,
            10,
        )
        self.create_subscription(
            Bool,
            f'{base}/set_gripper_position_result',
            self._on_position_result,
            10,
        )

        pick_topic = (
            f'{base}/set_gripper_pick_on_cmd'
            if self._pick_mode == 'pick_on'
            else f'{base}/set_gripper_pick_cmd'
        )
        self.get_logger().info(
            f'闭合模式 close_mode={self._close_mode} | 力控话题(辅): {pick_topic} | 位置: {base}/set_gripper_position_cmd'
        )
        if self._pub_hand_posture is not None:
            self.get_logger().info(
                f'灵巧手手势: {base}/set_hand_posture_cmd (posture_num={self._hand_posture_num})'
            )

    def _on_hand_posture_result(self, msg: Bool) -> None:
        self.get_logger().info(f'灵巧手手势结果 set_hand_posture_result: {msg.data}')

    def _on_pick_result(self, msg: Bool) -> None:
        self.get_logger().info(f'力控夹取结果 set_gripper_pick_result: {msg.data}')

    def _on_pick_on_result(self, msg: Bool) -> None:
        self.get_logger().info(f'力控夹取结果 set_gripper_pick_on_result: {msg.data}')

    def _on_position_result(self, msg: Bool) -> None:
        self.get_logger().info(f'位置设置结果 set_gripper_position_result: {msg.data}')

    def send_force_pick(self) -> None:
        m = Gripperpick()
        m.speed = self._pick_speed
        m.force = self._pick_force
        m.block = self._block
        m.timeout = self._timeout
        if self._pick_mode == 'pick_on':
            self._pub_pick_on.publish(m)
            tag = '持续力控 pick_on'
        else:
            self._pub_pick.publish(m)
            tag = '力控夹取 pick'
        self.get_logger().info(
            f'已发布{tag}: speed={m.speed} force={m.force} block={m.block} timeout={m.timeout}'
        )
        # 增加等待时间，确保夹爪有足够的时间完成力控夹取动作
        if self._block:
            # 如果是阻塞模式，等待一段时间让夹爪完成动作
            time.sleep(1.0)
        else:
            # 如果是非阻塞模式，也等待一段时间以确保消息被处理
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
        # 闭合后开口度变小，便于下次分段张开估计起点
        self._last_cmd_position = min(self._last_cmd_position or self._open_position, 120)

    def send_hand_posture(self) -> None:
        if self._pub_hand_posture is None or Handposture is None or self._hand_posture_num is None:
            self.get_logger().warn('灵巧手未配置：请使用 --hand-posture <1~40> 后按 t')
            return
        m = Handposture()
        m.posture_num = int(self._hand_posture_num)
        m.block = self._block
        self._pub_hand_posture.publish(m)
        self.get_logger().info(
            f'已发布灵巧手手势: posture_num={m.posture_num} block={m.block}'
        )

    def send_close_position(self) -> None:
        """与张开对称：用目标开口度闭合（平行夹爪常用）。"""
        target = self._close_position
        if self._open_ramp_step <= 0:
            positions = [target]
        else:
            start = (
                self._last_cmd_position
                if self._last_cmd_position is not None
                else max(1, min(800, self._open_position))  # 使用更合理的默认起始位置
            )
            positions = self._ramp_positions(start, target)

        for i, pos in enumerate(positions):
            m = Gripperset()
            m.position = int(pos)
            m.block = self._block
            m.timeout = self._timeout
            self._pub_position.publish(m)
            for _ in range(5):  # 增加spin次数，确保消息被处理
                rclpy.spin_once(self, timeout_sec=0.03)
            if self._open_ramp_step > 0 and len(positions) > 1 and i < len(positions) - 1:
                time.sleep(self._open_ramp_delay)

        self._last_cmd_position = target
        ramp_info = f' 分段{len(positions)}步' if len(positions) > 1 else ''
        self.get_logger().info(
            f'已发布位置闭合{ramp_info}: position->{target} block={self._block} timeout={self._timeout}'
        )

    def send_close(self) -> None:
        if self._close_mode == 'pick':
            self.send_force_pick()
        elif self._close_mode == 'position':
            self.send_close_position()
        elif self._close_mode == 'both':
            self.send_close_position()
            time.sleep(0.08)
            for _ in range(8):
                rclpy.spin_once(self, timeout_sec=0.02)
            self.send_force_pick()
        else:
            self.get_logger().error(f'未知 close_mode: {self._close_mode!r}')

    def _ramp_positions(self, start: int, end: int) -> list[int]:
        """单调从 start 走到 end（含端点），步长 open_ramp_step。"""
        start = max(1, min(1000, start))
        end = max(1, min(1000, end))
        if start == end:
            return [end]
        step = self._open_ramp_step if self._open_ramp_step > 0 else abs(end - start)
        if step <= 0:
            return [end]
        out: list[int] = []
        if end > start:
            p = start
            while p < end:
                p = min(p + step, end)
                out.append(p)
        else:
            p = start
            while p > end:
                p = max(p - step, end)
                out.append(p)
        return out

    def send_open(self) -> None:
        target = self._open_position
        if self._open_ramp_step <= 0:
            positions = [target]
        else:
            start = (
                self._last_cmd_position
                if self._last_cmd_position is not None
                else max(1, min(self._open_ramp_start_if_unknown, target - 1))
            )
            positions = self._ramp_positions(start, target)

        for i, pos in enumerate(positions):
            m = Gripperset()
            m.position = int(pos)
            m.block = self._block
            m.timeout = self._timeout
            self._pub_position.publish(m)
            for _ in range(3):
                rclpy.spin_once(self, timeout_sec=0.02)
            if self._open_ramp_step > 0 and len(positions) > 1 and i < len(positions) - 1:
                time.sleep(self._open_ramp_delay)

        self._last_cmd_position = target
        ramp_info = f' 分段{len(positions)}步' if len(positions) > 1 else ''
        self.get_logger().info(
            f'已发布张开{ramp_info}: position->{target} block={self._block} timeout={self._timeout}'
        )

    def print_help(self) -> None:
        mode = 'pick_on' if self._pick_mode == 'pick_on' else 'pick'
        extra = ' | t=灵巧手手势' if self._pub_hand_posture is not None else ''
        self.get_logger().info(
            f'c=闭合(close_mode={self._close_mode}, 辅力控={mode}) | o=张开{extra} | q/Esc=退出'
        )
        if self._open_ramp_step > 0:
            self.get_logger().info(
                f'分段张开: step={self._open_ramp_step} delay={self._open_ramp_delay}s'
            )
        if self._input_mode == 'line':
            self.get_logger().info('输入模式: 行模式 — 输入单字母后回车')

    def _setup_tty(self) -> None:
        self._old_term = None
        if sys.platform == 'win32' or not sys.stdin.isatty():
            return
        try:
            import termios
            import tty

            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            self._old_term = None

    def _restore_tty(self) -> None:
        if self._old_term is not None:
            try:
                import termios

                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
        self._old_term = None

    def _read_key(self) -> Optional[str]:
        if sys.platform == 'win32':
            try:
                import msvcrt
            except ImportError:
                return None
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x03',):
                    raise KeyboardInterrupt
                return ch.decode('latin-1', errors='ignore').lower()
            return None
        if self._old_term is not None:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
        return None

    def _dispatch(self, k: str) -> None:
        if k == 'c':
            self.send_close()
        elif k == 'o':
            self.send_open()
        elif k == 't':
            self.send_hand_posture()
        else:
            keys = 'c / o / q'
            if self._pub_hand_posture is not None:
                keys += ' / t'
            self.get_logger().warn(f'未知命令: {k!r}（仅 {keys}）')

    def _run_line(self) -> None:
        while rclpy.ok():
            try:
                cmd = input('命令 [c/o/t/q]: ').strip().lower()
            except EOFError:
                break
            except KeyboardInterrupt:
                self.get_logger().info('退出')
                break
            if not cmd:
                continue
            ch = cmd[0]
            if ch in ('q',):
                self.get_logger().info('退出')
                break
            self._dispatch(ch)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.02)

    def _run_keys(self) -> None:
        self._setup_tty()
        try:
            hint = 'c / o / t / q' if self._pub_hand_posture is not None else 'c / o / q'
            self.get_logger().info(f'按键模式：焦点在终端，按 {hint}')
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    k = self._read_key()
                except KeyboardInterrupt:
                    break
                if k is None:
                    continue
                if k in ('q', '\x1b'):
                    self.get_logger().info('退出')
                    break
                self._dispatch(k)
        finally:
            self._restore_tty()

    def run(self) -> None:
        # 给 discovery / publisher 一点时间
        time.sleep(0.3)
        self.print_help()
        if self._input_mode == 'line':
            self._run_line()
        else:
            self._run_keys()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='10 夹爪键盘测试（ROS2 + rm_driver）')
    p.add_argument(
        '--arm-side',
        choices=('left', 'right'),
        default='right',
        help='与 08 一致：自动设置 arm_ns（可被 --arm-ns 覆盖）',
    )
    p.add_argument(
        '--arm-ns',
        default=None,
        help='驱动 namespace，默认 left_arm_controller / right_arm_controller',
    )
    p.add_argument('--pick-speed', type=int, default=500, help='力控夹取 speed 1~1000')
    p.add_argument('--pick-force', type=int, default=500, help='力控夹取 force 50~1000（低于 50 会自动提到 50）')
    p.add_argument(
        '--gripper-pick-mode',
        choices=('pick', 'pick_on'),
        default='pick',
        help='pick=一次性力控夹取 set_gripper_pick_cmd（与 dual_arm_pick_place 一致）；'
        'pick_on=持续力控 set_gripper_pick_on_cmd（部分机型上可能只回成功位而不明显闭合）',
    )
    p.add_argument(
        '--close-mode',
        choices=('position', 'pick', 'both'),
        default='position',
        help='c 键闭合方式：position=仅 set_gripper_position_cmd 到较小开口度（与 o 对称，平行夹爪推荐）；'
        'pick=仅力控夹取；both=先位置再力控',
    )
    p.add_argument(
        '--close-position',
        type=int,
        default=80,
        help='close_mode 含 position 时的目标开口度 1~1000（越小越合，需小于 --open-position）',
    )
    p.add_argument(
        '--hand-posture',
        type=int,
        default=None,
        help='灵巧手预存手势序号 1~40；设置后按 t 发布 set_hand_posture_cmd（需手内已存该序号）',
    )
    p.add_argument(
        '--open-position',
        type=int,
        default=1000,
        help='张开目标 position 1~1000（1000≈最大开度）',
    )
    p.add_argument(
        '--open-ramp-step',
        type=int,
        default=0,
        help='分段张开时每步 position 增量（0=一次到位，最快；如 80~150 可明显变慢）',
    )
    p.add_argument(
        '--open-ramp-delay',
        type=float,
        default=0.05,
        help='分段张开时相邻两步之间的休眠秒数（仅 ramp-step>0 时有效）',
    )
    p.add_argument(
        '--open-ramp-start-if-unknown',
        type=int,
        default=200,
        help='启用分段张开且尚无历史位置时，假定起点开口度（1~999）；夹爪已很开时可改大以免先合再开',
    )
    p.add_argument(
        '--slow-open',
        action='store_true',
        help='等价于在未指定 --open-ramp-step 时使用 step=100（配合 --open-ramp-delay）',
    )
    p.add_argument('--block', action='store_true', help='驱动阻塞模式（消息 bool block）')
    p.add_argument(
        '--timeout-ms',
        type=int,
        default=0,
        help='阻塞超时(ms)，0 表示由驱动默认处理',
    )
    p.add_argument(
        '--input-mode',
        choices=('key', 'line'),
        default='key',
        help='key=单键；line=输入后回车',
    )
    return p.parse_args()


def main() -> None:
    if _IMPORT_ERR is not None:
        print(f'需要 rm_ros_interfaces: {_IMPORT_ERR}', file=sys.stderr)
        print('请先 source 工作空间后再运行。', file=sys.stderr)
        sys.exit(1)

    args = parse_args()
    if args.hand_posture is not None and Handposture is None:
        print('当前环境的 rm_ros_interfaces 不包含 Handposture 消息，无法使用 --hand-posture。', file=sys.stderr)
        sys.exit(1)
    side_cfg = {
        'left': 'left_arm_controller',
        'right': 'right_arm_controller',
    }
    if args.arm_ns is None:
        args.arm_ns = side_cfg[args.arm_side]

    if args.slow_open and int(args.open_ramp_step) == 0:
        args.open_ramp_step = 100

    if not sys.stdin.isatty() and args.input_mode == 'key':
        print('非交互终端，已自动改用 --input-mode line', file=sys.stderr)
        args.input_mode = 'line'

    rclpy.init()
    node = GripperKeyboardTestNode(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    