from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, UInt16

from .config_model import ArmConfig, MotionConfig, SafetyConfig
from .ros_types import RosTypes


QUERY_RESULT_SUFFIX = {
    "raw": "get_force_data_result",
    "zero": "get_zero_force_data_result",
    "work": "get_work_force_data_result",
    "tool": "get_tool_force_data_result",
}


def _topic_prefix(topic: str) -> str:
    if "/" not in topic:
        return ""
    return topic.rsplit("/", 1)[0]


def derive_query_topics(base_force_topic: str, query_result: str) -> tuple[str, str]:
    prefix = _topic_prefix(base_force_topic)
    result_suffix = QUERY_RESULT_SUFFIX[query_result]
    if prefix:
        return f"{prefix}/get_force_data_cmd", f"{prefix}/{result_suffix}"
    return "get_force_data_cmd", result_suffix


@dataclass
class ArmRuntimeState:
    latest_joint: Optional[JointState] = None
    latest_pose: Optional[Pose] = None
    latest_force_norm: float = 0.0
    latest_error_code: int = 0
    force_has_data: bool = False
    force_alarm_latched: bool = False
    force_alarm_count: int = 0


class RobotIO(Node):
    def __init__(
        self,
        arms: Dict[str, ArmConfig],
        motion: MotionConfig,
        safety: SafetyConfig,
        ros_types: RosTypes,
        dry_run: bool,
        confirm_before_motion: bool,
    ) -> None:
        super().__init__("demo_robot_io")
        self._arms = arms
        self._motion = motion
        self._safety = safety
        self._dry_run = dry_run
        self._confirm_before_motion = confirm_before_motion
        self._types = ros_types

        if not (0.0 < self._safety.release_ratio < 1.0):
            raise ValueError("safety.release_ratio 必须在 (0, 1) 区间内")
        for arm_name, arm_cfg in arms.items():
            if float(arm_cfg.threshold_n) <= 0.0:
                raise ValueError(f"arm={arm_name} threshold_n 必须大于 0")

        self._state: Dict[str, ArmRuntimeState] = {name: ArmRuntimeState() for name in arms.keys()}
        self._movej_pub = {}
        self._movel_pub = {}
        self._gripper_pick_pub = {}
        self._stop_pub = {}
        self._force_query_pub: Dict[str, object] = {}

        if self._safety.force_mode != "query":
            raise ValueError("demo_pick_place 力监控当前仅支持 query 模式，udp_six_force/udp_six_zero_force 不可用")
        if self._safety.force_query_hz <= 0.0:
            raise ValueError("safety.force_query_hz 必须大于 0")
        if self._safety.force_query_result not in QUERY_RESULT_SUFFIX:
            raise ValueError(
                f"safety.force_query_result 无效: {self._safety.force_query_result}, "
                f"可选 {list(QUERY_RESULT_SUFFIX.keys())}"
            )

        for arm_name, arm_cfg in arms.items():
            self._movej_pub[arm_name] = self.create_publisher(ros_types.Movej, arm_cfg.movej_topic, 10)
            self._movel_pub[arm_name] = self.create_publisher(ros_types.Movel, arm_cfg.movel_topic, 10)
            self._gripper_pick_pub[arm_name] = self.create_publisher(ros_types.Gripperpick, arm_cfg.gripper_pick_topic, 10)
            self._stop_pub[arm_name] = self.create_publisher(Bool, arm_cfg.move_stop_topic, 10)

            self.create_subscription(
                JointState,
                arm_cfg.joint_state_topic,
                lambda msg, n=arm_name: self._on_joint_state(n, msg),
                10,
            )
            self.create_subscription(
                Pose,
                f"/{arm_cfg.controller_ns}/rm_driver/udp_arm_position",
                lambda msg, n=arm_name: self._on_pose(n, msg),
                10,
            )
            self.create_subscription(
                UInt16,
                arm_cfg.arm_error_topic,
                lambda msg, n=arm_name: self._on_error(n, msg),
                10,
            )

            cmd_topic, result_topic = derive_query_topics(
                arm_cfg.six_force_topic,
                self._safety.force_query_result,
            )
            self._force_query_pub[arm_name] = self.create_publisher(Empty, cmd_topic, 10)
            self.create_subscription(
                ros_types.Sixforce,
                result_topic,
                lambda msg, n=arm_name: self._on_force(n, msg),
                10,
            )
            self.get_logger().info(
                f"[{arm_name}] 力监控(query): cmd_topic={cmd_topic}, result_topic={result_topic}, "
                f"query_hz={self._safety.force_query_hz:.1f}, threshold={arm_cfg.threshold_n:.2f}N"
            )

        self.create_timer(1.0 / self._safety.force_query_hz, self._request_force_data)

    def _request_force_data(self) -> None:
        for arm_name, pub in self._force_query_pub.items():
            if pub is None:
                continue
            pub.publish(Empty())

    def _on_joint_state(self, arm: str, msg: JointState) -> None:
        self._state[arm].latest_joint = msg

    def _on_pose(self, arm: str, msg: Pose) -> None:
        self._state[arm].latest_pose = msg

    def _on_error(self, arm: str, msg: UInt16) -> None:
        self._state[arm].latest_error_code = int(msg.data)

    def _on_force(self, arm: str, msg: object) -> None:
        # 优先使用 Sixforce 的 force_* 字段，同时兼容 fx/fy/fz 命名。
        fx = float(getattr(msg, "force_fx", getattr(msg, "fx", 0.0)))
        fy = float(getattr(msg, "force_fy", getattr(msg, "fy", 0.0)))
        fz = float(getattr(msg, "force_fz", getattr(msg, "fz", 0.0)))
        state = self._state[arm]
        state.latest_force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        state.force_has_data = True

        threshold_n = self._arm_force_threshold(arm)
        release_threshold_n = threshold_n * self._safety.release_ratio

        # 在力回调中即时判定阈值并急停，缩短碰撞到停机的响应路径。
        if state.latest_force_norm >= threshold_n and not state.force_alarm_latched:
            self._latch_force_alarm_and_stop(arm, state, threshold_n)

        # 使用释放迟滞，避免在阈值附近抖动反复触发。
        if state.force_alarm_latched and state.latest_force_norm < release_threshold_n:
            state.force_alarm_latched = False
            self.get_logger().info(
                f"[{arm}] 力告警释放: {state.latest_force_norm:.2f}N < "
                f"release_threshold={release_threshold_n:.2f}N"
            )

    def arm_state(self, arm: str) -> ArmRuntimeState:
        return self._state[arm]

    def safety_check_or_stop(self, arm: str) -> bool:
        state = self._state[arm]
        if state.latest_error_code in self._safety.emergency_error_codes:
            self.get_logger().error(
                f"[{arm}] 检测到紧急错误码: 0x{state.latest_error_code:04X}，触发急停"
            )
            self.stop_arm(arm)
            return False

        if not state.force_has_data:
            self.get_logger().error(f"[{arm}] 尚未收到 get_force_data_result，触发保护停止")
            self.stop_arm(arm)
            return False

        threshold_n = self._arm_force_threshold(arm)

        # 兜底：即使回调路径未及时触发，这里仍会补做阈值判定与急停。
        if state.latest_force_norm >= threshold_n and not state.force_alarm_latched:
            self._latch_force_alarm_and_stop(arm, state, threshold_n)
            return False

        # 若仍处于锁存态，流程检查保持失败。
        if state.force_alarm_latched:
            return False
        return True

    def _arm_force_threshold(self, arm: str) -> float:
        return float(self._arms[arm].threshold_n)

    def _latch_force_alarm_and_stop(
        self,
        arm: str,
        state: ArmRuntimeState,
        threshold_n: float,
    ) -> None:
        # 锁存后不重复下发 stop，避免日志和控制命令风暴。
        if state.force_alarm_latched:
            return

        state.force_alarm_latched = True
        state.force_alarm_count += 1
        self.get_logger().error(
            f"[{arm}] 力阈值超限: {state.latest_force_norm:.2f}N >= {threshold_n:.2f}N，"
            f"触发急停并锁存 (count={state.force_alarm_count})"
        )
        self.stop_arm(arm)

    def stop_arm(self, arm: str) -> None:
        msg = Bool()
        msg.data = True
        self._stop_pub[arm].publish(msg)

    def send_movej(self, arm: str, joints_deg: List[float]) -> None:
        self.get_logger().info(f"[{arm}] MoveJ 目标角度(度): {joints_deg}")
        if self._dry_run:
            return

        self._confirm_motion_if_needed(
            arm=arm,
            command_name="MoveJ",
            detail=f"joints_deg={joints_deg}, speed={self._motion.speed}, block={self._motion.block}",
        )

        cmd = self._types.Movej()
        cmd.joint = [math.radians(v) for v in joints_deg]
        cmd.speed = float(self._motion.speed)
        cmd.block = bool(self._motion.block)
        self._movej_pub[arm].publish(cmd)

    def send_movel(self, arm: str, pose_xyzrpy: List[float]) -> None:
        self.get_logger().info(f"[{arm}] MoveL 目标位姿(x,y,z,rx,ry,rz): {pose_xyzrpy}")
        if self._dry_run:
            return

        self._confirm_motion_if_needed(
            arm=arm,
            command_name="MoveL",
            detail=f"pose_xyzrpy={pose_xyzrpy}, speed={self._motion.speed}, block={self._motion.block}",
        )

        cmd = self._types.Movel()
        cmd.pose = list(pose_xyzrpy)
        cmd.speed = float(self._motion.speed)
        cmd.block = bool(self._motion.block)
        self._movel_pub[arm].publish(cmd)

    def send_gripper_pick(self, arm: str, speed: int, force: int) -> None:
        self.get_logger().info(f"[{arm}] 力控夹爪 speed={speed}, force={force}")
        if self._dry_run:
            return

        self._confirm_motion_if_needed(
            arm=arm,
            command_name="GripperPick",
            detail=(
                f"speed={speed}, force={force}, block={self._motion.block}, "
                f"timeout_ms={int(self._motion.timeout_sec * 1000)}"
            ),
        )

        cmd = self._types.Gripperpick()
        cmd.speed = int(speed)
        cmd.force = int(force)
        cmd.block = bool(self._motion.block)
        cmd.timeout = int(self._motion.timeout_sec * 1000)
        self._gripper_pick_pub[arm].publish(cmd)

    def wait(self, sec: float) -> None:
        # 等待期间持续处理 ROS 回调，保证状态实时更新。
        end = self.get_clock().now().nanoseconds + int(sec * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _confirm_motion_if_needed(self, arm: str, command_name: str, detail: str) -> None:
        if not self._confirm_before_motion:
            return

        self.get_logger().info(
            f"[{arm}] 准备发送 {command_name} 指令，等待终端确认: {detail}"
        )
        prompt = (
            f"[CONFIRM] arm={arm} cmd={command_name}\\n"
            f"{detail}\\n"
            "输入 y/yes 或直接回车继续，输入其他内容取消: "
        )
        try:
            answer = input(prompt).strip().lower()
        except EOFError as exc:
            raise RuntimeError("确认模式已开启，但当前终端无法读取输入（EOF）") from exc

        if answer not in {"", "y", "yes"}:
            raise RuntimeError(f"用户取消发送指令: arm={arm}, cmd={command_name}")
