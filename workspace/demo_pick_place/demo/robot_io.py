from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt16

from .config_model import ArmConfig, MotionConfig, SafetyConfig
from .ros_types import RosTypes


@dataclass
class ArmRuntimeState:
    latest_joint: Optional[JointState] = None
    latest_pose: Optional[Pose] = None
    latest_force_norm: float = 0.0
    latest_error_code: int = 0


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

        self._state: Dict[str, ArmRuntimeState] = {name: ArmRuntimeState() for name in arms.keys()}
        self._movej_pub = {}
        self._movel_pub = {}
        self._gripper_pick_pub = {}
        self._stop_pub = {}

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

            self.create_subscription(
                ros_types.Sixforce,
                arm_cfg.six_force_topic,
                lambda msg, n=arm_name: self._on_force(n, msg),
                10,
            )

    def _on_joint_state(self, arm: str, msg: JointState) -> None:
        self._state[arm].latest_joint = msg

    def _on_pose(self, arm: str, msg: Pose) -> None:
        self._state[arm].latest_pose = msg

    def _on_error(self, arm: str, msg: UInt16) -> None:
        self._state[arm].latest_error_code = int(msg.data)

    def _on_force(self, arm: str, msg: object) -> None:
        fx = float(getattr(msg, "force_fx", 0.0))
        fy = float(getattr(msg, "force_fy", 0.0))
        fz = float(getattr(msg, "force_fz", 0.0))
        self._state[arm].latest_force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)

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

        if state.latest_force_norm > self._safety.max_force_n:
            self.get_logger().error(
                f"[{arm}] 力阈值超限: {state.latest_force_norm:.2f}N > {self._safety.max_force_n:.2f}N，触发急停"
            )
            self.stop_arm(arm)
            return False
        return True

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
