from __future__ import annotations

from pathlib import Path
from typing import List

from rclpy.node import Node

from .config_model import DemoConfig, StepAction
from .robot_io import RobotIO
from .trajectory_loader import load_joint_trajectory_deg, sample_segment_midpoints


class SequenceRunner:
    """执行 0-11 工作流，并输出清晰的分步日志。"""

    def __init__(self, node: Node, io: RobotIO, cfg: DemoConfig) -> None:
        self._node = node
        self._io = io
        self._cfg = cfg
        self._trajectory_cache = {}

    def run_once(self) -> None:
        self._node.get_logger().info("=" * 70)
        self._node.get_logger().info("开始执行一轮 Demo 流程")
        self._node.get_logger().info(f"dry_run={self._cfg.dry_run}")
        self._node.get_logger().info("=" * 70)

        for idx, step in enumerate(self._cfg.sequence):
            self._run_step(idx, step)

        self._node.get_logger().info("本轮 Demo 流程执行完成")

    def _run_step(self, idx: int, step: StepAction) -> None:
        self._node.get_logger().info("-" * 70)
        self._node.get_logger().info(f"步骤 {idx}: {step.description}")
        self._node.get_logger().info(f"动作={step.action_type}, 机械臂={step.arm}")

        if step.arm in self._cfg.arms:
            if not self._io.safety_check_or_stop(step.arm):
                raise RuntimeError(f"步骤 {idx} 执行前安全检查失败")

        if step.action_type in {"movej", "trajectory_movej"}:
            if step.trajectory_file or step.action_type == "trajectory_movej":
                self._run_movej_from_trajectory(idx, step)
            else:
                self._require_len(step.joints_deg, min_len=6, msg=f"Step {idx} movej joints_deg")
                self._io.send_movej(step.arm, step.joints_deg)

        elif step.action_type == "movel":
            self._require_len(step.pose_xyzrpy, min_len=6, msg=f"Step {idx} movel pose_xyzrpy")
            self._io.send_movel(step.arm, step.pose_xyzrpy)

        elif step.action_type == "gripper_pick":
            self._io.send_gripper_pick(step.arm, step.gripper_speed, step.gripper_force)

        elif step.action_type == "pause":
            self._node.get_logger().info("暂停步骤：仅等待，不发送控制命令")
            pause_sec = step.pause_sec if step.pause_sec > 0 else self._cfg.motion.post_step_wait_sec
            self._io.wait(pause_sec)
            self._dump_runtime_state(step.arm)
            return

        else:
            raise ValueError(f"步骤 {idx} 存在未知 action_type: {step.action_type}")

        self._io.wait(self._cfg.motion.post_step_wait_sec)

        if step.arm in self._cfg.arms:
            if not self._io.safety_check_or_stop(step.arm):
                raise RuntimeError(f"步骤 {idx} 执行后安全检查失败")

        self._dump_runtime_state(step.arm)

    def _dump_runtime_state(self, arm: str) -> None:
        if arm not in self._cfg.arms:
            return
        state = self._io.arm_state(arm)
        self._node.get_logger().info(
            f"[{arm}] force_norm={state.latest_force_norm:.3f}N, error=0x{state.latest_error_code:04X}"
        )

    def _run_movej_from_trajectory(self, idx: int, step: StepAction) -> None:
        if not step.trajectory_file:
            raise ValueError(f"步骤 {idx} 缺少 trajectory_file 配置")

        path = Path(step.trajectory_file)
        if not path.is_absolute():
            path = Path(self._cfg.config_dir) / path
        path = path.resolve()

        cache_key = str(path)
        if cache_key not in self._trajectory_cache:
            self._trajectory_cache[cache_key] = load_joint_trajectory_deg(path)

        raw_points = self._trajectory_cache[cache_key]
        segment_count = max(1, step.segment_count)
        mid_points = sample_segment_midpoints(raw_points, segment_count)
        self._node.get_logger().info(
            f"步骤 {idx} 轨迹加载完成: {path}, 原始点数={len(raw_points)}, 中点数={len(mid_points)}"
        )

        for p_idx, joints_deg in enumerate(mid_points):
            if not self._io.safety_check_or_stop(step.arm):
                raise RuntimeError(f"步骤 {idx} 第 {p_idx} 个中点执行前安全检查失败")
            self._node.get_logger().info(f"步骤 {idx} 执行中点 {p_idx + 1}/{len(mid_points)}")
            self._io.send_movej(step.arm, joints_deg)
            self._io.wait(self._cfg.motion.post_step_wait_sec)
            if not self._io.safety_check_or_stop(step.arm):
                raise RuntimeError(f"步骤 {idx} 第 {p_idx} 个中点执行后安全检查失败")

    @staticmethod
    def _require_len(arr: List[float], min_len: int, msg: str) -> None:
        if len(arr) < min_len:
            raise ValueError(f"{msg} 长度不足，要求 >= {min_len}，实际为 {len(arr)}")
