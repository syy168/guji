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

        # 仅对已配置的机械臂执行安全检查（避免 pause 等非机械臂动作误判）。
        if step.arm in self._cfg.arms:
            # 执行前先检查错误码/力阈值；失败时会在 IO 层触发停止。
            if not self._io.safety_check_or_stop(step.arm):
                # 直接中断流程，防止继续发送后续动作。
                raise RuntimeError(f"步骤 {idx} 执行前安全检查失败")

        # 根据 action_type 分发到不同的执行分支。
        if step.action_type in {"movej", "trajectory_movej"}:
            # 配了 trajectory_file（或显式 trajectory_movej）就走“轨迹中点”执行逻辑。
            if step.trajectory_file or step.action_type == "trajectory_movej":
                self._run_movej_from_trajectory(idx, step)
            else:
                # 纯 movej 要求提供 6 轴关节角。
                self._require_len(step.joints_deg, min_len=6, msg=f"Step {idx} movej joints_deg")
                # 发送单次 MoveJ 指令。
                self._io.send_movej(step.arm, step.joints_deg)

        elif step.action_type == "movel":
            # movel 需要完整 6 维位姿 [x, y, z, rx, ry, rz]。
            self._require_len(step.pose_xyzrpy, min_len=6, msg=f"Step {idx} movel pose_xyzrpy")
            # 发送笛卡尔直线运动指令。
            self._io.send_movel(step.arm, step.pose_xyzrpy)

        elif step.action_type == "gripper_pick":
            # 发送夹爪夹取（速度/力度由步骤配置给定）。
            self._io.send_gripper_pick(step.arm, step.gripper_speed, step.gripper_force)

        elif step.action_type == "pause":
            # pause 不下发控制，仅等待一段时间。
            self._node.get_logger().info("暂停步骤：仅等待，不发送控制命令")
            # 若步骤配置了 pause_sec 用步骤值，否则退回全局默认等待时间。
            pause_sec = step.pause_sec if step.pause_sec > 0 else self._cfg.motion.post_step_wait_sec
            # 等待期间持续 spin_once，保证订阅状态实时刷新。
            self._io.wait(pause_sec)
            # 打印当前机械臂运行态（力/错误码）用于观察。
            self._dump_runtime_state(step.arm)
            # pause 分支执行完直接返回，不走后面的“动作后等待与二次检查”。
            return

        else:
            # 未知动作类型直接抛错，避免静默忽略配置问题。
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

        # 支持相对路径：相对路径按配置目录拼接，统一解析为绝对路径。
        path = Path(step.trajectory_file)
        if not path.is_absolute():
            path = Path(self._cfg.config_dir) / path
        path = path.resolve()

        # 轨迹文件做内存缓存，避免同一文件被重复读取与解析。
        cache_key = str(path)
        if cache_key not in self._trajectory_cache:
            self._trajectory_cache[cache_key] = load_joint_trajectory_deg(path)

        raw_points = self._trajectory_cache[cache_key]
        # 将整条轨迹按段采样为“中点序列”，减少指令数量并保留路径代表性。
        segment_count = max(1, step.segment_count)
        mid_points = sample_segment_midpoints(raw_points, segment_count)
        self._node.get_logger().info(
            f"步骤 {idx} 轨迹加载完成: {path}, 原始点数={len(raw_points)}, 中点数={len(mid_points)}"
        )

        # 逐个下发中点：每个点执行前后都进行一次安全检查，发现异常立即中断。
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
