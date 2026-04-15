from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

import yaml

from .config_model import ArmConfig, DemoConfig, MotionConfig, SafetyConfig, StepAction


def _required(data: Dict[str, Any], key: str) -> Any:
    if key not in data:
        raise ValueError(f"缺少必填配置项: {key}")
    return data[key]


def load_demo_config(config_path: str) -> DemoConfig:
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"配置文件不存在: {path}")

    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    motion_raw = raw.get("motion", {})
    motion = MotionConfig(
        speed=float(motion_raw.get("speed", 20.0)),
        block=bool(motion_raw.get("block", True)),
        timeout_sec=float(motion_raw.get("timeout_sec", 15.0)),
        post_step_wait_sec=float(motion_raw.get("post_step_wait_sec", 1.0)),
    )

    safety_raw = raw.get("safety", {})
    safety = SafetyConfig(
        max_force_n=float(safety_raw.get("max_force_n", 35.0)),
        emergency_error_codes=list(safety_raw.get("emergency_error_codes", [0x100D])),
    )

    arms_raw = _required(raw, "arms")
    arms: Dict[str, ArmConfig] = {}
    for arm_name, cfg in arms_raw.items():
        arms[arm_name] = ArmConfig(
            name=arm_name,
            controller_ns=str(_required(cfg, "controller_ns")),
            joint_state_topic=str(_required(cfg, "joint_state_topic")),
            movej_topic=str(_required(cfg, "movej_topic")),
            movel_topic=str(_required(cfg, "movel_topic")),
            gripper_pick_topic=str(_required(cfg, "gripper_pick_topic")),
            move_stop_topic=str(_required(cfg, "move_stop_topic")),
            six_force_topic=str(_required(cfg, "six_force_topic")),
            arm_error_topic=str(_required(cfg, "arm_error_topic")),
        )

    sequence_raw = _required(raw, "sequence")
    sequence: List[StepAction] = []
    for item in sequence_raw:
        sequence.append(
            StepAction(
                action_type=str(_required(item, "action_type")),
                arm=str(_required(item, "arm")),
                description=str(_required(item, "description")),
                joints_deg=list(item.get("joints_deg", [])),
                pose_xyzrpy=list(item.get("pose_xyzrpy", [])),
                trajectory_file=str(item.get("trajectory_file", "")),
                segment_count=int(item.get("segment_count", 5)),
                gripper_force=int(item.get("gripper_force", 0)),
                gripper_speed=int(item.get("gripper_speed", 0)),
                pause_sec=float(item.get("pause_sec", 0.0)),
            )
        )

    return DemoConfig(
        config_dir=str(path.parent),
        dry_run=bool(raw.get("dry_run", True)),
        loop_forever=bool(raw.get("loop_forever", False)),
        confirm_before_motion=bool(raw.get("confirm_before_motion", False)),
        motion=motion,
        safety=safety,
        arms=arms,
        sequence=sequence,
    )
