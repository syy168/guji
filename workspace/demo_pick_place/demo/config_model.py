from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class ArmConfig:
    name: str
    controller_ns: str
    joint_state_topic: str
    movej_topic: str
    movel_topic: str
    gripper_pick_topic: str
    move_stop_topic: str
    six_force_topic: str
    arm_error_topic: str
    threshold_n: float = 35.0


@dataclass
class MotionConfig:
    speed: float = 20.0
    block: bool = True
    timeout_sec: float = 15.0
    post_step_wait_sec: float = 1.0


@dataclass
class SafetyConfig:
    max_force_n: float = 35.0
    release_ratio: float = 0.8
    force_mode: str = "query"
    force_query_hz: float = 50.0
    force_query_result: str = "raw"
    emergency_error_codes: List[int] = field(default_factory=lambda: [0x100D])


@dataclass
class StepAction:
    action_type: str
    arm: str
    description: str
    joints_deg: List[float] = field(default_factory=list)
    pose_xyzrpy: List[float] = field(default_factory=list)
    trajectory_file: str = ""
    segment_count: int = 5
    gripper_force: int = 0
    gripper_speed: int = 0
    pause_sec: float = 0.0


@dataclass
class DemoConfig:
    config_dir: str
    dry_run: bool
    loop_forever: bool
    confirm_before_motion: bool
    motion: MotionConfig
    safety: SafetyConfig
    arms: Dict[str, ArmConfig]
    sequence: List[StepAction]
