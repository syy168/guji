#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Empty


@dataclass
class ArmConfig:
    six_force_topic: str
    threshold_n: float


@dataclass
class MonitorConfig:
    print_hz: float
    release_ratio: float
    mode: str
    query_hz: float
    query_result: str
    arms: Dict[str, ArmConfig]


@dataclass
class ArmState:
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    norm_n: float = 0.0
    has_data: bool = False
    collision_latched: bool = False
    collision_count: int = 0


QUERY_RESULT_SUFFIX = {
    "raw": "get_force_data_result",
    "zero": "get_zero_force_data_result",
    "work": "get_work_force_data_result",
    "tool": "get_tool_force_data_result",
}


def _required(data: dict, key: str):
    if key not in data:
        raise ValueError(f"缺少必填配置项: {key}")
    return data[key]


def load_monitor_config(path: str) -> MonitorConfig:
    cfg_path = Path(path).expanduser().resolve()
    if not cfg_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {cfg_path}")

    with cfg_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    monitor_raw = raw.get("monitor", {})
    print_hz = float(monitor_raw.get("print_hz", 10.0))
    release_ratio = float(monitor_raw.get("release_ratio", 0.8))
    mode = str(monitor_raw.get("mode", "udp"))
    query_hz = float(monitor_raw.get("query_hz", 30.0))
    query_result = str(monitor_raw.get("query_result", "raw"))

    arms_raw = _required(raw, "arms")
    arms: Dict[str, ArmConfig] = {}
    for arm_name, arm_cfg in arms_raw.items():
        arms[arm_name] = ArmConfig(
            six_force_topic=str(_required(arm_cfg, "six_force_topic")),
            threshold_n=float(arm_cfg.get("threshold_n", 35.0)),
        )

    return MonitorConfig(
        print_hz=print_hz,
        release_ratio=release_ratio,
        mode=mode,
        query_hz=query_hz,
        query_result=query_result,
        arms=arms,
    )


def import_sixforce_type():
    from rm_ros_interfaces.msg import Sixforce

    return Sixforce


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


class ForceCollisionMonitor(Node):
    def __init__(
        self,
        cfg: MonitorConfig,
        selected_arms: Iterable[str],
        mode: str,
        query_hz: float,
        query_result: str,
    ) -> None:
        super().__init__("force_collision_monitor")
        self._cfg = cfg
        self._mode = mode
        self._query_hz = query_hz
        self._query_result = query_result
        self._selected_arms = list(selected_arms)
        self._state: Dict[str, ArmState] = {name: ArmState() for name in self._selected_arms}
        self._query_cmd_pub: Dict[str, object] = {}

        sixforce_type = import_sixforce_type()

        if self._mode == "udp":
            for arm in self._selected_arms:
                topic = self._cfg.arms[arm].six_force_topic
                self.create_subscription(
                    sixforce_type,
                    topic,
                    lambda msg, arm_name=arm: self._on_force(arm_name, msg),
                    50,
                )
                self.get_logger().info(
                    f"UDP模式 arm={arm} 订阅={topic} threshold={self._cfg.arms[arm].threshold_n:.2f}N"
                )
        elif self._mode == "query":
            if self._query_hz <= 0.0:
                raise ValueError("query_hz 必须大于 0")

            for arm in self._selected_arms:
                base_topic = self._cfg.arms[arm].six_force_topic
                cmd_topic, result_topic = derive_query_topics(base_topic, self._query_result)

                self._query_cmd_pub[arm] = self.create_publisher(Empty, cmd_topic, 10)
                self.create_subscription(
                    sixforce_type,
                    result_topic,
                    lambda msg, arm_name=arm: self._on_force(arm_name, msg),
                    10,
                )
                self.get_logger().info(
                    "查询模式 "
                    f"arm={arm} cmd_topic={cmd_topic} result_topic={result_topic} "
                    f"query_hz={self._query_hz:.1f} threshold={self._cfg.arms[arm].threshold_n:.2f}N"
                )

            self.create_timer(1.0 / self._query_hz, self._request_force_data)
        else:
            raise ValueError(f"不支持的 mode: {self._mode}")

        if self._cfg.print_hz <= 0.0:
            raise ValueError("print_hz 必须大于 0")
        if not (0.0 < self._cfg.release_ratio < 1.0):
            raise ValueError("release_ratio 必须在 (0, 1) 区间内")

        self.create_timer(1.0 / self._cfg.print_hz, self._print_status)

    def _request_force_data(self) -> None:
        for arm in self._selected_arms:
            pub = self._query_cmd_pub.get(arm)
            if pub is None:
                continue
            pub.publish(Empty())

    def _on_force(self, arm: str, msg: object) -> None:
        # 优先使用 rm_ros_interfaces.msg.Sixforce 字段；同时保留兼容字段名。
        fx = float(getattr(msg, "force_fx", getattr(msg, "fx", 0.0)))
        fy = float(getattr(msg, "force_fy", getattr(msg, "fy", 0.0)))
        fz = float(getattr(msg, "force_fz", getattr(msg, "fz", 0.0)))

        state = self._state[arm]
        state.fx = fx
        state.fy = fy
        state.fz = fz
        state.norm_n = math.sqrt(fx * fx + fy * fy + fz * fz)
        state.has_data = True

        threshold_n = self._cfg.arms[arm].threshold_n
        release_threshold_n = threshold_n * self._cfg.release_ratio

        if state.norm_n >= threshold_n and not state.collision_latched:
            state.collision_latched = True
            state.collision_count += 1
            self.get_logger().error(
                "[碰撞] "
                f"arm={arm} norm={state.norm_n:.2f}N threshold={threshold_n:.2f}N "
                f"collision_detected=1 count={state.collision_count}"
            )

        if state.collision_latched and state.norm_n < release_threshold_n:
            state.collision_latched = False
            self.get_logger().info(
                "[恢复] "
                f"arm={arm} norm={state.norm_n:.2f}N < release_threshold={release_threshold_n:.2f}N "
                "collision_detected=0"
            )

    def _print_status(self) -> None:
        for arm in self._selected_arms:
            state = self._state[arm]
            threshold_n = self._cfg.arms[arm].threshold_n

            if not state.has_data:
                self.get_logger().warning(
                    f"[力监控] arm={arm} 暂无数据（等待话题消息）"
                )
                continue

            status = "COLLISION" if state.collision_latched else "OK"
            self.get_logger().info(
                "[力监控] "
                f"arm={arm} fx={state.fx:.2f} fy={state.fy:.2f} fz={state.fz:.2f} "
                f"norm={state.norm_n:.2f}N threshold={threshold_n:.2f}N status={status}"
            )


def parse_args() -> argparse.Namespace:
    default_cfg = Path(__file__).resolve().parent / "config" / "monitor_config.yaml"

    p = argparse.ArgumentParser(description="独立力碰撞监控器")
    p.add_argument(
        "--config",
        default=str(default_cfg),
        help="YAML 配置文件路径",
    )
    p.add_argument(
        "--arm",
        choices=["left", "right", "both"],
        default="both",
        help="选择监控哪只机械臂（left/right/both）",
    )
    p.add_argument(
        "--threshold",
        type=float,
        default=None,
        help="覆盖所选机械臂阈值，单位 N",
    )
    p.add_argument(
        "--print-hz",
        type=float,
        default=None,
        help="覆盖终端打印频率",
    )
    p.add_argument(
        "--release-ratio",
        type=float,
        default=None,
        help="覆盖释放比例 (0,1)，默认取 YAML",
    )
    p.add_argument(
        "--mode",
        choices=["udp", "query"],
        default=None,
        help="数据来源模式：udp 订阅推送，query 主动查询（不传则用配置）",
    )
    p.add_argument(
        "--query-hz",
        type=float,
        default=None,
        help="query 模式下主动查询频率（不传则用配置）",
    )
    p.add_argument(
        "--query-result",
        choices=["raw", "zero", "work", "tool"],
        default=None,
        help="query 模式下订阅哪类力结果：raw/zero/work/tool（不传则用配置）",
    )
    p.add_argument(
        "--topic-suffix",
        choices=["udp_six_force", "udp_six_zero_force", "udp_one_force", "udp_one_zero_force"],
        default=None,
        help="覆盖力话题后缀（仅对 udp 模式有意义）",
    )
    return p.parse_args()


def select_arms(available: Dict[str, ArmConfig], arm_arg: str) -> list[str]:
    if arm_arg == "both":
        return list(available.keys())
    if arm_arg not in available:
        raise ValueError(f"请求的机械臂 '{arm_arg}' 不在配置中，当前配置: {list(available.keys())}")
    return [arm_arg]


def apply_overrides(
    cfg: MonitorConfig,
    selected_arms: Iterable[str],
    threshold: Optional[float],
    print_hz: Optional[float],
    release_ratio: Optional[float],
) -> None:
    if threshold is not None:
        for arm in selected_arms:
            cfg.arms[arm].threshold_n = float(threshold)

    if print_hz is not None:
        cfg.print_hz = float(print_hz)

    if release_ratio is not None:
        cfg.release_ratio = float(release_ratio)


def apply_topic_suffix_override(
    cfg: MonitorConfig,
    selected_arms: Iterable[str],
    topic_suffix: Optional[str],
) -> None:
    if not topic_suffix:
        return

    for arm in selected_arms:
        old_topic = cfg.arms[arm].six_force_topic
        prefix = _topic_prefix(old_topic)
        if prefix:
            cfg.arms[arm].six_force_topic = f"{prefix}/{topic_suffix}"
        else:
            cfg.arms[arm].six_force_topic = topic_suffix


def main() -> None:
    args = parse_args()
    cfg = load_monitor_config(args.config)
    selected_arms = select_arms(cfg.arms, args.arm)

    mode = cfg.mode if args.mode is None else args.mode
    query_hz = cfg.query_hz if args.query_hz is None else float(args.query_hz)
    query_result = cfg.query_result if args.query_result is None else args.query_result

    if mode not in {"udp", "query"}:
        raise ValueError(f"配置中的 mode 无效: {mode}，可选 udp/query")
    if query_result not in QUERY_RESULT_SUFFIX:
        raise ValueError(
            f"配置中的 query_result 无效: {query_result}，可选 {list(QUERY_RESULT_SUFFIX.keys())}"
        )

    apply_overrides(
        cfg=cfg,
        selected_arms=selected_arms,
        threshold=args.threshold,
        print_hz=args.print_hz,
        release_ratio=args.release_ratio,
    )
    apply_topic_suffix_override(
        cfg=cfg,
        selected_arms=selected_arms,
        topic_suffix=args.topic_suffix,
    )

    rclpy.init()
    node = ForceCollisionMonitor(
        cfg=cfg,
        selected_arms=selected_arms,
        mode=mode,
        query_hz=query_hz,
        query_result=query_result,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户请求停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
