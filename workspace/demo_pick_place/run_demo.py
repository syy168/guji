#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from demo.main_node import run_demo


def parse_args() -> argparse.Namespace:
    default_cfg = Path(__file__).resolve().parent / "config" / "demo_config.yaml"
    p = argparse.ArgumentParser(description="双臂取放 Demo（模块清晰、解耦设计）")
    p.add_argument(
        "--config",
        default=str(default_cfg),
        help="YAML 配置文件路径",
    )
    p.add_argument(
        "--confirm-before-motion",
        action="store_true",
        help="每次发送运动指令前在终端等待人工确认",
    )
    p.add_argument(
        "--no-confirm-before-motion",
        action="store_true",
        help="关闭每次发送运动指令前的人工确认（覆盖配置）",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if args.confirm_before_motion and args.no_confirm_before_motion:
        raise ValueError("--confirm-before-motion 与 --no-confirm-before-motion 不能同时使用")

    confirm_override = None
    if args.confirm_before_motion:
        confirm_override = True
    elif args.no_confirm_before_motion:
        confirm_override = False

    run_demo(args.config, confirm_before_motion_override=confirm_override)


if __name__ == "__main__":
    main()
