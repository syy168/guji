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
    return p.parse_args()


def main() -> None:
    args = parse_args()
    run_demo(args.config)


if __name__ == "__main__":
    main()
