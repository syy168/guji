from __future__ import annotations

import rclpy
from rclpy.node import Node

from .config_loader import load_demo_config
from .robot_io import RobotIO
from .ros_types import import_rm_ros_types
from .sequence_runner import SequenceRunner


class DemoMain(Node):
    """取放 Demo 主节点：负责加载配置并启动流程。"""

    def __init__(self, config_path: str) -> None:
        super().__init__("demo_main")
        self._cfg = load_demo_config(config_path)
        self._types = import_rm_ros_types()

        self.get_logger().info(f"已加载配置: {config_path}")
        self.get_logger().info(f"机械臂列表: {list(self._cfg.arms.keys())}")

        self._io = RobotIO(
            arms=self._cfg.arms,
            motion=self._cfg.motion,
            safety=self._cfg.safety,
            ros_types=self._types,
            dry_run=self._cfg.dry_run,
        )
        self._runner = SequenceRunner(self, self._io, self._cfg)

    def run(self) -> None:
        # 给订阅者预热时间，避免刚启动时拿不到状态。
        self._io.wait(1.0)

        if self._cfg.loop_forever:
            while rclpy.ok():
                self._runner.run_once()
        else:
            self._runner.run_once()


def run_demo(config_path: str) -> None:
    rclpy.init()
    app = DemoMain(config_path)
    try:
        app.run()
    finally:
        app.destroy_node()
        app._io.destroy_node()
        rclpy.shutdown()
