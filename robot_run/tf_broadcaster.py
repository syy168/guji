#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
独立可运行版 TF 兼容桥接节点（直接 python3 运行）。

用途：
- 基于 RealMan 官方 TF（如 base_link -> Link6/Link7）
- 转发生成 guji 教程需要的 TF（right_base -> right_top）

示例：
1) 默认运行（自动在 Link6/Link7 中探测）
   python3 guji/rm_official_tf_broadcaster_run.py

2) 指定源末端帧
   python3 guji/rm_official_tf_broadcaster_run.py --source-tcp-frame Link7

3) 指定别名帧
   python3 guji/rm_official_tf_broadcaster_run.py --alias-base-frame right_base --alias-tcp-frame right_top
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster


class RmOfficialTFAliasBroadcaster(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("rm_official_tf_broadcaster")

        self.source_base = args.source_base_frame
        self.source_tcp = args.source_tcp_frame.strip()
        self.source_tcp_candidates = list(args.source_tcp_candidates)
        self.alias_base = args.alias_base_frame
        self.alias_tcp = args.alias_tcp_frame
        self.rate_hz = args.publish_rate_hz if args.publish_rate_hz > 0.0 else 30.0
        self.publish_static_base_alias = args.publish_static_base_alias

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self._resolved_tcp = None

        if self.publish_static_base_alias and self.alias_base != self.source_base:
            self._publish_static_base_alias()

        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        self.get_logger().info("rm_official_tf_broadcaster (standalone) 已启动")
        self.get_logger().info(f"  源基坐标系: {self.source_base}")
        self.get_logger().info(f"  源末端候选: {self.source_tcp_candidates}")
        self.get_logger().info(f"  目标别名: {self.alias_base} -> {self.alias_tcp}")

    def _publish_static_base_alias(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.alias_base
        t.child_frame_id = self.source_base
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f"  已发布静态别名 TF: {self.alias_base} -> {self.source_base} (identity)")

    def _resolve_source_tcp(self):
        if self.source_tcp:
            self._resolved_tcp = self.source_tcp
            return True

        for cand in self.source_tcp_candidates:
            try:
                self.tf_buffer.lookup_transform(
                    self.source_base,
                    cand,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                self._resolved_tcp = cand
                self.get_logger().info(f"  已自动选择源末端 frame: {cand}")
                return True
            except Exception:
                continue
        return False

    def _on_timer(self):
        if self._resolved_tcp is None:
            if not self._resolve_source_tcp():
                self.get_logger().warn(
                    "等待官方 TF 可用中：未找到 source_base -> source_tcp。"
                    f" 当前 source_base={self.source_base}, candidates={self.source_tcp_candidates}",
                    throttle_duration_sec=2.0,
                )
                return

        try:
            tr = self.tf_buffer.lookup_transform(
                self.source_base,
                self._resolved_tcp,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(f"查询官方 TF 失败: {e}", throttle_duration_sec=2.0)
            return

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.alias_base
        out.child_frame_id = self.alias_tcp
        out.transform = tr.transform
        self.tf_broadcaster.sendTransform(out)


def parse_args():
    p = argparse.ArgumentParser(description="RealMan 官方 TF -> guji 别名 TF 兼容桥接")
    p.add_argument("--source-base-frame", default="base_link", help="官方源基座 frame")
    p.add_argument(
        "--source-tcp-frame",
        default="",
        help="官方源末端 frame，留空则自动从候选中探测",
    )
    p.add_argument(
        "--source-tcp-candidates",
        nargs="+",
        default=["Link6", "Link7"],
        help="自动探测时的末端候选 frame 列表",
    )
    p.add_argument("--alias-base-frame", default="right_base", help="兼容层基座 frame")
    p.add_argument("--alias-tcp-frame", default="right_top", help="兼容层末端 frame")
    p.add_argument("--publish-rate-hz", type=float, default=30.0, help="动态 TF 发布频率")
    p.add_argument(
        "--publish-static-base-alias",
        action="store_true",
        default=True,
        help="是否发布静态别名 alias_base -> source_base（默认开）",
    )
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RmOfficialTFAliasBroadcaster(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

