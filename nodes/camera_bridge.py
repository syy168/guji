#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
相机诊断节点

功能：
- 订阅 RealSense D435 的图像话题，验证相机数据流正常
- 诊断检查：帧率、深度有效性、内参完整性、话题连通性
- 定时输出诊断摘要日志
- 提供 Service 接口查询当前相机状态

诊断项目：
  1. 话题连通性：color/depth/image_raw 话题是否有数据
  2. 帧率检查：实际帧率是否接近配置帧率（30fps）
  3. 深度有效性：深度图像中有效像素的比例
  4. 内参完整性：camera_info 中 K/D 矩阵是否完整

运行前检查：
  ros2 run guji camera_bridge
  ros2 topic hz /camera_right/color/image_raw     # 应 ≥ 15Hz
  ros2 topic echo /camera_right/color/camera_info  # 应有完整 K/D
  ros2 service call /right_arm_controller/get_camera_status std_srvs/srv/Trigger {}  # 查询状态

依赖：
  - realsense2_camera 节点已启动并发布 /camera_right/* 话题
  - system.yaml 配置文件
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np
import yaml
import os
import time


class CameraBridge(Node):
    """相机诊断与数据桥接节点"""

    def __init__(self):
        super().__init__('camera_bridge')

        self._load_config()

        # ==========================================
        # 状态变量
        # ==========================================
        self._color_image = None
        self._depth_image = None
        self._camera_info = None
        self._last_color_stamp = None
        self._last_depth_stamp = None

        # 帧率统计
        self._color_frame_times = []
        self._depth_frame_times = []

        # 诊断计数器
        self._diagnostic_count = 0
        self._diagnostic_ok = True
        self._diagnostic_messages = []

        # ==========================================
        # 订阅相机话题
        # ==========================================
        self._color_sub = self.create_subscription(
            Image,
            f'{self.topic_prefix}/color/image_raw',
            self._color_callback,
            10
        )

        self._depth_sub = self.create_subscription(
            Image,
            f'{self.topic_prefix}/aligned_depth_to_color/image_raw',
            self._depth_callback,
            10
        )

        self._info_sub = self.create_subscription(
            CameraInfo,
            f'{self.topic_prefix}/color/camera_info',
            self._info_callback,
            10
        )

        # ==========================================
        # 提供相机状态 Service
        # ==========================================
        self._status_srv = self.create_service(
            Trigger,
            '/right_arm_controller/get_camera_status',
            self._status_callback
        )

        # ==========================================
        # 诊断定时器（每 N 秒输出一次）
        # ==========================================
        diag_interval = self.cfg.get('camera_diagnostic_interval', 5.0)
        self._diag_timer = self.create_timer(diag_interval, self._publish_diagnostic)
        self._diag_start_time = time.time()

        self.get_logger().info('相机诊断节点已启动')
        self.get_logger().info(f'  监听话题: {self.topic_prefix}/color/image_raw')
        self.get_logger().info(f'  监听话题: {self.topic_prefix}/aligned_depth_to_color/image_raw')
        self.get_logger().info(f'  诊断间隔: {diag_interval}s')
        self.get_logger().info(f'  期望帧率: {self.expected_fps} Hz')

    def _load_config(self):
        """从 system.yaml 加载配置"""
        possible_paths = [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'system.yaml'),
            os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'system.yaml'),
        ]
        config_path = None
        for p in possible_paths:
            if os.path.exists(p):
                config_path = p
                break

        self.cfg = {}
        self.topic_prefix = '/camera_right'
        self.expected_fps = 30
        self.expected_width = 640
        self.expected_height = 480
        self.depth_valid_ratio_threshold = 0.5  # 深度有效像素至少 50%

        if config_path:
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    full_cfg = yaml.safe_load(f)
                sys_cfg = full_cfg.get('system', {})
                intr_cfg = full_cfg.get('camera', {}).get('intrinsic', {})

                ros2_cfg = sys_cfg.get('ros2', {})
                self.cfg['camera_diagnostic_interval'] = ros2_cfg.get('camera_diagnostic_interval', 5.0)
                self.cfg['debug_enabled'] = sys_cfg.get('debug', {}).get('enabled', False)
                self.cfg['show_image'] = sys_cfg.get('debug', {}).get('show_image', False)

                cam_cfg = sys_cfg.get('camera', {})
                self.topic_prefix = cam_cfg.get('topic_prefix', '/camera_right')
                self.expected_fps = intr_cfg.get('fps', 30)
                self.expected_width = intr_cfg.get('width', 640)
                self.expected_height = intr_cfg.get('height', 480)
                self.get_logger().info(f'已加载配置: {config_path}')
            except Exception as e:
                self.get_logger().warn(f'加载 system.yaml 失败: {e}，使用默认值')
        else:
            self.get_logger().warn('system.yaml 未找到，使用默认配置')

    # ==========================================
    # 回调函数
    # ==========================================
    def _color_callback(self, msg: Image):
        self._color_image = msg
        now = self.get_clock().now()
        self._color_frame_times.append(now)

        # 保留最近 60 个时间戳用于帧率计算
        if len(self._color_frame_times) > 60:
            self._color_frame_times.pop(0)

        if self._last_color_stamp is None:
            self.get_logger().info(f'Color 图像流已连接！width={msg.width}, height={msg.height}')

        self._last_color_stamp = now

    def _depth_callback(self, msg: Image):
        self._depth_image = msg
        now = self.get_clock().now()
        self._depth_frame_times.append(now)

        if len(self._depth_frame_times) > 60:
            self._depth_frame_times.pop(0)

        if self._last_depth_stamp is None:
            self.get_logger().info(f'Depth 图像流已连接！width={msg.width}, height={msg.height}')

        self._last_depth_stamp = now

    def _info_callback(self, msg: CameraInfo):
        self._camera_info = msg

        # 检查内参完整性
        if len(msg.K) == 9 and len(msg.D) >= 5:
            if self._camera_info is not None and len(self._camera_info.K) != 9:
                self.get_logger().info('CameraInfo 内参已就绪')
        else:
            if self._diagnostic_ok:
                self.get_logger().warn('CameraInfo 内参数据不完整！')

    def _status_callback(self, req: Trigger.Request) -> TriggerResponse:
        """Service 回调：返回当前相机状态"""
        resp = TriggerResponse()
        status, msg = self._check_camera_health()
        resp.success = status
        resp.message = msg
        return resp

    # ==========================================
    # 诊断检查
    # ==========================================
    def _calculate_fps(self, frame_times):
        """根据时间戳列表计算实际帧率"""
        if len(frame_times) < 2:
            return 0.0
        total_time = (frame_times[-1] - frame_times[0]).nanoseconds / 1e9
        if total_time <= 0:
            return 0.0
        return (len(frame_times) - 1) / total_time

    def _check_depth_valid_ratio(self):
        """检查深度图像有效像素比例"""
        if self._depth_image is None:
            return 0.0
        try:
            depth_data = np.frombuffer(self._depth_image.data, dtype=np.uint16)
            valid_count = np.count_nonzero(depth_data > 0)
            total_count = depth_data.size
            return valid_count / total_count if total_count > 0 else 0.0
        except Exception:
            return 0.0

    def _check_camera_health(self):
        """
        综合相机健康检查。
        返回: (success: bool, message: str)
        """
        messages = []
        all_ok = True

        # 1. 检查话题连通性
        if self._last_color_stamp is None:
            messages.append('[WARN] Color 图像话题无数据')
            all_ok = False
        else:
            elapsed = (self.get_clock().now() - self._last_color_stamp).nanoseconds / 1e9
            if elapsed > 3.0:
                messages.append(f'[WARN] Color 图像超时 {elapsed:.1f}s 无数据')
                all_ok = False

        if self._last_depth_stamp is None:
            messages.append('[WARN] Depth 图像话题无数据')
            all_ok = False
        else:
            elapsed = (self.get_clock().now() - self._last_depth_stamp).nanoseconds / 1e9
            if elapsed > 3.0:
                messages.append(f'[WARN] Depth 图像超时 {elapsed:.1f}s 无数据')
                all_ok = False

        # 2. 检查帧率
        color_fps = self._calculate_fps(self._color_frame_times)
        depth_fps = self._calculate_fps(self._depth_frame_times)

        if color_fps > 0:
            fps_ratio = color_fps / self.expected_fps
            if fps_ratio < 0.3:
                messages.append(f'[WARN] Color 帧率过低: {color_fps:.1f}Hz (期望≥{self.expected_fps}Hz)')
                all_ok = False
        else:
            messages.append('[WARN] Color 帧率无法计算')
            all_ok = False

        # 3. 检查深度有效性
        depth_ratio = self._check_depth_valid_ratio()
        if depth_ratio < self.depth_valid_ratio_threshold:
            messages.append(
                f'[WARN] 深度有效像素比例过低: {depth_ratio:.1%} '
                f'(期望≥{self.depth_valid_ratio_threshold:.0%})'
            )
            all_ok = False

        # 4. 检查内参
        if self._camera_info is None:
            messages.append('[WARN] CameraInfo 未收到')
            all_ok = False
        elif len(self._camera_info.K) != 9 or len(self._camera_info.D) < 5:
            messages.append('[WARN] CameraInfo 内参数据不完整')
            all_ok = False

        summary = 'OK' if all_ok else 'DEGRADED'
        msg = f'[{summary}] Color:{color_fps:.0f}Hz DepthValid:{depth_ratio:.0%} | {\" | \".join(messages) if messages else "All checks passed"}'
        return all_ok, msg

    def _publish_diagnostic(self):
        """定时输出诊断摘要"""
        self._diagnostic_count += 1
        status, msg = self._check_camera_health()

        elapsed = time.time() - self._diag_start_time
        prefix = '[OK  ]' if status else '[WARN]'

        self.get_logger().info(
            f'--- 相机诊断 #{self._diagnostic_count} (运行 {elapsed:.0f}s) ---')
        self.get_logger().info(f'  {prefix} {msg}')

        if not status:
            self.get_logger().warn(
                '  相机状态异常！请检查: \n'
                '    1. realsense2_camera 节点是否运行: ros2 node list\n'
                '    2. 相机话题是否发布: ros2 topic list | grep camera_right\n'
                '    3. 相机连接线是否松动\n'
                '    4. 序列号是否配置正确'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()

    print()
    print("========================================")
    print("  相机诊断节点 (Camera Bridge)")
    print("========================================")
    print()
    print("  订阅话题:")
    print("    /camera_right/color/image_raw")
    print("    /camera_right/aligned_depth_to_color/image_raw")
    print("    /camera_right/color/camera_info")
    print()
    print("  Service:")
    print("    /right_arm_controller/get_camera_status")
    print()
    print("  诊断命令:")
    print("    ros2 topic hz /camera_right/color/image_raw")
    print("    ros2 topic echo /camera_right/color/camera_info")
    print("    ros2 service call /right_arm_controller/get_camera_status std_srvs/srv/Trigger {}")
    print()
    print("========================================")
    print()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
