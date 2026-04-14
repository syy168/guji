#!/usr/bin/env python3
"""
相机功能测试脚本
测试相机的图像获取和基本诊断功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import time
import numpy as np

class CameraTestNode(Node):
    def __init__(self):
        super().__init__('camera_test_node')
        self.get_logger().info('相机功能测试开始...')
        
        self.bridge = CvBridge()
        
        # 订阅相机话题
        self.color_sub = self.create_subscription(
            Image,
            '/camera_right/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera_right/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_right/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 诊断数据
        self.color_received = False
        self.depth_received = False
        self.camera_info_received = False
        self.color_fps = 0.0
        self.depth_fps = 0.0
        self.depth_valid_ratio = 0.0
        
        # 时间戳
        self.last_color_time = time.time()
        self.last_depth_time = time.time()
        self.color_frame_count = 0
        self.depth_frame_count = 0
        
        # 启动诊断定时器
        self.diagnostic_timer = self.create_timer(5.0, self.diagnostic_callback)
        
        # 等待数据
        self.wait_for_camera_data()
    
    def color_callback(self, msg):
        if not self.color_received:
            self.color_received = True
            self.get_logger().info('彩色图像接收成功')
        
        # 计算帧率
        current_time = time.time()
        self.color_frame_count += 1
        if current_time - self.last_color_time >= 1.0:
            self.color_fps = self.color_frame_count / (current_time - self.last_color_time)
            self.color_frame_count = 0
            self.last_color_time = current_time
    
    def depth_callback(self, msg):
        if not self.depth_received:
            self.depth_received = True
            self.get_logger().info('深度图像接收成功')
        
        # 计算帧率
        current_time = time.time()
        self.depth_frame_count += 1
        if current_time - self.last_depth_time >= 1.0:
            self.depth_fps = self.depth_frame_count / (current_time - self.last_depth_time)
            self.depth_frame_count = 0
            self.last_depth_time = current_time
        
        # 计算深度有效像素比例
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            valid_pixels = np.count_nonzero(depth_image)
            total_pixels = depth_image.size
            self.depth_valid_ratio = (valid_pixels / total_pixels) * 100
        except Exception as e:
            self.get_logger().error(f'深度图像处理错误: {e}')
    
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info('相机内参接收成功')
            self.get_logger().info(f'相机分辨率: {msg.width}x{msg.height}')
            self.get_logger().info(f'相机内参矩阵: {msg.k[:3]}')
    
    def wait_for_camera_data(self):
        self.get_logger().info('等待相机数据...')
        start_time = time.time()
        timeout = 10.0
        
        while not (self.color_received and self.depth_received and self.camera_info_received):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('相机数据获取超时')
                return
        
        self.get_logger().info('相机数据获取成功！')
    
    def diagnostic_callback(self):
        self.get_logger().info('--- 相机诊断 ---')
        self.get_logger().info(f'彩色图像帧率: {self.color_fps:.1f} Hz')
        self.get_logger().info(f'深度图像帧率: {self.depth_fps:.1f} Hz')
        self.get_logger().info(f'深度有效像素比例: {self.depth_valid_ratio:.1f}%')
        
        # 检查状态
        if self.color_fps < 15.0:
            self.get_logger().warn('彩色图像帧率过低')
        if self.depth_fps < 15.0:
            self.get_logger().warn('深度图像帧率过低')
        if self.depth_valid_ratio < 50.0:
            self.get_logger().warn('深度有效像素比例过低')

def main(args=None):
    rclpy.init(args=args)
    node = CameraTestNode()
    
    # 运行一段时间进行测试
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
