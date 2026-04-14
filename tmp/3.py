#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机功能测试与图像采集脚本
测试相机的图像获取、基础诊断，并支持按键保存图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import time
import numpy as np
import sys

class CameraTestNode(Node):
    def __init__(self):
        super().__init__('camera_test_node')
        self.get_logger().info('相机功能测试与采集脚本开始运行...')
        
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
        
        # 状态标志
        self.color_received = False
        self.depth_received = False
        self.camera_info_received = False
        self.color_fps = 0.0
        self.depth_fps = 0.0
        self.depth_valid_ratio = 0.0
        
        # 图像存储（用于保存）
        self.latest_color_image = None
        self.latest_depth_image = None
        self.save_counter = 0  # 记录保存了多少组图片
        
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
            self.get_logger().info('✅ 彩色图像接收成功')
        
        # 计算帧率
        current_time = time.time()
        self.color_frame_count += 1
        if current_time - self.last_color_time >= 1.0:
            self.color_fps = self.color_frame_count / (current_time - self.last_color_time)
            self.color_frame_count = 0
            self.last_color_time = current_time
            
        # 将 ROS 图像转换为 OpenCV 格式，并显示
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_color_image = cv_image
            
            # 显示实时画面
            cv2.imshow("Camera View (Press 's' to Save, 'q' to Quit)", cv_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):
                self.save_images()
            elif key == ord('q'):
                self.get_logger().info('接收到退出指令...')
                sys.exit(0)
                
        except Exception as e:
            self.get_logger().error(f'彩色图像处理错误: {e}')
    
    def depth_callback(self, msg):
        if not self.depth_received:
            self.depth_received = True
            self.get_logger().info('✅ 深度图像接收成功')
        
        # 计算帧率
        current_time = time.time()
        self.depth_frame_count += 1
        if current_time - self.last_depth_time >= 1.0:
            self.depth_fps = self.depth_frame_count / (current_time - self.last_depth_time)
            self.depth_frame_count = 0
            self.last_depth_time = current_time
        
        # 计算深度有效像素并存储最新帧
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth_image  # 保存起来等待按键触发
            
            valid_pixels = np.count_nonzero(depth_image)
            total_pixels = depth_image.size
            self.depth_valid_ratio = (valid_pixels / total_pixels) * 100
        except Exception as e:
            self.get_logger().error(f'深度图像处理错误: {e}')

    def save_images(self):
        """保存当前时刻的彩色图和深度图"""
        if self.latest_color_image is not None and self.latest_depth_image is not None:
            self.save_counter += 1
            # 生成带时间戳的文件名，防止覆盖
            timestamp = time.strftime("%H%M%S")
            color_filename = f'color_{self.save_counter:02d}_{timestamp}.png'
            depth_filename = f'depth_{self.save_counter:02d}_{timestamp}.png'
            
            # 保存彩色图
            cv2.imwrite(color_filename, self.latest_color_image)
            # 保存深度图 (RealSense 深度图是 16位 无符号整型，OpenCV 会完美保存为 16位 PNG)
            cv2.imwrite(depth_filename, self.latest_depth_image)
            
            self.get_logger().info(f'�� 第 {self.save_counter} 组图像已保存: {color_filename} 和 {depth_filename}')
        else:
            self.get_logger().warn('⚠️ 图像数据不同步，保存失败，请重试！')
    
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info('✅ 相机内参接收成功')
            self.get_logger().info(f'   -> 分辨率: {msg.width}x{msg.height}')
            self.get_logger().info(f'   -> 内参(fx,fy,cx,cy): {msg.k[0]:.1f}, {msg.k[4]:.1f}, {msg.k[2]:.1f}, {msg.k[5]:.1f}')
    
    def wait_for_camera_data(self):
        self.get_logger().info('等待相机数据...')
        start_time = time.time()
        timeout = 10.0
        
        while not (self.color_received and self.depth_received and self.camera_info_received):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('❌ 数据获取超时！请检查 realsense 节点是否正确启动。')
                sys.exit(1)
        
        self.get_logger().info('�� 相机数据流获取成功！请在弹出的图像窗口中按 S 键保存图像。')
    
    def diagnostic_callback(self):
        self.get_logger().info('--- 相机诊断报告 ---')
        self.get_logger().info(f'彩色图像帧率: {self.color_fps:.1f} Hz')
        self.get_logger().info(f'深度图像帧率: {self.depth_fps:.1f} Hz')
        self.get_logger().info(f'深度有效像素比例: {self.depth_valid_ratio:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = CameraTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  # 安全关闭所有的 OpenCV 图像窗口
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()