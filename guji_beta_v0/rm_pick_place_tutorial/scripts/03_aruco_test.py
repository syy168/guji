#!/usr/bin/env python3
"""
ArUco标记识别测试脚本
测试ArUco标记的识别功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import time
import yaml
import os

class ArucoTestNode(Node):
    def __init__(self):
        super().__init__('aruco_test_node')
        self.get_logger().info('ArUco标记识别测试开始...')
        
        # 加载配置
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'camera.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # ArUco配置
        self.aruco_dict_type = config['camera']['aruco']['dict_type']
        self.marker_size = config['camera']['aruco']['marker_size']
        self.target_ids = config['camera']['aruco']['target_ids']
        
        # 选择ArUco字典
        if self.aruco_dict_type == 'DICT_5X5_1000':
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        elif self.aruco_dict_type == 'DICT_4X4_1000':
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        else:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()
        
        # 订阅相机话题
        self.color_sub = self.create_subscription(
            Image,
            '/camera_right/color/image_raw',
            self.color_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_right/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # 启动识别定时器
        self.recognize_timer = self.create_timer(1.0, self.recognize_aruco)
    
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_info_received = True
            # 提取相机内参
            self.camera_matrix = [[msg.k[0], msg.k[1], msg.k[2]],
                                 [msg.k[3], msg.k[4], msg.k[5]],
                                 [msg.k[6], msg.k[7], msg.k[8]]]
            self.dist_coeffs = msg.d
            self.get_logger().info('相机内参加载成功')
    
    def color_callback(self, msg):
        try:
            self.last_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def recognize_aruco(self):
        if not hasattr(self, 'last_color_image') or not self.camera_info_received:
            return
        
        try:
            # 转换为灰度图像
            gray = cv2.cvtColor(self.last_color_image, cv2.COLOR_BGR2GRAY)
            
            # 检测ArUco标记
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                # 绘制标记
                cv2.aruco.drawDetectedMarkers(self.last_color_image, corners, ids)
                
                # 估计位姿
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                # 绘制坐标轴
                for i in range(len(ids)):
                    cv2.aruco.drawAxis(self.last_color_image, self.camera_matrix, self.dist_coeffs,
                                      rvecs[i], tvecs[i], 0.1)
                
                # 打印识别结果
                self.get_logger().info(f'检测到 {len(ids)} 个ArUco标记')
                for i, (id_val, tvec) in enumerate(zip(ids, tvecs)):
                    self.get_logger().info(f'标记 ID: {id_val[0]}, 位置: ({tvec[0][0]:.3f}, {tvec[0][1]:.3f}, {tvec[0][2]:.3f})')
            else:
                self.get_logger().info('未检测到ArUco标记')
            
            # 显示图像
            cv2.imshow('ArUco Detection', self.last_color_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'识别错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
