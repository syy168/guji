#!/usr/bin/env python3
"""
TF坐标变换测试脚本
测试坐标变换功能
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import time

class TfTestNode(Node):
    def __init__(self):
        super().__init__('tf_test_node')
        self.get_logger().info('TF坐标变换测试开始...')
        
        # 创建TF缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 启动测试定时器
        self.test_timer = self.create_timer(2.0, self.test_tf_transforms)
    
    def test_tf_transforms(self):
        try:
            # 测试法兰到相机的变换（手眼标定结果）
            try:
                trans = self.tf_buffer.lookup_transform(
                    'right_top', 'camera_right', rclpy.time.Time()
                )
                self.get_logger().info('法兰到相机的变换:')
                self.get_logger().info(f'  平移: ({trans.transform.translation.x:.3f}, {trans.transform.translation.y:.3f}, {trans.transform.translation.z:.3f})')
                self.get_logger().info(f'  旋转: ({trans.transform.rotation.x:.3f}, {trans.transform.rotation.y:.3f}, {trans.transform.rotation.z:.3f}, {trans.transform.rotation.w:.3f})')
            except Exception as e:
                self.get_logger().warn(f'无法获取法兰到相机的变换: {e}')
            
            # 测试基座到法兰的变换（机械臂实时位姿）
            try:
                trans = self.tf_buffer.lookup_transform(
                    'right_base', 'right_top', rclpy.time.Time()
                )
                self.get_logger().info('基座到法兰的变换:')
                self.get_logger().info(f'  平移: ({trans.transform.translation.x:.3f}, {trans.transform.translation.y:.3f}, {trans.transform.translation.z:.3f})')
                self.get_logger().info(f'  旋转: ({trans.transform.rotation.x:.3f}, {trans.transform.rotation.y:.3f}, {trans.transform.rotation.z:.3f}, {trans.transform.rotation.w:.3f})')
            except Exception as e:
                self.get_logger().warn(f'无法获取基座到法兰的变换: {e}')
            
            # 测试基座到相机的变换（组合变换）
            try:
                trans = self.tf_buffer.lookup_transform(
                    'right_base', 'camera_right', rclpy.time.Time()
                )
                self.get_logger().info('基座到相机的变换:')
                self.get_logger().info(f'  平移: ({trans.transform.translation.x:.3f}, {trans.transform.translation.y:.3f}, {trans.transform.translation.z:.3f})')
                self.get_logger().info(f'  旋转: ({trans.transform.rotation.x:.3f}, {trans.transform.rotation.y:.3f}, {trans.transform.rotation.z:.3f}, {trans.transform.rotation.w:.3f})')
            except Exception as e:
                self.get_logger().warn(f'无法获取基座到相机的变换: {e}')
            
            # 测试基座到目标标记的变换（如果有ArUco标记在视野内）
            try:
                trans = self.tf_buffer.lookup_transform(
                    'right_base', 'target_right_camera', rclpy.time.Time()
                )
                self.get_logger().info('基座到目标标记的变换:')
                self.get_logger().info(f'  平移: ({trans.transform.translation.x:.3f}, {trans.transform.translation.y:.3f}, {trans.transform.translation.z:.3f})')
                self.get_logger().info(f'  旋转: ({trans.transform.rotation.x:.3f}, {trans.transform.rotation.y:.3f}, {trans.transform.rotation.z:.3f}, {trans.transform.rotation.w:.3f})')
            except Exception as e:
                self.get_logger().warn(f'无法获取基座到目标标记的变换: {e}')
            
        except Exception as e:
            self.get_logger().error(f'TF测试错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TfTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
