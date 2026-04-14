#!/usr/bin/env python3
"""
基础功能测试脚本
测试机械臂的基本连接和关节状态获取
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class BasicTestNode(Node):
    def __init__(self):
        super().__init__('basic_test_node')
        self.get_logger().info('基础功能测试开始...')
        
        # 订阅左右臂关节状态
        self.left_joint_sub = self.create_subscription(
            JointState,
            '/left_arm_controller/joint_states',
            self.left_joint_callback,
            10
        )
        
        self.right_joint_sub = self.create_subscription(
            JointState,
            '/right_arm_controller/joint_states',
            self.right_joint_callback,
            10
        )
        
        self.left_joint_data = None
        self.right_joint_data = None
        self.left_joint_received = False
        self.right_joint_received = False
        
        # 等待关节数据
        self.wait_for_joint_data()
    
    def left_joint_callback(self, msg):
        if not self.left_joint_received:
            self.left_joint_data = msg
            self.left_joint_received = True
            self.get_logger().info(f'左臂关节数据接收成功: {len(msg.name)} 个关节')
            for name, position in zip(msg.name, msg.position):
                self.get_logger().info(f'  {name}: {position:.2f} rad')
    
    def right_joint_callback(self, msg):
        if not self.right_joint_received:
            self.right_joint_data = msg
            self.right_joint_received = True
            self.get_logger().info(f'右臂关节数据接收成功: {len(msg.name)} 个关节')
            for name, position in zip(msg.name, msg.position):
                self.get_logger().info(f'  {name}: {position:.2f} rad')
    
    def wait_for_joint_data(self):
        self.get_logger().info('等待关节状态数据...')
        start_time = time.time()
        timeout = 10.0
        
        while not (self.left_joint_received and self.right_joint_received):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('关节数据获取超时')
                return
        
        self.get_logger().info('关节数据获取成功！')
        self.get_logger().info('基础功能测试完成。')

def main(args=None):
    rclpy.init(args=args)
    node = BasicTestNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
