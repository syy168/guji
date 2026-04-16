#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class BasicTestNode(Node):
    def __init__(self):
        super().__init__('basic_test_node')
        self.get_logger().info('基础功能测试开始...')
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # 使用字典来存储拼接左右臂的关节数据
        self.all_joints = {}
        self.test_finished = False
        
        self.wait_for_joint_data()
    
    def joint_callback(self, msg):
        # 只要测试还没结束，就不断把收到的关节更新到字典里
        if not self.test_finished:
            for name, position in zip(msg.name, msg.position):
                self.all_joints[name] = position
    
    def wait_for_joint_data(self):
        self.get_logger().info('正在收集双臂关节数据，请稍候...')
        start_time = time.time()
        timeout = 10.0
        
        # 循环等待，直到字典里收集到了 12 个关节
        while len(self.all_joints) < 12:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if time.time() - start_time > timeout:
                self.get_logger().error(f'获取超时！当前只收集到了 {len(self.all_joints)} 个关节。')
                return
                
        # 收集齐 12 个关节后，标记完成并打印
        self.test_finished = True
        self.get_logger().info('✅ 成功集齐双臂 12 个关节数据:')
        
        # 按照关节名字排序打印（让 l_joint 和 r_joint 整齐排列）
        for name in sorted(self.all_joints.keys()):
            self.get_logger().info(f'  {name}: {self.all_joints[name]:.4f} rad')
            
        self.get_logger().info('基础功能测试圆满完成！')

def main(args=None):
    rclpy.init(args=args)
    node = BasicTestNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()