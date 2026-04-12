#!/usr/bin/env python3
"""
单臂功能测试脚本
测试单个机械臂的运动和夹爪控制
"""

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import MovejCmd, MovelCmd, SetGripperPositionCmd, SetGripperPickCmd
from sensor_msgs.msg import JointState
import time
import yaml
import os
import math

class SingleArmTestNode(Node):
    def __init__(self):
        super().__init__('single_arm_test_node')
        self.get_logger().info('单臂功能测试开始...')
        
        # 加载配置
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'poses.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.poses = config['poses']
        
        # 创建发布者
        self.right_movej_pub = self.create_publisher(MovejCmd, '/right_arm_controller/rm_driver/movej_cmd', 10)
        self.right_gripper_set_pub = self.create_publisher(SetGripperPositionCmd, '/right_arm_controller/rm_driver/set_gripper_position_cmd', 10)
        self.right_gripper_pick_pub = self.create_publisher(SetGripperPickCmd, '/right_arm_controller/rm_driver/set_gripper_pick_cmd', 10)
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/right_arm_controller/joint_states',
            self.joint_callback,
            10
        )
        
        self.joint_data = None
        self.joint_received = False
        
        # 等待关节数据
        self.wait_for_joint_data()
        
        # 开始测试
        self.test_arm_motion()
    
    def joint_callback(self, msg):
        if not self.joint_received:
            self.joint_received = True
            self.joint_data = msg
            self.get_logger().info('关节数据接收成功')
    
    def wait_for_joint_data(self):
        self.get_logger().info('等待关节数据...')
        start_time = time.time()
        timeout = 10.0
        
        while not self.joint_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('关节数据获取超时')
                return
    
    def movej(self, joint_values, speed=30, block=True):
        """关节空间运动"""
        cmd = MovejCmd()
        cmd.joint = joint_values
        cmd.speed = speed
        self.right_movej_pub.publish(cmd)
        self.get_logger().info(f'发送MoveJ命令: {joint_values}, 速度: {speed}')
        
        if block:
            time.sleep(5.0)  # 等待运动完成
    
    def gripper_set(self, position, block=True):
        """设置夹爪位置"""
        cmd = SetGripperPositionCmd()
        cmd.position = position
        self.right_gripper_set_pub.publish(cmd)
        self.get_logger().info(f'发送夹爪位置命令: {position}')
        
        if block:
            time.sleep(2.0)  # 等待夹爪动作完成
    
    def gripper_pick(self, speed=300, force=300, block=True):
        """力控夹取"""
        cmd = SetGripperPickCmd()
        cmd.speed = speed
        cmd.force = force
        self.right_gripper_pick_pub.publish(cmd)
        self.get_logger().info(f'发送夹爪力控命令: 速度={speed}, 力={force}')
        
        if block:
            time.sleep(2.0)  # 等待夹取完成
    
    def test_arm_motion(self):
        """测试机械臂运动"""
        self.get_logger().info('开始测试机械臂运动...')
        
        # 1. 移动到初始位置
        self.get_logger().info('1. 移动到初始位置')
        self.movej(self.poses['right']['initial'])
        
        # 2. 移动到识别位置
        self.get_logger().info('2. 移动到识别位置')
        self.movej(self.poses['right']['recognize'])
        
        # 3. 测试夹爪
        self.get_logger().info('3. 测试夹爪')
        self.gripper_set(1000)  # 打开
        self.gripper_set(0)      # 关闭
        self.gripper_set(500)     # 半开
        
        # 4. 测试力控夹取
        self.get_logger().info('4. 测试力控夹取')
        self.gripper_pick()
        
        # 5. 移动到安全位置
        self.get_logger().info('5. 移动到安全位置')
        self.movej(self.poses['right']['pick_safe'])
        
        self.get_logger().info('单臂功能测试完成！')

def main(args=None):
    rclpy.init(args=args)
    node = SingleArmTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
