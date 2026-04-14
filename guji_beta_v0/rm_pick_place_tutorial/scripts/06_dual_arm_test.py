#!/usr/bin/env python3
"""
双臂协同测试脚本
测试双臂的协同运动
"""

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import MovejCmd, SetGripperPositionCmd, SetGripperPickCmd
from sensor_msgs.msg import JointState
import time
import yaml
import os

class DualArmTestNode(Node):
    def __init__(self):
        super().__init__('dual_arm_test_node')
        self.get_logger().info('双臂协同测试开始...')
        
        # 加载配置
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'poses.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.poses = config['poses']
        
        # 创建发布者
        # 右臂
        self.right_movej_pub = self.create_publisher(MovejCmd, '/right_arm_controller/rm_driver/movej_cmd', 10)
        self.right_gripper_set_pub = self.create_publisher(SetGripperPositionCmd, '/right_arm_controller/rm_driver/set_gripper_position_cmd', 10)
        
        # 左臂
        self.left_movej_pub = self.create_publisher(MovejCmd, '/left_arm_controller/rm_driver/movej_cmd', 10)
        self.left_gripper_set_pub = self.create_publisher(SetGripperPositionCmd, '/left_arm_controller/rm_driver/set_gripper_position_cmd', 10)
        
        # 订阅关节状态
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
        
        self.left_joint_received = False
        self.right_joint_received = False
        
        # 等待关节数据
        self.wait_for_joint_data()
        
        # 开始测试
        self.test_dual_arm_motion()
    
    def left_joint_callback(self, msg):
        if not self.left_joint_received:
            self.left_joint_received = True
            self.get_logger().info('左臂关节数据接收成功')
    
    def right_joint_callback(self, msg):
        if not self.right_joint_received:
            self.right_joint_received = True
            self.get_logger().info('右臂关节数据接收成功')
    
    def wait_for_joint_data(self):
        self.get_logger().info('等待关节数据...')
        start_time = time.time()
        timeout = 10.0
        
        while not (self.left_joint_received and self.right_joint_received):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('关节数据获取超时')
                return
    
    def movej(self, arm, joint_values, speed=30):
        """关节空间运动"""
        cmd = MovejCmd()
        cmd.joint = joint_values
        cmd.speed = speed
        
        if arm == 'left':
            self.left_movej_pub.publish(cmd)
            self.get_logger().info(f'发送左臂MoveJ命令: {joint_values}, 速度: {speed}')
        else:
            self.right_movej_pub.publish(cmd)
            self.get_logger().info(f'发送右臂MoveJ命令: {joint_values}, 速度: {speed}')
    
    def gripper_set(self, arm, position):
        """设置夹爪位置"""
        cmd = SetGripperPositionCmd()
        cmd.position = position
        
        if arm == 'left':
            self.left_gripper_set_pub.publish(cmd)
            self.get_logger().info(f'发送左臂夹爪位置命令: {position}')
        else:
            self.right_gripper_set_pub.publish(cmd)
            self.get_logger().info(f'发送右臂夹爪位置命令: {position}')
    
    def movej_both(self, left_joints, right_joints, speed=30, block=True):
        """双臂协同运动"""
        self.movej('left', left_joints, speed)
        self.movej('right', right_joints, speed)
        
        if block:
            time.sleep(5.0)  # 等待运动完成
    
    def test_dual_arm_motion(self):
        """测试双臂协同运动"""
        self.get_logger().info('开始测试双臂协同运动...')
        
        # 1. 双臂同时移动到初始位置
        self.get_logger().info('1. 双臂同时移动到初始位置')
        self.movej_both(self.poses['left']['initial'], self.poses['right']['initial'])
        
        # 2. 双臂同时移动到识别位置
        self.get_logger().info('2. 双臂同时移动到识别位置')
        self.movej_both(self.poses['left']['recognize'], self.poses['right']['recognize'])
        
        # 3. 测试双臂夹爪
        self.get_logger().info('3. 测试双臂夹爪')
        self.gripper_set('left', 1000)   # 左臂打开
        self.gripper_set('right', 1000)  # 右臂打开
        time.sleep(2.0)
        
        self.gripper_set('left', 0)      # 左臂关闭
        self.gripper_set('right', 0)     # 右臂关闭
        time.sleep(2.0)
        
        # 4. 双臂同时移动到安全位置
        self.get_logger().info('4. 双臂同时移动到安全位置')
        self.movej_both(self.poses['left']['pick_safe'], self.poses['right']['pick_safe'])
        
        self.get_logger().info('双臂协同测试完成！')

def main(args=None):
    rclpy.init(args=args)
    node = DualArmTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
