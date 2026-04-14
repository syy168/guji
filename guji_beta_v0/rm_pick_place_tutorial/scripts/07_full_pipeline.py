#!/usr/bin/env python3
"""
完整取放料流程测试脚本
测试完整的取放料流程
"""

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import MovejCmd, SetGripperPositionCmd, SetGripperPickCmd
from sensor_msgs.msg import JointState
import time
import yaml
import os

class FullPipelineTestNode(Node):
    def __init__(self):
        super().__init__('full_pipeline_test_node')
        self.get_logger().info('完整取放料流程测试开始...')
        
        # 加载配置
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'poses.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.poses = config['poses']
        
        # 创建发布者
        # 右臂
        self.right_movej_pub = self.create_publisher(MovejCmd, '/right_arm_controller/rm_driver/movej_cmd', 10)
        self.right_gripper_set_pub = self.create_publisher(SetGripperPositionCmd, '/right_arm_controller/rm_driver/set_gripper_position_cmd', 10)
        self.right_gripper_pick_pub = self.create_publisher(SetGripperPickCmd, '/right_arm_controller/rm_driver/set_gripper_pick_cmd', 10)
        
        # 左臂
        self.left_movej_pub = self.create_publisher(MovejCmd, '/left_arm_controller/rm_driver/movej_cmd', 10)
        self.left_gripper_set_pub = self.create_publisher(SetGripperPositionCmd, '/left_arm_controller/rm_driver/set_gripper_position_cmd', 10)
        self.left_gripper_pick_pub = self.create_publisher(SetGripperPickCmd, '/left_arm_controller/rm_driver/set_gripper_pick_cmd', 10)
        
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
        
        # 开始测试完整流程
        self.run_full_pipeline()
    
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
    
    def movej(self, arm, joint_values, speed=30, block=True):
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
        
        if block:
            time.sleep(5.0)  # 等待运动完成
    
    def movej_both(self, left_joints, right_joints, speed=30, block=True):
        """双臂协同运动"""
        self.movej('left', left_joints, speed, block=False)
        self.movej('right', right_joints, speed, block=False)
        
        if block:
            time.sleep(5.0)  # 等待运动完成
    
    def gripper_set(self, arm, position, block=True):
        """设置夹爪位置"""
        cmd = SetGripperPositionCmd()
        cmd.position = position
        
        if arm == 'left':
            self.left_gripper_set_pub.publish(cmd)
            self.get_logger().info(f'发送左臂夹爪位置命令: {position}')
        else:
            self.right_gripper_set_pub.publish(cmd)
            self.get_logger().info(f'发送右臂夹爪位置命令: {position}')
        
        if block:
            time.sleep(2.0)  # 等待夹爪动作完成
    
    def gripper_pick(self, arm, speed=300, force=300, block=True):
        """力控夹取"""
        cmd = SetGripperPickCmd()
        cmd.speed = speed
        cmd.force = force
        
        if arm == 'left':
            self.left_gripper_pick_pub.publish(cmd)
            self.get_logger().info(f'发送左臂夹爪力控命令: 速度={speed}, 力={force}')
        else:
            self.right_gripper_pick_pub.publish(cmd)
            self.get_logger().info(f'发送右臂夹爪力控命令: 速度={speed}, 力={force}')
        
        if block:
            time.sleep(2.0)  # 等待夹取完成
    
    def go_initial_position(self):
        """双臂回到初始位置"""
        self.get_logger().info('=== 状态1: 回到初始位置 ===')
        self.movej_both(self.poses['left']['initial'], self.poses['right']['initial'])
    
    def go_recognize_position(self):
        """双臂移动到识别位置"""
        self.get_logger().info('=== 状态2: 移动到识别位置 ===')
        self.movej_both(self.poses['left']['recognize'], self.poses['right']['recognize'])
    
    def recognize_arcode(self):
        """识别ArUco标记"""
        self.get_logger().info('=== 状态3: 识别ArUco标记 ===')
        # 这里简化处理，实际应该调用detect_aruco服务
        self.get_logger().info('模拟识别ARcode12...')
        time.sleep(2.0)
        self.get_logger().info('识别完成，假设找到标记')
    
    def pre_grasp_alignment(self):
        """预抓取对齐"""
        self.get_logger().info('=== 状态4: 预抓取对齐 ===')
        # 右臂移动到工件前方
        self.movej('right', self.poses['right']['pick_prep'])
        # 左臂移动到工件侧面
        self.movej('left', self.poses['left']['pick_prep'])
        # 右臂前推对齐
        self.movej('right', self.poses['right']['pick_insert'])
        # 左臂右推对齐
        self.movej('left', self.poses['left']['pick_left_support'])
    
    def pick_workpiece(self):
        """取料流程"""
        self.get_logger().info('=== 状态5: 取料流程 ===')
        
        # Step 1: 左臂移动到托住位置
        self.get_logger().info('取料 Step 1: 左臂移动到托住位置')
        self.movej('left', self.poses['left']['pick_left_support'])
        
        # Step 2: 左臂力控夹取（托住工件）
        self.get_logger().info('取料 Step 2: 左臂力控夹取（托住工件）')
        self.gripper_pick('left')
        
        # Step 3: 左臂弧线抬起托住位
        self.get_logger().info('取料 Step 3: 左臂弧线抬起托住位')
        self.movej('left', self.poses['left']['pick_left_lift'])
        
        # Step 4: 右臂伸入工件下方
        self.get_logger().info('取料 Step 4: 右臂伸入工件下方')
        self.movej('right', self.poses['right']['pick_insert'])
        
        # Step 5: 右臂力控夹取（夹住工件）
        self.get_logger().info('取料 Step 5: 右臂力控夹取（夹住工件）')
        self.gripper_pick('right')
        
        # Step 6: 右臂回到垂直姿态
        self.get_logger().info('取料 Step 6: 右臂回到垂直姿态')
        self.movej('right', self.poses['right']['pick_grasp_vertical'])
        
        # Step 7: 右臂夹爪松开（让工件回落到左爪）
        self.get_logger().info('取料 Step 7: 右臂夹爪松开（让工件回落到左爪）')
        self.gripper_set('right', 1000)
        
        # Step 8: 左臂前顶工件到位
        self.get_logger().info('取料 Step 8: 左臂前顶工件到位')
        self.movej('left', self.poses['left']['pick_left_push'])
        
        # Step 9: 左臂夹爪松开
        self.get_logger().info('取料 Step 9: 左臂夹爪松开')
        self.gripper_set('left', 1000)
        
        # Step 10: 右臂带着工件上移
        self.get_logger().info('取料 Step 10: 右臂带着工件上移')
        self.movej('right', self.poses['right']['pick_lift'])
        
        # Step 11: 左臂回缩到安全位置
        self.get_logger().info('取料 Step 11: 左臂回缩到安全位置')
        self.movej('left', self.poses['left']['pick_left_safe'])
    
    def go_place_position(self):
        """移动到放料位置"""
        self.get_logger().info('=== 状态6: 移动到放料位置 ===')
        self.movej_both(self.poses['left']['place_above'], self.poses['right']['place_above'])
    
    def recognize_arcode21(self):
        """识别放料位置标记"""
        self.get_logger().info('=== 状态7: 识别放料位置标记 ===')
        # 这里简化处理，实际应该调用detect_aruco服务
        self.get_logger().info('模拟识别ARcode21...')
        time.sleep(2.0)
        self.get_logger().info('识别完成，假设找到标记')
    
    def place_workpiece(self):
        """放料流程"""
        self.get_logger().info('=== 状态8: 放料流程 ===')
        
        # Step 1: 右臂移动到放料上方位
        self.get_logger().info('放料 Step 1: 右臂移动到放料上方位')
        self.movej('right', self.poses['right']['place_above'])
        
        # Step 2: 右臂移动到放料位
        self.get_logger().info('放料 Step 2: 右臂移动到放料位')
        self.movej('right', self.poses['right']['place_drop'])
        
        # Step 3: 夹爪张开释放物料
        self.get_logger().info('放料 Step 3: 夹爪张开释放物料')
        self.gripper_set('right', 1000)
        
        # Step 4: 右臂移动到退出位
        self.get_logger().info('放料 Step 4: 右臂移动到退出位')
        self.movej('right', self.poses['right']['place_exit'])
        
        # Step 5: 夹爪闭合
        self.get_logger().info('放料 Step 5: 夹爪闭合')
        self.gripper_set('right', 0)
        
        # Step 6: 右臂回安全位
        self.get_logger().info('放料 Step 6: 右臂回安全位')
        self.movej('right', self.poses['right']['place_safe'])
    
    def return_to_pick_origin(self):
        """回到取料处原点"""
        self.get_logger().info('=== 状态9: 回到取料处原点 ===')
        self.movej_both(self.poses['left']['initial'], self.poses['right']['initial'])
    
    def run_full_pipeline(self):
        """运行完整取放料流程"""
        try:
            self.go_initial_position()
            self.go_recognize_position()
            self.recognize_arcode()
            self.pre_grasp_alignment()
            self.pick_workpiece()
            self.go_place_position()
            self.recognize_arcode21()
            self.place_workpiece()
            self.return_to_pick_origin()
            
            self.get_logger().info('完整取放料流程测试完成！')
        except Exception as e:
            self.get_logger().error(f'流程执行错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FullPipelineTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
