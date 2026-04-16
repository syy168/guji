#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单臂极简安全测试脚本
仅测试末端关节(4,5,6轴)的微小相对运动，且必须终端确认
"""

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import Movej  # ⚠️ 这里修改成了正确的名字 Movej
from sensor_msgs.msg import JointState
import time
import sys

class SafeSingleArmTest(Node):
    def __init__(self):
        super().__init__('safe_single_arm_test')
        self.get_logger().info('��️ 终极安全测试模式已启动...')
        
        # 创建 MoveJ 发布者
        self.right_movej_pub = self.create_publisher(
            Movej,  # ⚠️ 这里也改成了 Movej
            '/right_arm_controller/rm_driver/movej_cmd', 
            10
        )
        
        # 订阅全局关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # 用于存储当前右臂的 6 个关节角度
        self.current_right_joints = {}
        
        # 等待获取初始状态
        self.wait_for_joint_data()
        
        # 开始交互式安全测试
        self.interactive_test()
    
    def joint_callback(self, msg):
        # 提取右臂的关节数据并存入字典
        for name, position in zip(msg.name, msg.position):
            if name.startswith('r_joint'):
                self.current_right_joints[name] = position

    def wait_for_joint_data(self):
        self.get_logger().info('正在读取机械臂当前实际姿态...')
        start_time = time.time()
        timeout = 5.0
        
        # 确保 r_joint1 到 r_joint6 这 6 个关节都读到了
        while len(self.current_right_joints) < 6:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('❌ 读取关节角度超时！请确认底层驱动正在运行。')
                sys.exit(1)
        
        self.get_logger().info('✅ 成功获取当前姿态！')
    
    def get_current_joint_array(self):
        """将字典按顺序转换为长度为6的数组"""
        # 确保每次获取都是最新的数据
        rclpy.spin_once(self, timeout_sec=0.1) 
        try:
            return [
                self.current_right_joints['r_joint1'],
                self.current_right_joints['r_joint2'],
                self.current_right_joints['r_joint3'],
                self.current_right_joints['r_joint4'],
                self.current_right_joints['r_joint5'],
                self.current_right_joints['r_joint6']
            ]
        except KeyError:
            return None

    def interactive_test(self):
        """交互式安全测试序列"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('⚠️ 即将进行【手腕关节微调】测试')
        self.get_logger().info('规则: 1/2/3轴(基座大臂)将被完全锁定，仅4/5/6轴进行微小运动')
        self.get_logger().info('='*50 + '\n')
        
        # 1. 获取当前绝对真实的关节角度
        current_joints = self.get_current_joint_array()
        if not current_joints:
            self.get_logger().error('关节数据异常，退出测试。')
            return
            
        # 2. 构造安全的目标角度 (复制当前角度)
        target_joints = list(current_joints)
        
        # 3. 仅修改靠后的关节 (例如: 6轴转动约 10度/0.17弧度)
        small_move = 0.17  # 弧度，约 2.8 度，非常安全
        target_joints[3] += small_move  # J4 轴
        target_joints[4] += small_move  # J5 轴
        target_joints[5] += small_move  # J6 轴
        
        # 4. 打印极其显眼的对比信息
        print('\n【运动指令核对】')
        print('轴号 |    当前角度(rad)   ->   目标角度(rad)')
        print('-'*45)
        for i in range(6):
            # 如果是前三个关节，显示为锁定状态
            if i < 3:
                status = "(锁定 ��)" 
            else:
                status = "(微调 ��)" if abs(target_joints[i] - current_joints[i]) > 0.01 else "(保持 ��)"
                
            print(f" J{i+1} | {current_joints[i]:15.4f} -> {target_joints[i]:15.4f}  {status}")
        print('-'*45)
        
        # 5. 终端拦截与用户确认
        print('\n�� 系统已拦截下发指令。')
        user_input = input("�� 请检查周围环境安全。若确认执行，请按 's' 键并回车 (输入其他任意键取消): ")
        
        if user_input.strip().lower() == 's':
            print('\n�� 收到确认指令，正在下发...')
            cmd = Movej()  # ⚠️ 这里也改成了 Movej
            cmd.joint = [float(j) for j in target_joints]
            cmd.speed = 5  # 设置一个非常缓慢的安全速度 (0-100)
            
            self.right_movej_pub.publish(cmd)
            print('✅ 指令已发送！等待运动完成...')
            time.sleep(3.0)  # 等待运动执行
            print('�� 测试完成！')
        else:
            print('\n�� 已取消运动指令，机械臂保持原位。安全退出。')

def main(args=None):
    rclpy.init(args=args)
    node = SafeSingleArmTest()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()