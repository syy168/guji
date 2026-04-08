#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
双臂取放料视觉流水线 launch 启动文件

启动顺序：
  1. rm_driver (双臂)          — 左臂 + 右臂 驱动（left_arm_controller / right_arm_controller）
  2. realsense2_camera        — D435 相机驱动
  3. camera_bridge            — 相机诊断节点
  4. tf_broadcaster           — 手眼标定 TF 广播
  5. aruco_detector           — ArUco 视觉识别节点
  6. dual_arm_pick_place      — 主控制器

使用方法：
  # 一键启动（推荐）
  ros2 launch guji vision_pipeline.launch.py

  # 指定参数
  ros2 launch guji vision_pipeline.launch.py \
    camera_serial:=123422072697 \
    left_ns:=left_arm_controller \
    right_ns:=right_arm_controller \
    computer_ip:=192.168.151.119 \
    left_arm_ip:=192.168.150.111 \
    right_arm_ip:=192.168.150.112

  # 仅启动视觉部分（不含双臂控制）
  ros2 launch guji vision_pipeline.launch.py \
    launch_controller:=false \
    launch_drivers:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # ==========================================
    # 声明 Launch 参数
    # ==========================================
    camera_serial = DeclareLaunchArgument(
        'camera_serial',
        default_value='REPLACE_WITH_YOUR_SERIAL_NUMBER',
        description='RealSense D435 序列号'
    )
    left_ns = DeclareLaunchArgument(
        'left_ns',
        default_value='left_arm_controller',
        description='左臂 ROS namespace'
    )
    right_ns = DeclareLaunchArgument(
        'right_ns',
        default_value='right_arm_controller',
        description='右臂 ROS namespace'
    )
    camera_prefix = DeclareLaunchArgument(
        'camera_prefix',
        default_value='/camera_right',
        description='相机话题前缀'
    )
    launch_controller = DeclareLaunchArgument(
        'launch_controller',
        default_value='true',
        description='是否同时启动主控制器节点'
    )
    launch_drivers = DeclareLaunchArgument(
        'launch_drivers',
        default_value='true',
        description='是否同时启动双臂驱动节点'
    )
    computer_ip = DeclareLaunchArgument(
        'computer_ip',
        default_value='192.168.151.119',
        description='计算机 IP（双臂 UDP 上报目标）'
    )
    left_arm_ip = DeclareLaunchArgument(
        'left_arm_ip',
        default_value='192.168.150.111',
        description='左臂 TCP 连接 IP'
    )
    right_arm_ip = DeclareLaunchArgument(
        'right_arm_ip',
        default_value='192.168.150.112',
        description='右臂 TCP 连接 IP'
    )

    # ==========================================
    # 0. 双臂驱动（rm_driver x2）
    # ==========================================
    left_arm_params = {
        'arm_ip': LaunchConfiguration('left_arm_ip'),
        'tcp_port': 8080,
        'arm_type': 'RM_65',
        'arm_dof': 6,
        'udp_ip': LaunchConfiguration('computer_ip'),
        'udp_cycle': 5,
        'udp_port': 8089,
        'udp_force_coordinate': 0,
        'udp_hand': False,
        'udp_plus_base': False,
        'udp_plus_state': False,
        'udp_joint_speed_state': True,
        'udp_lift_state': False,
        'udp_expand_state': False,
        'udp_arm_current_status': True,
        'udp_aloha_state': False,
        'trajectory_mode': 0,
        'radio': 0,
        'arm_joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    }
    right_arm_params = {
        'arm_ip': LaunchConfiguration('right_arm_ip'),
        'tcp_port': 8080,
        'arm_type': 'RM_65',
        'arm_dof': 6,
        'udp_ip': LaunchConfiguration('computer_ip'),
        'udp_cycle': 5,
        'udp_port': 8089,
        'udp_force_coordinate': 0,
        'udp_hand': False,
        'udp_plus_base': False,
        'udp_plus_state': False,
        'udp_joint_speed_state': True,
        'udp_lift_state': False,
        'udp_expand_state': False,
        'udp_arm_current_status': True,
        'udp_aloha_state': False,
        'trajectory_mode': 0,
        'radio': 0,
        'arm_joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    }

    left_driver_node = Node(
        package='rm_driver',
        executable='rm_driver',
        namespace=LaunchConfiguration('left_ns'),
        parameters=[left_arm_params],
        output='screen',
        emulate_tty=True,
        condition=launch.actions.IfCondition(LaunchConfiguration('launch_drivers')),
    )
    right_driver_node = Node(
        package='rm_driver',
        executable='rm_driver',
        namespace=LaunchConfiguration('right_ns'),
        parameters=[right_arm_params],
        output='screen',
        emulate_tty=True,
        condition=launch.actions.IfCondition(LaunchConfiguration('launch_drivers')),
    )

    # ==========================================
    # 1. realsense2_camera 相机驱动
    # ==========================================
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        namespace=LaunchConfiguration('camera_prefix'),
        parameters=[{
            'serial_no': LaunchConfiguration('camera_serial'),
            'enable_color': True,
            'enable_depth': True,
            'enable_aligned_depth': True,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'align_depth': True,
            'pointcloud.enable': False,
            'enable_rgbd_camera_align': True,
            'publish_tf': False,   # TF 由 tf_broadcaster 节点统一管理
        }],
        output='screen',
        emulate_tty=True,
    )

    # ==========================================
    # 2. camera_bridge 相机诊断节点
    # ==========================================
    camera_bridge_node = Node(
        package='guji',
        executable='camera_bridge.py',
        name='camera_bridge',
        parameters=[{
            'camera_prefix': LaunchConfiguration('camera_prefix'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # ==========================================
    # 3. tf_broadcaster 手眼标定 TF 广播节点
    # ==========================================
    tf_broadcaster_node = Node(
        package='guji',
        executable='tf_broadcaster.py',
        name='hand_eye_tf_broadcaster',
        parameters=[{
            'right_ns': LaunchConfiguration('right_ns'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # ==========================================
    # 4. aruco_detector ArUco 视觉识别节点
    # ==========================================
    aruco_detector_node = Node(
        package='guji',
        executable='aruco_detector.py',
        name='aruco_detector',
        parameters=[{
            'camera_prefix': LaunchConfiguration('camera_prefix'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # ==========================================
    # 5. dual_arm_pick_place 主控制器
    # ==========================================
    controller_node = Node(
        package='guji',
        executable='dual_arm_pick_place.py',
        name='dual_arm_pick_place_controller',
        parameters=[{
            'left_ns': LaunchConfiguration('left_ns'),
            'right_ns': LaunchConfiguration('right_ns'),
        }],
        output='screen',
        emulate_tty=True,
        condition=launch.actions.IfCondition(LaunchConfiguration('launch_controller')),
    )

    # ==========================================
    # Lifecycle 管理（确保节点按顺序启动）
    # ==========================================
    camera_bridge_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=realsense_node,
            on_start=[
                LogInfo(msg='[Launch] realsense2_camera 已启动，等待相机话题就绪...'),
            ]
        )
    )

    # ==========================================
    # 构建 LaunchDescription
    # ==========================================
    ld = LaunchDescription([
        # 启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='  双臂取放料视觉流水线启动'),
        LogInfo(msg='========================================'),
        LogInfo(msg='  0. rm_driver (left_arm_controller) — 左臂驱动'),
        LogInfo(msg='  0. rm_driver (right_arm_controller) — 右臂驱动'),
        LogInfo(msg='  1. realsense2_camera — D435 相机驱动'),
        LogInfo(msg='  2. camera_bridge    — 相机诊断'),
        LogInfo(msg='  3. tf_broadcaster   — 手眼标定 TF'),
        LogInfo(msg='  4. aruco_detector   — ArUco 识别'),
        LogInfo(msg='  5. dual_arm_pick_place — 主控制器'),
        LogInfo(msg='========================================'),

        # 参数声明
        camera_serial,
        left_ns,
        right_ns,
        camera_prefix,
        launch_controller,
        launch_drivers,
        computer_ip,
        left_arm_ip,
        right_arm_ip,

        # 节点启动
        left_driver_node,
        right_driver_node,
        realsense_node,
        camera_bridge_node,
        tf_broadcaster_node,
        aruco_detector_node,
        controller_node,
    ])

    return ld
