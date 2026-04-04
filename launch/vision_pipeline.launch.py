#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
双臂取放料视觉流水线 launch 启动文件

启动顺序：
  1. realsense2_camera      — D435 相机驱动
  2. camera_bridge          — 相机诊断节点
  3. tf_broadcaster          — 手眼标定 TF 广播
  4. aruco_detector           — ArUco 视觉识别节点
  5. dual_arm_pick_place     — 主控制器

使用方法：
  ros2 launch guji vision_pipeline.launch.py

  # 指定参数
  ros2 launch guji vision_pipeline.launch.py \
    camera_serial:=123422072697 \
    left_ns:=l_arm \
    right_ns:=r_arm

  # 仅启动视觉部分（不含双臂控制）
  ros2 launch guji vision_pipeline.launch.py \
    launch_controller:=false
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
        default_value='l_arm',
        description='左臂 ROS namespace'
    )
    right_ns = DeclareLaunchArgument(
        'right_ns',
        default_value='r_arm',
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
    # 6. Lifecycle 管理（确保节点按顺序启动）
    # ==========================================
    # 等待相机节点启动后再启动 camera_bridge
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

        # 节点启动
        realsense_node,
        camera_bridge_node,
        tf_broadcaster_node,
        aruco_detector_node,
        controller_node,
    ])

    return ld
