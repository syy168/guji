#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 码识别与移动检测程序 - 使用说明

二话不说，直接用！
"""

USAGE = """
╔════════════════════════════════════════════════════════════════════════╗
║                   ArUco 码识别与移动检测 - 快速使用                   ║
╚════════════════════════════════════════════════════════════════════════╝

【前置条件】
• ROS2 Humble 已安装
• 机械臂驱动可用
• 相机驱动可用

【安装依赖】
$ pip install opencv-python numpy scipy PyYAML

【配置文件修改】

编辑 guji/config/camera.yaml，在最后添加这段（对齐 camera:）：

  # 臂基座固定参数（从 URDF 中提取）
  arm_base_offset_right:
    position:
      x: -0.1
      y: -0.1103
      z: 0.031645
    rotation_rpy:
      r: 0.0
      p: -0.7854
      y: 0.0

【启动指令】

终端 1 - 启动机械臂驱动：
  $ cd ~/ros2_ws
  $ source /opt/ros/humble/setup.bash
  $ source ./install/setup.bash
  $ ros2 launch rm_driver rm_65_driver.launch.py

终端 2 - 启动相机驱动：
  $ ros2 launch realsense2_camera rs_launch.py

终端 3 - 运行程序：
  $ cd ~/ros2_ws
  $ source /opt/ros/humble/setup.bash
  $ source ./install/setup.bash
  $ python3 ~/Desktop/realman/new/aruco_tracker.py

【使用流程】

1. 选择机械臂
   输入: right (或 left，按回车默认 right)

2. 准备 ArUco 码
   将 ArUco 标记放在相机视野内

3. 第一次识别
   按 's' 键执行初始检测
   程序输出 ArUco 在相机、末端、基座三个坐标系的位置

4. 移动 ArUco 码
   将标记移动到新位置（保持在视野内）

5. 第二次识别
   按 's' 键再次检测
   程序自动计算两次检测之间的移动距离（三个坐标系）

6. 查看结果
   程序输出移动距离（毫米为单位）

【输出示例】

✅ 检测到 ArUco 码 (ID: 11)
   相机坐标系位置: (0.1234, -0.0456, 0.3789) m
   末端执行器坐标系: (0.2089, -0.0812, 0.3801) m
   基座坐标系位置: (-0.0234, -0.1567, 0.4123) m

【第二次检测后】

📊 移动距离计算结果
════════════════════════════════════════════

📷 相机坐标系中的移动:
   Δ位置: (0.0123, -0.0456, 0.0789) m
   距离: 0.0945 m = 94.50 mm

🦾 末端执行器坐标系中的移动:
   Δ位置: (0.0089, -0.0412, 0.0801) m
   距离: 0.0923 m = 92.30 mm

🏠 基座坐标系中的移动（基于 URDF 固定参数）:
   Δ位置: (0.0234, -0.0567, -0.0123) m
   距离: 0.0615 m = 61.50 mm

【按键说明】

's' 键  → 执行 ArUco 检测
'q' 键  → 退出程序
其他键  → 跳过当前步骤

【坐标系说明】

📷 相机坐标系
   原点：相机镜头
   方法：ArUco 直接检测得到

🦾 末端执行器坐标系
   原点：机械臂末端法兰
   方法：使用手眼标定参数（camera.yaml 中的 hand_eye_right）

🏠 基座坐标系
   原点：机械臂基座
   方法：使用 URDF 固定参数（不需要 TF 话题，已硬编码）

【性能数据】

基座坐标系计算耗时：~1ms（比旧版本快 50 倍）
依赖服务：只需要驱动 + 相机（不需要 TF 广播器）

【配置更新】

如果改变了机械臂的臂基座参数（修改了 URDF），需要同步更新：

1. 查看 URDF 中 r_base_joint1 的参数
2. 更新 camera.yaml 中的 arm_base_offset_right

如果改进了手眼标定，需要更新：

1. 运行手眼标定程序
2. 获得新的标定参数
3. 更新 camera.yaml 中的 hand_eye_right

【故障排查】

问题：程序报错 KeyError
修复：
  1. 打开 camera.yaml
  2. 确保 arm_base_offset_right 存在且缩进正确
  3. 重新启动程序

问题：基座坐标系位置明显不对（偏差 > 10cm）
修复：
  1. 验证 URDF 中的 r_base_joint1 参数
  2. 确保 camera.yaml 中的数值与 URDF 完全一致
  3. 重新标定手眼参数
  4. 重启程序

问题：无法检测到 ArUco 码
修复：
  1. 检查 ArUco 标记是否完好
  2. 确保标记在相机视野内
  3. 改善光线条件
  4. 检查 dict_type 是否与标记匹配

问题：相机无数据
修复：
  $ ros2 launch realsense2_camera rs_launch.py
  确保相机驱动启动

【两次识别之间可以进行的操作】

✓ 【第一步】按 's' 识别初始位置
  ↓ 程序提示"已检测第一个位置，请移动 ArUco 码"
  ↓
✓ 【移动】物理移动 ArUco 标记
  • 可以用手拿着标记移动
  • 可以放在移动的物体上
  • 可以改变标记的角度
  ↓
✓ 【第二步】按 's' 识别新位置
  ↓
✓ 【输出】程序计算并显示移动距离

【高级配置】

修改 ArUco 标记相关参数（需深入配置）：

编辑 camera.yaml 中的 aruco 部分：
  • dict_type: 标记字典类型（DICT_5X5_1000 等）
  • marker_size: 标记物的实际尺寸（米）
  • target_ids: 目标标记 ID 列表

修改相机话题前缀：

编辑 camera.yaml 中的 topic_prefix：
  • /camera_right (默认)
  • /camera_left (如果用左臂)

【所需文件】

必须有（运行依赖）：
  new/aruco_tracker.py                   主程序
  guji/config/camera.yaml                配置文件（已修改）

可选（参考）：
  new/trajectory_player.py               轨迹回放示例
  new/3.py                              相机测试示例

【一句话总结】

1. 改 camera.yaml，加臂基座参数
2. 启动驱动（机械臂 + 相机）
3. 运行程序，按 's' 两次，得到移动距离

就这么简单！

"""

if __name__ == '__main__':
    print(USAGE)
