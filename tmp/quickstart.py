#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
快速开始指南：ArUco 码识别与移动检测

此脚本提供一个完整的使用示例流程
"""

import subprocess
import time
import sys
from pathlib import Path

def print_section(title):
    """打印标题"""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70 + "\n")

def main():
    print_section("ArUco 码识别与移动检测 - 快速开始指南")
    
    print("""
本指南将帮助您快速开始使用 ArUco 追踪程序。

【系统要求】
✓ ROS2 Humble
✓ Python 3.10+
✓ OpenCV 4.7+
✓ numpy, scipy, PyYAML

【前置步骤】

1️⃣  确保机械臂驱动已启动
   $ cd ~/ros2_ws
   $ source /opt/ros/humble/setup.bash
   $ source ./install/setup.bash
   $ ros2 launch rm_driver rm_65_driver.launch.py

2️⃣  确保相机驱动已启动（另一个终端）
   $ ros2 launch realsense2_camera rs_launch.py

3️⃣  安装必要的 Python 依赖
   $ pip install opencv-python numpy scipy PyYAML

【运行程序】

在第三个终端中运行：

   $ cd ~/ros2_ws
   $ source /opt/ros/humble/setup.bash
   $ source ./install/setup.bash
   $ python3 new/aruco_tracker.py

【使用流程】

┌──────────────────────────────────────────────────────┐
│ 【步骤 1】选择机械臂                                 │
│                                                      │
│ 输入: left 或 right（默认 right）                    │
└──────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────┐
│ 【步骤 2】初始检测                                   │
│                                                      │
│ • 将 ArUco 码放在相机视野内                          │
│ • 程序会显示实时相机画面                            │
│ • 按 's' 键执行首次检测                              │
│ • 程序会输出 ArUco 码在三个坐标系中的位置           │
└──────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────┐
│ 【步骤 3】移动 ArUco 码                               │
│                                                      │
│ • 物理移动 ArUco 码到新位置                          │
│ • 确保新位置仍在相机视野内                          │
│ • 按 's' 键执行第二次检测                            │
└──────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────┐
│ 【步骤 4】查看结果                                   │
│                                                      │
│ 程序会自动计算并输出：                              │
│ ✓ 相机坐标系中的移动距离                            │
│ ✓ 末端执行器坐标系中的移动距离                      │
│ ✓ 基座坐标系中的移动距离（如果可用）                │
└──────────────────────────────────────────────────────┘

【配置文件位置】

手眼标定外参: guji/config/camera.yaml

示例内容:
```yaml
hand_eye_right:
  translation:
    x: 0.0850      # 相机 X 方向偏移（米）
    y: -0.0400     # 相机 Y 方向偏移（米）
    z: 0.0100      # 相机 Z 方向偏移（米）
  quaternion:
    x: 0.0         # 旋转四元数
    y: 0.0
    z: 0.0
    w: 1.0
```

【常见问题排查】

❌ 问题：未检测到 ArUco 码
✓ 解决方案：
  • 检查 ArUco 码是否完好无损
  • 检查 dict_type 是否与标记一致
  • 改善光线条件
  • 确保标记在相机视野内

❌ 问题：相机内参未获取
✓ 解决方案：
  • 检查相机驱动是否启动
  • 验证相机话题：ros2 topic list | grep camera

❌ 问题：关节状态未更新
✓ 解决方案：
  • 检查机械臂驱动是否启动
  • 验证关节话题：ros2 topic echo /joint_states

【性能优化建议】

1. 确保充分的光线
2. 使用高质量的 ArUco 标记
3. 保持相机清洁
4. 定期更新手眼标定外参
5. 使用合适的标记尺寸（通常 30mm 效果最好）

【高级选项】

• 修改 ArUco 字典类型：编辑 guji/config/camera.yaml
• 修改标记尺寸：编辑 marker_size 参数
• 改用左臂相机：运行时选择 'left'

【输出示例】

┌──────────────────────────────────────────────────────┐
│ 📊 移动距离计算结果                                  │
├──────────────────────────────────────────────────────┤
│                                                      │
│ 📷 相机坐标系中的移动:                               │
│    Δ位置: (0.0123, -0.0456, 0.0789) m               │
│    距离: 0.0945 m = 94.50 mm                        │
│                                                      │
│ 🦾 末端执行器坐标系中的移动:                         │
│    Δ位置: (0.0089, -0.0412, 0.0801) m               │
│    距离: 0.0923 m = 92.30 mm                        │
│                                                      │
│ 🏠 基座坐标系中的移动:                               │
│    Δ位置: (0.0234, -0.0567, -0.0123) m              │
│    距离: 0.0615 m = 61.50 mm                        │
│                                                      │
└──────────────────────────────────────────────────────┘

【更多信息】

详细文档：ARUCO_TRACKER_README.md
参考代码：trajectory_player.py（机械臂控制示例）
参考代码：3.py（相机获取示例）

【获得帮助】

1. 查看程序输出的日志信息
2. 检查 ROS2 话题和节点状态
3. 参考 ARUCO_TRACKER_README.md 文档
4. 运行诊断程序验证各模块

    """)
    
    print_section("配置检查清单")
    
    checklist = [
        ("guji/config/camera.yaml 存在", Path("guji/config/camera.yaml").exists()),
        ("new/aruco_tracker.py 存在", Path("new/aruco_tracker.py").exists()),
        ("相机话题配置", "检查 camera.yaml 中的 topic_prefix"),
        ("手眼标定参数", "检查 camera.yaml 中的 hand_eye_right"),
        ("ArUco 标记尺寸", "应与实际打印尺寸一致"),
    ]
    
    for item, status in checklist:
        if isinstance(status, bool):
            symbol = "✅" if status else "❌"
            print(f"{symbol} {item}")
        else:
            print(f"ℹ️  {item}: {status}")
    
    print_section("准备启动")
    
    print("""
✅ 所有准备工作已完成！

下一步：
1. 启动机械臂驱动
2. 启动相机驱动
3. 运行: python3 new/aruco_tracker.py

祝您使用愉快！
    """)

if __name__ == '__main__':
    main()
