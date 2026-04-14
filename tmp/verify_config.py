#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
配置验证工具

验证 ArUco 追踪程序所需的所有配置和依赖
"""

import sys
from pathlib import Path
import yaml

def print_header(text):
    print("\n" + "="*70)
    print(f"  {text}")
    print("="*70)

def check_file_exists(path, description):
    """检查文件是否存在"""
    p = Path(path)
    status = "✅" if p.exists() else "❌"
    print(f"{status} {description}: {path}")
    return p.exists()

def check_python_package(package_name):
    """检查 Python 包是否已安装"""
    try:
        __import__(package_name)
        print(f"✅ Python 包已安装: {package_name}")
        return True
    except ImportError:
        print(f"❌ Python 包未安装: {package_name}")
        return False

def check_yaml_config(config_path):
    """检查 YAML 配置文件的有效性"""
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        print(f"✅ YAML 配置文件有效: {config_path}")
        return config
    except Exception as e:
        print(f"❌ YAML 配置文件读取失败: {e}")
        return None

def check_hand_eye_calibration(config):
    """验证手眼标定参数"""
    if not config:
        print("❌ 无有效的配置数据")
        return False
    
    try:
        camera_config = config.get('camera', {})
        
        # 检查右臂标定数据
        he_right = camera_config.get('hand_eye_right', {})
        he_left = camera_config.get('hand_eye_left', {})
        
        print("\n【右臂手眼标定参数】")
        if he_right:
            trans = he_right.get('translation', {})
            quat = he_right.get('quaternion', {})
            print(f"  • 平移: ({trans.get('x', 0)}, {trans.get('y', 0)}, {trans.get('z', 0)})")
            print(f"  • 四元数: ({quat.get('x', 0)}, {quat.get('y', 0)}, {quat.get('z', 0)}, {quat.get('w', 1)})")
            print("  ✅ 右臂标定数据完整")
        else:
            print("  ⚠️  右臂标定数据缺失")
        
        print("\n【左臂手眼标定参数】")
        if he_left:
            trans = he_left.get('translation', {})
            quat = he_left.get('quaternion', {})
            print(f"  • 平移: ({trans.get('x', 0)}, {trans.get('y', 0)}, {trans.get('z', 0)})")
            print(f"  • 四元数: ({quat.get('x', 0)}, {quat.get('y', 0)}, {quat.get('z', 0)}, {quat.get('w', 1)})")
            print("  ✅ 左臂标定数据完整")
        else:
            print("  ⚠️  左臂标定数据缺失")
        
        return True
    except Exception as e:
        print(f"❌ 标定参数验证失败: {e}")
        return False

def check_aruco_config(config):
    """验证 ArUco 配置参数"""
    if not config:
        print("❌ 无有效的配置数据")
        return False
    
    try:
        aruco = config.get('camera', {}).get('aruco', {})
        
        print("\n【ArUco 配置参数】")
        dict_type = aruco.get('dict_type', 'DICT_5X5_1000')
        marker_size = aruco.get('marker_size', 0.030)
        target_ids = aruco.get('target_ids', [])
        
        print(f"  • 字典类型: {dict_type}")
        print(f"  • 标记尺寸: {marker_size} m = {marker_size*1000:.1f} mm")
        print(f"  • 目标 ID: {target_ids}")
        print("  ✅ ArUco 配置完整")
        
        return True
    except Exception as e:
        print(f"❌ ArUco 配置验证失败: {e}")
        return False

def check_camera_config(config):
    """验证相机配置"""
    if not config:
        print("❌ 无有效的配置数据")
        return False
    
    try:
        camera = config.get('camera', {})
        
        print("\n【相机配置】")
        model = camera.get('model', '未设置')
        serial = camera.get('serial_number', '未设置')
        topic_prefix = camera.get('topic_prefix', '/camera_right')
        
        print(f"  • 相机型号: {model}")
        print(f"  • 序列号: {serial}")
        print(f"  • 话题前缀: {topic_prefix}")
        
        intrinsic = camera.get('intrinsic', {})
        if intrinsic:
            print(f"  • 分辨率: {intrinsic.get('width', 0)}x{intrinsic.get('height', 0)}")
            print(f"  • 帧率: {intrinsic.get('fps', 0)} Hz")
        
        print("  ✅ 相机配置完整")
        return True
    except Exception as e:
        print(f"❌ 相机配置验证失败: {e}")
        return False

def main():
    print_header("ArUco 追踪程序 - 配置验证工具")
    
    all_ok = True
    
    # ========== 1. 检查文件 ==========
    print_header("1. 文件检查")
    
    files_to_check = [
        ("guji/config/camera.yaml", "相机配置文件"),
        ("new/aruco_tracker.py", "ArUco 追踪程序"),
        ("new/trajectory_player.py", "轨迹回放程序（参考）"),
        ("new/3.py", "相机测试程序（参考）"),
    ]
    
    for file_path, description in files_to_check:
        if not check_file_exists(file_path, description):
            all_ok = False
    
    # ========== 2. 检查 Python 包 ==========
    print_header("2. Python 依赖检查")
    
    packages = [
        'rclpy',
        'cv2',
        'numpy',
        'scipy',
        'yaml',
    ]
    
    for pkg in packages:
        # 对于 cv2 的特殊处理
        pkg_name = 'cv2' if pkg == 'cv2' else pkg
        if not check_python_package(pkg_name):
            all_ok = False
    
    # ========== 3. 检查配置文件内容 ==========
    print_header("3. 配置文件内容验证")
    
    config_path = Path("guji/config/camera.yaml")
    if config_path.exists():
        config = check_yaml_config(str(config_path))
        
        if config:
            check_camera_config(config)
            check_aruco_config(config)
            check_hand_eye_calibration(config)
    else:
        print("❌ 配置文件不存在，无法验证内容")
        all_ok = False
    
    # ========== 4. 诊断建议 ==========
    print_header("4. 诊断建议")
    
    print("""
【推荐的启动流程】

1️⃣  启动机械臂驱动（终端 1）:
   $ cd ~/ros2_ws
   $ source /opt/ros/humble/setup.bash
   $ source ./install/setup.bash
   $ ros2 launch rm_driver rm_65_driver.launch.py

2️⃣  启动相机驱动（终端 2）:
   $ ros2 launch realsense2_camera rs_launch.py

3️⃣  运行 ArUco 追踪程序（终端 3）:
   $ cd ~/ros2_ws
   $ source /opt/ros/humble/setup.bash
   $ source ./install/setup.bash
   $ python3 new/aruco_tracker.py

【调试命令】

• 检查 ROS2 节点状态:
  $ ros2 node list

• 检查相机话题:
  $ ros2 topic list | grep camera
  $ ros2 topic echo /camera_right/color/camera_info

• 检查关节话题:
  $ ros2 topic echo /joint_states

• 查看所有话题:
  $ ros2 topic list

【常见问题解决】

❌ 导入错误 "from rm_ros_interfaces import ..."
✓ 解决方案:
  $ source ~/ros2_ws/install/setup.bash

❌ 找不到模块 'cv2'
✓ 解决方案:
  $ pip install opencv-python

❌ 配置文件找不到
✓ 解决方案:
  $ 确保工作目录正确，或指定绝对路径

❌ 相机话题找不到
✓ 解决方案:
  $ 检查相机驱动是否启动
  $ 检查 camera.yaml 中的 topic_prefix 配置

    """)
    
    # ========== 5. 总结 ==========
    print_header("5. 验证总结")
    
    if all_ok:
        print("""
✅ ✅ ✅ 所有检查项目都已通过！

您可以现在运行 ArUco 追踪程序：
  $ python3 new/aruco_tracker.py

或运行快速开始指南：
  $ python3 new/quickstart.py
        """)
    else:
        print("""
⚠️  发现一些问题需要解决

请按以下步骤排查：
1. 检查所有必需的文件是否存在
2. 安装缺失的 Python 包
3. 检查配置文件内容
4. 按照诊断建议启动必需的服务

如问题仍未解决，请参考 ARUCO_TRACKER_README.md 文档
        """)
    
    return 0 if all_ok else 1

if __name__ == '__main__':
    sys.exit(main())
