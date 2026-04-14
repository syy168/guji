from setuptools import setup
import os

# 所有可执行脚本
scripts = [
    'dual_arm_pick_place.py',
    'dual_arm_controller.py',
    'teach_record.py',
    'arm_monitor.py',
    'rm_joint_reader.py',
]

# nodes 目录下的脚本
nodes_scripts = [
    'nodes/camera_bridge.py',
    'nodes/tf_broadcaster.py',
    'nodes/aruco_detector.py',
    'nodes/agv_navigator.py',
    'nodes/gazebo_arm_adapter.py',
]

all_scripts = scripts + nodes_scripts

setup(
    name='guji',
    version='1.0.0',
    packages=[],
    data_files=[
        # 配置文件（安装到 share/guji/config/）
        (os.path.join('share', 'guji', 'config'), [
            'config/poses.yaml',
            'config/camera.yaml',
            'config/system.yaml',
            'config/navigation.yaml',
        ]),
        # launch 文件（安装到 share/guji/launch/）
        (os.path.join('share', 'guji', 'launch'), [
            'launch/vision_pipeline.launch.py',
        ]),
        # srv 定义（安装到 share/guji/srv/）
        (os.path.join('share', 'guji', 'srv'), [
            'srv/DetectAruco.srv',
        ]),
        # package.xml
        (os.path.join('share', 'guji'), ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='睿尔曼双臂取放料系统 — 视觉识别 + 双臂协同控制',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 主控制器
            'dual_arm_pick_place = dual_arm_pick_place:main',
            # 基础控制器
            'dual_arm_controller = dual_arm_controller:main',
            # 示教记录
            'teach_record = teach_record:main',
            # 臂状态监控
            'arm_monitor = arm_monitor:main',
            # 关节数据读取
            'rm_joint_reader = rm_joint_reader:main',
            # 相机诊断
            'camera_bridge = nodes.camera_bridge:main',
            # 手眼标定 TF 广播
            'tf_broadcaster = nodes.tf_broadcaster:main',
            # ArUco 视觉识别
            'aruco_detector = nodes.aruco_detector:main',
            # AGV 导航
            'agv_navigator = nodes.agv_navigator:main',
            # Gazebo 适配
            'gazebo_arm_adapter = nodes.gazebo_arm_adapter:main',
        ],
    },
)
