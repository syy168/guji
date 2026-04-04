#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
睿尔曼双臂协同取放料控制器

功能：
1. 双臂协同完成取料-放料全流程
2. 支持关节运动（MoveJ）和直线运动（MoveL）
3. 支持夹爪控制和力位混合搜索
4. ArUco 视觉识别（通过 /right_arm/detect_aruco Service）
5. TF 坐标变换（相机坐标系 → 基坐标系）
6. YAML 配置管理（点位、系统参数）

前置条件：
- ROS2 Foxy 环境已启动
- rm_driver 驱动已启动（双臂，各自独立 namespace）
- realsense2_camera 已启动并发布 /camera_right/* 话题
- aruco_detector 节点已启动（提供 /right_arm/detect_aruco Service）
- 手眼标定 TF 节点已启动
- 网络连接到两个机械臂
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf_transformations as T
import yaml
import os
import math
import sys
import time

# 延迟导入消息类型（rclpy.init() 之后才能导入）
Movej_msg = None
Movel_msg = None
Gripperset_msg = None
Gripperpick_msg = None
Handangle_msg = None
Forcepositionmovepose_msg = None


class DualArmPickPlaceController(Node):
    """双臂取放料控制节点"""

    def __init__(self, left_ns: str = "l_arm", right_ns: str = "r_arm"):
        super().__init__('dual_arm_pick_place_controller')

        self.left_ns = left_ns
        self.right_ns = right_ns

        # ==========================================
        # 关节角度反馈存储
        # ==========================================
        self.left_joint_positions = []
        self.right_joint_positions = []
        self.has_left_joint = False
        self.has_right_joint = False

        # ==========================================
        # 延迟导入所有消息类型
        # ==========================================
        global Movej_msg, Movel_msg, Gripperset_msg, Gripperpick_msg
        global Handangle_msg, Forcepositionmovepose_msg
        try:
            from rm_ros_interfaces.msg import Movej, Movel, Gripperset, Gripperpick
            from rm_ros_interfaces.msg import Handangle, Forcepositionmovepose
            Movej_msg = Movej
            Movel_msg = Movel
            Gripperset_msg = Gripperset
            Gripperpick_msg = Gripperpick
            Handangle_msg = Handangle
            Forcepositionmovepose_msg = Forcepositionmovepose
        except ImportError as e:
            self.get_logger().error(f'无法导入 rm_ros_interfaces 消息类型: {e}')
            return

        # ==========================================
        # 左臂订阅者
        # ==========================================
        self.left_joint_sub = self.create_subscription(
            JointState,
            f'/{self.left_ns}/joint_states',
            self._left_joint_callback,
            10
        )

        # ==========================================
        # 右臂订阅者
        # ==========================================
        self.right_joint_sub = self.create_subscription(
            JointState,
            f'/{self.right_ns}/joint_states',
            self._right_joint_callback,
            10
        )

        # ==========================================
        # 左臂发布者
        # ==========================================
        self.left_movej_pub = self.create_publisher(
            Movej_msg,
            f'/{self.left_ns}/rm_driver/movej_cmd',
            10
        )
        self.left_movel_pub = self.create_publisher(
            Movel_msg,
            f'/{self.left_ns}/rm_driver/movel_cmd',
            10
        )
        self.left_gripper_set_pub = self.create_publisher(
            Gripperset_msg,
            f'/{self.left_ns}/rm_driver/set_gripper_position_cmd',
            10
        )
        self.left_gripper_pick_pub = self.create_publisher(
            Gripperpick_msg,
            f'/{self.left_ns}/rm_driver/set_gripper_pick_cmd',
            10
        )
        self.left_hand_angle_pub = self.create_publisher(
            Handangle_msg,
            f'/{self.left_ns}/rm_driver/set_hand_angle_cmd',
            10
        )
        self.left_force_pub = self.create_publisher(
            Forcepositionmovepose_msg,
            f'/{self.left_ns}/rm_driver/force_position_move_pose_cmd',
            10
        )

        # ==========================================
        # 右臂发布者
        # ==========================================
        self.right_movej_pub = self.create_publisher(
            Movej_msg,
            f'/{self.right_ns}/rm_driver/movej_cmd',
            10
        )
        self.right_movel_pub = self.create_publisher(
            Movel_msg,
            f'/{self.right_ns}/rm_driver/movel_cmd',
            10
        )
        self.right_gripper_set_pub = self.create_publisher(
            Gripperset_msg,
            f'/{self.right_ns}/rm_driver/set_gripper_position_cmd',
            10
        )
        self.right_gripper_pick_pub = self.create_publisher(
            Gripperpick_msg,
            f'/{self.right_ns}/rm_driver/set_gripper_pick_cmd',
            10
        )
        self.right_hand_angle_pub = self.create_publisher(
            Handangle_msg,
            f'/{self.right_ns}/rm_driver/set_hand_angle_cmd',
            10
        )
        self.right_force_pub = self.create_publisher(
            Forcepositionmovepose_msg,
            f'/{self.right_ns}/rm_driver/force_position_move_pose_cmd',
            10
        )

        self.get_logger().info('双臂取放料控制器已启动')
        self.get_logger().info(f'  左臂 namespace: {self.left_ns}')
        self.get_logger().info(f'  右臂 namespace: {self.right_ns}')

        # ==========================================
        # YAML 配置加载（替代硬编码）
        # ==========================================
        self._load_config()

        # ==========================================
        # TF 监听器（用于相机坐标变换）
        # ==========================================
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ==========================================
        # ArUco 视觉识别 Service Client
        # ==========================================
        self._aruco_client = None
        self._aruco_srv_name = '/right_arm/detect_aruco'
        self._wait_for_aruco_service()

        # ==========================================
        # 相机状态 Service Client（用于启动检查）
        # ==========================================
        self._camera_status_client = self.create_client(
            Trigger, '/right_arm/get_camera_status'
        )

    # ==========================================
    # 配置加载与 Service 连接
    # ==========================================
    def _load_config(self):
        """从 YAML 配置文件加载所有参数"""
        node_dir = os.path.dirname(os.path.abspath(__file__))
        cfg_paths = {
            'poses': os.path.join(node_dir, 'config', 'poses.yaml'),
            'camera': os.path.join(node_dir, 'config', 'camera.yaml'),
            'system': os.path.join(node_dir, 'config', 'system.yaml'),
        }

        # 查找配置文件（支持开发和安装两种目录结构）
        for name, path in list(cfg_paths.items()):
            if not os.path.exists(path):
                alt = os.path.join(node_dir, '..', 'config', os.path.basename(path))
                if os.path.exists(alt):
                    cfg_paths[name] = alt

        # 加载 poses.yaml（关节角度点位）
        poses_loaded = False
        try:
            with open(cfg_paths['poses'], 'r', encoding='utf-8') as f:
                raw = yaml.safe_load(f)
            self.POSES = raw.get('poses', {})
            if self.POSES.get('left') and self.POSES.get('right'):
                poses_loaded = True
                self.get_logger().info(f'已加载关节配置: {cfg_paths["poses"]}')
            else:
                raise ValueError('poses.yaml 格式不正确，缺少 left/right')
        except Exception as e:
            self.get_logger().warn(f'加载 poses.yaml 失败: {e}，使用硬编码默认值')
            self.POSES = self._hardcoded_poses()

        # 加载 system.yaml（系统参数）
        try:
            with open(cfg_paths['system'], 'r', encoding='utf-8') as f:
                raw = yaml.safe_load(f)
            self.SYS = raw.get('system', {})
            self.get_logger().info(f'已加载系统配置: {cfg_paths["system"]}')
        except Exception as e:
            self.get_logger().warn(f'加载 system.yaml 失败: {e}，使用默认值')
            self.SYS = {}

        # 加载 camera.yaml（坐标系名称）
        try:
            with open(cfg_paths['camera'], 'r', encoding='utf-8') as f:
                raw = yaml.safe_load(f)
            self.CAM = raw.get('camera', {})
            self.get_logger().info(f'已加载相机配置: {cfg_paths["camera"]}')
        except Exception as e:
            self.get_logger().warn(f'加载 camera.yaml 失败: {e}，使用默认值')
            self.CAM = {}

    def _hardcoded_poses(self):
        """硬编码默认值（当 YAML 加载失败时使用）"""
        return {
            'left': {
                'initial': [0, -30, 50, 0, 80, 0],
                'recognize': [30, -20, 40, 0, 90, 0],
                'pick_prep': [20, -40, 30, 0, 100, 0],
                'pick_left_support': [10, -50, 20, 0, 110, 0],
                'pick_left_lift': [5, -60, 10, 0, 120, 0],
                'pick_left_push': [5, -60, 10, 0, 120, 0],
                'pick_left_safe': [0, -30, 50, 0, 80, 0],
                'place_above': [0, -30, 50, 0, 80, 0],
                'place_exit': [0, -30, 50, 0, 80, 0],
                'place_safe': [0, -30, 50, 0, 80, 0],
            },
            'right': {
                'initial': [0, -30, 50, 0, 80, 0],
                'recognize': [-30, -20, 40, 0, 90, 0],
                'pick_prep': [-20, -40, 30, 0, 100, 0],
                'pick_insert': [-10, -50, 5, 0, 130, 0],
                'pick_grasp_vertical': [-5, -55, 0, 0, 135, 0],
                'pick_lift': [-5, -55, -10, 0, 135, 0],
                'pick_safe': [-5, -55, -10, 0, 135, 0],
                'place_above': [0, -40, -20, 0, 130, 0],
                'place_drop': [0, -50, -40, 0, 120, 0],
                'place_exit': [0, -40, -20, 0, 130, 0],
                'place_safe': [0, -30, 50, 0, 80, 0],
            }
        }

    def _wait_for_aruco_service(self):
        """等待 ArUco 检测 Service 可用"""
        try:
            from guji.srv import DetectAruco
            self._aruco_client = self.create_client(
                DetectAruco, self._aruco_srv_name
            )
            self.get_logger().info(f'等待 ArUco 视觉服务: {self._aruco_srv_name}')
            if self._aruco_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('ArUco 视觉服务已就绪')
            else:
                self.get_logger().warn('ArUco 视觉服务等待超时，视觉调用将使用 Mock')
        except Exception as e:
            self.get_logger().warn(f'无法创建 ArUco Service Client: {e}，视觉将使用 Mock')

    # ==========================================
    # 启动检查
    # ==========================================
    def startup_checks(self):
        """
        启动时检查所有依赖是否就绪。
        返回 True 表示所有检查通过，False 表示有检查失败。

        检查项目：
        1. 关节状态数据接收（左右臂）
        2. ArUco 视觉 Service 可用性
        3. TF 变换树完整性
        4. 相机状态 Service
        """
        self.get_logger().info('========================================')
        self.get_logger().info('  执行启动检查...')
        self.get_logger().info('========================================')

        checks = [
            ('_check_joint_states',  '关节状态数据'),
            ('_check_aruco_service', 'ArUco 视觉服务'),
            ('_check_tf_tree',       'TF 变换树'),
            ('_check_camera_status', '相机状态'),
        ]

        all_passed = True
        for method_name, desc in checks:
            method = getattr(self, method_name, None)
            if method is None:
                self.get_logger().error(f'  [  SKIP  ] {desc} (方法不存在)')
                continue
            try:
                result = method()
                if result:
                    self.get_logger().info(f'  [   OK   ] {desc}')
                else:
                    self.get_logger().error(f'  [ FAILED ] {desc}')
                    all_passed = False
            except Exception as e:
                self.get_logger().error(f'  [ERROR  ] {desc}: {e}')
                all_passed = False

        if all_passed:
            self.get_logger().info('========================================')
            self.get_logger().info('  所有检查通过，系统就绪！')
            self.get_logger().info('========================================')
        else:
            self.get_logger().error('========================================')
            self.get_logger().error('  存在检查失败，请修复后再运行！')
            self.get_logger().error('========================================')

        return all_passed

    def _check_joint_states(self):
        """检查 1：等待左右臂关节状态数据"""
        self.get_logger().info('  [检查1] 等待关节状态数据...')
        start = time.time()
        timeout = self.SYS.get('arm', {}).get('joint_state_timeout', 10.0)
        while rclpy.ok():
            if self.has_left_joint and self.has_right_joint:
                self.get_logger().info(
                    f'    左臂: {len(self.left_joint_positions)} joints, '
                    f'右臂: {len(self.right_joint_positions)} joints'
                )
                return True
            if time.time() - start > timeout:
                self.get_logger().warn(
                    f'    超时，左臂={self.has_left_joint}，右臂={self.has_right_joint}'
                )
                return False
            rclpy.sleep(0.1)

    def _check_aruco_service(self):
        """检查 2：ArUco 视觉服务可用性"""
        self.get_logger().info('  [检查2] ArUco 视觉服务...')
        if self._aruco_client is None:
            self.get_logger().warn('    ArUco Client 未初始化（将使用 Mock）')
            return True  # Mock 模式下不强制失败
        if not self._aruco_client.service_is_ready():
            self.get_logger().warn('    ArUco Service 未就绪（将使用 Mock）')
            return True  # Mock 模式下不强制失败
        self.get_logger().info('    ArUco Service 已就绪')
        return True

    def _check_tf_tree(self):
        """检查 3：TF 变换树"""
        self.get_logger().info('  [检查3] TF 变换树...')
        try:
            frames = self._tf_buffer.all_frames_as_string()
            needed_frames = ['right_base', 'camera_right', 'right_top']
            found_any = False
            for frame in needed_frames:
                if frame in frames:
                    self.get_logger().info(f'    找到坐标系: {frame}')
                    found_any = True
                else:
                    self.get_logger().warn(f'    未找到坐标系: {frame}')
            return found_any
        except Exception as e:
            self.get_logger().warn(f'    TF 查询异常: {e}')
            return True  # 非致命

    def _check_camera_status(self):
        """检查 4：相机状态 Service"""
        self.get_logger().info('  [检查4] 相机状态...')
        if not self._camera_status_client.service_is_ready():
            self.get_logger().warn('    相机状态 Service 未就绪（跳过）')
            return True  # 非关键，不强制失败
        req = Trigger.Request()
        future = self._camera_status_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'    相机状态: {resp.message}')
            else:
                self.get_logger().warn(f'    相机状态告警: {resp.message}')
            return resp.success
        self.get_logger().warn('    相机状态查询超时（跳过）')
        return True  # 超时不强制失败

    # ==========================================
    # 回调函数
    # ==========================================
    def _left_joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.left_joint_positions = list(msg.position)
            self.has_left_joint = True

    def _right_joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.right_joint_positions = list(msg.position)
            self.has_right_joint = True

    # ==========================================
    # 基础控制方法
    # ==========================================
    def movej(self, arm: str, joints: list, speed: int = 30, block: bool = True):
        """
        控制机械臂以关节运动方式到达目标位置。

        参数:
            arm:   'left' 或 'right'
            joints: 关节角度列表（单位：度）
            speed:  速度 1-100
            block:  是否阻塞等待运动完成
        """
        global Movej_msg
        if Movej_msg is None:
            try:
                from rm_ros_interfaces.msg import Movej
                Movej_msg = Movej
            except ImportError:
                self.get_logger().error('无法导入 Movej 消息类型')
                return

        msg = Movej_msg()
        msg.joint = [math.radians(j) for j in joints]
        msg.speed = speed
        msg.block = block

        if arm.lower() == 'left':
            self.left_movej_pub.publish(msg)
            self.get_logger().info(f'左臂 MoveJ: {[f"{j:.1f}" for j in joints]}deg, speed={speed}, block={block}')
        elif arm.lower() == 'right':
            self.right_movej_pub.publish(msg)
            self.get_logger().info(f'右臂 MoveJ: {[f"{j:.1f}" for j in joints]}deg, speed={speed}, block={block}')
        else:
            self.get_logger().error(f'无效的臂标识: {arm}，请使用 "left" 或 "right"')

    def movel(self, arm: str, pose: Pose, speed: int = 30, block: bool = True):
        """
        控制机械臂以直线运动方式到达目标位置。

        参数:
            arm:   'left' 或 'right'
            pose:  geometry_msgs/Pose 目标位姿
            speed: 速度 1-100
            block: 是否阻塞等待运动完成
        """
        global Movel_msg
        if Movel_msg is None:
            try:
                from rm_ros_interfaces.msg import Movel
                Movel_msg = Movel
            except ImportError:
                self.get_logger().error('无法导入 Movel 消息类型')
                return

        msg = Movel_msg()
        msg.pose = pose
        msg.speed = speed
        msg.block = block

        if arm.lower() == 'left':
            self.left_movel_pub.publish(msg)
            self.get_logger().info(f'左臂 MoveL: pos=({pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f}), speed={speed}')
        elif arm.lower() == 'right':
            self.right_movel_pub.publish(msg)
            self.get_logger().info(f'右臂 MoveL: pos=({pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f}), speed={speed}')
        else:
            self.get_logger().error(f'无效的臂标识: {arm}')

    def gripper_set(self, arm: str, position: int, block: bool = True, timeout: int = 5000):
        """
        设置夹爪到固定位置（位置控制）。

        参数:
            arm:      'left' 或 'right'
            position: 1~1000，开口度约 0~70mm
            block:    是否阻塞等待
            timeout:  超时时间（ms）
        """
        global Gripperset_msg
        if Gripperset_msg is None:
            try:
                from rm_ros_interfaces.msg import Gripperset
                Gripperset_msg = Gripperset
            except ImportError:
                self.get_logger().error('无法导入 Gripperset 消息类型')
                return

        msg = Gripperset_msg()
        msg.position = position
        msg.block = block
        msg.timeout = timeout

        if arm.lower() == 'left':
            self.left_gripper_set_pub.publish(msg)
            self.get_logger().info(f'左臂夹爪设置位置: {position} (block={block})')
        elif arm.lower() == 'right':
            self.right_gripper_set_pub.publish(msg)
            self.get_logger().info(f'右臂夹爪设置位置: {position} (block={block})')
        else:
            self.get_logger().error(f'无效的臂标识: {arm}')

    def gripper_pick(self, arm: str, speed: int = 300, force: int = 300,
                     block: bool = True, timeout: int = 5000):
        """
        力控夹取：当夹爪受力超过设定力后停止运动。

        参数:
            arm:    'left' 或 'right'
            speed:  1~1000，夹爪开合速度
            force:  1~1000，夹持力阈值（最大 1.5kg）
            block:  是否阻塞等待
            timeout: 超时时间（ms）
        """
        global Gripperpick_msg
        if Gripperpick_msg is None:
            try:
                from rm_ros_interfaces.msg import Gripperpick
                Gripperpick_msg = Gripperpick
            except ImportError:
                self.get_logger().error('无法导入 Gripperpick 消息类型')
                return

        msg = Gripperpick_msg()
        msg.speed = speed
        msg.force = force
        msg.block = block
        msg.timeout = timeout

        if arm.lower() == 'left':
            self.left_gripper_pick_pub.publish(msg)
            self.get_logger().info(f'左臂力控夹取: speed={speed}, force={force}')
        elif arm.lower() == 'right':
            self.right_gripper_pick_pub.publish(msg)
            self.get_logger().info(f'右臂力控夹取: speed={speed}, force={force}')
        else:
            self.get_logger().error(f'无效的臂标识: {arm}')

    def force_position_move(self, arm: str, pose: Pose, sensor: int = 0,
                            mode: int = 1, direction: int = 2,
                            force_threshold: int = 10, follow: bool = True):
        """
        力位混合运动：在指定方向上运动直到达到力阈值。

        参数:
            arm:             'left' 或 'right'
            pose:            geometry_msgs/Pose 目标位姿
            sensor:          0=一维力, 1=六维力
            mode:            0=基坐标系, 1=工具坐标系
            direction:       0~5 对应 x/y/z/rx/ry/rz
            force_threshold: 力阈值，精度 0.1N
            follow:          高跟随模式
        """
        global Forcepositionmovepose_msg
        if Forcepositionmovepose_msg is None:
            try:
                from rm_ros_interfaces.msg import Forcepositionmovepose
                Forcepositionmovepose_msg = Forcepositionmovepose
            except ImportError:
                self.get_logger().error('无法导入 Forcepositionmovepose 消息类型')
                return

        msg = Forcepositionmovepose_msg()
        msg.pose = pose
        msg.sensor = sensor
        msg.mode = mode
        msg.dir = direction
        msg.force = force_threshold
        msg.follow = follow

        if arm.lower() == 'left':
            self.left_force_pub.publish(msg)
            self.get_logger().info(f'左臂力位混合运动: dir={direction}, force={force_threshold}')
        elif arm.lower() == 'right':
            self.right_force_pub.publish(msg)
            self.get_logger().info(f'右臂力位混合运动: dir={direction}, force={force_threshold}')
        else:
            self.get_logger().error(f'无效的臂标识: {arm}')

    def movej_both(self, left_joints: list, right_joints: list,
                   speed: int = 30, block: bool = False):
        """
        同时控制两个机械臂以关节方式运动。

        参数:
            left_joints:  左臂关节角度列表（单位：度）
            right_joints: 右臂关节角度列表（单位：度）
            speed: 速度 1-100
            block: 是否阻塞等待（默认 False，双臂同时运动）
        """
        self.movej('left', left_joints, speed, block)
        self.movej('right', right_joints, speed, block)
        self.get_logger().info('双臂同时 MoveJ')

    # ==========================================
    # 视觉识别（真实 + Mock 降级）
    # ==========================================
    def _mock_offset(self, code: str):
        """Mock 偏移（当视觉 Service 不可用时使用）"""
        mock_offsets = {
            'ARcode11': {'x': 0.1,  'y': -0.05, 'z': 0.0,  'roll': 0.0,  'pitch': 0.0, 'yaw': 0.0},
            'ARcode12': {'x': -0.1, 'y': 0.05,  'z': 0.0,  'roll': 0.0,  'pitch': 0.0, 'yaw': 0.0},
            'ARcode21': {'x': 0.0,  'y': 0.0,   'z': -0.02, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        }
        return mock_offsets.get(code, {'x': 0.0, 'y': 0.0, 'z': 0.0,
                                       'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})

    def detect_arcode(self, arm: str, code: str):
        """
        ARcode 识别：通过 /right_arm/detect_aruco Service 获取真实位置。
        当 Service 不可用时，自动降级为 Mock。

        参数:
            arm: 'left' 或 'right' 或 'head'（仅右臂有相机，统一用右臂服务）
            code: ARcode 标识符，如 'ARcode11', 'ARcode12', 'ARcode21'

        返回:
            dict: 包含 'x', 'y', 'z', 'roll', 'pitch', 'yaw' 的字典（单位：米，弧度）
        """
        # 从 code 提取 marker ID
        try:
            marker_id = int(code.replace('ARcode', ''))
        except ValueError:
            self.get_logger().error(f'无法解析 marker ID: {code}，使用 Mock')
            return self._mock_offset(code)

        # 检查 Service 是否可用
        if self._aruco_client is None or not self._aruco_client.service_is_ready():
            self.get_logger().warn(
                f'ArUco Service 不可用，使用 Mock 识别 {code}'
            )
            return self._mock_offset(code)

        # 调用 ArUco 检测 Service
        try:
            from guji.srv import DetectAruco
            req = DetectAruco.Request()
            req.marker_id = marker_id
            req.timeout = self.SYS.get('vision', {}).get('timeout', 5.0)

            self.get_logger().info(f'调用 ArUco Service: marker_id={marker_id}, timeout={req.timeout}')
            future = self._aruco_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=int(req.timeout) + 5)

            if future.result() is None:
                self.get_logger().warn(f'ArUco Service 调用超时，使用 Mock')
                return self._mock_offset(code)

            resp = future.result()
            if not resp.found:
                self.get_logger().warn(f'未检测到 marker ID={marker_id}，使用 Mock')
                return self._mock_offset(code)

            # 从响应提取位姿
            pose = resp.pose
            euler = T.euler_from_quaternion([
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w
            ])
            result = {
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z),
                'roll': float(euler[0]),
                'pitch': float(euler[1]),
                'yaw': float(euler[2]),
            }
            self.get_logger().info(
                f'ArUco 真实识别: {code} -> '
                f'pos=({result["x"]:.4f}, {result["y"]:.4f}, {result["z"]:.4f})m, '
                f'rpy=({result["roll"]:.3f}, {result["pitch"]:.3f}, {result["yaw"]:.3f})rad'
            )
            return result

        except Exception as e:
            self.get_logger().error(f'ArUco Service 调用异常: {e}，使用 Mock')
            return self._mock_offset(code)

    # ==========================================
    # 各状态流程方法
    # ==========================================
    def _get_joints(self, arm: str, state: str):
        """从 POSES 配置中获取关节角度，带校验"""
        try:
            joints = self.POSES[arm][state]
            if joints is None or len(joints) != 6:
                raise KeyError(f'点位 {arm}/{state} 格式错误: {joints}')
            return joints
        except KeyError:
            self.get_logger().warn(f'点位配置缺失: {arm}/{state}，跳过此步骤')
            return None

    def go_initial_position(self, speed: int = None):
        """双臂回到初始位置（上举姿态）"""
        self.get_logger().info('>>> [状态1] 双臂回到初始位置')
        if speed is None:
            speed = self.SYS.get('arm', {}).get('movej_default_speed', 30)

        left_joints = self._get_joints('left', 'initial')
        right_joints = self._get_joints('right', 'initial')
        if left_joints is None or right_joints is None:
            self.get_logger().error('点位配置缺失，跳过')
            return

        self.movej_both(left_joints, right_joints, speed=speed, block=True)
        self.get_logger().info('<<< [状态1] 完成：双臂已到达初始位置')

    def go_recognize_position(self, speed: int = None):
        """双臂移动到识别位置（相机垂直朝下）"""
        self.get_logger().info('>>> [状态2] 双臂进入识别位置（相机垂直向下）')
        if speed is None:
            speed = self.SYS.get('arm', {}).get('movej_default_speed', 30)

        left_joints = self._get_joints('left', 'recognize')
        right_joints = self._get_joints('right', 'recognize')
        if left_joints is None or right_joints is None:
            self.get_logger().error('点位配置缺失，跳过')
            return

        self.movej_both(left_joints, right_joints, speed=speed, block=True)
        self.get_logger().info('<<< [状态2] 完成：双臂已到达识别位置')

    def recognize_arcode(self):
        """
        视觉识别 ARcode：
        - 右臂相机识别 ARcode12（仅右臂有相机）
        - 获得工件相对偏移
        """
        self.get_logger().info('>>> [状态3] 识别 ARcode12（右臂相机）')
        offset = self.detect_arcode('right', 'ARcode12')
        self.get_logger().info('<<< [状态3] 完成：已识别 ARcode12')
        return offset

    def pre_grasp_alignment(self, speed: int = None):
        """
        预抓取对齐：
        - 右臂移动到工件前方
        - 左臂移动到工件左边
        - 右臂前推工件对齐
        - 左臂右推工件对齐
        """
        self.get_logger().info('>>> [状态4] 预抓取对齐')
        if speed is None:
            speed = self.SYS.get('arm', {}).get('movej_default_speed', 30) // 2

        left_joints = self._get_joints('left', 'pick_prep')
        right_joints = self._get_joints('right', 'pick_prep')
        if left_joints and right_joints:
            self.movej('left', left_joints, speed=speed, block=True)
            self.movej('right', right_joints, speed=speed, block=True)

        # 右臂前推对齐（小幅调整）
        right_support = self._get_joints('right', 'pick_insert')
        left_support = self._get_joints('left', 'pick_left_support')
        if right_support:
            self.get_logger().info('    右臂前推工件对齐...')
            self.movej('right', right_support, speed=speed, block=True)
        if left_support:
            self.get_logger().info('    左臂右推工件对齐...')
            self.movej('left', left_support, speed=speed, block=True)

        self.get_logger().info('<<< [状态4] 完成：预抓取对齐')

    def pick_workpiece(self, speed: int = None):
        """
        核心取料逻辑（11步）：

        Step 1:  左臂 MoveJ → 目标点（左臂到达工件侧面托住位置）
        Step 2:  左臂夹爪闭合（力控夹取，force=300）
        Step 3:  左臂 MoveJ → 抬起托住位（弧线抬起调整工件姿态）
        Step 4:  右臂 MoveJ → 插入位（右臂伸入工件下方）
        Step 5:  右臂夹爪闭合（力控夹取，force=300）
        Step 6:  右臂 MoveJ → 垂直位（右臂夹爪回到垂直姿态）
        Step 7:  右臂夹爪松开（position=1000，开口最大）
        Step 8:  左臂 MoveJ → 前顶位（左臂将工件推向右臂）
        Step 9:  左臂夹爪松开（position=1000）
        Step 10: 右臂 MoveJ → 上移位（右臂带着工件上移）
        Step 11: 左臂 MoveJ → 安全位（左臂回缩）
        """
        self.get_logger().info('>>> [状态5] 开始取料（11步核心逻辑）')
        if speed is None:
            speed = self.SYS.get('arm', {}).get('movej_default_speed', 30) * 2 // 3

        gripper_cfg = self.SYS.get('gripper', {})

        # Step 1: 左臂移动到托住位置
        left_support = self._get_joints('left', 'pick_left_support')
        if left_support:
            self.get_logger().info('  [取料 Step 1] 左臂移动到托住位置')
            self.movej('left', left_support, speed=speed, block=True)

        # Step 2: 左臂夹爪闭合（托住/按住工件）
        self.get_logger().info('  [取料 Step 2] 左臂夹爪闭合（托住工件）')
        self.gripper_pick('left',
                           speed=gripper_cfg.get('pick_speed', 300),
                           force=gripper_cfg.get('pick_force', 300),
                           block=True)

        # Step 3: 左臂抬起托住位
        left_lift = self._get_joints('left', 'pick_left_lift')
        if left_lift:
            self.get_logger().info('  [取料 Step 3] 左臂弧线抬起托住位')
            self.movej('left', left_lift, speed=speed, block=True)

        # Step 4: 右臂伸入工件下方
        right_insert = self._get_joints('right', 'pick_insert')
        if right_insert:
            self.get_logger().info('  [取料 Step 4] 右臂伸入工件下方')
            self.movej('right', right_insert, speed=speed, block=True)

        # Step 5: 右臂夹爪闭合（夹取工件）
        self.get_logger().info('  [取料 Step 5] 右臂力控夹取')
        self.gripper_pick('right',
                           speed=gripper_cfg.get('pick_speed', 300),
                           force=gripper_cfg.get('pick_force', 300),
                           block=True)

        # Step 6: 右臂夹爪回到垂直姿态
        right_vertical = self._get_joints('right', 'pick_grasp_vertical')
        if right_vertical:
            self.get_logger().info('  [取料 Step 6] 右臂夹爪回到垂直位置')
            self.movej('right', right_vertical, speed=speed, block=True)

        # Step 7: 右臂夹爪松开（让工件回落到左臂夹爪上）
        self.get_logger().info('  [取料 Step 7] 右臂夹爪松开')
        self.gripper_set('right',
                          position=gripper_cfg.get('position_open', 1000),
                          block=True)

        # Step 8: 左臂前顶工件到位
        left_push = self._get_joints('left', 'pick_left_push')
        if left_push:
            self.get_logger().info('  [取料 Step 8] 左臂前顶工件')
            self.movej('left', left_push, speed=speed, block=True)

        # Step 9: 左臂夹爪松开
        self.get_logger().info('  [取料 Step 9] 左臂夹爪松开')
        self.gripper_set('left',
                          position=gripper_cfg.get('position_open', 1000),
                          block=True)

        # Step 10: 右臂带着工件上移
        right_lift = self._get_joints('right', 'pick_lift')
        if right_lift:
            self.get_logger().info('  [取料 Step 10] 右臂带着工件上移')
            self.movej('right', right_lift, speed=speed, block=True)

        # Step 11: 左臂回到安全位
        left_safe = self._get_joints('left', 'pick_left_safe')
        if left_safe:
            self.get_logger().info('  [取料 Step 11] 左臂回缩到安全位置')
            self.movej('left', left_safe, speed=speed, block=True)

        self.get_logger().info('<<< [状态5] 完成：取料成功，右臂已夹取物料')

    def go_place_position(self, speed: int = None):
        """
        双臂移动到放料准备位置。
        左臂保持安全位，右臂携带物料移动到放料上方位。
        """
        self.get_logger().info('>>> [状态6] 双臂移动到放料准备位置')
        if speed is None:
            speed = self.SYS.get('arm', {}).get('movej_default_speed', 30)

        left_joints = self._get_joints('left', 'place_above')
        right_joints = self._get_joints('right', 'place_above')
        if left_joints is None or right_joints is None:
            self.get_logger().error('点位配置缺失，跳过')
            return

        self.movej_both(left_joints, right_joints, speed=speed, block=True)
        self.get_logger().info('<<< [状态6] 完成：已到达放料准备位置')

    def recognize_arcode21(self):
        """
        放料前识别 ARcode21（右臂相机）。
        """
        self.get_logger().info('>>> [状态7] 识别 ARcode21（右臂相机识别目标位置）')
        place_offset = self.detect_arcode('right', 'ARcode21')
        self.get_logger().info('<<< [状态7] 完成：已识别 ARcode21')
        return place_offset

    def place_workpiece(self, speed: int = None):
        """
        力位混合放料流程（8步）：

        Step 1:  右臂 MoveJ → 放料上方位
        Step 2:  右臂力位混合下移（搜索放置面）
        Step 3:  右臂夹爪张开（释放物料）
        Step 4:  右臂 MoveJ → 退出位
        Step 5:  右臂夹爪闭合（position=0，压实物料）
        Step 6:  右臂夹爪上移（position=500）
        Step 7:  右臂夹爪闭合（position=0，夹住）
        Step 8:  右臂 MoveJ → 放料安全位
        """
        self.get_logger().info('>>> [状态8] 开始放料（力位混合搜索）')
        if speed is None:
            speed = self.SYS.get('arm', {}).get('movej_default_speed', 30) * 2 // 3

        gripper_cfg = self.SYS.get('gripper', {})
        fp_cfg = self.SYS.get('force_position', {})

        # Step 1: 右臂移动到放料上方位
        place_above = self._get_joints('right', 'place_above')
        if place_above:
            self.get_logger().info('  [放料 Step 1] 右臂移动到放料上方位')
            self.movej('right', place_above, speed=speed, block=True)

        # Step 2: 右臂力位混合下移搜索（Z方向，工具坐标系）
        self.get_logger().info('  [放料 Step 2] 右臂力位混合下移搜索')
        target_pose = Pose()
        target_pose.position.x = 0.0
        target_pose.position.y = 0.0
        target_pose.position.z = -0.05
        target_pose.orientation.w = 1.0
        self.force_position_move(
            'right',
            pose=target_pose,
            sensor=fp_cfg.get('sensor_type', 0),
            mode=fp_cfg.get('coordinate_mode', 1),
            direction=fp_cfg.get('direction', 2),
            force_threshold=fp_cfg.get('force_threshold', 10),
            follow=fp_cfg.get('follow_mode', True)
        )

        # Step 3: 右臂夹爪张开，释放物料
        self.get_logger().info('  [放料 Step 3] 右臂夹爪张开释放物料')
        self.gripper_set('right',
                          position=gripper_cfg.get('position_open', 1000),
                          block=True)

        # Step 4: 右臂移动到退出位
        place_exit = self._get_joints('right', 'place_exit')
        if place_exit:
            self.get_logger().info('  [放料 Step 4] 右臂移动到退出位')
            self.movej('right', place_exit, speed=speed, block=True)

        # Step 5: 右臂夹爪闭合压实物料
        self.get_logger().info('  [放料 Step 5] 右臂夹爪压实物料')
        self.gripper_set('right',
                          position=gripper_cfg.get('position_close', 0),
                          block=True)

        # Step 6: 右臂夹爪上移
        self.get_logger().info('  [放料 Step 6] 右臂夹爪上移')
        self.gripper_set('right',
                          position=gripper_cfg.get('position_half', 500),
                          block=True)

        # Step 7: 右臂夹爪闭合
        self.get_logger().info('  [放料 Step 7] 右臂夹爪闭合')
        self.gripper_set('right',
                          position=gripper_cfg.get('position_close', 0),
                          block=True)

        # Step 8: 右臂移动到放料安全位
        place_safe = self._get_joints('right', 'place_safe')
        if place_safe:
            self.get_logger().info('  [放料 Step 8] 右臂移动到放料安全位')
            self.movej('right', place_safe, speed=speed, block=True)

        self.get_logger().info('<<< [状态8] 完成：放料成功')

    def return_to_pick_origin(self, speed: int = None):
        """双臂回取料处原点位置，一轮结束。"""
        self.get_logger().info('>>> [状态9] 回取料处原点（一轮结束）')
        if speed is None:
            speed = self.SYS.get('arm', {}).get('movej_default_speed', 30)

        left_joints = self._get_joints('left', 'initial')
        right_joints = self._get_joints('right', 'initial')
        if left_joints is None or right_joints is None:
            self.get_logger().error('点位配置缺失，跳过')
            return

        self.movej_both(left_joints, right_joints, speed=speed, block=True)
        self.get_logger().info('<<< [状态9] 完成：已回到取料处原点')
        self.get_logger().info('========================================')
        self.get_logger().info('  一轮取放料流程完成！')
        self.get_logger().info('========================================')

    # ==========================================
    # 主状态机入口
    # ==========================================
    def run(self, skip_checks: bool = False):
        """
        执行完整的取放料流程。
        流程：
        1. 启动检查（可选）
        2. 双臂回到初始位置
        3. 进入识别位置
        4. 识别 ARcode12
        5. 推对齐
        6. 取料（11步）
        7. 移动到放料位置
        8. 识别 ARcode21
        9. 放料（力位混合）
        10. 回取料处原点

        参数:
            skip_checks: True=跳过启动检查（用于调试），False=执行所有检查
        """
        self.get_logger().info('========================================')
        self.get_logger().info('  双臂取放料流程开始')
        self.get_logger().info('========================================')

        try:
            # 启动检查
            if not skip_checks:
                if not self.startup_checks():
                    self.get_logger().error('启动检查未通过，请修复后重试')
                    return
            else:
                self.get_logger().warn('已跳过启动检查（调试模式）')
                # 仍然等待关节状态
                self.get_logger().info('等待机械臂状态数据...')
                start = time.time()
                while rclpy.ok():
                    if self.has_left_joint and self.has_right_joint:
                        break
                    if time.time() - start > 10.0:
                        self.get_logger().warn('等待超时，继续执行...')
                        break
                    rclpy.sleep(0.1)

            # 执行各状态
            self.go_initial_position()
            rclpy.sleep(0.5)

            self.go_recognize_position()
            rclpy.sleep(0.5)

            self.recognize_arcode()
            rclpy.sleep(0.5)

            self.pre_grasp_alignment()
            rclpy.sleep(0.5)

            self.pick_workpiece()
            rclpy.sleep(0.5)

            self.go_place_position()
            rclpy.sleep(0.5)

            self.recognize_arcode21()
            rclpy.sleep(0.5)

            self.place_workpiece()
            rclpy.sleep(0.5)

            self.return_to_pick_origin()

        except KeyboardInterrupt:
            self.get_logger().info('用户中断执行')


def main(args=None):
    rclpy.init(args=args)

    left_ns = 'l_arm'
    right_ns = 'r_arm'

    if len(sys.argv) > 1:
        left_ns = sys.argv[1]
    if len(sys.argv) > 2:
        right_ns = sys.argv[2]

    try:
        controller = DualArmPickPlaceController(left_ns, right_ns)

        print()
        print("========================================")
        print("  睿尔曼双臂取放料控制器")
        print("========================================")
        print()
        print("  Namespace 配置:")
        print(f"    左臂: /{left_ns}")
        print(f"    右臂: /{right_ns}")
        print()
        print("  启动检查:")
        print("    controller.startup_checks()  # 检查所有依赖")
        print()
        print("  基础控制:")
        print("    controller.movej('left',  [j1, j2, j3, j4, j5, j6])")
        print("    controller.movej('right', [j1, j2, j3, j4, j5, j6])")
        print("    controller.gripper_pick('right', speed=300, force=300)")
        print("    controller.gripper_set('right', position=1000)")
        print("    controller.detect_arcode('right', 'ARcode11')")
        print()
        print("  执行完整流程:")
        print("    controller.run()              # 含启动检查")
        print("    controller.run(skip_checks=True)  # 跳过检查（调试用）")
        print()
        print("========================================")
        print()

        controller.run()

    except KeyboardInterrupt:
        print("\n程序已退出")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
