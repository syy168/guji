#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 码识别与移动检测程序

功能：
1. 第一部分：识别 ArUco 码的初始位置
2. 第二部分：移动 ArUco 码后再识别，计算移动距离

工作流程：
1. 启动程序
2. 第一段：按 's' 进行初始 ArUco 识别，保存其在坐标系中的位置
3. 移动 ArUco 码（改变其在物理空间中的位置）
4. 第二段：再次按 's' 进行识别，程序输出两次识别的移动距离

坐标系说明：
- 相机坐标系：以相机为原点
- 末端执行器坐标系：以机械臂末端法兰为原点
- 基座坐标系：以机械臂基座为原点

依赖：
- ROS2
- rclpy
- OpenCV (cv2)
- numpy
- scipy (四元数操作)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import Pose
import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml
from pathlib import Path
import time
from scipy.spatial.transform import Rotation as R
from typing import Optional, Tuple, Dict
import math


class ArucoTracker(Node):
    """ArUco 码识别与追踪节点"""
    
    def __init__(self, arm: str = 'right', config_path: str = None):
        super().__init__('aruco_tracker')
        
        self.arm = arm.lower()  # 'left' or 'right'
        self.bridge = CvBridge()
        
        # 配置参数
        self.config = {}
        self.load_config(config_path)
        
        # 获取 ArUco 参数
        aruco_config = self.config['camera'].get('aruco', {})
        self.dict_type = aruco_config.get('dict_type', 'DICT_5X5_1000')
        self.marker_size = aruco_config.get('marker_size', 0.03)  # 米
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # 手眼标定外参
        self.hand_eye_calibration = self.load_hand_eye_calibration()
        
        # 臂基座固定偏移（从 URDF 中定义的固定关节参数）
        self.arm_base_offset = self.load_arm_base_offset()
        
        # 当前数据
        self.latest_color_image = None
        self.current_joint_states = {}
        
        # 识别结果存储
        self.first_detection = None  # (tvec, rvec, marker_id, timestamp)
        self.second_detection = None
        self.stage = 1  # 当前阶段
        
        # ArUco 字典
        self.aruco_dict = self.get_aruco_dict()
        self.parameters = cv2.aruco.DetectorParameters()
        
        # ==========================================
        # 创建订阅者
        # ==========================================
        arm_prefix = 'r' if self.arm == 'right' else 'l'
        
        # 相机话题
        self.color_sub = self.create_subscription(
            Image,
            f"{self.config['camera']['topic_prefix']}/color/image_raw",
            self.color_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.config['camera']['topic_prefix']}/color/camera_info",
            self.camera_info_callback,
            10
        )
        
        # 关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.get_logger().info(f'🎯 ArUco 追踪程序已启动 (臂: {self.arm})')
        self.get_logger().info(f'   字典: {self.dict_type}')
        self.get_logger().info(f'   标记尺寸: {self.marker_size*1000:.1f} mm')
        
        # 等待相机数据
        self.wait_for_camera_data()
        
        # 启动交互界面
        self.start_interactive_mode()
    
    def load_config(self, config_path: Optional[str] = None):
        """加载配置文件"""
        if config_path is None:
            config_path = 'guji/config/camera.yaml'
        
        config_file = Path(config_path)
        if not config_file.exists():
            self.get_logger().warn(f'⚠️  配置文件不存在: {config_file}，使用默认参数')
            self.config = self.get_default_config()
        else:
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    self.config = yaml.safe_load(f)
                self.get_logger().info(f'✅ 加载配置文件: {config_file}')
            except Exception as e:
                self.get_logger().error(f'❌ 配置文件读取失败: {e}，使用默认参数')
                self.config = self.get_default_config()
    
    def get_default_config(self) -> Dict:
        """获取默认配置"""
        return {
            'camera': {
                'topic_prefix': '/camera_right',
                'aruco': {
                    'dict_type': 'DICT_5X5_1000',
                    'marker_size': 0.030,
                },
                'hand_eye_right': {
                    'translation': {'x': 0.085, 'y': -0.040, 'z': 0.010},
                    'quaternion': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                # URDF 中定义的臂基座固定偏移
                'arm_base_offset_right': {
                    'position': {'x': -0.1, 'y': -0.1103, 'z': 0.031645},
                    'rotation_rpy': {'r': 0.0, 'p': -0.7854, 'y': 0.0}
                },
                'arm_base_offset_left': {
                    'position': {'x': 0.1, 'y': -0.1103, 'z': 0.031645},
                    'rotation_rpy': {'r': 0.0, 'p': -0.7854, 'y': 3.1416}
                }
            }
        }
    
    def load_hand_eye_calibration(self) -> Dict:
        """加载手眼标定参数"""
        if self.arm == 'right':
            calib_data = self.config['camera'].get('hand_eye_right', {})
        else:
            calib_data = self.config['camera'].get('hand_eye_left', {})
        
        # 提取平移和旋转
        translation = calib_data.get('translation', {'x': 0, 'y': 0, 'z': 0})
        quaternion = calib_data.get('quaternion', {'x': 0, 'y': 0, 'z': 0, 'w': 1})
        
        return {
            'translation': np.array([translation['x'], translation['y'], translation['z']]),
            'quaternion': np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']]),
            'rotation_matrix': R.from_quat([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']]).as_matrix()
        }
    
    def load_arm_base_offset(self) -> Dict:
        """加载臂基座固定偏移（从 URDF 中定义的固定关节参数）
        
        臂基座是末端执行器相对于基座的固定变换关系
        """
        if self.arm == 'right':
            offset_data = self.config['camera'].get('arm_base_offset_right', {})
        else:
            offset_data = self.config['camera'].get('arm_base_offset_left', {})
        
        # 提取位置
        position = offset_data.get('position', {'x': 0, 'y': 0, 'z': 0})
        position_array = np.array([position['x'], position['y'], position['z']])
        
        # 提取旋转（欧拉角 roll, pitch, yaw）
        rotation_rpy = offset_data.get('rotation_rpy', {'r': 0, 'p': 0, 'y': 0})
        rpy_array = np.array([rotation_rpy['r'], rotation_rpy['p'], rotation_rpy['y']])
        
        # 将欧拉角转换为旋转矩阵
        rotation_matrix = R.from_euler('xyz', rpy_array).as_matrix()
        
        return {
            'position': position_array,
            'rotation_matrix': rotation_matrix,
            'rpy': rpy_array
        }

    
    def get_aruco_dict(self):
        """获取 ArUco 字典"""
        dict_name = self.dict_type
        try:
            if hasattr(cv2.aruco, dict_name):
                return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
            else:
                self.get_logger().warn(f'⚠️  未知字典类型: {dict_name}，使用 DICT_5X5_1000')
                return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        except Exception as e:
            self.get_logger().error(f'❌ ArUco 字典加载失败: {e}')
            return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    
    def camera_info_callback(self, msg: CameraInfo):
        """相机内参回调"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('✅ 相机内参已获取')
    
    def color_callback(self, msg: Image):
        """彩色图像回调"""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'❌ 图像转换失败: {e}')
    
    def joint_callback(self, msg: JointState):
        """关节状态回调"""
        for name, position in zip(msg.name, msg.position):
            self.current_joint_states[name] = position
    
    def wait_for_camera_data(self):
        """等待获取相机数据"""
        self.get_logger().info('⏳ 正在获取相机数据...')
        start_time = time.time()
        timeout = 10.0
        
        while not self.camera_info_received or self.latest_color_image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('❌ 相机数据获取超时')
                return False
        
        self.get_logger().info('✅ 相机数据获取成功')
        return True
    
    def detect_aruco(self, image: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[int]]:
        """
        检测 ArUco 码
        
        返回:
            (tvec, rvec, marker_id) 或 (None, None, None)
        """
        if self.camera_matrix is None:
            self.get_logger().error('❌ 相机内参未获取')
            return None, None, None
        
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 使用新的 ArUco API (OpenCV 4.7+)
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
            
            if ids is None or len(ids) == 0:
                return None, None, None
            
            # 使用第一个检测到的标记
            marker_id = int(ids[0][0])
            marker_corners = corners[0]
            
            # 估计标记的位姿
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners.reshape(1, -1, 2),
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )
            
            return tvec[0], rvec[0], marker_id
            
        except Exception as e:
            self.get_logger().error(f'❌ ArUco 检测失败: {e}')
            return None, None, None
    
    def transform_point_camera_to_camera(self, tvec: np.ndarray) -> np.ndarray:
        """ArUco 在相机坐标系中的位置（已有）"""
        return tvec
    
    def transform_point_camera_to_eef(self, point_camera: np.ndarray) -> np.ndarray:
        """
        将点从相机坐标系转换到末端执行器坐标系
        
        使用手眼标定参数进行变换
        """
        # 点 = 旋转 * 点_相机 + 平移
        point_eef = self.hand_eye_calibration['rotation_matrix'] @ point_camera + self.hand_eye_calibration['translation']
        return point_eef
    
    def transform_point_eef_to_base(self, point_eef: np.ndarray) -> np.ndarray:
        """
        将点从末端执行器坐标系转换到基座坐标系
        
        使用 URDF 中定义的固定关节参数进行变换
        不需要运行时的末端执行器位姿信息，使用硬编码的臂基座偏移
        
        变换关系: P_base = R_eef_to_base @ P_eef + T_eef_to_base
        """
        # 使用从 URDF 中加载的固定变换
        point_base = self.arm_base_offset['rotation_matrix'] @ point_eef + self.arm_base_offset['position']
        return point_base
    
    def visualize_detection(self, image: np.ndarray, tvec: np.ndarray, rvec: np.ndarray, marker_id: int) -> np.ndarray:
        """在图像上绘制检测结果"""
        img_copy = image.copy()
        
        # 绘制坐标轴
        img_copy = cv2.drawFrameAxes(
            img_copy, 
            self.camera_matrix, 
            self.dist_coeffs, 
            rvec, 
            tvec, 
            length=0.05
        )
        
        # 绘制 ArUco 码边框
        corners = cv2.aruco.drawDetectedMarkers(img_copy, [np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)])
        
        # 添加文字信息
        text = f"ID: {marker_id} | Pos: ({tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}) m"
        cv2.putText(img_copy, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return img_copy
    
    def perform_detection(self, stage_name: str) -> bool:
        """执行一次检测"""
        print(f'\n【{stage_name}】')
        print('='*70)
        
        # 获取最新图像
        if self.latest_color_image is None:
            print('❌ 没有获取到图像数据')
            return False
        
        image = self.latest_color_image.copy()
        
        # 检测 ArUco
        print('🔍 正在检测 ArUco 码...')
        tvec, rvec, marker_id = self.detect_aruco(image)
        
        if tvec is None:
            print('❌ 未检测到 ArUco 码')
            return False
        
        print(f'✅ 检测到 ArUco 码 (ID: {marker_id})')
        print(f'   相机坐标系位置: ({tvec[0]:.4f}, {tvec[1]:.4f}, {tvec[2]:.4f}) m')
        
        # 转换到末端执行器坐标系
        point_eef = self.transform_point_camera_to_eef(tvec)
        print(f'   末端执行器坐标系: ({point_eef[0]:.4f}, {point_eef[1]:.4f}, {point_eef[2]:.4f}) m')
        
        # 转换到基座坐标系（使用 URDF 中的固定关节参数）
        point_base = self.transform_point_eef_to_base(point_eef)
        print(f'   基座坐标系位置: ({point_base[0]:.4f}, {point_base[1]:.4f}, {point_base[2]:.4f}) m')
        
        # 显示检测结果
        vis_image = self.visualize_detection(image, tvec, rvec, marker_id)
        cv2.imshow(f'{stage_name} - ArUco Detection', vis_image)
        cv2.waitKey(1)
        
        # 保存结果
        timestamp = time.time()
        if stage_name == '第一步：初始检测':
            self.first_detection = {
                'tvec': tvec.copy(),
                'rvec': rvec.copy(),
                'tvec_eef': point_eef.copy(),
                'tvec_base': point_base.copy(),
                'marker_id': marker_id,
                'timestamp': timestamp
            }
        else:
            self.second_detection = {
                'tvec': tvec.copy(),
                'rvec': rvec.copy(),
                'tvec_eef': point_eef.copy(),
                'tvec_base': point_base.copy(),
                'marker_id': marker_id,
                'timestamp': timestamp
            }
        
        return True
    
    def calculate_movement(self):
        """计算移动距离"""
        if self.first_detection is None or self.second_detection is None:
            print('❌ 两次检测数据不完整')
            return
        
        print('\n' + '='*70)
        print('📊 移动距离计算结果')
        print('='*70 + '\n')
        
        # 相机坐标系中的移动
        delta_camera = self.second_detection['tvec'] - self.first_detection['tvec']
        distance_camera = np.linalg.norm(delta_camera)
        print(f'📷 相机坐标系中的移动:')
        print(f'   Δ位置: ({delta_camera[0]:.4f}, {delta_camera[1]:.4f}, {delta_camera[2]:.4f}) m')
        print(f'   距离: {distance_camera:.4f} m = {distance_camera*1000:.2f} mm')
        
        # 末端执行器坐标系中的移动
        delta_eef = self.second_detection['tvec_eef'] - self.first_detection['tvec_eef']
        distance_eef = np.linalg.norm(delta_eef)
        print(f'\n🦾 末端执行器坐标系中的移动:')
        print(f'   Δ位置: ({delta_eef[0]:.4f}, {delta_eef[1]:.4f}, {delta_eef[2]:.4f}) m')
        print(f'   距离: {distance_eef:.4f} m = {distance_eef*1000:.2f} mm')
        
        # 基座坐标系中的移动（使用 URDF 中的固定关节参数）
        delta_base = self.second_detection['tvec_base'] - self.first_detection['tvec_base']
        distance_base = np.linalg.norm(delta_base)
        print(f'\n🏠 基座坐标系中的移动（基于 URDF 固定参数）:')
        print(f'   Δ位置: ({delta_base[0]:.4f}, {delta_base[1]:.4f}, {delta_base[2]:.4f}) m')
        print(f'   距离: {distance_base:.4f} m = {distance_base*1000:.2f} mm')
        
        print('\n' + '='*70)
    
    def start_interactive_mode(self):
        """启动交互模式"""
        print('\n' + '='*70)
        print('🎯 ArUco 码识别与移动检测程序')
        print('='*70 + '\n')
        
        # 第一步：初始检测
        print('【步骤 1 - 初始检测】')
        print('准备好 ArUco 码，使其在相机视野内，然后按 s 键进行初始检测')
        print('(按其他键跳过)\n')
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # 显示实时图像
            if self.latest_color_image is not None:
                cv2.imshow('Camera Feed - Press s for Detection', self.latest_color_image)
            
            if key == ord('s') or key == ord('S'):
                if self.perform_detection('第一步：初始检测'):
                    print('✅ 第一步完成，请现在移动 ArUco 码...\n')
                    break
                else:
                    print('❌ 检测失败，请重试\n')
            elif key == ord('q') or key == ord('Q'):
                print('⏹️  程序已退出')
                return
        
        # 第二步：等待 ArUco 码被移动
        print('【步骤 2 - 等待移动】')
        print('已检测第一个位置。请移动 ArUco 码到新位置，然后按 s 键进行第二次检测')
        print('(按其他键跳过)\n')
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # 显示实时图像
            if self.latest_color_image is not None:
                cv2.imshow('Camera Feed - Press s for Detection', self.latest_color_image)
            
            if key == ord('s') or key == ord('S'):
                if self.perform_detection('第二步：移动后检测'):
                    print('✅ 第二步完成\n')
                    self.calculate_movement()
                    break
                else:
                    print('❌ 检测失败，请重试\n')
            elif key == ord('q') or key == ord('Q'):
                print('⏹️  程序已退出')
                return


def main(args=None):
    rclpy.init(args=args)
    
    print('\n' + '='*70)
    print('🎯 ArUco 码识别与移动检测程序')
    print('='*70 + '\n')
    
    # 选择机械臂
    arm_input = input('请选择机械臂 (left/right，默认 right): ').strip().lower()
    if arm_input in ['left', 'l']:
        arm = 'left'
    else:
        arm = 'right'
    
    try:
        node = ArucoTracker(arm=arm)
    except KeyboardInterrupt:
        print('\n\n⏹️  用户中断程序')
    except Exception as e:
        print(f'\n❌ 程序出错: {e}')
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
