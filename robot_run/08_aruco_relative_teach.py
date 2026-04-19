#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
08 - ArUco 与末端法兰相对位姿示教 / 回放

场景1（底座不动）：a 记录码在基座下位姿；s 依次记录多个目标法兰位姿（编号递增）；
                 d 计算各目标相对码的位姿并保存 YAML。

场景2（基座可移动）：a 仅识别并打印码位姿（便于确认视野）；s 再次识别码、读取 YAML，
                 计算当前基座下各目标位姿并依次 Movel（目标间默认间歇 3s）。

法兰位姿默认由 TF r_base_link1 -> r_link6 得到（适配 dual_rm_65b real_moveit_demo）。
"""

from __future__ import annotations

import argparse
import cv2
import cv_bridge
import math
import select
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

try:
    from rm_ros_interfaces.msg import Movel
except ImportError as _e:
    Movel = None  # type: ignore
    _IMPORT_ERR = _e
else:
    _IMPORT_ERR = None


def _script_dir() -> Path:
    return Path(__file__).resolve().parent


def _load_vision_timeout() -> float:
    p = _script_dir().parent / 'config' / 'system.yaml'
    try:
        with open(p, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        return float(cfg.get('system', {}).get('vision', {}).get('timeout', 5.0))
    except Exception:
        return 5.0


ARUCO_DICT_MAP = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
    'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
    'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
    'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
    'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
}


def _load_camera_cfg() -> dict:
    p = _script_dir().parent / 'config' / 'camera.yaml'
    with open(p, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def pose_to_mat(pose: Pose) -> np.ndarray:
    x, y, z = pose.position.x, pose.position.y, pose.position.z
    qx, qy, qz, qw = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)
    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)
    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)
    t = np.eye(4, dtype=float)
    t[0, 0], t[0, 1], t[0, 2] = r00, r01, r02
    t[1, 0], t[1, 1], t[1, 2] = r10, r11, r12
    t[2, 0], t[2, 1], t[2, 2] = r20, r21, r22
    t[0, 3], t[1, 3], t[2, 3] = x, y, z
    return t


def mat_to_pose(T: np.ndarray) -> Pose:
    r = T[:3, :3]
    tvec = T[:3, 3]
    tr = float(np.trace(r))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    p = Pose()
    p.position.x = float(tvec[0])
    p.position.y = float(tvec[1])
    p.position.z = float(tvec[2])
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p


def transform_to_mat(tr) -> np.ndarray:
    p = Pose()
    p.position.x = tr.transform.translation.x
    p.position.y = tr.transform.translation.y
    p.position.z = tr.transform.translation.z
    p.orientation = tr.transform.rotation
    return pose_to_mat(p)


def mat_to_nested(T: np.ndarray) -> List[List[float]]:
    return [[float(T[i, j]) for j in range(4)] for i in range(4)]


def nested_to_mat(rows) -> np.ndarray:
    return np.array(rows, dtype=float)


def print_pose(label: str, pose: Pose, logger_or_node) -> None:
    logger = logger_or_node.get_logger() if hasattr(logger_or_node, 'get_logger') else logger_or_node
    logger.info(
        f'{label} 位置(m): x={pose.position.x:.5f}, y={pose.position.y:.5f}, z={pose.position.z:.5f}'
    )
    logger.info(
        f'{label} 姿态(xyzw): {pose.orientation.x:.5f}, {pose.orientation.y:.5f}, '
        f'{pose.orientation.z:.5f}, {pose.orientation.w:.5f}'
    )


class ArucoRelativeTeachNode(Node):
    """交互式示教 / 回放节点"""

    def __init__(self, args) -> None:
        super().__init__('aruco_relative_teach_08')
        self._args = args
        self._base = args.base_frame
        self._tcp = args.tcp_frame
        self._marker_id = int(args.marker_id)
        self._vision_timeout = float(args.vision_timeout)
        self._arm_ns = args.arm_ns.strip('/')
        self._movel_speed = int(args.movel_speed)
        self._pause = float(args.pause_between)
        self._output_path = Path(args.output).expanduser().resolve()
        self._input_mode = args.input_mode
        self._show_camera_view = bool(args.show_camera_view)
        self._camera_frame_override = (args.camera_frame or '').strip()
        self._camera_frame_cfg = ''
        self._hand_eye_flange_frame = ''
        self._hand_eye_camera_frame = ''
        self._T_flange_camera: Optional[np.ndarray] = None
        self._preview_win = '08 ArUco Live View'
        self._bridge = cv_bridge.CvBridge()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cam_cfg = _load_camera_cfg().get('camera', {})
        aruco_cfg = cam_cfg.get('aruco', {})
        frames_cfg = cam_cfg.get('frames', {})
        self._topic_prefix = str(cam_cfg.get('topic_prefix', '/camera_right')).rstrip('/')
        self._camera_frame_cfg = str(frames_cfg.get('camera_frame', '')).strip()
        flange_frame_cfg = str(frames_cfg.get('flange_frame', '')).strip()
        if not self._camera_frame_override and self._camera_frame_cfg:
            # 优先使用配置文件给出的相机 TF frame，减少对 CameraInfo.frame_id 的依赖。
            self._camera_frame_override = self._camera_frame_cfg
        dict_name = str(aruco_cfg.get('dict_type', 'DICT_5X5_1000'))
        self._marker_size = float(aruco_cfg.get('marker_size', 0.03))
        dict_id = ARUCO_DICT_MAP.get(dict_name, cv2.aruco.DICT_5X5_1000)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._aruco_detector = cv2.aruco.ArucoDetector(
            self._aruco_dict, cv2.aruco.DetectorParameters()
        )
        self._latest_image: Optional[np.ndarray] = None
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._camera_frame_id: Optional[str] = None
        self._latest_joint_state: Optional[JointState] = None

        # 场景1 状态
        self._T_base_marker: Optional[np.ndarray] = None
        self._targets: List[Tuple[int, np.ndarray]] = []

        self.create_subscription(
            Image, f'{self._topic_prefix}/color/image_raw', self._on_image, 10
        )
        self.create_subscription(
            CameraInfo, f'{self._topic_prefix}/color/camera_info', self._on_camera_info, 10
        )
        self.create_subscription(
            JointState, f'/{self._arm_ns}/joint_states', self._on_joint_state, 10
        )
        self._publish_hand_eye_static_tf(cam_cfg, flange_frame_cfg, self._camera_frame_cfg)

        if Movel is not None:
            self._movel_pub = self.create_publisher(
                Movel,
                f'/{self._arm_ns}/rm_driver/movel_cmd',
                10,
            )
        else:
            self._movel_pub = None

    def _publish_hand_eye_static_tf(self, cam_cfg: dict, flange_frame: str, camera_frame: str) -> None:
        hand_eye = cam_cfg.get('hand_eye_right', {})
        t_cfg = hand_eye.get('translation', {})
        q_cfg = hand_eye.get('quaternion', {})
        keys = ('x', 'y', 'z')
        q_keys = ('x', 'y', 'z', 'w')
        if not flange_frame or not camera_frame:
            return
        if not all(k in t_cfg for k in keys) or not all(k in q_cfg for k in q_keys):
            return
        try:
            tx, ty, tz = (float(t_cfg[k]) for k in keys)
            qx, qy, qz, qw = (float(q_cfg[k]) for k in q_keys)
        except (TypeError, ValueError):
            self.get_logger().warn('hand_eye_right 存在非法数值，跳过自动静态 TF 发布')
            return
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n <= 1e-9:
            self.get_logger().warn('hand_eye_right 四元数范数接近 0，跳过自动静态 TF 发布')
            return
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        p = Pose()
        p.position.x = tx
        p.position.y = ty
        p.position.z = tz
        p.orientation.x = qx
        p.orientation.y = qy
        p.orientation.z = qz
        p.orientation.w = qw
        self._T_flange_camera = pose_to_mat(p)
        self._hand_eye_flange_frame = flange_frame
        self._hand_eye_camera_frame = camera_frame
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id = flange_frame
        tr.child_frame_id = camera_frame
        tr.transform.translation.x = tx
        tr.transform.translation.y = ty
        tr.transform.translation.z = tz
        tr.transform.rotation.x = qx
        tr.transform.rotation.y = qy
        tr.transform.rotation.z = qz
        tr.transform.rotation.w = qw
        self._static_tf_broadcaster.sendTransform(tr)
        self.get_logger().info(
            f'已自动发布手眼静态TF: {flange_frame} -> {camera_frame} (来源: camera.hand_eye_right)'
        )

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera_matrix is not None:
            return
        if len(msg.k) != 9:
            return
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self._camera_frame_id = msg.header.frame_id.strip()
        self.get_logger().info('CameraInfo 已就绪')

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _detect_marker_pose_once(self) -> Optional[Pose]:
        if self._latest_image is None or self._camera_matrix is None or self._dist_coeffs is None:
            return None
        gray = cv2.cvtColor(self._latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        if ids is None:
            return None
        flat = ids.flatten()
        idxs = np.where(flat == self._marker_id)[0]
        if idxs.size == 0:
            return None
        idx = int(idxs[0])
        corner = corners[idx]
        rvec, tvec = self._estimate_marker_pose(corner)
        if rvec is None or tvec is None:
            return None
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = tvec.reshape(3)
        return mat_to_pose(T)

    def _estimate_marker_pose(self, corner: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self._camera_matrix is None or self._dist_coeffs is None:
            return None, None
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, self._marker_size, self._camera_matrix, self._dist_coeffs
            )
            rv = np.asarray(rvec[0, 0, :], dtype=np.float64).reshape(3, 1)
            tv = np.asarray(tvec[0, 0, :], dtype=np.float64).reshape(3, 1)
            return rv, tv

        # 兼容旧版 OpenCV：直接用 marker 四角 solvePnP
        half = self._marker_size * 0.5
        obj_pts = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )
        img_pts = np.asarray(corner, dtype=np.float32).reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            self._camera_matrix,
            self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self._camera_matrix, self._dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not ok:
                return None, None
        return np.asarray(rvec, dtype=np.float64).reshape(3, 1), np.asarray(tvec, dtype=np.float64).reshape(3, 1)

    def _render_live_view(self) -> None:
        if not self._show_camera_view or self._latest_image is None:
            return
        vis = self._latest_image.copy()
        txt = f'Target ID={self._marker_id} (press command keys in terminal)'
        cv2.putText(
            vis,
            txt,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        gray = cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            if self._camera_matrix is not None and self._dist_coeffs is not None:
                flat = ids.flatten()
                idxs = np.where(flat == self._marker_id)[0]
                if idxs.size > 0:
                    idx = int(idxs[0])
                    rvec, tvec = self._estimate_marker_pose(corners[idx])
                    if rvec is not None and tvec is not None:
                        try:
                            cv2.drawFrameAxes(
                                vis, self._camera_matrix, self._dist_coeffs, rvec, tvec, self._marker_size * 0.5, 2
                            )
                        except cv2.error:
                            pass
        try:
            cv2.imshow(self._preview_win, vis)
            cv2.waitKey(1)
        except cv2.error:
            # 无显示环境（例如 SSH）时自动关闭可视化，避免反复报错
            self._show_camera_view = False
            self.get_logger().warn('无法创建 OpenCV 窗口，已自动关闭实时画面（可使用 --no-camera-view）')

    def call_detect_aruco(self) -> Tuple[bool, Optional[Pose], Optional[Pose], str]:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < self._vision_timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._render_live_view()
            pose_cam = self._detect_marker_pose_once()
            if pose_cam is not None:
                ok_tf, pose_base, tf_msg = self._marker_pose_cam_to_base(pose_cam)
                if ok_tf and pose_base is not None:
                    return True, pose_cam, pose_base, ''
                return False, pose_cam, None, tf_msg
        if self._camera_matrix is None:
            return False, None, None, '未收到 CameraInfo'
        if self._latest_image is None:
            return False, None, None, '未收到图像'
        return False, None, None, '超时未检测到标记'

    def _marker_pose_cam_to_base(self, pose_cam: Pose) -> Tuple[bool, Optional[Pose], str]:
        cam_frame_raw = (self._camera_frame_id or '').strip()
        if not cam_frame_raw and not self._camera_frame_override:
            return False, None, 'CameraInfo 缺少 frame_id，无法转换到基座坐标系'
        # 优先使用显式指定的相机 frame；其次使用 CameraInfo.frame_id；最后回退到 camera_right。
        # 这样可兼容“话题是 optical_frame，但 TF 只发布 camera_right”的场景。
        candidates = []
        if self._camera_frame_override:
            candidates.append(self._camera_frame_override)
        if self._camera_frame_cfg and self._camera_frame_cfg not in candidates:
            candidates.append(self._camera_frame_cfg)
        if cam_frame_raw:
            candidates.append(cam_frame_raw)
        if 'camera_right' not in candidates:
            candidates.append('camera_right')
        err_msgs = []
        tr = None
        cam_frame_used = ''
        try:
            for cam_frame in candidates:
                try:
                    tr = self._tf_buffer.lookup_transform(
                        self._base,
                        cam_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=1.0),
                    )
                    cam_frame_used = cam_frame
                    break
                except Exception as e:
                    err_msgs.append(f'{cam_frame}: {e}')
            if tr is None:
                # 兜底：若 camera frame 仍未进入 TF Buffer，则用 base<-flange（实时）和
                # flange<-camera（手眼标定）直接组合出 base<-camera。
                if (
                    self._T_flange_camera is not None
                    and self._hand_eye_flange_frame
                    and self._hand_eye_camera_frame
                ):
                    try:
                        tr_bf = self._tf_buffer.lookup_transform(
                            self._base,
                            self._hand_eye_flange_frame,
                            rclpy.time.Time(),
                            timeout=Duration(seconds=1.0),
                        )
                        T_bf = transform_to_mat(tr_bf)
                        T_bc = T_bf @ self._T_flange_camera
                        T_cm = pose_to_mat(pose_cam)
                        T_bm = T_bc @ T_cm
                        self.get_logger().warn(
                            f'直接 TF 查询相机 frame 失败，已回退用手眼矩阵链路: '
                            f'{self._base} <- {self._hand_eye_flange_frame} <- {self._hand_eye_camera_frame}'
                        )
                        return True, mat_to_pose(T_bm), ''
                    except Exception as e:
                        err_msgs.append(f'fallback({self._base}<-{self._hand_eye_flange_frame}<-{self._hand_eye_camera_frame}): {e}')
                msg = ' | '.join(err_msgs)
                return False, None, f'TF 查询失败({self._base} <- camera_frame_candidates): {msg}'
        except Exception as e:
            return False, None, f'TF 查询异常: {e}'

        T_bc = transform_to_mat(tr)
        T_cm = pose_to_mat(pose_cam)
        T_bm = T_bc @ T_cm
        if cam_frame_used and cam_frame_used != cam_frame_raw:
            self.get_logger().warn(
                f'CameraInfo.frame_id={cam_frame_raw or "<empty>"} 无法用于 TF，已回退使用 {cam_frame_used}'
            )
        return True, mat_to_pose(T_bm), ''

    def lookup_flange_mat(self) -> np.ndarray:
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.05)
        tr = self._tf_buffer.lookup_transform(
            self._base,
            self._tcp,
            rclpy.time.Time(),
            timeout=Duration(seconds=3.0),
        )
        return transform_to_mat(tr)

    def lookup_flange_pose(self) -> Pose:
        return mat_to_pose(self.lookup_flange_mat())

    def _print_aruco_detection_details(self, pose_cam: Pose, pose_base: Pose) -> None:
        cam_frame = (self._camera_frame_id or '').strip() or '<unknown_camera_frame>'
        self.get_logger().info(f'ArUco 识别结果: id={self._marker_id}')
        self.get_logger().info(f'坐标系关系: {cam_frame} -> marker_{self._marker_id}')
        print_pose(f'ArUco(ID={self._marker_id}, 相机系:{cam_frame})', pose_cam, self)
        self.get_logger().info(f'坐标系关系: {self._base} -> marker_{self._marker_id}')
        print_pose(f'ArUco(ID={self._marker_id}, 基座系:{self._base})', pose_base, self)

    def _print_current_and_target_joint_hint(self, goal_pose: Pose, target_index: int) -> None:
        if self._latest_joint_state is None:
            self.get_logger().warn('[d] 尚未收到 joint_states，无法打印当前关节角')
        else:
            names = list(self._latest_joint_state.name)
            vals = list(self._latest_joint_state.position)
            if not names:
                names = [f'joint_{i + 1}' for i in range(len(vals))]
            self.get_logger().info('[d] 当前关节角(rad / deg):')
            for n, v in zip(names, vals):
                self.get_logger().info(f'  {n}: {v:.6f} rad ({math.degrees(v):.2f} deg)')

        self.get_logger().info(
            f'[d] 目标 #{target_index} 的目标位姿(基座系:{self._base})如下:'
        )
        print_pose(f'目标#{target_index} 法兰', goal_pose, self)
        self.get_logger().warn(
            '[d] 目标关节角当前无法直接给出：本脚本使用 Movel 笛卡尔目标，'
            '关节解由控制器/规划器在线生成。'
        )

    def _wait_scene2_decision(self, target_index: int, goal_pose: Pose) -> str:
        self.get_logger().info(
            f'目标 #{target_index}: 按 d 查看关节信息，按 s 执行运动，按 q 取消后续目标。'
        )
        if self._input_mode == 'line':
            while rclpy.ok():
                cmd = input('场景2 命令 [d=查看关节/s=执行/q=取消]: ').strip().lower()
                if not cmd:
                    continue
                ch = cmd[0]
                if ch == 'd':
                    self._print_current_and_target_joint_hint(goal_pose, target_index)
                    continue
                if ch in ('s', 'q'):
                    return ch
                self.get_logger().warn(f'未知命令: {ch!r}')
            return 'q'

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._render_live_view()
            try:
                k = self._read_key()
            except KeyboardInterrupt:
                return 'q'
            if k is None:
                continue
            if k == 'd':
                self._print_current_and_target_joint_hint(goal_pose, target_index)
                continue
            if k in ('s', 'q', '\x1b'):
                return 'q' if k in ('q', '\x1b') else 's'
            self.get_logger().warn(f'未知命令: {k!r}')
        return 'q'

    def on_key_a_scene1(self) -> None:
        ok, pose_cam, pose_base, msg = self.call_detect_aruco()
        if not ok or pose_cam is None or pose_base is None:
            self.get_logger().error(f'[a] ArUco 失败: {msg}')
            return
        self._T_base_marker = pose_to_mat(pose_base)
        self.get_logger().info(
            f'[a] 已记录 ArUco(ID={self._marker_id}) 在 {self._base} 下的位姿（用于后续 d 保存）'
        )
        self._print_aruco_detection_details(pose_cam, pose_base)

    def on_key_s_scene1(self) -> None:
        idx = len(self._targets) + 1
        try:
            T = self.lookup_flange_mat()
        except Exception as e:
            self.get_logger().error(f'[s] TF 查询失败: {e}')
            return
        self._targets.append((idx, T))
        pose = mat_to_pose(T)
        self.get_logger().info(f'[s] 已记录目标 #{idx}（法兰在 {self._base} 下）')
        print_pose(f'法兰 目标#{idx}', pose, self)

    def on_key_d_scene1(self) -> None:
        if self._T_base_marker is None:
            self.get_logger().error('[d] 请先按 a 记录 ArUco 位姿')
            return
        if not self._targets:
            self.get_logger().error('[d] 请至少按 s 记录一个目标位姿')
            return
        T_bm = self._T_base_marker
        inv_bm = np.linalg.inv(T_bm)
        doc = {
            'version': 2,
            'scene': 1,
            'marker_id': self._marker_id,
            'base_frame': self._base,
            'tcp_frame': self._tcp,
            'arm_ns': self._arm_ns,
            'vision_mode': 'local_aruco',
            'T_base_marker_at_teach': mat_to_nested(T_bm),
            'targets': [],
        }
        for idx, T_bt in self._targets:
            T_mt = inv_bm @ T_bt
            doc['targets'].append({
                'index': idx,
                'T_marker_tcp': mat_to_nested(T_mt),
            })
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._output_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(doc, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f'[d] 已保存 {len(self._targets)} 组相对位姿 -> {self._output_path}')

    def on_key_a_scene2(self) -> None:
        ok, pose_cam, pose_base, msg = self.call_detect_aruco()
        if not ok or pose_cam is None or pose_base is None:
            self.get_logger().error(f'[a] ArUco 失败: {msg}')
            return
        self.get_logger().info('[a] ArUco 识别成功')
        self._print_aruco_detection_details(pose_cam, pose_base)

    def on_key_s_scene2(self) -> None:
        if Movel is None or self._movel_pub is None:
            self.get_logger().error('Movel 消息不可用，无法执行场景2')
            return
        path = self._output_path
        if not path.is_file():
            self.get_logger().error(f'[s] 文件不存在: {path}')
            return
        with open(path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f)
        ok, pose_cam, pose_m, msg = self.call_detect_aruco()
        if not ok or pose_cam is None or pose_m is None:
            self.get_logger().error(f'[s] ArUco 失败，取消运动: {msg}')
            return
        T_bm_new = pose_to_mat(pose_m)
        self.get_logger().info('[s] 已获取当前 ArUco 位姿')
        self._print_aruco_detection_details(pose_cam, pose_m)
        targets = sorted(doc.get('targets', []), key=lambda x: int(x['index']))
        for i, t in enumerate(targets):
            T_mt = nested_to_mat(t['T_marker_tcp'])
            T_goal = T_bm_new @ T_mt
            goal_pose = mat_to_pose(T_goal)
            target_index = int(t['index'])
            try:
                current_pose = self.lookup_flange_pose()
                self.get_logger().info(
                    f'[s] 当前位姿与下一目标预览 ({i + 1}/{len(targets)}):'
                )
                print_pose(f'当前法兰({self._base}->{self._tcp})', current_pose, self)
            except Exception as e:
                self.get_logger().warn(f'[s] 读取当前法兰位姿失败: {e}')
            print_pose(f'下一目标#{target_index} 法兰({self._base}系)', goal_pose, self)

            decision = self._wait_scene2_decision(target_index, goal_pose)
            if decision == 'q':
                self.get_logger().warn('[s] 用户取消后续目标执行')
                break

            m = Movel()
            m.pose = goal_pose
            # rm_ros_interfaces/Movel.speed 在当前驱动中是 int 类型
            # 并按 1~100 的速度档位使用，这里做一次类型和范围保护
            speed_i = int(round(self._movel_speed))
            speed_i = max(1, min(100, speed_i))
            m.speed = speed_i
            m.block = True
            self.get_logger().info(
                f'[s] -> 执行目标 #{target_index} ({i + 1}/{len(targets)})'
            )
            self._movel_pub.publish(m)
            if i < len(targets) - 1:
                time.sleep(max(self._pause, 0.0))
        self.get_logger().info('[s] 全部目标已下发完成（请确认驱动 block 行为与到位情况）')

    def print_help(self, scene: int) -> None:
        if scene == 1:
            self.get_logger().info(
                '场景1：a=识别并记录码位姿 | s=记录当前法兰为目标(编号递增) | '
                'd=保存相对位姿到文件 | q=退出'
            )
        else:
            self.get_logger().info(
                '场景2：a=识别并打印码位姿 | s=读文件并逐目标预览，'
                '每个目标可 d=看关节信息 / s=执行 / q=取消 | q=退出'
            )
        if self._input_mode == 'line':
            self.get_logger().info('输入模式: 行模式 — 输入单字母命令后按回车')

    def run(self, scene: int) -> None:
        self.print_help(scene)
        if self._input_mode == 'line':
            self._run_line(scene)
        else:
            self._run_keys(scene)

    def _run_line(self, scene: int) -> None:
        while rclpy.ok():
            try:
                cmd = input('命令 [a/s/d/q]: ').strip().lower()
            except EOFError:
                break
            except KeyboardInterrupt:
                self.get_logger().info('退出')
                break
            if not cmd:
                continue
            ch = cmd[0]
            self._dispatch(scene, ch)
            rclpy.spin_once(self, timeout_sec=0.01)

    def _setup_tty(self):
        self._old_term = None
        if sys.platform == 'win32' or not sys.stdin.isatty():
            return
        try:
            import termios
            import tty

            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            self._old_term = None

    def _restore_tty(self):
        if self._old_term is not None:
            try:
                import termios

                termios.tcsetattr(
                    sys.stdin, termios.TCSADRAIN, self._old_term
                )
            except Exception:
                pass
        self._old_term = None

    def _read_key(self) -> Optional[str]:
        if sys.platform == 'win32':
            try:
                import msvcrt
            except ImportError:
                return None
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x03',):
                    raise KeyboardInterrupt
                return ch.decode('latin-1', errors='ignore').lower()
            return None
        if self._old_term is not None:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
        return None

    def _run_keys(self, scene: int) -> None:
        self._setup_tty()
        try:
            self.get_logger().info('按键模式已启用（单键 a/s/d/q，勿关终端焦点）')
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    k = self._read_key()
                except KeyboardInterrupt:
                    break
                if k is None:
                    continue
                if k in ('q', '\x1b'):
                    self.get_logger().info('退出')
                    break
                self._dispatch(scene, k)
        finally:
            self._restore_tty()

    def _dispatch(self, scene: int, k: str) -> None:
        if scene == 1:
            if k == 'a':
                self.on_key_a_scene1()
            elif k == 's':
                self.on_key_s_scene1()
            elif k == 'd':
                self.on_key_d_scene1()
            else:
                self.get_logger().warn(f'未知命令: {k!r}')
        else:
            if k == 'a':
                self.on_key_a_scene2()
            elif k == 's':
                self.on_key_s_scene2()
            elif k == 'd':
                self.get_logger().warn('场景2 不需要 d 键')
            else:
                self.get_logger().warn(f'未知命令: {k!r}')


def parse_args():
    default_out = _script_dir().parent / 'data' / 'aruco_relative_targets.yaml'
    p = argparse.ArgumentParser(description='08 ArUco 相对位姿示教 / 回放')
    p.add_argument(
        '--scene',
        type=int,
        choices=(1, 2),
        required=True,
        help='1=示教并保存，2=读取并回放 Movel',
    )
    p.add_argument('--marker-id', type=int, required=True, help='ArUco 标记 ID')
    p.add_argument(
        '-o',
        '--output',
        default=str(default_out),
        help='YAML 保存/读取路径（场景1 写，场景2 读）',
    )
    p.add_argument(
        '--arm-side',
        choices=('left', 'right'),
        default='right',
        help='机械臂侧别，自动选择 base/tcp 坐标系（可被 --base-frame/--tcp-frame 覆盖）',
    )
    p.add_argument(
        '--base-frame',
        default=None,
        help='基坐标系；未设置时按 --arm-side 自动选择',
    )
    p.add_argument(
        '--tcp-frame',
        default=None,
        help='末端法兰坐标系；未设置时按 --arm-side 自动选择',
    )
    p.add_argument(
        '--arm-ns',
        default=None,
        help='发布 movel_cmd 的 namespace（仅场景2）；未设置时按 --arm-side 自动选择',
    )
    p.add_argument(
        '--vision-timeout',
        type=float,
        default=None,
        help='单次识别超时(s)，默认读 config/system.yaml',
    )
    p.add_argument('--movel-speed', type=int, default=30, help='Movel 速度 1-100')
    p.add_argument(
        '--pause-between',
        type=float,
        default=3.0,
        help='场景2 每个目标 Movel 之后间歇(秒)',
    )
    p.add_argument(
        '--input-mode',
        choices=('key', 'line'),
        default='key',
        help='key=单键(需终端)；line=输入字母后回车',
    )
    p.add_argument(
        '--no-camera-view',
        action='store_false',
        dest='show_camera_view',
        help='关闭检测过程中的实时相机画面',
    )
    p.add_argument(
        '--camera-frame',
        default='',
        help='可选：强制用于 TF 查询的相机 frame（如 camera_right）',
    )
    p.set_defaults(show_camera_view=True)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    side_cfg = {
        'left': {
            'base_frame': 'l_base_link1',
            'tcp_frame': 'l_link6',
            'arm_ns': 'left_arm_controller',
        },
        'right': {
            'base_frame': 'r_base_link1',
            'tcp_frame': 'r_link6',
            'arm_ns': 'right_arm_controller',
        },
    }
    cfg = side_cfg[args.arm_side]
    if args.base_frame is None:
        args.base_frame = cfg['base_frame']
    if args.tcp_frame is None:
        args.tcp_frame = cfg['tcp_frame']
    if args.arm_ns is None:
        args.arm_ns = cfg['arm_ns']
    if args.vision_timeout is None:
        args.vision_timeout = _load_vision_timeout()

    if not sys.stdin.isatty() and args.input_mode == 'key':
        print('非交互终端，已自动改用 --input-mode line', file=sys.stderr)
        args.input_mode = 'line'

    if args.scene == 2 and _IMPORT_ERR is not None:
        print(f'场景2 需要 rm_ros_interfaces.msg.Movel: {_IMPORT_ERR}', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = ArucoRelativeTeachNode(args)
    try:
        node.run(args.scene)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
