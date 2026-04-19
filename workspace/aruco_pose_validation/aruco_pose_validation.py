#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 两次识别验证程序（独立版）

满足需求：
1. 右臂保持不动，从 ROS 话题读取右臂关节角度（仅用于记录与核对）
2. 第一次识别 ArUco，输出三个坐标系下位置：
   - 相机坐标系
   - 右臂基坐标系
   - 机器人基坐标系
3. 移动 ArUco 后第二次识别，输出同样三坐标系结果
4. 计算并输出“机器人基坐标系”下两次识别的平移差

坐标变换策略：
- camera -> right_arm_base: 优先按图像时间戳查 TF，失败回退 latest TF
- right_arm_base -> robot_base: 优先查 TF；仅在固定 frame 且 TF 失败时回退 xacro 固定参数
"""

from __future__ import annotations

import argparse
import math
import re
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener


ARUCO_DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
}

XACRO_FIXED_ROBOT_BASE_FRAME = "platform_base_link"
XACRO_FIXED_RIGHT_BASE_FRAME = "r_base_link1"


def detect_project_root() -> Path:
    here = Path(__file__).resolve()
    for candidate in [here.parent, *here.parents]:
        if (candidate / "guji_beta_v0").is_dir() and (candidate / "ros2_ws").is_dir():
            return candidate
    return here.parent


PROJECT_ROOT = detect_project_root()
DEFAULT_CAMERA_CONFIG = PROJECT_ROOT / "guji_beta_v0" / "config" / "camera.yaml"
DEFAULT_JOINT_XACRO = (
    PROJECT_ROOT
    / "ros2_ws"
    / "src"
    / "ros2_rm_robot"
    / "dual_rm_description"
    / "dual_rm_65b_description"
    / "urdf"
    / "joint.urdf.xacro"
)


@dataclass
class CaptureResult:
    marker_id: int
    camera_xyz: np.ndarray
    right_base_xyz: np.ndarray
    robot_base_xyz: np.ndarray
    sample_count: int
    camera_std_xyz: np.ndarray
    right_base_std_xyz: np.ndarray
    robot_base_std_xyz: np.ndarray
    tf_source: str
    robot_transform_source: str
    joint_snapshot: Dict[str, float]


@dataclass
class LoadedConfig:
    camera_topic_prefix: str
    camera_frame: str
    right_base_frame: str
    robot_base_frame: str
    flange_frame: str
    aruco_dict: str
    marker_size: float
    target_ids: List[int]
    hand_eye_translation: np.ndarray
    hand_eye_quaternion: np.ndarray
    robot_to_right_t: np.ndarray
    robot_to_right_r: np.ndarray


def load_yaml_camera_config(config_path: Path) -> LoadedConfig:
    import yaml

    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    camera = data.get("camera", {})
    hand_eye = camera.get("hand_eye_right", {})
    aruco = camera.get("aruco", {})
    frames = camera.get("frames", {})

    translation = hand_eye.get("translation", {})
    quaternion = hand_eye.get("quaternion", {})

    target_ids = aruco.get("target_ids", []) or []
    return LoadedConfig(
        camera_topic_prefix=str(camera.get("topic_prefix", "/camera_right")),
        camera_frame=str(frames.get("camera_frame", "camera_right")),
        right_base_frame=str(frames.get("base_frame", "r_base_link1")),
        robot_base_frame=str(frames.get("robot_base_frame", "platform_base_link")),
        flange_frame=str(frames.get("flange_frame", "right_top")),
        aruco_dict=str(aruco.get("dict_type", "DICT_5X5_1000")),
        marker_size=float(aruco.get("marker_size", 0.03)),
        target_ids=[int(v) for v in target_ids],
        hand_eye_translation=np.array(
            [
                float(translation.get("x", 0.0)),
                float(translation.get("y", 0.0)),
                float(translation.get("z", 0.0)),
            ],
            dtype=float,
        ),
        hand_eye_quaternion=np.array(
            [
                float(quaternion.get("x", 0.0)),
                float(quaternion.get("y", 0.0)),
                float(quaternion.get("z", 0.0)),
                float(quaternion.get("w", 1.0)),
            ],
            dtype=float,
        ),
        robot_to_right_t=np.zeros(3, dtype=float),
        robot_to_right_r=np.eye(3, dtype=float),
    )


def load_robot_fixed_joint_xacro(xacro_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    text = xacro_path.read_text(encoding="utf-8")
    pattern = re.compile(
        r'<joint\s+name="r_base_joint1"[\s\S]*?<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"',
        re.MULTILINE,
    )
    match = pattern.search(text)
    if not match:
        raise RuntimeError(f"未能从 {xacro_path} 解析 r_base_joint1")

    xyz = np.array([float(v) for v in match.group(1).split()], dtype=float)
    rpy = [float(v) for v in match.group(2).split()]
    rot = euler_xyz_to_rot(rpy[0], rpy[1], rpy[2])
    return xyz, rot


def euler_xyz_to_rot(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def transform_point(tr: TransformStamped, point_xyz: np.ndarray) -> np.ndarray:
    tx = tr.transform.translation.x
    ty = tr.transform.translation.y
    tz = tr.transform.translation.z
    qx = tr.transform.rotation.x
    qy = tr.transform.rotation.y
    qz = tr.transform.rotation.z
    qw = tr.transform.rotation.w

    # 四元数转旋转矩阵
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rot = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=float,
    )
    trans = np.array([tx, ty, tz], dtype=float)
    return rot @ point_xyz + trans


class ArucoPoseValidationNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("aruco_pose_validation")
        self.args = args

        self.bridge = CvBridge()
        self.last_image: Optional[np.ndarray] = None
        self.last_image_stamp: Optional[Time] = None
        self.last_image_seq: int = 0
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_info_frame: str = ""
        self.joints: Dict[str, float] = {}
        self.capture_timeout_sec = max(float(args.capture_timeout), 0.2)
        self.min_samples = max(int(args.min_samples), 1)
        self.max_samples = max(int(args.max_samples), self.min_samples)
        self._warned_tf_latest_fallback = False
        self._warned_fixed_robot_fallback = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        loaded_cfg = load_yaml_camera_config(args.config)
        self.loaded_cfg = loaded_cfg

        # 优先使用命令行覆盖，否则读取 camera.yaml 中的默认值。
        self.image_topic = args.image_topic or f"{loaded_cfg.camera_topic_prefix}/color/image_raw"
        self.camera_info_topic = args.camera_info_topic or f"{loaded_cfg.camera_topic_prefix}/color/camera_info"
        self.joint_states_topic = args.joint_states_topic or "/joint_states"
        self.camera_frame = args.camera_frame or loaded_cfg.camera_frame
        self.right_base_frame = args.right_base_frame or loaded_cfg.right_base_frame
        self.robot_base_frame = args.robot_base_frame or loaded_cfg.robot_base_frame
        self.marker_size = args.marker_size or loaded_cfg.marker_size
        self.aruco_dict_name = args.aruco_dict or loaded_cfg.aruco_dict
        self.target_ids = loaded_cfg.target_ids
        self.marker_id = args.marker_id if args.marker_id is not None else (self.target_ids[0] if self.target_ids else -1)

        dict_id = ARUCO_DICT_MAP.get(self.aruco_dict_name, cv2.aruco.DICT_5X5_1000)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.create_subscription(JointState, self.joint_states_topic, self.on_joint_states, 10)

        # 来自 joint.urdf.xacro 的固定关节 r_base_joint1: platform_base_link -> r_base_link1
        self.robot_to_right_t, self.robot_to_right_r = load_robot_fixed_joint_xacro(args.joint_xacro)

        self.get_logger().info("程序已启动，等待图像/内参/关节数据就绪...")
        self.get_logger().info(f"配置文件: {args.config}")
        self.get_logger().info(f"xacro 文件: {args.joint_xacro}")
        self.get_logger().info(f"相机话题前缀: {loaded_cfg.camera_topic_prefix}")
        self.get_logger().info(f"相机 frame: {self.camera_frame}")
        self.get_logger().info(f"右臂基座 frame: {self.right_base_frame}")
        self.get_logger().info(f"机器人基座 frame: {self.robot_base_frame}")
        self.get_logger().info(f"ArUco 字典: {self.aruco_dict_name}, marker_size={self.marker_size:.4f}m, target_ids={self.target_ids}")
        self.get_logger().info(
            f"稳健采样参数: timeout={self.capture_timeout_sec:.2f}s min_samples={self.min_samples} max_samples={self.max_samples}"
        )
        self.get_logger().info(
            "手眼标定参数(右臂): "
            f"t=({loaded_cfg.hand_eye_translation[0]:.4f}, {loaded_cfg.hand_eye_translation[1]:.4f}, {loaded_cfg.hand_eye_translation[2]:.4f}), "
            f"q=({loaded_cfg.hand_eye_quaternion[0]:.4f}, {loaded_cfg.hand_eye_quaternion[1]:.4f}, {loaded_cfg.hand_eye_quaternion[2]:.4f}, {loaded_cfg.hand_eye_quaternion[3]:.4f})"
        )

    def on_image(self, msg: Image) -> None:
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            stamp = Time.from_msg(msg.header.stamp)
            self.last_image_stamp = stamp if stamp.nanoseconds > 0 else None
            self.last_image_seq += 1
        except Exception as exc:
            self.get_logger().error(f"图像转换失败: {exc}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_matrix is not None:
            return
        if len(msg.k) != 9:
            return
        self.camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=float)
        self.camera_info_frame = (msg.header.frame_id or "").strip()

    def on_joint_states(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def wait_ready(self, timeout_sec: float = 10.0) -> None:
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_image is not None and self.camera_matrix is not None and self.joints:
                return
            if time.time() - start > timeout_sec:
                raise TimeoutError("等待数据超时：请检查图像、相机内参、关节话题")

    def _camera_frame_candidates(self) -> List[str]:
        candidates: List[str] = []
        for frame in [
            self.camera_frame,
            self.loaded_cfg.camera_frame,
            self.camera_info_frame,
            "camera_right",
        ]:
            value = (frame or "").strip()
            if value and value not in candidates:
                candidates.append(value)
        return candidates

    def _lookup_transform_with_fallback(
        self,
        target_frame: str,
        source_candidates: List[str],
        stamp: Optional[Time],
        timeout_sec: float,
    ) -> Tuple[TransformStamped, str]:
        errors: List[str] = []

        if stamp is not None:
            for source_frame in source_candidates:
                try:
                    tr = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        stamp,
                        timeout=Duration(seconds=timeout_sec),
                    )
                    return tr, f"{target_frame} <- {source_frame} @ image_stamp"
                except TransformException as exc:
                    errors.append(f"{target_frame}<-{source_frame}@image_stamp: {exc}")

        for source_frame in source_candidates:
            try:
                tr = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=timeout_sec),
                )
                if stamp is not None and not self._warned_tf_latest_fallback:
                    self.get_logger().warn(
                        "按图像时间戳查询 TF 失败，已回退使用 latest TF（后续不再重复提示）"
                    )
                    self._warned_tf_latest_fallback = True
                return tr, f"{target_frame} <- {source_frame} @ latest"
            except TransformException as exc:
                errors.append(f"{target_frame}<-{source_frame}@latest: {exc}")

        raise RuntimeError("TF 查询失败: " + " | ".join(errors))

    def _point_right_to_robot(self, p_right: np.ndarray, stamp: Optional[Time]) -> Tuple[np.ndarray, str]:
        try:
            tr, source_desc = self._lookup_transform_with_fallback(
                target_frame=self.robot_base_frame,
                source_candidates=[self.right_base_frame],
                stamp=stamp,
                timeout_sec=0.3,
            )
            return transform_point(tr, p_right), f"tf({source_desc})"
        except RuntimeError as tf_exc:
            can_use_fixed = (
                self.robot_base_frame == XACRO_FIXED_ROBOT_BASE_FRAME
                and self.right_base_frame == XACRO_FIXED_RIGHT_BASE_FRAME
            )
            if not can_use_fixed:
                raise RuntimeError(
                    "right_base -> robot_base TF 查询失败，且当前 frame 组合不匹配固定回退链路；"
                    f"当前为 {self.robot_base_frame} <- {self.right_base_frame}，"
                    f"固定回退仅支持 {XACRO_FIXED_ROBOT_BASE_FRAME} <- {XACRO_FIXED_RIGHT_BASE_FRAME}；"
                    f"原始错误: {tf_exc}"
                )

            rot_right_to_robot = self.robot_to_right_r.T
            trans_right_to_robot = -rot_right_to_robot @ self.robot_to_right_t
            p_robot = rot_right_to_robot @ p_right + trans_right_to_robot
            if not self._warned_fixed_robot_fallback:
                self.get_logger().warn(
                    "right_base -> robot_base TF 查询失败，已回退使用 joint.urdf.xacro 固定参数（后续不再重复提示）"
                )
                self._warned_fixed_robot_fallback = True
            return p_robot, "xacro_fixed(platform_base_link<-r_base_link1)"

    def _detect_marker_tvec(self, image_bgr: np.ndarray) -> Optional[Tuple[int, np.ndarray]]:
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None

        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return None

        flat_ids = ids.flatten().tolist()
        if self.marker_id >= 0:
            if self.marker_id not in flat_ids:
                return None
            idx = flat_ids.index(self.marker_id)
            marker_id = self.marker_id
        else:
            idx = 0
            marker_id = int(flat_ids[0])

        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )
        tvec = tvecs[idx][0].astype(float)
        return marker_id, tvec

    def capture_once(self) -> CaptureResult:
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise RuntimeError("图像或内参未就绪")

        start = time.time()
        last_error = ""
        last_processed_image_seq = -1

        marker_ids: List[int] = []
        camera_samples: List[np.ndarray] = []
        right_base_samples: List[np.ndarray] = []
        robot_base_samples: List[np.ndarray] = []
        tf_sources: List[str] = []
        robot_sources: List[str] = []

        while rclpy.ok() and (time.time() - start) < self.capture_timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_image is None:
                continue

            image_seq = self.last_image_seq
            if image_seq == last_processed_image_seq:
                continue
            last_processed_image_seq = image_seq

            stamp = self.last_image_stamp
            image = self.last_image.copy()
            detection = self._detect_marker_tvec(image)
            if detection is None:
                continue

            marker_id, p_camera = detection

            try:
                tr_cam_to_right, tf_source = self._lookup_transform_with_fallback(
                    target_frame=self.right_base_frame,
                    source_candidates=self._camera_frame_candidates(),
                    stamp=stamp,
                    timeout_sec=0.3,
                )
            except RuntimeError as exc:
                last_error = str(exc)
                continue

            p_right = transform_point(tr_cam_to_right, p_camera)

            try:
                p_robot, robot_source = self._point_right_to_robot(p_right, stamp)
            except RuntimeError as exc:
                last_error = str(exc)
                continue

            marker_ids.append(marker_id)
            camera_samples.append(p_camera)
            right_base_samples.append(p_right)
            robot_base_samples.append(p_robot)
            tf_sources.append(tf_source)
            robot_sources.append(robot_source)

            if len(camera_samples) >= self.max_samples:
                break

        if len(camera_samples) < self.min_samples:
            detail = f" 最近错误: {last_error}" if last_error else ""
            raise RuntimeError(
                f"有效样本不足: {len(camera_samples)} < min_samples={self.min_samples}，"
                f"请检查 marker 可见性、TF 与 frame 配置。{detail}"
            )

        marker_stat = Counter(marker_ids)
        marker_id = int(marker_stat.most_common(1)[0][0])
        if len(marker_stat) > 1:
            self.get_logger().warn(f"本次捕获出现多个 marker_id={dict(marker_stat)}，已使用众数 ID={marker_id}")

        cam_np = np.vstack(camera_samples)
        right_np = np.vstack(right_base_samples)
        robot_np = np.vstack(robot_base_samples)

        camera_xyz = np.median(cam_np, axis=0)
        right_base_xyz = np.median(right_np, axis=0)
        robot_base_xyz = np.median(robot_np, axis=0)

        camera_std_xyz = np.std(cam_np, axis=0)
        right_base_std_xyz = np.std(right_np, axis=0)
        robot_base_std_xyz = np.std(robot_np, axis=0)

        tf_source = Counter(tf_sources).most_common(1)[0][0]
        robot_transform_source = Counter(robot_sources).most_common(1)[0][0]

        joint_snapshot = {
            k: v
            for k, v in sorted(self.joints.items())
            if k.startswith("r_joint")
        }

        return CaptureResult(
            marker_id=marker_id,
            camera_xyz=camera_xyz,
            right_base_xyz=right_base_xyz,
            robot_base_xyz=robot_base_xyz,
            sample_count=len(camera_samples),
            camera_std_xyz=camera_std_xyz,
            right_base_std_xyz=right_base_std_xyz,
            robot_base_std_xyz=robot_base_std_xyz,
            tf_source=tf_source,
            robot_transform_source=robot_transform_source,
            joint_snapshot=joint_snapshot,
        )


def print_capture(label: str, cap: CaptureResult, right_base_frame: str, robot_base_frame: str) -> None:
    print("\n" + "=" * 80)
    print(label)
    print("=" * 80)
    print(f"Aruco ID: {cap.marker_id}")
    print(
        "采样统计: "
        f"samples={cap.sample_count}, camera->right={cap.tf_source}, right->robot={cap.robot_transform_source}"
    )
    print("相机坐标系(camera):")
    print(f"  x={cap.camera_xyz[0]: .6f}, y={cap.camera_xyz[1]: .6f}, z={cap.camera_xyz[2]: .6f} (m)")
    print(
        f"  std=({cap.camera_std_xyz[0]:.6f}, {cap.camera_std_xyz[1]:.6f}, {cap.camera_std_xyz[2]:.6f}) (m)"
    )
    print(f"右臂基坐标系({right_base_frame}):")
    print(f"  x={cap.right_base_xyz[0]: .6f}, y={cap.right_base_xyz[1]: .6f}, z={cap.right_base_xyz[2]: .6f} (m)")
    print(
        f"  std=({cap.right_base_std_xyz[0]:.6f}, {cap.right_base_std_xyz[1]:.6f}, {cap.right_base_std_xyz[2]:.6f}) (m)"
    )
    print(f"机器人基坐标系({robot_base_frame}):")
    print(f"  x={cap.robot_base_xyz[0]: .6f}, y={cap.robot_base_xyz[1]: .6f}, z={cap.robot_base_xyz[2]: .6f} (m)")
    print(
        f"  std=({cap.robot_base_std_xyz[0]:.6f}, {cap.robot_base_std_xyz[1]:.6f}, {cap.robot_base_std_xyz[2]:.6f}) (m)"
    )

    print("右臂关节角(弧度，采样时刻快照):")
    if not cap.joint_snapshot:
        print("  未读取到 r_joint*，请确认 joint_states 命名")
    else:
        for k, v in cap.joint_snapshot.items():
            print(f"  {k}: {v:.6f}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ArUco 两次识别精度验证")
    p.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CAMERA_CONFIG,
        help="camera.yaml 路径，默认读取项目中的配置文件",
    )
    p.add_argument(
        "--joint-xacro",
        type=Path,
        default=DEFAULT_JOINT_XACRO,
        help="joint.urdf.xacro 路径，默认读取项目中的固定关节定义",
    )
    p.add_argument("--image-topic", default=None, help="图像话题，留空则读取 camera.yaml")
    p.add_argument("--camera-info-topic", default=None, help="相机内参话题，留空则读取 camera.yaml")
    p.add_argument("--joint-states-topic", default=None, help="关节状态话题，默认 /joint_states")
    p.add_argument("--camera-frame", default=None, help="相机 frame，留空则读取 camera.yaml")
    p.add_argument("--right-base-frame", default=None, help="右臂基座 frame，留空则读取 camera.yaml")
    p.add_argument("--robot-base-frame", default=None, help="机器人基座 frame，留空则读取 camera.yaml")
    p.add_argument("--aruco-dict", default=None, help="ArUco 字典，留空则读取 camera.yaml")
    p.add_argument("--marker-size", type=float, default=None, help="Aruco 边长（米），留空则读取 camera.yaml")
    p.add_argument("--marker-id", type=int, default=None, help="目标 ID，留空则使用 camera.yaml 的第一个 target_id")
    p.add_argument(
        "--capture-timeout",
        type=float,
        default=2.0,
        help="每次识别采样窗口时长(秒)，在窗口内采集多帧并做稳健统计",
    )
    p.add_argument(
        "--min-samples",
        type=int,
        default=6,
        help="每次识别最少有效样本数，不足会报错",
    )
    p.add_argument(
        "--max-samples",
        type=int,
        default=25,
        help="每次识别最多采样数，达到后提前结束窗口",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ArucoPoseValidationNode(args)

    try:
        node.wait_ready(timeout_sec=15.0)
        print("\n数据已就绪。请保持右臂不动。")
        input("按回车执行【第一次识别】...")

        cap1 = node.capture_once()
        print_capture("第一次识别结果", cap1, node.right_base_frame, node.robot_base_frame)

        input("\n请物理平移 ArUco 后，按回车执行【第二次识别】...")
        cap2 = node.capture_once()
        print_capture("第二次识别结果", cap2, node.right_base_frame, node.robot_base_frame)

        delta_robot = cap2.robot_base_xyz - cap1.robot_base_xyz
        dist = float(np.linalg.norm(delta_robot))

        print("\n" + "=" * 80)
        print("机器人基坐标系下平移差")
        print("=" * 80)
        print(
            f"dx={delta_robot[0]: .6f}, dy={delta_robot[1]: .6f}, dz={delta_robot[2]: .6f} (m), "
            f"distance={dist:.6f} m"
        )

    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
