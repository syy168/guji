#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
09 - Eye-in-Hand 手眼标定（棋盘格标定板固定于环境 + 机械臂末端持相机）

使用 OpenCV calibrateHandEye，采集多组：
  - 法兰在基座系下位姿（来自 rm_driver/udp_arm_position）
  - 标定板相对相机的位姿（棋盘角点 + solvePnP）

结果写入 rm_pick_place_tutorial/config/camera.yaml 的 hand_eye_right，
与 tf_broadcaster 约定一致：静态 TF 父 right_top、子 camera_right，
即 p_flange = R * p_camera + t（平移+四元数为相机相对法兰的外参）。

前置：RealSense 彩色流、CameraInfo、右臂驱动发布 udp_arm_position；
标定过程中无需已正确的 hand_eye（可不运行 tf_broadcaster，或运行与否均可），
但须保证标定板固定不动。
"""

from __future__ import annotations

import argparse
import math
import shutil
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import cv_bridge
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# OpenCV 手眼算法常量（不同版本可能略有差异）
def _hand_eye_methods():
    ms = []
    for name in ('CALIB_HAND_EYE_TSAI', 'CALIB_HAND_EYE_PARK', 'CALIB_HAND_EYE_HORAUD',
                 'CALIB_HAND_EYE_ANDREFF', 'CALIB_HAND_EYE_DANIILIDIS'):
        if hasattr(cv2, name):
            ms.append(getattr(cv2, name))
    return ms if ms else [0]


def build_board_object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
    """
    生成棋盘格角点 3D 坐标（单位: 米）。
    cols: 每行内角点数（棋盘横向）
    rows: 每列内角点数（棋盘纵向）
    """
    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid.astype(np.float32) * float(square_size)
    return objp


def _script_dir() -> Path:
    return Path(__file__).resolve().parent


def pose_to_Rt(pose: Pose) -> Tuple[np.ndarray, np.ndarray]:
    """法兰在基座系：p_base = R @ p_flange + t。返回 R(3x3), t(3x1)。"""
    x, y, z = pose.position.x, pose.position.y, pose.position.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
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
    R = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], dtype=np.float64)
    t = np.array([[x], [y], [z]], dtype=np.float64)
    return R, t


def R_to_quaternion_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    """旋转矩阵 -> 四元数 x,y,z,w（与 camera.yaml / geometry_msgs 一致）。"""
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


class HandEyeCalibrateNode(Node):
    def __init__(self, args) -> None:
        super().__init__('hand_eye_calibrate_09')
        self._args = args
        self._bridge = cv_bridge.CvBridge()

        cfg_path = Path(args.config).expanduser().resolve()
        with open(cfg_path, 'r', encoding='utf-8') as f:
            self._full_cfg = yaml.safe_load(f)
        cam = self._full_cfg['camera']
        self._topic_prefix = cam['topic_prefix'].rstrip('/')
        board_cfg = cam.get('chessboard', {})
        self._pattern_cols = int(
            args.pattern_cols if args.pattern_cols is not None else board_cfg.get('pattern_cols', 9)
        )
        self._pattern_rows = int(
            args.pattern_rows if args.pattern_rows is not None else board_cfg.get('pattern_rows', 6)
        )
        self._square_size = float(
            args.square_size if args.square_size is not None else board_cfg.get('square_size', 0.025)
        )
        self._board_object_points = build_board_object_points(
            self._pattern_cols,
            self._pattern_rows,
            self._square_size,
        )
        self._criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        self._arm_ns = args.arm_ns.strip('/')
        self._min_samples = int(args.min_samples)

        self._latest_image: Optional[np.ndarray] = None
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._latest_gripper_pose: Optional[Pose] = None

        self._Rgrip2base: List[np.ndarray] = []
        self._tgrip2base: List[np.ndarray] = []
        self._Rtarget2cam: List[np.ndarray] = []
        self._ttarget2cam: List[np.ndarray] = []

        self.create_subscription(
            Image,
            f'{self._topic_prefix}/color/image_raw',
            self._on_image,
            10,
        )
        self.create_subscription(
            CameraInfo,
            f'{self._topic_prefix}/color/camera_info',
            self._on_camera_info,
            10,
        )
        self.create_subscription(
            Pose,
            f'/{self._arm_ns}/rm_driver/udp_arm_position',
            self._on_gripper_pose,
            10,
        )

        self.get_logger().info(f'配置文件: {cfg_path}')
        self.get_logger().info(f'图像: {self._topic_prefix}/color/image_raw')
        self.get_logger().info(f'法兰位姿: /{self._arm_ns}/rm_driver/udp_arm_position')
        self.get_logger().info(
            f'棋盘格内角点: cols={self._pattern_cols}, rows={self._pattern_rows}, '
            f'方格边长={self._square_size}m'
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
        self.get_logger().info('CameraInfo 已就绪')

    def _on_gripper_pose(self, msg: Pose) -> None:
        self._latest_gripper_pose = msg

    def _detect_board_rt(self, image: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        if self._camera_matrix is None or self._dist_coeffs is None:
            return None
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern_size = (self._pattern_cols, self._pattern_rows)
        found = False
        corners = None
        if hasattr(cv2, 'findChessboardCornersSB'):
            found, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    self._criteria,
                )
        if not found or corners is None:
            return None
        ok, rvec, tvec = cv2.solvePnP(
            self._board_object_points,
            corners,
            self._camera_matrix,
            self._dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec, corners

    def capture_sample(self) -> bool:
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._latest_image is None:
            self.get_logger().error('无图像')
            return False
        if self._latest_gripper_pose is None:
            self.get_logger().error('无法获取法兰位姿（udp_arm_position）')
            return False
        rt = self._detect_board_rt(self._latest_image)
        if rt is None:
            self.get_logger().error('当前帧未检测到棋盘格角点')
            return False
        R_tc, t_tc, _ = rt
        R_gb, t_gb = pose_to_Rt(self._latest_gripper_pose)
        self._Rgrip2base.append(R_gb)
        self._tgrip2base.append(t_gb)
        self._Rtarget2cam.append(R_tc)
        self._ttarget2cam.append(t_tc)
        n = len(self._Rgrip2base)
        self.get_logger().info(f'已采集样本 #{n}（请更换姿态后继续，建议 ≥{self._min_samples} 组）')
        return True

    def undo_last(self) -> None:
        if not self._Rgrip2base:
            self.get_logger().warn('无样本可撤销')
            return
        for lst in (self._Rgrip2base, self._tgrip2base, self._Rtarget2cam, self._ttarget2cam):
            lst.pop()
        self.get_logger().info(f'已撤销，剩余 {len(self._Rgrip2base)} 组')

    def run_calibration(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        n = len(self._Rgrip2base)
        if n < 3:
            self.get_logger().error('至少需要 3 组样本（建议 10 组以上、姿态差异大）')
            return None
        methods = _hand_eye_methods()
        last_err = None
        for method in methods:
            try:
                R_cg, t_cg = cv2.calibrateHandEye(
                    self._Rgrip2base,
                    self._tgrip2base,
                    self._Rtarget2cam,
                    self._ttarget2cam,
                    method=method,
                )
                self.get_logger().info(f'calibrateHandEye 成功 (method={method})')
                return R_cg, t_cg
            except cv2.error as e:
                last_err = e
                continue
        self.get_logger().error(f'calibrateHandEye 失败: {last_err}')
        return None

    def save_camera_yaml(self, R_cg: np.ndarray, t_cg: np.ndarray) -> None:
        """R_cg, t_cg: 相机相对法兰，p_flange = R @ p_cam + t（与 tf_broadcaster 一致）。"""
        tv = np.asarray(t_cg, dtype=np.float64).reshape(3)
        tx, ty, tz = float(tv[0]), float(tv[1]), float(tv[2])
        qx, qy, qz, qw = R_to_quaternion_xyzw(R_cg)
        cam = self._full_cfg.setdefault('camera', {})
        cam['hand_eye_right'] = {
            'translation': {'x': tx, 'y': ty, 'z': tz},
            'quaternion': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
        }
        out_path = Path(self._args.config).expanduser().resolve()
        if self._args.backup:
            bak = out_path.with_suffix('.yaml.bak')
            try:
                shutil.copy2(out_path, bak)
                self.get_logger().info(f'已备份: {bak}')
            except Exception as e:
                self.get_logger().warn(f'备份失败: {e}')
        with open(out_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(
                self._full_cfg,
                f,
                allow_unicode=True,
                default_flow_style=False,
                sort_keys=False,
            )
        self.get_logger().info(f'已写入 hand_eye_right -> {out_path}')
        self.get_logger().info(
            f'  translation: ({tx:.5f}, {ty:.5f}, {tz:.5f}) m\n'
            f'  quaternion(xyzw): ({qx:.5f}, {qy:.5f}, {qz:.5f}, {qw:.5f})'
        )
        self.get_logger().info('请重启 tf_broadcaster（及依赖 TF 的节点）使新标定生效。')

    def draw_overlay(self, img: np.ndarray) -> np.ndarray:
        vis = img.copy()
        n = len(self._Rgrip2base)
        cv2.putText(
            vis, f'Samples: {n} (min {self._min_samples})', (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
        )
        cv2.putText(
            vis, 'c=capture u=undo w=calib+save q=quit', (10, 56),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
        )
        if self._camera_matrix is not None and self._latest_image is not None:
            rt = self._detect_board_rt(self._latest_image)
            if rt is not None:
                R_tc, tvec, corners = rt
                cv2.drawChessboardCorners(
                    vis,
                    (self._pattern_cols, self._pattern_rows),
                    corners,
                    True,
                )
                rvec, _ = cv2.Rodrigues(R_tc)
                try:
                    cv2.drawFrameAxes(
                        vis,
                        self._camera_matrix,
                        self._dist_coeffs,
                        rvec,
                        tvec,
                        max(self._square_size * 2.0, 0.02),
                        3,
                    )
                except (AttributeError, cv2.error):
                    cv2.drawFrameAxes(
                        vis,
                        self._camera_matrix,
                        self._dist_coeffs,
                        rvec,
                        tvec,
                        max(self._square_size * 2.0, 0.02),
                    )
        return vis

    def run_gui_loop(self) -> None:
        win = '09 Hand-Eye Calibrate'
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._latest_image is None:
                time.sleep(0.05)
                continue
            vis = self.draw_overlay(self._latest_image)
            cv2.imshow(win, vis)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                break
            if key == ord('c'):
                self.capture_sample()
            elif key == ord('u'):
                self.undo_last()
            elif key == ord('w'):
                if len(self._Rgrip2base) < self._min_samples:
                    self.get_logger().warn(
                        f'样本数 {len(self._Rgrip2base)} < 建议最小 {self._min_samples}，仍尝试计算…'
                    )
                out = self.run_calibration()
                if out is not None:
                    R, t = out
                    if self._args.dry_run:
                        self.get_logger().info('dry-run：不写入文件')
                        qx, qy, qz, qw = R_to_quaternion_xyzw(R)
                        self.get_logger().info(
                            f't=({t[0,0]:.5f},{t[1,0]:.5f},{t[2,0]:.5f}) '
                            f'q=({qx:.5f},{qy:.5f},{qz:.5f},{qw:.5f})'
                        )
                    else:
                        self.save_camera_yaml(R, t)
                else:
                    self.get_logger().error('标定失败，未写入')
        cv2.destroyAllWindows()

    def run_line_loop(self) -> None:
        self.get_logger().info('无界面模式：输入 c=采集 u=撤销 w=计算并保存 q=退出')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                line = input('命令 [c/u/w/q]: ').strip().lower()
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            if not line:
                continue
            ch = line[0]
            if ch == 'q':
                break
            if ch == 'c':
                self.capture_sample()
            elif ch == 'u':
                self.undo_last()
            elif ch == 'w':
                out = self.run_calibration()
                if out is not None:
                    R, t = out
                    if self._args.dry_run:
                        self.get_logger().info('dry-run')
                    else:
                        self.save_camera_yaml(R, t)
                else:
                    self.get_logger().error('标定失败')


def parse_args():
    default_cfg = _script_dir().parent / 'config' / 'camera.yaml'
    p = argparse.ArgumentParser(description='09 Eye-in-Hand 手眼标定（棋盘格标定板）')
    p.add_argument(
        '--config',
        default=str(default_cfg),
        help='camera.yaml 路径（读取并写回 hand_eye_right）',
    )
    p.add_argument('--pattern-cols', type=int, default=None, help='棋盘格每行内角点数（如 9）')
    p.add_argument('--pattern-rows', type=int, default=None, help='棋盘格每列内角点数（如 6）')
    p.add_argument('--square-size', type=float, default=None, help='棋盘格单格边长（米）')
    p.add_argument('--arm-ns', default='right_arm_controller', help='右臂 namespace')
    p.add_argument('--min-samples', type=int, default=10, help='建议最少采集组数')
    p.add_argument('--no-gui', action='store_true', help='无 OpenCV 窗口，改用终端命令')
    p.add_argument('--dry-run', action='store_true', help='w 时只计算不写入 yaml')
    p.add_argument('--backup', action='store_true', help='写入前备份 .yaml.bak')
    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = HandEyeCalibrateNode(args)
    try:
        t0 = time.time()
        while node._camera_matrix is None and time.time() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node._camera_matrix is None:
            node.get_logger().error('30s 内未收到 CameraInfo，退出')
            return
        if args.no_gui:
            node.run_line_loop()
        else:
            node.run_gui_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
