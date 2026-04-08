# 双臂取放料系统 — 完整实现文档

> 本文档是 `dualeft_arm_controller_pick_place.py` 及相关视觉系统的完整技术参考手册，
> 参照 `Warehouse_handling_robot` 项目的文档风格编写，涵盖架构设计、
> 硬件连接、软件流程、配置文件说明、启动检查与调试方法。

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件组成与连接](#2-硬件组成与连接)
3. [系统架构](#3-系统架构)
4. [软件模块详解](#4-软件模块详解)
5. [相机与视觉识别系统](#5-相机与视觉识别系统)
6. [TF 坐标变换体系](#6-tf-坐标变换体系)
7. [配置文件说明](#7-配置文件说明)
8. [启动与运行](#8-启动与运行)
9. [启动检查与验证流程](#9-启动检查与验证流程)
10. [手眼标定指南](#10-手眼标定指南)
11. [完整取放料流程](#11-完整取放料流程)
12. [调试与故障排查](#12-调试与故障排查)
13. [扩展开发指南](#13-扩展开发指南)

---

## 1. 系统概述

### 1.1 项目定位

本项目是一套面向 **睿尔曼 RM65 双臂机械臂** 的协同取放料控制系统，运行于 **ROS2 Foxy** 环境，部署在 **Jetson（ARM64/aarch64）** 或 x86_64 PC 上。项目目标：

- 实现双臂协同完成"识别 → 抓取 → 搬运 → 放置"全流程
- 通过 RealSense D435 相机 + ArUco 标记实现工件视觉定位
- 以 YAML 配置文件管理所有点位、参数和坐标系关系
- 提供完整的启动检查机制，便于实际部署验证

### 1.2 与 Warehouse 项目的对比

| 对比项 | Warehouse_handling_robot | dualeft_arm_controller_pick_place |
|--------|--------------------------|---------------------|
| 导航系统 | Woosh AGV + SLAM | 本项目不含导航（留待后续扩展） |
| 升降机控制 | 有（多层货架） | 无（单层工作台） |
| 机械臂数量 | 双臂（睿尔曼 RM65） | 双臂（睿尔曼 RM65） |
| 相机数量 | 3 台（左右手 + 头部） | 1 台（仅右臂，D435） |
| 视觉识别 | ArUco + YOLO | ArUco（当前）+ YOLO（扩展） |
| 坐标系管理 | tf_transform（C++） | tf_broadcaster（Python） |
| 配置管理 | launch 混合 | YAML 独立配置 |
| 标定方式 | 手动标定参数 | 手眼标定结果写入 YAML |
| 主控制器 | body_handling_action | dualeft_arm_controller_pick_place |

### 1.3 硬件配置

| 设备 | 型号 | 数量 | 说明 |
|------|------|------|------|
| 机械臂 | 睿尔曼 RM65 | 2 | 左臂 + 右臂 |
| 相机 | Intel RealSense D435 | 1 | 仅右臂末端（手眼安装） |
| 计算机 | Jetson / PC | 1 | 运行 ROS2 |
| 网络 | 千兆有线 | - | 计算机与机械臂通信 |

**网络 IP 分配：**

| 节点 | IP 地址 |
|------|---------|
| 计算机（Jetson） | `192.168.151.119` |
| 左臂 | `192.168.150.111` |
| 右臂 | `192.168.150.112` |

---

## 2. 硬件组成与连接

### 2.1 RealSense D435 相机连接

RealSense D435 通过 **USB 3.0** 连接到 Jetson/PC，相机固件由 `realsense2_camera` ROS2 功能包驱动。

**物理连接示意：**

```
Jetson/PC (USB 3.0 端口)
    │
    └── USB 3.0 线 ──► Intel RealSense D435
                              │
                              ├── RGB 彩色图像（1280×720 @ 30fps）
                              ├── 深度图像（1280×720 @ 30fps）
                              └── 红外左右纹理（1280×720 @ 30fps）
```

**D435 技术参数（与本项目相关）：**

| 参数 | 值 |
|------|-----|
| RGB 分辨率 | 1280×720（默认 640×480） |
| 深度范围 | 0.2m ~ 10m |
| RGB 视场角 | 69° × 42° |
| 深度视场角 | 85° × 58° |
| RGB 焦距（640×480） | fx=615, fy=615, cx=320, cy=240 |
| USB 接口 | USB 3.0 |

### 2.2 相机在机械臂上的安装

D435 安装于右臂末端法兰侧面（手眼配置，Eye-in-Hand），其坐标系相对于法兰的偏移由手眼标定测定。

**安装位置参考值（camera.yaml 默认值）：**

```yaml
hand_eye_right:
  translation:
    x: 0.0850   # 相机朝前 8.5cm
    y: -0.0400  # 相机偏左 4cm
    z: 0.0100   # 相机朝上 1cm
  quaternion:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

> 以上为初始估计值，实际值需要通过手眼标定精确测定。标定方法详见第十章。

### 2.3 机械臂与计算机网络连接

```
Jetson/PC
    │
    ├── eth0 (有线网卡) ──► 路由器 ──► 左臂（192.168.150.111:8080）
    │                                    │
    │                                    └──► 右臂（192.168.150.112:8080）
    │
    └── USB 3.0 ──► RealSense D435
```

---

## 3. 系统架构

### 3.1 ROS2 节点拓扑图

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                            ROS2 通信拓扑                                     │
└──────────────────────────────────────────────────────────────────────────────┘

┌─────────────────┐    TCP (8080/8089)    ┌─────────────────┐
│   rm_driver     │◄─────────────────────►│   rm_driver     │
│  (left_arm_controller ns)     │                       │  (right_arm_controller ns)     │
└────────┬────────┘                       └────────┬────────┘
         │                                         │
         │  /left_arm_controller/joint_states                    │  /right_arm_controller/joint_states
         │  /left_arm_controller/rm_driver/...                   │  /right_arm_controller/rm_driver/...
         ▼                                         ▼
┌─────────────────────────────────────────────────────────────────┐
│              dualeft_arm_controller_pick_place (主控制器)                      │
│                                                                  │
│  - 订阅左右臂关节状态                                             │
│  - 发布 MoveJ/MoveL/Gripper/Force 控制指令                        │
│  - 调用 /right_arm_controller/detect_aruco Service                          │
│  - 监听 TF 变换                                                  │
│  - 执行完整取放料状态机                                           │
└────────────────────────────┬────────────────────────────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌──────────────┐  ┌─────────────────┐  ┌─────────────────────┐
│camera_bridge │  │tf_broadcaster   │  │  aruco_detector     │
│ (诊断节点)    │  │ (TF 广播节点)   │  │   (视觉识别节点)     │
└──────┬───────┘  └────────┬────────┘  └──────────┬──────────┘
       │                    │                      │
       │                    │  static TF           │  Service
       │                    │  right_base──►       │  /right_arm_controller/detect_aruco
       ▼                    ▼  camera_right        ▼
┌─────────────────────────────────────────────────────────────────┐
│                  realsense2_camera (D435 驱动)                   │
│                                                                  │
│  /camera_right/color/image_raw                                   │
│  /camera_right/color/camera_info                                │
│  /camera_right/aligned_depth_to_color/image_raw                 │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 数据流

```
[RealSense D435]
        │
        ├── RGB 图像 ──────► [camera_bridge] ──诊断──► [终端日志]
        │                              │
        │                              └──► /right_arm_controller/get_camera_status (Service)
        │
        ├── RGB 图像 ──────► [aruco_detector]
        │                         │
        │                         ├── ArUco 检测（OpenCV）
        │                         │
        │                         ├── TF 发布
        │                         │   camera_right → target_right_camera
        │                         │
        │                         └──► /right_arm_controller/detect_aruco (Service)
        │                                   │
        │                                   └───► [dualeft_arm_controller_pick_place]
        │                                              │
        ├── 深度对齐图 ──► [camera_bridge] ──诊断──► [终端日志]
        │
        └── CameraInfo ──► [aruco_detector]（PnP 姿态估计用）

[dualeft_arm_controller_pick_place]
        │
        ├── 订阅 /right_arm_controller/joint_states
        ├── 订阅 /left_arm_controller/joint_states
        │
        ├── 发布 /right_arm_controller/rm_driver/movej_cmd
        ├── 发布 /right_arm_controller/rm_driver/movel_cmd
        ├── 发布 /right_arm_controller/rm_driver/set_gripper_*_cmd
        ├── 发布 /right_arm_controller/rm_driver/force_position_move_pose_cmd
        │
        ├── TF 监听：
        │   target_right_camera → right_base
        │
        └── 接收 /right_arm_controller/detect_aruco 响应
```

### 3.3 模块依赖关系

```
dualeft_arm_controller_pick_place.py
  ├── rm_ros_interfaces（Movej/Movel/Gripper/Force 消息）
  ├── tf2_ros（Buffer + TransformListener）
  ├── geometry_msgs（Pose）
  ├── sensor_msgs（JointState）
  ├── guji.srv（DetectAruco）
  ├── yaml（配置加载）
  └── config/*.yaml（点位、相机、系统参数）
       │
       ├── poses.yaml ───► 所有点位定义
       ├── camera.yaml ───► 相机参数 + 手眼标定 + ArUco 参数
       └── system.yaml ───► 运动参数 + 夹爪参数 + 视觉参数

aruco_detector.py
  ├── cv_bridge（ROS ↔ OpenCV 图像转换）
  ├── cv2.aruco（ArUco 检测）
  ├── tf2_ros（TransformBroadcaster + Buffer + TransformListener）
  ├── sensor_msgs（Image, CameraInfo）
  ├── geometry_msgs（Pose, TransformStamped）
  └── config/*.yaml

camera_bridge.py
  ├── sensor_msgs（Image, CameraInfo）
  ├── std_srvs（Trigger）
  ├── numpy（深度图像处理）
  └── config/*.yaml

tf_broadcaster.py
  ├── tf2_ros（StaticTransformBroadcaster）
  ├── sensor_msgs（JointState）
  ├── geometry_msgs（TransformStamped）
  └── config/*.yaml
```

---

## 4. 软件模块详解

### 4.1 主控制器 `dualeft_arm_controller_pick_place.py`

**类名：** `DualArmPickPlaceController`

**职责：** 协调双臂完成取放料全流程，是整个系统的中央调度节点。

#### 4.1.1 初始化流程

```
DualArmPickPlaceController.__init__()
    │
    ├── 设置 namespace（默认 left_arm_controller / right_arm_controller）
    │
    ├── 创建左右臂订阅者
    │   ├── left_joint_sub   → /left_arm_controller/joint_states
    │   └── right_joint_sub  → /right_arm_controller/joint_states
    │
    ├── 创建左右臂发布者
    │   ├── left/right_movej_pub      → /{ns}/rm_driver/movej_cmd
    │   ├── left/right_movel_pub      → /{ns}/rm_driver/movel_cmd
    │   ├── left/right_gripper_set_pub → /{ns}/rm_driver/set_gripper_position_cmd
    │   ├── left/right_gripper_pick_pub → /{ns}/rm_driver/set_gripper_pick_cmd
    │   ├── left/right_hand_angle_pub  → /{ns}/rm_driver/set_hand_angle_cmd
    │   └── left/right_force_pub       → /{ns}/rm_driver/force_position_move_pose_cmd
    │
    ├── _load_config() 加载 YAML
    │   ├── poses.yaml  → self.POSES
    │   ├── system.yaml → self.SYS
    │   └── camera.yaml → self.CAM
    │
    ├── TF 监听器
    │   └── self._tf_buffer + self._tf_listener
    │
    └── ArUco Service Client
        └── self._aruco_client → /right_arm_controller/detect_aruco
```

#### 4.1.2 启动检查机制

`startup_checks()` 方法在 `run()` 执行前逐项验证依赖：

| 检查项 | 方法 | 超时 | 失败处理 |
|--------|------|------|---------|
| 关节状态数据 | `_check_joint_states()` | 10s | 阻塞等待 |
| ArUco 视觉服务 | `_check_aruco_service()` | 0s（立即） | 降级 Mock |
| TF 变换树 | `_check_tf_tree()` | 0s（立即） | 警告，继续 |
| 相机状态 | `_check_camera_status()` | 3s | 警告，跳过 |

**检查输出格式：**

```
========================================
  执行启动检查...
========================================
  [检查1] 等待关节状态数据...
    左臂: 6 joints, 右臂: 6 joints
  [   OK   ] 关节状态数据
  [   OK   ] ArUco 视觉服务
  [   OK   ] TF 变换树
  [   OK   ] 相机状态
========================================
  所有检查通过，系统就绪！
========================================
```

#### 4.1.3 核心控制方法

| 方法 | 功能 | 关键参数 |
|------|------|---------|
| `movej(arm, joints, speed, block)` | 关节运动（弧线） | joints=[6], speed=1~100, block=是否等待完成 |
| `movel(arm, pose, speed, block)` | 直线运动 | pose=geometry_msgs/Pose |
| `gripper_set(arm, pos, block, timeout)` | 位置控制夹爪 | pos=1~1000 |
| `gripper_pick(arm, speed, force, block, timeout)` | 力控夹取 | speed=1~1000, force=1~1000 |
| `force_position_move(arm, pose, ...)` | 力位混合运动 | direction=0~5(X/Y/Z/RX/RY/RZ) |
| `detect_arcode(arm, code)` | ArUco 视觉识别 | code='ARcode11'等，返回位姿字典 |
| `movej_both(left, right, speed, block)` | 双臂协同运动 | - |

#### 4.1.4 状态机设计

`run()` 方法按固定顺序执行各状态：

```
run()
  │
  ├── startup_checks()  ──► 验证所有依赖
  │
  ├── go_initial_position()      ──► [状态1] 双臂回初始位
  │      调用: movej_both(POSES['left']['initial'], POSES['right']['initial'])
  │
  ├── go_recognize_position()    ──► [状态2] 进入识别位（相机朝下）
  │
  ├── recognize_arcode()          ──► [状态3] 识别 ARcode12
  │      调用: detect_arcode('right', 'ARcode12')
  │
  ├── pre_grasp_alignment()       ──► [状态4] 预抓取对齐（推工件）
  │
  ├── pick_workpiece()            ──► [状态5] 取料（11步核心逻辑）
  │
  ├── go_place_position()         ──► [状态6] 移动到放料位置
  │
  ├── recognize_arcode21()        ──► [状态7] 识别 ARcode21
  │      调用: detect_arcode('right', 'ARcode21')
  │
  ├── place_workpiece()           ──► [状态8] 放料（力位混合）
  │
  └── return_to_pick_origin()    ──► [状态9] 回初始位
```

### 4.2 相机诊断节点 `camera_bridge.py`

**类名：** `CameraBridge`

**职责：** 监控 RealSense D435 数据流，验证图像帧率、深度有效性和内参完整性。

#### 4.2.1 订阅的话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/camera_right/color/image_raw` | `sensor_msgs/Image` | RGB 彩色图 |
| `/camera_right/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 对齐后的深度图 |
| `/camera_right/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |

#### 4.2.2 诊断检查项

| 检查项 | 方法 | 阈值 | 说明 |
|--------|------|------|------|
| Color 帧率 | `_calculate_fps()` | ≥ 15Hz | 深度图同理 |
| Color 连通性 | 时间戳差值 | ≤ 3s | 超过 3s 无数据告警 |
| 深度有效像素比 | `np.count_nonzero()` | ≥ 50% | 深度值 > 0 的像素比例 |
| CameraInfo 完整性 | K 矩阵 9 个，D ≥ 5 个 | 全部 | 内参矩阵维度检查 |

#### 4.2.3 定时诊断日志

每 5 秒（可配置）输出一次诊断摘要：

```
[INFO] --- 相机诊断 #3 (运行 15s) ---
[INFO]   [OK  ] [OK] Color:30Hz DepthValid:78% | All checks passed

[WARN] --- 相机诊断 #5 (运行 25s) ---
[WARN]   [WARN] Color:8Hz DepthValid:31% | [WARN] Color 帧率过低: 8.0Hz (期望≥30Hz) | [WARN] 深度有效像素比例过低: 31% (期望≥50%)
[WARN]   相机状态异常！请检查:
[WARN]     1. realsense2_camera 节点是否运行: ros2 node list
[WARN]     2. 相机话题是否发布: ros2 topic list | grep camera_right
[WARN]     3. 相机连接线是否松动
[WARN]     4. 序列号是否配置正确
```

#### 4.2.4 Service 接口

**`/right_arm_controller/get_camera_status`**（类型：`std_srvs/Trigger`）

调用示例：

```bash
ros2 service call /right_arm_controller/get_camera_status std_srvs/srv/Trigger "{}"
```

返回：
- `success: true` → 相机正常
- `success: false` → 存在异常，`message` 字段包含详细信息

### 4.3 手眼标定 TF 广播节点 `tf_broadcaster.py`

**类名：** `HandEyeTFBroadcaster`

**职责：** 发布相机相对于机械臂基座的静态 TF 变换，建立完整的坐标链。

#### 4.3.1 发布的 TF

| 父坐标系 | 子坐标系 | 类型 | 说明 |
|---------|---------|------|------|
| `right_base` | `camera_right` | 静态（一次性广播） | 手眼标定结果 |

> 注意：`right_base → right_top`（法兰 TF）由 `rm_driver` 节点通过关节状态计算后发布，本节点不重复广播。

#### 4.3.2 默认标定参数

当 `camera.yaml` 未找到时，使用以下默认值（相机垂直朝下安装）：

```python
cam_tx = 0.085   # 相机朝前 8.5cm
cam_ty = -0.040  # 相机偏左 4cm
cam_tz = 0.010   # 相机朝上 1cm
# 旋转：绕 Y 轴 90°（相机朝下）
yaw = π/2
qw = cos(yaw/2), qy = sin(yaw/2)
qx = 0, qz = 0
```

> **重要：** 这些值是初始估计，**必须通过手眼标定获得精确值**后填入 `camera.yaml`。详见第十章。

### 4.4 ArUco 检测节点 `aruco_detector.py`

**类名：** `ArucoDetector`

**职责：** 识别 ArUco 标记，计算 6DOF 位姿，提供 Service 接口。

#### 4.4.1 支持的 ArUco 字典

| 字典名称 | OpenCV 常量 | 标记数量 |
|---------|-----------|---------|
| DICT_4X4_50/100/250/1000 | `cv2.aruco.DICT_4X4_*` | 50~1000 |
| DICT_5X5_50/100/250/1000 | `cv2.aruco.DICT_5X5_*` | 50~1000 |
| DICT_6X6_50/100/250/1000 | `cv2.aruco.DICT_6X6_*` | 50~1000 |
| DICT_7X7_50/100/250/1000 | `cv2.aruco.DICT_7X7_*` | 50~1000 |

本项目默认使用 **DICT_5X5_1000**（与 Warehouse 项目一致）。

#### 4.4.2 检测流程（Service 回调）

```
_detect_callback(marker_id, timeout)
    │
    ├── 清除历史稳定性数据
    │
    ├── 等待图像帧到达（超时 5s）
    │
    ├── while (time < timeout):
    │     ├── spin_once() 获取最新图像
    │     ├── 灰度化 cvtColor(..., BGR2GRAY)
    │     ├── aruco_detector.detectMarkers(gray)
    │     │     │
    │     │     └──► corners[], ids[], rejected[]
    │     │
    │     ├── if ids 为空: 继续循环
    │     │
    │     ├── 查找目标 marker_id
    │     │
    │     ├── PnP 姿态估计
    │     │   cv2.aruco.estimatePoseSingleMarkers()
    │     │     │
    │     │     └──► rvec[3], tvec[3]（相机坐标系下）
    │     │
    │     ├── 角度稳定性过滤
    │     │   ├── 滑动窗口累积历史角度
    │     │   ├── max_diff < stability_threshold ?
    │     │   │     (当前帧与窗口最大偏差)
    │     │   └──► 通过 → 平滑 → 发布 TF
    │     │         未通过 → 清除窗口 → 继续检测
    │     │
    │     ├── 发布 TF: camera_right → target_right_camera
    │     │
    │     ├── TF 监听: target_right_camera → right_base
    │     │     └──► 获取基坐标系下位姿
    │     │
    │     └── return found=True, pose, header
    │
    └── return found=False（超时未检测到）
```

#### 4.4.3 角度稳定性过滤详解

单帧 ArUco 检测结果容易受噪声影响（机械臂轻微晃动、相机曝光变化等），因此本节点引入滑动窗口稳定性过滤：

```
滑动窗口（history_size=10）
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ a1  │ a2  │ a3  │ a4  │ a5  │ a6  │ a7  │ a8  │ a9  │ a10 │
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
                                              ↑
                                         当前帧 angle

稳定性判断：
  max_diff = max(|angle - a_i|) for i in 1..9
  is_stable = (max_diff < stability_threshold)

角度平滑：
  smoothed_angle = smoothing_factor * angle[-2] + (1-smoothing_factor) * angle[-1]
                 = 0.8 * a9 + 0.2 * a10
```

#### 4.4.4 Service 定义

**文件：** `guji/srv/DetectAruco.srv`

```yaml
# 请求
int32 marker_id       # ArUco 标记 ID（0~999）
float64 timeout       # 检测超时（秒），0=使用默认值
---
# 响应
bool found            # 是否检测到
geometry_msgs/Pose pose  # 基坐标系下的 6DOF 位姿
std_msgs/Header header   # 时间戳和参考坐标系
```

#### 4.4.5 调用示例

```bash
# 识别 ID=12 的标记，超时 5 秒
ros2 service call /right_arm_controller/detect_aruco guji/srv/DetectAruco \
  "{marker_id: 12, timeout: 5.0}"

# 响应示例
ros2 srv call result:
  found: true
  pose:
    position:
      x: 0.3421
      y: -0.0893
      z: 0.1547
    orientation:
      x: 0.0012
      y: -0.0023
      z: 0.7071
      w: 0.7071
  header:
    stamp:
      sec: 1234
      nanosec: 567000000
    frame_id: right_base
```

---

## 5. 相机与视觉识别系统

### 5.1 相机如何连接到系统

RealSense D435 通过 USB 3.0 连接到 Jetson，数据流如下：

```
[RealSense D435 硬件]
        │
        │ USB 3.0 数据流
        ▼
[realsense2_camera ROS2 功能包]
        │
        ├── 图像处理 + 深度对齐
        │
        ├── 发布话题:
        │   ├── /camera_right/color/image_raw              (RGB)
        │   ├── /camera_right/color/camera_info           (内参)
        │   ├── /camera_right/depth/image_rect_raw        (原始深度)
        │   └── /camera_right/aligned_depth_to_color/image_raw  (对齐深度)
        │
        └── 不发布 TF（publish_tf=False，由 tf_broadcaster 统一管理）
                │
                ▼
        [camera_bridge] ← 订阅 RGB + Depth + CameraInfo，诊断
        [aruco_detector] ← 订阅 RGB + CameraInfo，识别
```

### 5.2 realsense2_camera 参数配置

在 `vision_pipeline.launch.py` 中配置：

```python
realsense_node = Node(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    namespace='/camera_right',  # 话题前缀
    parameters=[{
        'serial_no': 'REPLACE_WITH_YOUR_SERIAL_NUMBER',  # 填入实际序列号
        'enable_color': True,
        'enable_depth': True,
        'enable_aligned_depth': True,    # 深度图与 RGB 对齐
        'color_width': 640,
        'color_height': 480,
        'color_fps': 30,
        'depth_width': 640,
        'depth_height': 480,
        'depth_fps': 30,
        'align_depth': True,
        'publish_tf': False,   # TF 由 tf_broadcaster 统一管理
    }]
)
```

### 5.3 查找相机序列号

```bash
# 方法1：使用 realsense-viewer
rs-viewer

# 方法2：命令行列出所有相机
rs-enumerate-devices

# 方法3：列出已连接的 USB 设备
lsusb | grep -i intel

# 方法4：查看系统设备
ls /dev/video*
```

序列号格式示例：`123422072697`

### 5.4 深度图与 RGB 对齐

对齐后的深度图（`aligned_depth_to_color`）使得每个深度像素与 RGB 像素在空间上严格对应，便于精确计算目标的三维坐标。

```
未对齐：                    对齐后：
┌──────────────┐            ┌──────────────┐
│  RGB 图像     │            │  RGB 图像     │
│  1280×720    │            │  640×480     │
│              │   ───►     │              │
│              │   对齐     │  深度叠加    │
└──────────────┘            └──────────────┘
┌──────────────┐            每个像素可直接获得
│  深度图像     │               三维坐标
│  1280×720    │
└──────────────┘
```

### 5.5 ArUco 标记准备

| 参数 | 值 | 说明 |
|------|-----|------|
| 字典类型 | DICT_5X5_1000 | 1000 个不同标记 |
| 标记边长 | 0.030m（30mm） | 需与 camera.yaml 一致 |
| 标记材质 | 打印在白纸上（黑白） | 建议覆膜避免反光 |
| ID 分配 | 11=取料标记, 12=对齐标记, 21=放料标记 | 与 poses.yaml 对应 |

**生成 ArUco 标记：**

```python
import cv2
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
img = cv2.aruco.drawMarker(aruco_dict, marker_id=11, sidePixels=200)
cv2.imwrite('aruco_11.png', img)
```

---

## 6. TF 坐标变换体系

### 6.1 坐标链

```
right_base
    │
    │  [rm_driver 节点通过关节状态计算并发布]
    │
    └──► right_top  （法兰坐标系）
              │
              │  [tf_broadcaster 节点发布静态 TF]
              │  手眼标定结果: translation + quaternion
              │
              └──► camera_right  （D435 相机坐标系）
                        │
                        │  [aruco_detector 节点发布]
                        │
                        └──► target_right_camera  （检测到的标记）
```

### 6.2 各坐标系含义

| 坐标系 | 定义 | 用途 |
|--------|------|------|
| `right_base` | 右臂基座坐标系原点 | 所有位姿的参考基准 |
| `right_top` | 右臂末端法兰中心 | 描述机械臂末端位置 |
| `camera_right` | D435 相机光学中心 | 相机视角下的位置 |
| `target_right_camera` | ArUco 标记坐标系 | 标记物在相机中的位姿 |

### 6.3 位姿变换流程

```
[ArUco 标记在真实空间中]
        │
        │  D435 相机拍摄
        ▼
[相机坐标系下的 6DOF]
  tvec = [tx, ty, tz]   ← 标记原点到相机的平移
  rvec = [rx, ry, rz]   ← 标记相对于相机的旋转
        │
        │  _publish_target_tf()
        │
        ▼
camera_right ──► target_right_camera
        │
        │  tf_buffer.lookup_transform()
        │
        ▼
right_base ──► target_right_camera
        │
        │  返回 TransformStamped
        │
        ▼
[基坐标系下的 6DOF 位姿]
  pos = (x, y, z)        ← 基座标系下的位置
  quat = (x, y, z, w)    ← 基坐标系下的姿态
```

### 6.4 TF 验证命令

```bash
# 查看两个坐标系之间的实时变换
ros2 run tf2_ros tf2_echo right_base camera_right
ros2 run tf2_ros tf2_echo right_base target_right_camera

# 可视化整个 TF 树
ros2 run tf2_ros view_frames

# 列出所有可用坐标系
ros2 run tf2_ros tf2_echo right_base right_top
ros2 run tf2_ros tf2_echo right_base camera_right

# 检查 TF 缓冲区的内容
ros2 run tf2_ros tf2_monitor right_base target_right_camera
```

---

## 7. 配置文件说明

### 7.1 文件总览

```
guji/
├── config/
│   ├── poses.yaml      # 双臂关节角度点位（示教值）
│   ├── camera.yaml     # 相机参数、手眼标定、ArUco 配置
│   └── system.yaml     # 系统全局参数
```

### 7.2 `poses.yaml` — 双臂关节角度配置

**用途：** 存储所有关键点位的关节角度，替代代码中的硬编码值。

**结构：**

```yaml
poses:
  left:
    initial:          # 初始位置（启动/结束）
    recognize:        # 识别位置（相机朝下）
    pick_prep:        # 取料准备
    pick_left_support:# 托住位
    pick_left_lift:   # 抬起托住位
    pick_left_push:   # 前顶位
    pick_left_safe:   # 安全位
    place_above:      # 放料上方位
    place_exit:       # 放料退出位
    place_safe:       # 放料安全位
  right:
    initial:          # ...
    recognize:
    pick_prep:
    pick_insert:      # 插入位（伸入工件下方）
    pick_grasp_vertical:# 夹取垂直位
    pick_lift:        # 上移位
    pick_safe:        # 安全位
    place_above:
    place_drop:
    place_exit:
    place_safe:
```

**示教方法：**

1. 运行 `teach_record.py` 拖动机械臂到目标位置
2. 记录各关节角度
3. 填入对应状态的列表中
4. 重启程序使配置生效

**单位：** 所有角度为**度（°）**，在 `movej()` 中会转换为弧度发布。

### 7.3 `camera.yaml` — 相机与标定配置

**用途：** 管理相机硬件参数、手眼标定结果和 ArUco 检测参数。

```yaml
camera:
  model: "D435"                          # 相机型号
  serial_number: "123422072697"          # ← 填入实际序列号
  topic_prefix: "/camera_right"           # 话题前缀

  intrinsic:                             # 内参（仅供参考）
    width: 640
    height: 480
    fx: 615.0
    fy: 615.0
    cx: 320.0
    cy: 240.0
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]

  hand_eye_right:                         # 手眼标定结果
    translation:
      x: 0.0850    # ← 标定后填入
      y: -0.0400   # ← 标定后填入
      z: 0.0100    # ← 标定后填入
    quaternion:
      x: 0.0       # ← 标定后填入
      y: 0.0       # ← 标定后填入
      z: 0.0       # ← 标定后填入
      w: 1.0       # ← 标定后填入

  aruco:
    dict_type: "DICT_5X5_1000"
    marker_size: 0.030   # ← 填入实际打印的标记边长（米）
    target_ids: [11, 12, 21]

  frames:
    base_frame: "right_base"
    flange_frame: "right_top"
    camera_frame: "camera_right"
    target_frame: "target_right_camera"
```

### 7.4 `system.yaml` — 系统全局参数

**用途：** 集中管理机械臂运动、夹爪、力位混合和视觉识别的所有参数。

```yaml
system:
  arm:
    movej_default_speed: 30          # MoveJ 默认速度（1~100）
    movel_default_speed: 30
    movej_timeout: 30.0              # 超时（秒）
    joint_state_timeout: 10.0         # 等待关节状态超时

  gripper:
    position_open: 1000              # 夹爪全开（1~1000）
    position_close: 0                # 夹爪全闭
    position_half: 500               # 半开
    pick_speed: 300                  # 力控夹取速度（1~1000）
    pick_force: 300                  # 力控夹取力（1~1000）

  force_position:
    sensor_type: 0                   # 0=一维力, 1=六维力
    coordinate_mode: 1               # 0=基坐标系, 1=工具坐标系
    direction: 2                    # 0~5=X/Y/Z/RX/RY/RZ
    force_threshold: 10              # 力阈值（0.1N）
    follow_mode: true

  vision:
    timeout: 5.0                     # 识别超时（秒）
    stability_threshold: 0.1         # 角度稳定性阈值（弧度）
    history_size: 10                 # 滑动窗口大小（帧）
    smoothing_factor: 0.8            # 平滑因子（0~1）
    depth_averaging_radius: 3         # 深度平均半径（像素）

  ros2:
    camera_qos: "BEST_EFFORT"        # 图像流 QoS
    arm_qos: "RELIABLE"              # 指令 QoS
    tf_timeout: 1.0
    service_timeout: 5.0

  debug:
    enabled: false
    camera_diagnostic_interval: 5.0
    show_image: false
    image_window: "Right Arm Camera"
```

---

## 8. 启动与运行

### 8.1 一键启动（推荐）

```bash
# 1. 启动 ROS2 环境
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# 2. 一键启动所有节点
ros2 launch guji vision_pipeline.launch.py \
  camera_serial:=123422072697 \
  left_ns:=left_arm_controller \
  right_ns:=right_arm_controller
```

### 8.2 分步启动（便于排查问题）

```bash
# 终端1：启动机械臂驱动（参考 ros2_ws 文档）
ros2 launch rm_driver rm_65_driver.launch.py

# 终端2：启动相机驱动
ros2 run realsense2_camera realsense2_camera_node \
  _serial_no:=123422072697 \
  _enable_color:=true \
  _enable_depth:=true \
  _align_depth:=true \
  _namespace:=/camera_right

# 终端3：启动相机诊断
ros2 run guji camera_bridge

# 终端4：启动手眼标定 TF
ros2 run guji tf_broadcaster

# 终端5：启动 ArUco 检测
ros2 run guji aruco_detector

# 终端6：启动主控制器
ros2 run guji dualeft_arm_controller_pick_place
```

### 8.3 快速验证命令

```bash
# 验证 ROS2 环境
ros2 topic list

# 验证相机话题
ros2 topic list | grep camera_right

# 验证相机帧率
ros2 topic hz /camera_right/color/image_raw

# 验证相机内参
ros2 topic echo /camera_right/color/camera_info

# 验证 Service 列表
ros2 service list | grep detect_aruco

# 验证 TF 树
ros2 run tf2_ros tf2_echo right_base camera_right

# 测试 ArUco 识别（放置标记后在另一终端运行）
ros2 service call /right_arm_controller/detect_aruco guji/srv/DetectAruco \
  "{marker_id: 11, timeout: 5.0}"
```

---

## 9. 启动检查与验证流程

### 9.1 逐项检查清单

| 步骤 | 检查项 | 验证命令 | 期望结果 |
|------|--------|---------|---------|
| **1** | ROS2 环境 | `ros2 topic list` | 显示话题列表 |
| **2** | 左臂驱动 | `ros2 topic echo /left_arm_controller/joint_states` | 有数据输出 |
| **3** | 右臂驱动 | `ros2 topic echo /right_arm_controller/joint_states` | 有数据输出 |
| **4** | 相机连接 | `lsusb \| grep -i realsense` | 显示 Intel 设备 |
| **5** | 相机驱动 | `ros2 topic list \| grep camera_right` | 看到 `/camera_right/...` |
| **6** | Color 帧率 | `ros2 topic hz /camera_right/color/image_raw` | ≥ 15Hz |
| **7** | Depth 帧率 | `ros2 topic hz /camera_right/aligned_depth...` | ≥ 15Hz |
| **8** | 相机内参 | `ros2 topic echo /camera_right/color/camera_info` | K 矩阵 9 个数 |
| **9** | Camera Bridge | 观察终端日志 | 每 5s 输出诊断 |
| **10** | TF 广播 | `ros2 run tf2_ros tf2_echo right_base camera_right` | 显示实时变换 |
| **11** | ArUco Service | `ros2 service list \| grep detect_aruco` | 看到服务 |
| **12** | ArUco 识别 | 放置 ArUco 标记，`ros2 service call ...` | `found=True` |
| **13** | TF 目标变换 | `ros2 run tf2_ros tf2_echo right_base target_right_camera` | 显示标记位置 |
| **14** | 主控制器 | 运行 `dualeft_arm_controller_pick_place.py` | 启动检查全部 OK |
| **15** | 完整流程 | `controller.run()` | 取放料流程无报错 |

### 9.2 相机诊断节点验证详解

启动 `camera_bridge` 后，观察终端输出：

**正常情况（每 5 秒）：**
```
[INFO] [camera_bridge]: --- 相机诊断 #1 (运行 5s) ---
[INFO] [camera_bridge]:   [OK  ] [OK] Color:30Hz DepthValid:82% | All checks passed
```

**异常情况：**

```
[WARN] [camera_bridge]: --- 相机诊断 #3 (运行 15s) ---
[WARN] [camera_bridge]:   [WARN] Color:8Hz DepthValid:31% | [WARN] Color 帧率过低: 8.0Hz (期望≥30Hz) | [WARN] 深度有效像素比例过低: 31% (期望≥50%)
```

### 9.3 TF 验证详解

```bash
# 验证手眼标定 TF（相机相对于基座）
ros2 run tf2_ros tf2_echo right_base camera_right
# 期望：translation 显示固定的偏移值
# At time 0.000
# - Translation: [0.085, -0.040, 0.010]

# 验证标记检测 TF（标记相对于基座）
ros2 run tf2_ros tf2_echo right_base target_right_camera
# 期望：当 ArUco 标记在相机视野内时，显示实时跟踪的位姿
# At time 1234.567
# - Translation: [0.342, -0.089, 0.155]
```

### 9.4 主控制器启动检查详解

```python
# 在 Python 中逐步检查
import rclpy
from guji.dualeft_arm_controller_pick_place import DualArmPickPlaceController

rclpy.init()
ctrl = DualArmPickPlaceController()

# 逐项检查
ctrl._check_joint_states()   # 等待关节数据
ctrl._check_aruco_service()  # 检查 Service
ctrl._check_tf_tree()        # 检查 TF 树
ctrl._check_camera_status()  # 检查相机

# 完整检查
ctrl.startup_checks()
```

---

## 10. 手眼标定指南

### 10.1 什么是手眼标定

手眼标定（Hand-Eye Calibration）用于确定**相机坐标系**与**机械臂末端法兰坐标系**之间的精确位置关系（6DOF 变换）。

```
            hand_eye_calibration
              ┌──────────┐
              │ 相机坐标系│
              └────┬─────┘
                   │
                   │ 手眼标定
                   │  (Tx, Ty, Tz, qx, qy, qz, qw)
                   ▼
┌────────┐    ┌──────────┐
│法兰坐标系│◄──│法兰→相机 │
└────────┘    └──────────┘
```

### 10.2 为什么必须标定

没有精确的手眼标定结果：
- ArUco 标记在**相机坐标系**下的位置（tvec）无法转换到**基坐标系**
- 视觉识别得到的工件位置是错误的
- 机械臂无法精确抓取识别到的目标

### 10.3 标定方法概述

手眼标定的一般流程：

1. **准备**：打印 ArUco 标定板（棋盘格或 ArUco 标定板）
2. **采集**：在相机视野内，以不同姿态拍摄多张（10~20 张）标定板图像
3. **计算**：使用 OpenCV `calibrateHandEye()` 或 ROS `easy_handeye` 包计算变换
4. **验证**：将标定结果填入 `camera.yaml`，重启 `tf_broadcaster`，验证 TF

### 10.4 使用 `easy_handeye` 进行标定（推荐）

**安装：**

```bash
cd ~/ros2_ws/src
git clone https://github.com送出/JenniferBuehler/easy_handeye.git
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select easy_handeye
```

**Launch 启动：**

```bash
# 终端1：启动机械臂和相机
ros2 launch rm_driver rm_65_driver.launch.py
ros2 launch realsense2_camera rs_camera.launch.py \
  namespace:=/camera_right \
  serial_no:=你的序列号

# 终端2：启动 MoveIt! 手动示教
ros2 launch moveit2izards moveit2izards.launch.py

# 终端3：启动 easy_handeye
ros2 launch easy_handeye easy_handeye.launch.py \
  name:=right_hand_eye \
  tracking_camera:=camera_right \
  robot_base_frame:=right_base \
  robot_effector_frame:=right_top \
  tracking_marker_frame:=aruco_marker
```

**采集步骤：**

1. 打开 RViz 中的 `easy_handeye` 界面
2. 确保 ArUco 标定板在相机视野内
3. 拖动机械臂到不同姿态（至少 10 个），每个姿态点击 "Take Sample"
4. 点击 "Compute" 计算标定结果
5. 点击 "Save" 保存结果到 YAML

**标定结果格式：**

```yaml
# eye_on_hand (相机在手上 / 手眼配置)
# translation: [x, y, z] 米
# rotation: [x, y, z, w] 四元数
translation:
  x: 0.0850
  y: -0.0400
  z: 0.0100
rotation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

### 10.5 手动标定验证

填入标定结果后，按以下步骤验证：

**步骤 1：** 重启 `tf_broadcaster`

```bash
# 停止旧节点
ros2 node list | grep tf_broadcaster
ros2 node kill /hand_eye_tf_broadcaster

# 重启
ros2 run guji tf_broadcaster
```

**步骤 2：** 验证静态 TF

```bash
ros2 run tf2_ros tf2_echo right_base camera_right
# At time 0.000
# - Translation: [0.085, -0.040, 0.010]
# - Rotation: [0.000, 0.000, 0.000, 1.000]
```

**步骤 3：** 放置 ArUco 标记在相机视野内

**步骤 4：** 验证目标 TF 的 Z 坐标

```bash
ros2 run tf2_ros tf2_echo right_base target_right_camera
# At time 1234.567
# - Translation: [x, y, z]
```

将 `z` 值与标记的实际高度对比：
- 如果标定正确：z 值误差 < 5mm
- 如果误差 > 10mm：需要重新标定

**步骤 5：** 验证工件抓取

1. 将 ArUco 标记贴在工件上
2. 启动 `aruco_detector`，调用 Service 获取位置
3. 手动计算/估计抓取位置
4. 对比实际抓取结果

### 10.6 常见标定问题

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| 标定结果差异大 | 采集姿态不够多样化 | 增加采集数量，覆盖更多角度 |
| 保存结果后 TF 不对 | `tf_broadcaster` 未重启 | 重启节点 |
| Z 坐标误差大 | 四元数方向错误 | 检查 quaternion 的 x/y/z/w 顺序 |
| 标定值接近零 | 量纲错误（可能是米/毫米混淆） | 确认 YAML 中单位是米 |

---

## 11. 完整取放料流程

### 11.1 流程概览（9 个状态）

```
[初始位] → [识别位] → [识别AR12] → [推对齐] → [取料11步]
                                                          │
                                                          ▼
                                              [回初始位] ← [放料8步] ← [识别AR21] ← [放料位]
```

### 11.2 各状态详解

#### 状态 1：初始位置

```python
go_initial_position()
```

**动作：** 双臂同时 MoveJ 到初始位置（上举姿态）

**点位：** `POSES['left']['initial']`, `POSES['right']['initial']`

**目的：** 确保机械臂在安全、可观测的位置，为后续动作做准备

#### 状态 2：识别位置

```python
go_recognize_position()
```

**动作：** 双臂移动到识别位置（相机垂直朝下，观察工作台）

**点位：** `POSES['left']['recognize']`, `POSES['right']['recognize']`

**目的：** 将相机对准工件和 ArUco 标记

#### 状态 3：识别 ARcode12

```python
recognize_arcode()
  └─► detect_arcode('right', 'ARcode12')
        └─► 调用 /right_arm_controller/detect_aruco Service
              └─► 返回工件相对于基座的位置
```

**动作：** 右臂相机识别工作台上的 ARcode12 标记

**输出：** 工件的三维位置偏移（用于后续对齐）

**目的：** 定位工件在工作台上的精确位置

#### 状态 4：预抓取对齐

```python
pre_grasp_alignment()
```

**动作：**
- 右臂移动到工件前方（`pick_prep`）
- 左臂移动到工件侧面（`pick_prep`）
- 右臂前推对齐（`pick_insert`）
- 左臂右推对齐（`pick_left_support`）

**目的：** 物理对齐工件，为后续取料做准备

#### 状态 5：取料（11 步核心逻辑）

```python
pick_workpiece()
```

| Step | 动作 | 涉及点位/操作 |
|------|------|-------------|
| 1 | 左臂移动到托住位置 | `pick_left_support` |
| 2 | 左臂力控夹取（托住工件） | `gripper_pick(force=300)` |
| 3 | 左臂弧线抬起托住位 | `pick_left_lift` |
| 4 | 右臂伸入工件下方 | `pick_insert` |
| 5 | 右臂力控夹取（夹住工件） | `gripper_pick(force=300)` |
| 6 | 右臂回到垂直姿态 | `pick_grasp_vertical` |
| 7 | 右臂夹爪松开（让工件回落到左爪） | `gripper_set(position=1000)` |
| 8 | 左臂前顶工件到位 | `pick_left_push` |
| 9 | 左臂夹爪松开 | `gripper_set(position=1000)` |
| 10 | 右臂带着工件上移 | `pick_lift` |
| 11 | 左臂回缩到安全位置 | `pick_left_safe` |

**结果：** 右臂成功夹取物料，左臂处于安全位置

#### 状态 6：移动到放料位置

```python
go_place_position()
```

**动作：** 双臂移动到放料区上方

**点位：** `POSES['left']['place_above']`, `POSES['right']['place_above']`

#### 状态 7：识别 ARcode21

```python
recognize_arcode21()
  └─► detect_arcode('right', 'ARcode21')
```

**动作：** 右臂相机识别放料位置标记

**目的：** 精确定位放料位置

#### 状态 8：放料（力位混合）

```python
place_workpiece()
```

| Step | 动作 | 说明 |
|------|------|------|
| 1 | 右臂移动到放料上方位 | MoveJ 到 `place_above` |
| 2 | 力位混合下移搜索放置面 | `force_position_move(dir=2)` 沿 Z 轴 |
| 3 | 夹爪张开释放物料 | `gripper_set(position=1000)` |
| 4 | 右臂移动到退出位 | MoveJ 到 `place_exit` |
| 5 | 夹爪闭合压实物料 | `gripper_set(position=0)` |
| 6 | 夹爪上移 | `gripper_set(position=500)` |
| 7 | 夹爪再次闭合 | `gripper_set(position=0)` |
| 8 | 右臂回安全位 | MoveJ 到 `place_safe` |

**力位混合原理：** 在 Z 轴方向施加恒定的轻柔下压力，当夹爪接触放置面时，力反馈增大，系统判定接触后停止下移，实现精准放置。

#### 状态 9：回取料处原点

```python
return_to_pick_origin()
```

**动作：** 双臂回到初始位置，完成一轮取放料

---

## 12. 调试与故障排查

### 12.1 常见问题速查表

| 现象 | 可能原因 | 排查方法 | 解决方案 |
|------|---------|---------|---------|
| `camera_right` 话题不存在 | `realsense2_camera` 未启动 | `ros2 node list` | 启动 realsense2_camera 节点 |
| 相机帧率 < 15Hz | USB 带宽不足 | `ros2 topic hz /camera_right/...` | 更换 USB 3.0 口，降低分辨率 |
| CameraInfo 全零 | 驱动版本问题 | `ros2 topic echo /camera_right/color/camera_info` | 更新 realsense2_camera |
| 深度全黑/无效 | USB 连接不稳定 | 检查 USB 线，重新插拔 | 更换 USB 3.0 线 |
| ArUco 检测 `found=False` | 标记不在视野 | 移动机械臂确认标记在相机中 | 调整 `recognize` 点位 |
| TF 查询失败 | `right_base` 未广播 | `ros2 run tf2_ros tf2_echo right_base camera_right` | 检查 rm_driver 节点 |
| YAML 加载失败 | 文件路径错误 | 查看终端错误日志 | 检查 config 目录位置 |
| 关节数据无更新 | namespace 错误 | `ros2 topic echo /left_arm_controller/joint_states` | 确认 namespace 配置 |
| Service 调用超时 | aruco_detector 未启动 | `ros2 service list \| grep detect` | 启动 aruco_detector 节点 |
| 抓取偏移大 | 手眼标定未做 | 验证 TF Z 坐标 | 执行手眼标定 |

### 12.2 日志分析指南

**camera_bridge 日志：**

```
[INFO] --- 相机诊断 #2 (运行 10s) ---
[INFO]   [OK  ] [OK] Color:30Hz DepthValid:78% | All checks passed
```
→ 相机正常工作

```
[WARN] [WARN] Color:8Hz DepthValid:31% | [WARN] Color 帧率过低: 8.0Hz
```
→ 帧率不足，检查 USB 连接

**aruco_detector 日志：**

```
[INFO] 开始检测 ArUco ID=12, 超时=5s
[INFO]   [ID=12] 检测成功! pos=(0.3421, -0.0893, 0.1547)m, angle=0.0012rad, attempts=3
[INFO]   基坐标系位姿: pos=(0.3421, -0.0893, 0.1547)m
```
→ 检测成功

```
[WARN] 检测超时，未找到 ID=12（尝试 60 帧）
```
→ 标记不在视野内，调整机械臂姿态

**dualeft_arm_controller_pick_place 日志：**

```
[INFO] >>> [状态5] 开始取料（11步核心逻辑）
[INFO]   [取料 Step 1] 左臂移动到托住位置
[INFO] 左臂 MoveJ: ['10.0', '-50.0', ...]deg, speed=20, block=True
...
[INFO] <<< [状态5] 完成：取料成功，右臂已夹取物料
```
→ 取料流程正常执行

### 12.3 网络连接测试

```bash
# 测试网络连通性
ping 192.168.150.111  # 左臂
ping 192.168.150.112  # 右臂

# 测试 TCP 端口
nc -vz 192.168.150.111 8080
nc -vz 192.168.150.112 8080

# 查看端口占用
sudo lsof -i :8080
sudo lsof -i :8089

# 测试 UDP 通信
nc -vuz 192.168.150.111 8089
```

### 12.4 ROS2 调试命令汇总

```bash
# 列出所有节点
ros2 node list

# 列出所有话题
ros2 topic list

# 列出所有服务
ros2 service list

# 查看节点信息
ros2 node info /dualeft_arm_controller_pick_place_controller

# 查看话题带宽
ros2 topic bw /camera_right/color/image_raw

# 测量延迟
ros2 topic delay /camera_right/color/image_raw

# 重新映射话题（调试用）
ros2 run guji aruco_detector --ros-args -r /camera_right/color/image_raw:=/my_custom/image

# 查看参数
ros2 param list
ros2 param get /camera_bridge camera_prefix
```

### 12.5 视觉调试技巧

**1. 实时预览相机图像（rviz）：**

```bash
ros2 run rviz2 rviz2
# 添加 Image Display，topic 选择 /camera_right/color/image_raw
```

**2. 保存图像供离线分析：**

```python
import cv2
import imageio as iio

# 录制 10 秒图像
writer = iio.get_writer('debug_video.mp4', fps=30)
for _ in range(300):  # 10s @ 30fps
    frame = self._latest_image.copy()
    writer.append_data(frame)
writer.close()
```

**3. 调试 ArUco 检测（单独测试）：**

```python
import cv2
import numpy as np

# 加载图像
img = cv2.imread('test_image.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 检测 ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)

# 可视化
output = cv2.aruco.drawDetectedMarkers(img, corners, ids)
cv2.imshow('debug', output)
cv2.waitKey(0)
```

---

## 13. 扩展开发指南

### 13.1 添加 YOLO 目标检测

当前系统使用 ArUco 标记定位。在需要通用物体识别时，可扩展 YOLO：

```python
# nodes/yolo_detector.py
class YoloDetector(Node):
    def __init__(self):
        # 加载 YOLO 模型
        self._net = cv2.dnn.readNet('yolov8n.onnx')
        self._classes = ['part1', 'part2', 'part3']

        self._sub = self.create_subscription(
            Image,
            '/camera_right/color/image_raw',
            self._callback, 10
        )

        self._srv = self.create_service(
            DetectObject,
            '/right_arm_controller/detect_object',
            self._detect_callback
        )

    def _callback(self, msg):
        self._latest_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _detect_callback(self, req, resp):
        blob = cv2.dnn.blobFromImage(self._latest_image, 1/255, (640, 640))
        self._net.setInput(blob)
        outputs = self._net.forward(self._net.getUnconnectedOutLayersNames())
        # ... NMS 和后处理
        return resp
```

### 13.2 添加多目标抓取

扩展 `pick_workpiece()` 支持多物料顺序抓取：

```python
def pick_multiple(self, num: int = 3):
    for i in range(num):
        marker_id = 10 + i  # ARcode10, ARcode11, ARcode12
        offset = self.detect_arcode('right', f'ARcode{marker_id}')
        self.pick_single(offset)
        self.go_place_position()
        self.place_workpiece()
    self.return_to_pick_origin()
```

### 13.3 添加导航集成

参照 `Warehouse_handling_robot` 的导航集成方式：

```python
# 添加导航依赖
# woosh_msgs 或 nav2_msgs

class DualArmPickPlaceWithNav(DualArmPickPlaceController):
    def __init__(self):
        super().__init__()

        # 导航 Service Client
        self._nav_client = self.create_client(
            ExecTask, '/exec_task'
        )

    def go_to_station(self, station_no: str):
        req = ExecTaskRequest()
        req.task_id = 1
        req.task_type = 1
        req.mark_no = station_no
        future = self._nav_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def run_with_nav(self):
        self.startup_checks()

        # 1. 导航到取料工位
        self.go_to_station('11')
        self.go_initial_position()
        self.go_recognize_position()
        self.recognize_arcode()
        self.pre_grasp_alignment()
        self.pick_workpiece()

        # 2. 导航到放料工位
        self.go_to_station('21')
        self.go_place_position()
        self.recognize_arcode21()
        self.place_workpiece()

        # 3. 回充电桩（低电量时）
        battery = self.get_battery_level()
        if battery < 20:
            self.go_to_station('120')  # 充电桩
```

### 13.4 添加力传感反馈

当前放料使用力位混合（Force-Position Hybrid）。可扩展力传感的更多应用：

```python
def force_guardian(self, arm: str, max_force: float = 30.0):
    """
    力监护模式：监控运动过程中的力反馈，
    如果力超过阈值立即停止并告警
    """
    force_threshold = max_force  # N

    def force_callback(msg):
        fx, fy, fz = msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z
        total_force = math.sqrt(fx**2 + fy**2 + fz**2)
        if total_force > force_threshold:
            self.get_logger().warn(f'力超限: {total_force:.1f}N > {force_threshold}N')
            self._emergency_stop(arm)

    force_sub = self.create_subscription(
        WrenchStamped,
        f'/{arm}_arm/ft_sensor/data',
        force_callback, 10
    )
```

### 13.5 添加日志记录与回放

```python
import json
from datetime import datetime

class PickPlaceLogger:
    """记录取放料流程日志，便于回放和故障分析"""

    def __init__(self, log_dir: str = 'logs/'):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.events = []

    def log_event(self, event_type: str, data: dict):
        event = {
            'timestamp': time.time(),
            'type': event_type,
            'data': data
        }
        self.events.append(event)
        self.get_logger().info(f'[LOG] {event_type}: {data}')

    def save(self):
        path = os.path.join(self.log_dir, f'pick_place_{self.session_id}.json')
        with open(path, 'w') as f:
            json.dump(self.events, f, indent=2)
        return path
```

---

## 附录 A：文件清单

| 文件 | 说明 |
|------|------|
| `guji/dualeft_arm_controller_pick_place.py` | 主控制器（1140 行） |
| `guji/nodes/camera_bridge.py` | 相机诊断节点（345 行） |
| `guji/nodes/aruco_detector.py` | ArUco 检测节点（520 行） |
| `guji/nodes/tf_broadcaster.py` | 手眼标定 TF 广播（224 行） |
| `guji/launch/vision_pipeline.launch.py` | 一键启动文件（196 行） |
| `guji/config/poses.yaml` | 双臂关节角度配置 |
| `guji/config/camera.yaml` | 相机与标定配置 |
| `guji/config/system.yaml` | 系统全局参数 |
| `guji/srv/DetectAruco.srv` | 视觉识别 Service 定义 |
| `guji/README.md` | 项目总览文档 |

## 附录 B：ROS2 话题完整列表

```
/camera_right/color/image_raw
/camera_right/color/camera_info
/camera_right/depth/image_rect_raw
/camera_right/aligned_depth_to_color/image_raw
/camera_right/extrinsic/depth_to_color

/left_arm_controller/joint_states
/left_arm_controller/rm_driver/udp_arm_position
/left_arm_controller/rm_driver/udp_joint_pose_euler
/left_arm_controller/rm_driver/movej_cmd
/left_arm_controller/rm_driver/movel_cmd
/left_arm_controller/rm_driver/set_gripper_position_cmd
/left_arm_controller/rm_driver/set_gripper_pick_cmd
/left_arm_controller/rm_driver/set_hand_angle_cmd
/left_arm_controller/rm_driver/force_position_move_pose_cmd

/right_arm_controller/joint_states
/right_arm_controller/rm_driver/udp_arm_position
/right_arm_controller/rm_driver/udp_joint_pose_euler
/right_arm_controller/rm_driver/movej_cmd
/right_arm_controller/rm_driver/movel_cmd
/right_arm_controller/rm_driver/set_gripper_position_cmd
/right_arm_controller/rm_driver/set_gripper_pick_cmd
/right_arm_controller/rm_driver/set_hand_angle_cmd
/right_arm_controller/rm_driver/force_position_move_pose_cmd

/right_arm_controller/detect_aruco           # Service
/right_arm_controller/get_camera_status      # Service
/right_arm_controller/aruco/debug_image      # Image topic
```

## 附录 C：坐标系汇总

| 坐标系名称 | 类型 | 发布者 | 父坐标系 | 说明 |
|-----------|------|--------|---------|------|
| `right_base` | 动态 | `rm_driver` | - | 右臂基座原点 |
| `right_top` | 动态 | `rm_driver` | `right_base` | 右臂法兰 |
| `camera_right` | 静态 | `tf_broadcaster` | `right_top` | D435 相机光学中心 |
| `target_right_camera` | 动态 | `aruco_detector` | `camera_right` | 检测到的 ArUco 标记 |
| `left_arm_controller/*` | 动态 | `rm_driver` | - | 左臂相关坐标系 |
| `base_link` | 动态 | `rm_driver` | - | 全局基座（参考） |
