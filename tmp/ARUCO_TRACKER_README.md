# ArUco 码识别与移动检测程序

## 概述

这是一个基于 ROS2 的机械臂视觉应用程序，用于识别 ArUco 标记码的位置，并通过两次识别来计算 ArUco 码在不同坐标系中的移动距离。

## 功能说明

### 程序分为两个阶段：

#### 第一阶段：初始检测
- 识别 ArUco 码的初始位置
- 获取 ArUco 码在相机、末端执行器、基座坐标系中的位置
- 保存第一次检测结果

#### 第二阶段：移动后检测  
- 物理移动 ArUco 码到新位置
- 进行第二次识别
- 自动计算两次检测之间的移动距离

## 坐标系说明

程序支持三个坐标系的位置计算：

1. **相机坐标系（Camera Frame）**
   - 原点：相机镜头中心
   - 直接从ArUco检测得到
   
2. **末端执行器坐标系（EEF / Tool Frame）**
   - 原点：机械臂末端法兰中心
   - 使用手眼标定外参从相机坐标系转换而来
   
3. **基座坐标系（Base Frame）**
   - 原点：机械臂基座
   - 使用 URDF 中定义的固定关节参数进行转换
   - **无需运行时 TF 话题支持，使用硬编码参数**
   - 对右臂的固定变换已从 `joint.urdf.xacro` 中提取

## 手眼标定外参与臂基座参数

### 手眼标定外参

外参存储在 `guji/config/camera.yaml` 中，包含：

```yaml
hand_eye_right:
  translation:
    x: 0.0850    # X 方向偏移（米）
    y: -0.0400   # Y 方向偏移（米）
    z: 0.0100    # Z 方向偏移（米）
  quaternion:
    x: 0.0       # 四元数 X 分量
    y: 0.0       # 四元数 Y 分量
    z: 0.0       # 四元数 Z 分量
    w: 1.0       # 四元数 W 分量
```

### 臂基座固定参数

臂基座固定变换从 URDF 中的固定关节参数提取（`joint.urdf.xacro`）：

```yaml
arm_base_offset_right:
  position:
    x: -0.1           # 相对于基座的 X 偏移
    y: -0.1103        # 相对于基座的 Y 偏移
    z: 0.031645       # 相对于基座的 Z 偏移
  rotation_rpy:
    r: 0.0            # 滚动角（弧度）
    p: -0.7854        # 俯仰角（弧度，约 -45°）
    y: 0.0            # 偏航角（弧度）
```

这些参数定义了机械臂末端执行器相对于基座的固定位置关系，无需运行时的 TF 话题。

### 如何更新这些参数

1. **手眼标定更新**：运行手眼标定程序，获得新的外参后，更新 `guji/config/camera.yaml` 中的 `hand_eye_right` 部分。

2. **臂基座参数更新**：如果 URDF 中的固定关节参数改变，需要同步更新 `guji/config/camera.yaml` 中的 `arm_base_offset_right` 部分。

## 使用方法

### 1. 前置准备

确保以下模块正确启动：

```bash
# 终端 1：启动机械臂驱动
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch rm_driver rm_65_driver.launch.py

# 终端 2：启动相机驱动
ros2 launch realsense2_camera rs_launch.py
```

**注意**：程序使用 URDF 中的固定关节参数进行基座坐标系转换，无需启动 TF 广播器。

### 2. 运行程序

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ./install/setup.bash
python3 new/aruco_tracker.py
```

### 3. 交互流程

**第一步 - 初始检测：**
```
请选择机械臂 (left/right，默认 right): [输入或直接回车]

【步骤 1 - 初始检测】
准备好 ArUco 码，使其在相机视野内，然后按 s 键进行初始检测
(按其他键跳过)

[按 's' 键执行检测]
```

**第二步 - 移动后检测：**
```
【步骤 2 - 等待移动】
已检测第一个位置。请移动 ArUco 码到新位置，然后按 s 键进行第二次检测

[物理移动 ArUco 码]
[按 's' 键执行第二次检测]
```

**输出结果：**
```
📊 移动距离计算结果
====================================================================

📷 相机坐标系中的移动:
   Δ位置: (0.0123, -0.0456, 0.0789) m
   距离: 0.0945 m = 94.50 mm

🦾 末端执行器坐标系中的移动:
   Δ位置: (0.0089, -0.0412, 0.0801) m
   距离: 0.0923 m = 92.30 mm

🏠 基座坐标系中的移动:
   Δ位置: (0.0234, -0.0567, -0.0123) m
   距离: 0.0615 m = 61.50 mm
```

## 键盘控制

| 按键 | 功能 |
|-----|------|
| `s` 或 `S` | 执行 ArUco 检测 |
| `q` 或 `Q` | 退出程序 |

## 配置参数

### ArUco 标记配置

在 `guji/config/camera.yaml` 中修改：

```yaml
aruco:
  dict_type: "DICT_5X5_1000"      # 字典类型
  marker_size: 0.030              # 标记尺寸（米），需与实际打印尺寸一致
  target_ids:
    - 11
    - 12
    - 21
```

常见标记尺寸：
- 20mm: `marker_size: 0.020`
- 30mm: `marker_size: 0.030`
- 50mm: `marker_size: 0.050`

### 相机话题

默认使用右臂相机：`/camera_right/color/image_raw`

如需更改，编辑 `camera.yaml`：

```yaml
camera:
  topic_prefix: "/camera_right"   # 或 "/camera_left"
```

## 常见问题

### 1. 未检测到 ArUco 码

**原因：**
- ArUco 码不在相机视野内
- 标记已损坏或打印不清晰
- 光线不足
- 字典类型不匹配

**解决方案：**
- 确认标记印刷清晰
- 确保相机能看到标记
- 检查 `dict_type` 是否正确
- 改善光线条件

### 2. 相机内参未获取

**原因：**
- 相机驱动未正确启动
- 相机话题配置错误

**解决方案：**
```bash
# 检查相机话题
ros2 topic list | grep camera

# 查看相机内参
ros2 topic echo /camera_right/color/camera_info
```

### 3. 末端执行器话题配置正确

**说明**：程序不再依赖末端执行器位姿话题，使用 URDF 中的固定关节参数。

### 4. 检测精度不理想

**改进方法：**
- 检查手眼标定是否准确（重新运行标定程序）
- 确保相机镜头清洁
- 调整光线以获得更清晰的图像
- 增大 ArUco 标记尺寸

## 依赖库

```bash
pip install opencv-python
pip install numpy
pip install scipy
pip install PyYAML
```

## 算法说明

### 坐标变换流程

```
相机坐标系
    ↓ [使用手眼标定外参]
末端执行器坐标系
    ↓ [使用 URDF 固定关节参数]
基座坐标系
```

### 手眼标定外参的含义

```
P_eef = R_camera_to_eef * P_camera + T_camera_to_eef
```

其中：
- `P_camera`: ArUco 码在相机坐标系中的位置
- `R_camera_to_eef`: 手眼标定旋转矩阵（由四元数转换）
- `T_camera_to_eef`: 手眼标定平移向量
- `P_eef`: ArUco 码在末端执行器坐标系中的位置

### 臂基座固定变换的含义

```
P_base = R_eef_to_base * P_eef + T_eef_to_base
```

其中：
- `P_eef`: ArUco 码在末端执行器坐标系中的位置
- `R_eef_to_base`: URDF 中定义的旋转矩阵（由欧拉角转换）
- `T_eef_to_base`: URDF 中定义的平移向量（固定的臂基座偏移）
- `P_base`: ArUco 码在基座坐标系中的位置

**关键特性**：臂基座变换是固定的，不依赖机械臂的当前姿态

### 移动距离计算

```
Δ位置 = P2 - P1
距离 = ||Δ位置|| = sqrt(Δx² + Δy² + Δz²)
```

## 输出文件

程序会在控制台输出结果，可以通过以下方式保存：

```bash
python3 new/aruco_tracker.py | tee output.log
```

## 扩展功能

### 批量检测多个 ArUco 码

修改 `perform_detection()` 方法中的标记选择逻辑：

```python
# 检测所有标记而不是只取第一个
for marker_id in ids:
    # 处理每个标记
```

### 连续追踪

添加定时器，定期执行检测：

```python
self.track_timer = self.create_timer(1.0, self.continuous_tracking)
```

### 保存检测结果

添加数据保存功能：

```python
import json
with open('detection_results.json', 'w') as f:
    json.dump(self.first_detection, f)
```

## 参考资源

- [OpenCV ArUco 模块](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [机械臂驱动文档](../guji/README.md)

## 常见改进方向

1. **多标记追踪** - 同时检测并追踪多个 ArUco 码
2. **性能优化** - 使用多线程提高检测速度
3. **鲁棒性增强** - 添加滤波算法处理检测噪声
4. **实时监控** - 添加 Web 界面实时展示检测结果
5. **数据记录** - 自动保存检测数据用于离线分析

## 故障排查

如遇到问题，可按以下步骤排查：

1. **检查 ROS2 连接**
   ```bash
   ros2 node list
   ros2 topic list
   ```

2. **验证相机工作**
   ```bash
   python3 new/3.py
   ```

3. **验证机械臂工作**
   ```bash
   python3 guji/arm_monitor.py
   ```

4. **查看程序日志**
   ```bash
   # 启用详细日志
   export ROS_LOG_LEVEL=DEBUG
   python3 new/aruco_tracker.py
   ```

## 许可证

遵循项目主许可证

## 维护者

请在遇到问题时提交 Issue
