# 双臂取放料系统 - 入门指南

## 项目简介

本教程项目是基于睿尔曼 RM65 双臂机械臂的简化版取放料系统，旨在帮助用户由浅入深地学习和测试整个系统的各个组件。项目采用与原项目相同的方法，但拆分为多个小部分，便于理解和测试。

## 目录结构

```
rm_pick_place_tutorial/
├── config/          # 配置文件
│   ├── poses.yaml   # 点位配置
│   ├── camera.yaml  # 相机配置
│   └── system.yaml  # 系统参数
├── scripts/         # 测试脚本
│   ├── 01_basic_test.py       # 基础功能测试
│   ├── 02_camera_test.py      # 相机功能测试
│   ├── 03_aruco_test.py       # ArUco标记识别测试
│   ├── 04_tf_test.py          # TF坐标变换测试
│   ├── 05_single_arm_test.py  # 单臂功能测试
│   ├── 06_dual_arm_test.py    # 双臂协同测试
│   └── 07_full_pipeline.py    # 完整取放料流程
└── docs/            # 文档
    └── GETTING_STARTED.md  # 本入门指南
```

## 系统要求

1. **硬件**：
   - 睿尔曼 RM65 双臂机械臂
   - Intel RealSense D435 相机
   - 计算机（Jetson 或 PC）
   - 网络连接（机械臂与计算机）

2. **软件**：
   - ROS2 Foxy Fitzroy
   - Python 3.8+
   - OpenCV 4.5+
   - realsense2_camera ROS2 包
   - dual_rm_driver ROS2 包（双臂驱动）

## 安装与设置

### 1. 环境准备

```bash
# 1. 安装 ROS2 Foxy（参考官方文档）
# 2. 安装依赖
pip3 install opencv-python numpy pyyaml

# 3. 安装 realsense2_camera 包
cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development

# 4. 安装双臂驱动包（已在 ros2_ws/src/ros2_rm_robot 目录中）

# 5. 编译工作空间
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

### 2. 配置文件设置

1. **相机序列号配置**：
   - 运行 `rs-enumerate-devices` 查看相机序列号
   - 修改 `config/camera.yaml` 中的 `serial_number` 字段

2. **网络配置**：
   - 确保计算机与机械臂在同一网络
   - 默认 IP 配置：
     - 计算机：192.168.151.119
     - 左臂：192.168.150.111
     - 右臂：192.168.150.112

3. **点位配置**：
   - 根据实际机械臂情况，修改 `config/poses.yaml` 中的关节角度

## 测试流程

### 步骤 1：基础功能测试

```bash
# 启动双臂机械臂驱动
ros2 launch dual_rm_driver dual_rm_65_driver.launch.py

# 运行基础测试脚本
cd rm_pick_place_tutorial
python scripts/01_basic_test.py
```

此脚本测试机械臂的基本连接和关节状态获取。

### 步骤 2：相机功能测试

```bash
# 启动相机驱动
ros2 run realsense2_camera realsense2_camera_node \
  _serial_no:=你的相机序列号 \
  _enable_color:=true \
  _enable_depth:=true \
  _align_depth:=true \
  _namespace:=/camera_right

# 运行相机测试脚本
python scripts/02_camera_test.py
```

此脚本测试相机的图像获取和基本诊断功能。

### 步骤 3：ArUco 标记识别测试

```bash
# 确保相机驱动正在运行
# 打印 ArUco 标记（ID: 11, 12, 21）并放置在相机视野内

# 运行 ArUco 测试脚本
python scripts/03_aruco_test.py
```

此脚本测试 ArUco 标记的识别功能。

### 步骤 4：TF 坐标变换测试

```bash
# 启动 tf_broadcaster
ros2 run guji tf_broadcaster

# 运行 TF 测试脚本
python scripts/04_tf_test.py
```

此脚本测试坐标变换功能。

### 步骤 5：单臂功能测试

```bash
# 运行单臂测试脚本
python scripts/05_single_arm_test.py
```

此脚本测试单个机械臂的运动和夹爪控制。

### 步骤 6：双臂协同测试

```bash
# 运行双臂协同测试脚本
python scripts/06_dual_arm_test.py
```

此脚本测试双臂的协同运动。

### 步骤 7：完整取放料流程

```bash
# 运行完整流程测试脚本
python scripts/07_full_pipeline.py
```

此脚本测试完整的取放料流程。

## 故障排查

### 常见问题

1. **机械臂连接失败**
   - 检查网络连接
   - 确认 IP 地址配置正确
   - 检查机械臂电源

2. **相机无法识别**
   - 检查 USB 连接
   - 确认相机序列号正确
   - 检查 realsense2_camera 包是否正确安装

3. **ArUco 标记识别失败**
   - 确保标记在相机视野内
   - 检查标记质量（无模糊、无反光）
   - 确认标记尺寸与配置文件一致

4. **TF 变换失败**
   - 检查 tf_broadcaster 是否运行
   - 确认手眼标定参数正确

## 扩展开发

1. **添加新的取放料点位**：
   - 修改 `config/poses.yaml` 添加新点位
   - 在测试脚本中使用新点位

2. **调整视觉识别参数**：
   - 修改 `config/camera.yaml` 中的 ArUco 相关参数

3. **优化运动轨迹**：
   - 修改 `config/system.yaml` 中的运动参数

4. **添加新的视觉识别算法**：
   - 扩展 `scripts/03_aruco_test.py` 添加新的识别算法

## 安全注意事项

1. **机械臂操作安全**：
   - 在测试前确保工作区域无人员和障碍物
   - 开始测试时保持距离机械臂
   - 准备紧急停止按钮

2. **相机安全**：
   - 避免相机受到机械臂碰撞
   - 保持相机镜头清洁

3. **网络安全**：
   - 确保网络环境稳定
   - 避免网络干扰

## 联系方式

如有任何问题，请参考原项目文档或联系技术支持。
