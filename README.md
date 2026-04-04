# 睿尔曼机械臂 ROS2 控制程序

## 一、项目简介

本项目包含多个用于睿尔曼机械臂的 Python 控制程序：

| 程序 | 说明 |
|------|------|
| `arm_monitor.py` | ROS2 状态监控（支持单/双臂） |
| `dual_arm_controller.py` | ROS2 双臂关节控制 |
| `dual_arm_pick_place.py` | ROS2 双臂协同取放料控制器 |
| `teach_record.py` | 示教轨迹记录 |
| `rm_joint_reader.py` | **Python SDK 读取关节角度**（无需 ROS2） |

## 二、环境依赖

### 2.1 硬件要求

| 设备 | 要求 |
|------|------|
| 睿尔曼机械臂 | RM65 / RM75 / RM63 / ECO65 / ECO63 / GEN72 系列 |
| 计算机 | **Jetson 系列（ARM64 / aarch64）**或 x86_64 PC，与机械臂同一网络，建议有线连接 |
| 网线 | 千兆网线 |

**本项目实际部署环境：**
- 硬件平台：Jetson（ARM64 / aarch64）
- 机械臂数量：双臂（左臂 + 右臂）

### 2.2 软件环境

| 软件 | 版本要求 |
|------|----------|
| 操作系统 | **Ubuntu 18.04**（Jetson 原生系统）或 Ubuntu 20.04+ / Windows 10/11 |
| Python | 3.6+ |
| ROS2 | Foxy（仅 ROS2 相关程序需要） |

### 2.3 网络配置

| 节点 | IP 地址 |
|------|---------|
| Jetson（计算机） | `192.168.151.119` |
| 左臂 | `192.168.150.111` |
| 右臂 | `192.168.150.112` |

> 注：机械臂默认 IP 为 `192.168.1.25`，上述 IP 为实际配置值。

**网络拓扑：**`Jetson` — `路由器` — `机械臂`

**配置方法：**

```bash
# 临时配置有线网卡 IP（重启后失效）
sudo ip addr add 192.168.150.100/24 dev eth0

# 验证网卡状态
ip addr show eth0
```

```bash
# 永久配置 - 编辑网络配置
sudo nano /etc/netplan/01-netcfg.yaml
```

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      addresses:
        - 192.168.150.100/24
      nameservers:
        addresses:
          - 8.8.8.8
```

```bash
sudo netplan apply
```

### 2.4 环境配置注意事项

#### Conda 环境冲突

ROS2 环境与 miniconda 可能发生冲突。操作前记得执行：

```bash
deactivate
```

#### 环境变量清理

检查并去除 `~/.bashrc` 中 `source` 另一个 ROS 工作目录的指令，避免环境交叉污染：

```bash
nano ~/.bashrc
# 删除或注释掉类似以下的行
# source ~/other_ros_ws/install/setup.bash
source ~/.bashrc
```

#### CMake 版本问题

编译时如果遇到 CMake 版本过低报错，需要手动修改功能包中 `CMakeLists.txt` 的最低版本要求（例如 `cmake_minimum_required(VERSION 3.14)` 改为 `3.1`）。

#### 编译跳过目录

在不需要编译的文件夹下创建 `COLCON_IGNORE` 文件，编译时将自动跳过该目录：

```bash
touch COLCON_IGNORE
```

### 2.5 RM_API2 Python SDK 安装 (SDK 方式)

如果使用 `rm_joint_reader.py`，需要安装 RM_API2 SDK：

#### Linux ARM64 (Jetson)

> **重要：**仓库默认的动态链接库（.so）为 **X86-64** 版本，运行在 Jetson（ARM64）上需要替换为 **ARM64 版本（V1.1.3）**。

1. 找到 SDK 路径：`RM_API2/Python/Robotic_Arm/`
2. 将 `libs/linux_arm64/` 目录下的动态库替换为 **V1.1.3 ARM64 版本**
3. 确保 `libs/linux_arm64/libapi_c.so` 存在
4. 程序会自动添加路径

```bash
# 安装依赖
sudo apt install libgl1-mesa-glx libglib2.0-0

# 确保动态库可访问
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/RM_API2/Python/Robotic_Arm/libs/linux_arm64/
```

## 三、安装步骤

### 3.1 安装 ROS2 Foxy

ROS2 Foxy 官方支持 Ubuntu 18.04（Bionic）。

```bash
# 安装 ROS2（如果尚未安装）
sudo bash ros2_install.sh

# 或参考官方文档：https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
```

### 3.2 编译 ros2_rm_robot 功能包

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆仓库
git clone https://github.com/RealManRobot/ros2_rm_robot.git

# 或者使用本地仓库（如果已克隆到本地）
# cp -r /path/to/ros2_rm_robot ~/ros2_ws/src/

# 编译
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select rm_ros_interfaces
source ./install/setup.bash
colcon build
```

### 3.4 配置机械臂 IP

**实际环境 IP 分配：**

| 节点 | IP |
|------|-----|
| Jetson | `192.168.151.119` |
| 左臂 | `192.168.150.111` |
| 右臂 | `192.168.150.112` |

编辑左臂配置文件：
```bash
nano ~/ros2_ws/src/ros2_rm_robot/rm_driver/config/rm_65_config.yaml
```

修改 `arm_ip` 和 `udp_ip` 为实际 IP：

```yaml
rm_driver:
  ros__parameters:
    arm_ip: "192.168.150.111"   # 左臂 IP
    tcp_port: 8080
    arm_type: "RM_65"           # 根据实际型号修改
    arm_dof: 6
    udp_ip: "192.168.151.119"   # Jetson IP
    udp_cycle: 5
    udp_port: 8089
```

> 双臂模式需要分别为左臂和右臂创建两套配置文件和 launch 文件，使用不同的 namespace（如 `l_arm`、`r_arm`）进行区分。

### 3.4 安装 Python 依赖

```bash
# 安装 rclpy 和标准消息类型
pip install rclpy sensor-msgs geometry-msgs
```

## 四、使用方法

### 4.1 启动 ROS2 驱动

```bash
# 在 ros2_ws 目录下
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# 启动机械臂驱动
ros2 launch rm_driver rm_65_driver.launch.py

# 如果需要查看详细输出
ros2 launch rm_driver rm_65_driver.launch.py --show-args
```

### 4.2 运行验证程序

```bash
# 重新打开一个终端
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# 运行验证程序
python3 arm_monitor.py

# 或者如果需要指定功能包路径
ros2 run guji arm_monitor.py
```

### 4.3 运行效果

程序启动后，你应该看到类似以下输出：

```
=======================================
  睿尔曼机械臂 ROS2 连接验证
  时间: 2026-04-02 12:00:00
=======================================

[✓] ROS2 节点初始化成功
[✓] 关节状态订阅成功
[✓] 末端位姿订阅成功

实时数据:
----------------------------------------
关节角度 (度):
  Joint1: 0.00
  Joint2: -45.00
  Joint3: 45.00
  Joint4: 0.00
  Joint5: 90.00
  Joint6: 0.00

末端位姿:
  X: 0.2345 m
  Y: -0.1234 m
  Z: 0.4567 m

姿态 (欧拉角):
  Roll:  0.0000 rad
  Pitch: 1.5708 rad
  Yaw:   0.0000 rad
----------------------------------------
```

## 五、程序说明

### 5.1 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节角度 |
| `/rm_driver/udp_arm_position` | `geometry_msgs/Pose` | 末端位置 |
| `/rm_driver/udp_joint_pose_euler` | `rm_ros_interfaces/Jointposeeuler` | 欧拉角姿态 |

### 5.2 消息字段说明

**JointState:**
- `position[]` - 关节角度（弧度）

**Pose:**
- `position.x/y/z` - 末端位置（米）
- `orientation.x/y/z/w` - 末端姿态（四元数）

**Jointposeeuler:**
- `euler[]` - 欧拉角 [roll, pitch, yaw]（弧度）
- `position[]` - 位置 [x, y, z]（米）

### 5.3 单位转换

- 关节角度：ROS2 发布的是**弧度**，程序会自动转换为**度**显示
- 位置：单位是**米**
- 速度：程序中设置为百分比 0-100

## 六、常见问题

### Q1: 有线网卡没有 IP 地址？

有线网卡状态为 UP 但无 IP 时，手动配置：

```bash
sudo ip addr add 192.168.150.100/24 dev eth0
```

### Q2: 连接失败，检查哪些内容？

1. 网络是否互通：
```bash
ping 192.168.150.111   # 左臂
ping 192.168.150.112   # 右臂
```

2. 端口连接测试（TCP）：
```bash
# 测试左臂端口
nc -vz 192.168.150.111 8080

# 测试右臂端口
nc -vz 192.168.150.112 8080
```

3. 端口是否被占用：
```bash
sudo lsof -i :8080
sudo lsof -i :8089
```

4. 防火墙是否阻止：
```bash
sudo ufw allow 8080/tcp
sudo ufw allow 8089/udp
```

### Q3: 话题没有数据？

1. 检查驱动是否正常运行：
```bash
ros2 topic list
ros2 topic echo /joint_states
```

2. 检查机械臂是否使能：
```bash
ros2 service list
```

### Q4: 编译报错 CMake 版本过低？

手动修改功能包中 `CMakeLists.txt` 的 `cmake_minimum_required`，将版本要求降低。

### Q5: 动态库加载失败 (libapi_c.so)？

确认运行平台与动态库架构匹配。Jetson（ARM64）需要使用 **ARM64 版本（V1.1.3）** 的 `.so` 文件，不能使用默认的 X86-64 版本。

## 七、扩展：发送运动指令

如果需要发送运动指令，可以发布到以下话题：

```bash
# MoveJ 关节运动
ros2 topic pub /rm_driver/movej_cmd rm_ros_interfaces/msg/Movej \
    "{joint: [0.0, -0.785, 1.571, 0.0, 0.785, 0.0], speed: 50, block: true}"

# MoveL 直线运动
ros2 topic pub /rm_driver/movel_cmd rm_ros_interfaces/msg/Movel \
    "{pose: {position: {x: 0.2, y: 0.0, z: 0.3}}, speed: 50, block: true}"
```

## 八、示教记录程序 (teach_record.py)

### 8.1 功能介绍

示教记录程序用于记录拖动示教过程中的关键数据，支持导出为 CSV 文件供后续分析或回放使用。

### 8.2 记录的数据

| 数据类型 | 字段名 | 单位 | 说明 |
|---------|--------|------|------|
| 基本信息 | index, time | - | 点位索引、相对时间 |
| 关节角度 | joint_1 ~ joint_7 | 度 (°) | 最多7个关节 |
| 末端位置 | pos_x, pos_y, pos_z | 米 (m) | 笛卡尔坐标 |
| 末端姿态 | quat_w/x/y/z | - | 四元数 |
| 欧拉角 | roll, pitch, yaw | 弧度 (rad) | 姿态表示 |
| 关节速度 | speed_1 ~ speed_7 | 度/秒 (°/s) | 可选 |
| 六维力 | force_fx/fy/fz/mx/my/mz | N / N·m | 可选 |
| 夹爪状态 | gripper | 0/1 | 打开/关闭 |
| 关键标记 | marker | 0/1 | 手动标记 |

### 8.3 使用方法

```bash
# 启动示教记录程序
python3 teach_record.py

# 指定 namespace (双臂模式)
python3 teach_record.py --namespace l_arm
python3 teach_record.py --namespace r_arm

# 程序启动后自动开始记录
python3 teach_record.py --auto-start
```

### 8.4 操作说明

| 按键 | 功能 | 说明 |
|------|------|------|
| R | 开始/停止记录 | 切换记录状态 |
| 空格 | 标记关键点 | 标记抓取点、放置点等 |
| G | 切换夹爪状态 | 记录夹爪开闭动作 |
| S | 保存数据 | 导出为 CSV 文件 |
| Q | 退出程序 | 退出前询问是否保存 |

### 8.5 输出文件

数据保存到 `records/` 目录，文件格式为 CSV：

```
records/teach_record_20260402_120000.csv
```

CSV 文件包含所有记录的点位数据，可使用 Excel、Python pandas 等工具分析。

### 8.6 示例：分析记录数据

```python
import pandas as pd

# 读取记录数据
df = pd.read_csv('records/teach_record_20260402_120000.csv')

# 查看关键点位
key_points = df[df['marker'] == 1]
print(key_points[['time', 'joint_1', 'joint_2', 'joint_3', 'pos_x', 'pos_y', 'pos_z']])

# 查看夹爪动作点
gripper_changes = df[df['gripper'].diff() != 0]
print(gripper_changes[['time', 'gripper']])
```

## 九、Python SDK 方式 (RM_API2)

### 9.1 SDK 方式 vs ROS2 方式

| 对比项 | Python SDK | ROS2 |
|--------|------------|------|
| 依赖 | 仅 Python | 需要 ROS2 环境 |
| 延迟 | 更低 | 略高 |
| 功能 | 完整 API | 订阅话题 |
| 适用场景 | 快速测试、简单控制 | 复杂系统集成 |
| 双臂支持 | 需要创建两个连接 | 通过 namespace 区分 |

### 9.2 SDK 文件位置

```
RM_API2/
└── Python/
    └── Robotic_Arm/
        ├── rm_robot_interface.py    # 主接口
        ├── rm_ctypes_wrap.py        # C库封装
        └── libs/
            ├── linux_arm64/libapi_c.so # Linux ARM64
```

### 9.3 rm_joint_reader.py 使用方法

```bash
# 基本用法 - 持续读取
python3 rm_joint_reader.py

# 指定 IP
python3 rm_joint_reader.py -i 192.168.1.18

# 单次读取
python3 rm_joint_reader.py --once

# 调整读取频率 (0.1秒间隔)
python3 rm_joint_reader.py -t 0.1
```

### 9.4 API 主要函数

```python
from rm_robot_interface import RoboticArm, rm_thread_mode_e

# 创建连接
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
robot.rm_create_robot_arm("192.168.1.25", 8080)

# 读取关节角度 (单位: 度)
ret, joints = robot.rm_get_joint_degree()
print(f"关节角度: {joints}")

# 读取末端位置
ret, pose = robot.rm_get_tcp_position()
print(f"末端位置: {pose}")

# 读取完整状态
ret, state = robot.rm_get_arm_all_state()

# 断开连接
robot.rm_delete_robot_arm()
```

### 9.5 常用 API 列表

| 函数 | 说明 | 返回值 |
|------|------|--------|
| `rm_get_joint_degree()` | 获取关节角度 | `(ret, [j1,j2...])` 度 |
| `rm_get_tcp_position()` | 获取 TCP 位置 | `(ret, [x,y,z,rx,ry,rz])` 米/弧度 |
| `rm_get_end_pose()` | 获取末端位姿 | |
| `rm_get_arm_all_state()` | 获取所有状态 | 字典 |
| `rm_movej(joints, v, r, block)` | 关节运动 | |
| `rm_movel(pose, v, r, block)` | 直线运动 | |
| `rm_set_arm_stop()` | 急停 | |

### 9.6 错误码

| 错误码 | 说明 |
|--------|------|
| 0 | 成功 |
| -1 | 数据发送失败 |
| -2 | 数据接收失败 |
| -3 | 返回值解析失败 |

## 十、演示功能 (Demo)

### 10.1 电流环控制

涉及开关角度以及控制逻辑。

### 10.2 六轴力传感

使用六轴力传感功能时，需要按住机械臂上的**力传感按钮**，数据才会正常输出。

### 10.3 示教记录

拖动机械臂进行示教，记录关节角度、末端位姿、六维力等数据，导出为 CSV 文件供后续分析。

### 10.4 双臂协同控制

通过 `dual_arm_controller.py` 同时控制左臂和右臂，支持双臂协同运动。

### 10.5 双臂协同取放料控制器

通过 `dual_arm_pick_place.py` 实现完整的取料-放料流程，支持双臂协作夹取、视觉识别、力位混合放置。

## 十一、双臂取放料控制器 (dual_arm_pick_place.py)

### 11.1 功能概述

`dual_arm_pick_place.py` 实现双臂协同取放料任务，流程如下：

```
初始位置 → 识别位置 → 识别ARcode11/12 → 推对齐 → 取料(11步) →
移动到放料位置 → 识别ARcode21 → 放料(力位混合) → 回原点
```

### 11.2 运行方法

```bash
# 方式1：直接运行（需要先启动双臂驱动）
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash
python3 guji/dual_arm_pick_place.py

# 方式2：指定 namespace
python3 guji/dual_arm_pick_place.py l_arm r_arm

# 方式3：交互式使用
python3
>>> from guji.dual_arm_pick_place import DualArmPickPlaceController
>>> import rclpy
>>> rclpy.init()
>>> ctrl = DualArmPickPlaceController()
>>> ctrl.run()  # 执行完整流程
```

### 11.3 支持的控制方法

| 方法 | 说明 |
|------|------|
| `movej(arm, joints, speed, block)` | 关节运动到指定角度 |
| `movel(arm, pose, speed, block)` | 直线运动到目标位姿 |
| `gripper_set(arm, position, block, timeout)` | 设置夹爪固定位置（1~1000） |
| `gripper_pick(arm, speed, force, block, timeout)` | 力控夹取 |
| `force_position_move(arm, pose, ...)` | 力位混合运动 |
| `detect_arcode(arm, code)` | Mock ARcode 视觉识别 |
| `run()` | 执行完整取放料流程 |

### 11.4 取料核心逻辑（11步）

```
Step 1:  左臂 MoveJ → 托住位置
Step 2:  左臂夹爪闭合（托住工件）
Step 3:  左臂 MoveJ → 抬起托住位
Step 4:  右臂 MoveJ → 插入位
Step 5:  右臂力控夹取
Step 6:  右臂 MoveJ → 垂直位
Step 7:  右臂夹爪松开
Step 8:  左臂 MoveJ → 前顶位
Step 9:  左臂夹爪松开
Step 10: 右臂 MoveJ → 上移位
Step 11: 左臂 MoveJ → 安全位
```

### 11.5 放料流程（力位混合）

```
Step 1:  右臂移动到放料上方位
Step 2:  力位混合下移搜索
Step 3:  夹爪张开释放物料
Step 4:  右臂移动到退出位
Step 5-7: 夹爪压实/夹紧
Step 8:  右臂移动到安全位
```

### 11.6 注意事项

- 暂不包含导航部分和异常处理逻辑
- 视觉部分已集成 ArUco 真实识别，Service 不可用时自动降级为 Mock（见第十二节）
- 所有点位从 `config/poses.yaml` 加载，可通过示教记录替换为实际值
- 运行前需确保双臂驱动已通过 launch 文件启动
- 右臂有相机，左臂无相机（与硬件配置一致）

## 十二、相机与视觉识别系统

### 12.1 系统架构

```
RealSense D435 ──(USB)──► realsense2_camera ──(image)──► aruco_detector
                                                          │
                                                          ├──► /right_arm/detect_aruco (Service)
                                                          │
                                                          └──► TF: camera_right → target_right_camera

tf_broadcaster ──(static TF)──► right_base → camera_right
                                            ↓
                                      dual_arm_pick_place
                                            │
                                            └──► TF: target_right_camera → right_base
```

### 12.2 文件清单

| 文件 | 说明 |
|------|------|
| `config/camera.yaml` | 相机参数（序列号、内参、手眼标定结果、ArUco 参数） |
| `config/system.yaml` | 系统全局参数（速度、夹爪、视觉参数） |
| `nodes/camera_bridge.py` | 相机诊断节点，验证图像流、帧率、深度有效性 |
| `nodes/tf_broadcaster.py` | 手眼标定 TF 广播节点 |
| `nodes/aruco_detector.py` | ArUco 标记检测节点，发布 `/right_arm/detect_aruco` Service |
| `srv/DetectAruco.srv` | 视觉识别 Service 接口定义 |

### 12.3 启动方法

```bash
# 方式1：一键启动（推荐）
ros2 launch guji vision_pipeline.launch.py camera_serial:=你的序列号

# 方式2：分步启动（便于排查问题）
# 终端1：启动相机驱动
ros2 run realsense2_camera realsense2_camera_node \
  _serial_no:=你的序列号 \
  _enable_color:=true \
  _enable_depth:=true \
  _align_depth:=true

# 终端2：启动相机诊断
ros2 run guji camera_bridge

# 终端3：启动手眼标定 TF
ros2 run guji tf_broadcaster

# 终端4：启动 ArUco 检测
ros2 run guji aruco_detector

# 终端5：启动主控制器
ros2 run guji dual_arm_pick_place
```

### 12.4 Service 调用示例

```bash
# 调用 ArUco 检测（识别 ID=12 的标记，超时 5 秒）
ros2 service call /right_arm/detect_aruco guji/srv/DetectAruco \
  "{marker_id: 12, timeout: 5.0}"

# 查询相机状态
ros2 service call /right_arm/get_camera_status std_srvs/srv/Trigger "{}"
```

### 12.5 相机诊断检查项目

| 检查项 | 方法 | 期望值 |
|--------|------|--------|
| Color 图像流 | `ros2 topic hz /camera_right/color/image_raw` | ≥ 15Hz |
| Depth 图像流 | `ros2 topic hz /camera_right/aligned_depth...` | ≥ 15Hz |
| 相机内参 | `ros2 topic echo /camera_right/color/camera_info` | K 矩阵 9 个数，D 矩阵 ≥ 5 个数 |
| 深度有效性 | 深度图像中 >0 像素比例 | ≥ 50% |

### 12.6 手眼标定验证

```bash
# 查看 TF 关系
ros2 run tf2_ros tf2_echo right_base camera_right

# 可视化 TF 树
ros2 run tf2_ros view_frames

# 验证坐标系名称
ros2 run tf2_ros run
```

标定正确的判断标准：在相机视野内放置 ArUco 标记，
`tf2_echo right_base target_right_camera` 显示的 Z 坐标应该接近标记实际高度（误差 < 5mm）。

## 十三、配置文件说明

### 13.1 poses.yaml — 关节角度配置

```yaml
poses:
  left:
    initial: [0, -30, 50, 0, 80, 0]    # 6 个关节角度，单位：度
    recognize: [30, -20, 40, 0, 90, 0]
    ...
  right:
    initial: [0, -30, 50, 0, 80, 0]
    ...
```

示教替换方法：
1. 手动示教记录各状态的关节角度
2. 将示教值填入对应状态
3. 重启程序或重新加载 YAML

### 13.2 camera.yaml — 相机与标定配置

```yaml
camera:
  serial_number: "REPLACE_WITH_YOUR_SERIAL_NUMBER"  # ← 修改为你的相机序列号
  hand_eye_right:
    translation:
      x: 0.0850   # ← 手眼标定测定后填入
      y: -0.0400
      z: 0.0100
    quaternion:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  aruco:
    dict_type: "DICT_5X5_1000"    # 与 Warehouse 项目一致
    marker_size: 0.030            # ← 修改为实际标记物边长（米）
```

### 13.3 system.yaml — 系统全局参数

| 参数路径 | 说明 | 默认值 |
|----------|------|--------|
| `arm.movej_default_speed` | MoveJ 默认速度 | 30 |
| `gripper.pick_speed` | 力控夹取速度 | 300 |
| `gripper.pick_force` | 力控夹取力阈值 | 300 |
| `gripper.position_open` | 夹爪全开 | 1000 |
| `vision.timeout` | 视觉识别超时 | 5.0s |
| `vision.stability_threshold` | 角度稳定性阈值 | 0.1 rad |

## 十四、启动检查与验证流程

### 14.1 启动检查函数

主控制器提供 `startup_checks()` 方法，可在 `run()` 中自动执行或手动调用：

```python
controller = DualArmPickPlaceController()
controller.startup_checks()  # 逐项检查，打印结果
controller.run()               # 启动检查通过后执行流程
controller.run(skip_checks=True)  # 调试模式，跳过检查
```

检查结果格式：
```
  [   OK   ] 关节状态数据
  [   OK   ] ArUco 视觉服务
  [   OK   ] TF 变换树
  [   OK   ] 相机状态
```

### 14.2 完整验证步骤

| 步骤 | 操作 | 验证方法 |
|------|------|---------|
| 1 | 启动 realsense2_camera | `ros2 topic list` 看到 `/camera_right/...` |
| 2 | 启动 camera_bridge | 每 5s 输出诊断日志，无 `[WARN]` |
| 3 | 启动 tf_broadcaster | `ros2 run tf2_ros tf2_echo right_base camera_right` |
| 4 | 启动 aruco_detector | `ros2 service list` 看到 `/right_arm/detect_aruco` |
| 5 | 调用视觉 Service | 放置 ArUco 标记，`ros2 service call ...` 返回 `found=True` |
| 6 | 启动主控制器 | `startup_checks()` 全部通过 |
| 7 | 运行完整流程 | `controller.run()` 无报错 |

### 14.3 常见问题排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| `camera_right` 话题不存在 | realsense2_camera 未启动 | `ros2 launch realsense2_camera rs_camera.launch.py` |
| 相机帧率 < 15Hz | USB 带宽不足 | 降低分辨率或帧率，或更换 USB3.0 口 |
| CameraInfo 全零 | realsense2_camera 版本问题 | 更新驱动或检查参数 |
| ArUco 检测返回 `found=False` | 标记不在视野或 ID 错误 | 调整机械臂姿态，确认 `marker_size` 正确 |
| TF 查询失败 | `right_base` 未广播 | 检查 arm_driver 节点是否运行 |
| YAML 加载失败 | 文件路径不对 | 检查 config 目录是否在 guji 包下 |

## 十五、参考链接

- [睿尔曼 ROS2 功能包](https://github.com/RealManRobot/ros2_rm_robot)
- [ROS2 Foxy 官方文档](https://docs.ros.org/en/foxy/)
- [MoveIt2 官方文档](https://moveit.ros.org/)
