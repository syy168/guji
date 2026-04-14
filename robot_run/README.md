# 睿尔曼机械臂 ROS2 连接验证程序

## 一、项目简介

本程序用于验证与睿尔曼机械臂的 ROS2 通信连接，实时显示机械臂的关节角度和末端位姿数据。

## 二、环境依赖

### 2.1 硬件要求

| 设备 | 要求 |
|------|------|
| 睿尔曼机械臂 | RM65 / RM75 / RM63 / ECO65 / ECO63 / GEN72 系列 |
| 计算机 | 与机械臂同一网络，建议有线连接 |
| 网线 | 千兆网线 |

### 2.2 软件环境

| 软件 | 版本要求 |
|------|----------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| Python | 3.9+ |

### 2.3 网络配置

机械臂默认 IP：`192.168.1.25`
计算机 IP：需设置为同一网段（如 `192.168.1.10`）

**配置方法：**

```bash
# 临时配置（重启后失效）
sudo ip addr add 192.168.1.10/24 dev eth0

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
        - 192.168.1.10/24
      nameservers:
        addresses:
          - 8.8.8.8
```

```bash
sudo netplan apply
```

## 三、安装步骤

### 3.1 安装 ROS2 Humble

```bash
# 安装 ROS2（如果尚未安装）
sudo bash ros2_install.sh

# 或参考官方文档：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
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
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_ros_interfaces
source ./install/setup.bash
colcon build
```

### 3.3 配置机械臂 IP

编辑配置文件：
```bash
nano ~/ros2_ws/src/ros2_rm_robot/rm_driver/config/rm_65_config.yaml
```

修改 `arm_ip` 为你的机械臂 IP：
```yaml
rm_driver:
  ros__parameters:
    arm_ip: "192.168.1.25"   # 修改为你的机械臂IP
    tcp_port: 8080
    arm_type: "RM_65"        # 根据实际型号修改
    arm_dof: 6
    udp_ip: "192.168.1.10"   # 修改为你的计算机IP
    udp_cycle: 5
    udp_port: 8089
```

### buxvyao3.4 安装 Python 依赖

```bash
# 安装 rclpy 和标准消息类型
pip install rclpy sensor-msgs geometry-msgs
```

## 四、使用方法

### 4.1 启动 ROS2 驱动

```bash
# 在 ros2_ws 目录下
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
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
source /opt/ros/humble/setup.bash
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

### Q1: 连接失败，检查哪些内容？

1. 网络是否互通：
```bash
ping 192.168.1.25
```

2. 端口是否被占用：
```bash
sudo lsof -i :8080
sudo lsof -i :8089
```

3. 防火墙是否阻止：
```bash
sudo ufw allow 8080/tcp
sudo ufw allow 8089/udp
```

### Q2: 话题没有数据？

1. 检查驱动是否正常运行：
```bash
ros2 topic list
ros2 topic echo /joint_states
```

2. 检查机械臂是否使能：
```bash
ros2 service list
```

### Q3: 机械臂 IP 不是默认的？

修改 `rm_65_config.yaml` 中的 `arm_ip` 参数。

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

## 九、参考链接

- [睿尔曼 ROS2 功能包](https://github.com/RealManRobot/ros2_rm_robot)
- [ROS2 Humble 官方文档](https://docs.ros.org/en/humble/)
- [MoveIt2 官方文档](https://moveit.ros.org/)
