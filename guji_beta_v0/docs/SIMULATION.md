# Gazebo 仿真测试指南

本文档说明如何使用 Gazebo + MoveIt2 对双臂取放料系统进行仿真测试。

---

## 一、环境准备

### 1.1 确认 ros2_ws 已编译

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# 确认包可用
ros2 pkg list | grep dual_rm
```

预期输出应包含：
- `dual_rm_gazebo`
- `dual_rm_65b_moveit_config`
- `dual_rm_description`

### 1.2 确认 Gazebo 已安装

```bash
gazebo --version
```

如果未安装：
```bash
sudo apt install ros-foxy-gazebo-*
```

---

## 二、基础仿真测试

### 2.1 仅启动 Gazebo 仿真环境

```bash
# 终端 1：启动 Gazebo 双臂仿真
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

ros2 launch dual_rm_gazebo dual_rm_65b_gazebo.launch.py
```

等待 Gazebo 窗口出现，显示双臂机器人模型。

### 2.2 启动 MoveIt2 + RViz 可视化

```bash
# 终端 2：启动 MoveIt2 + RViz
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

ros2 launch dual_rm_65b_moveit_config demo.launch.py
```

RViz 窗口将打开，显示机器人模型和运动规划界面。

### 2.3 手动验证双臂运动

在 RViz 中：
1. 选择 `Interact` 标签
2. 拖动任一机械臂的末端
3. 点击 `Plan` 规划路径
4. 点击 `Execute` 执行运动
5. 观察 Gazebo 中的机器人同步运动

### 2.4 验证话题发布

```bash
# 新开终端 3：查看话题
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# 查看关节状态
ros2 topic list | grep joint_states

# 查看控制器状态
ros2 topic list | grep arm_controller

# 查看控制器列表
ros2 controller list
```

预期话题：
```
/left_arm_controller/follow_joint_trajectory/goal
/left_arm_controller/follow_joint_trajectory/result
/left_arm_controller/follow_joint_trajectory/feedback
/left_arm_controller/state
/right_arm_controller/follow_joint_trajectory/goal
/right_arm_controller/follow_joint_trajectory/result
/right_arm_controller/follow_joint_trajectory/feedback
/right_arm_controller/state
/joint_states
```

---

## 三、使用 guji 仿真适配器

### 3.1 启动仿真适配器

在完成 2.1 和 2.2 后，执行：

```bash
# 终端 3：启动 guji 仿真适配器
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

python3 guji/nodes/gazebo_arm_adapter.py
```

适配器会自动：
- 订阅 `/joint_states` 获取 Gazebo 中的关节角度
- 将 `movej_cmd` 命令转换为 `/follow_joint_trajectory/goal`
- 模拟真实的控制接口给 `dual_arm_controller.py` 使用

### 3.2 运行 guji 控制器（仿真模式）

```bash
# 终端 4：运行 guji 双臂控制器（仿真模式）
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

python3 guji/dual_arm_controller.py --use_gazebo
```

参数说明：
- `--use_gazebo`：启用 Gazebo 仿真模式，话题自动切换到 Gazebo 接口

### 3.3 运行完整取放料流程（仿真模式）

```bash
# 终端 4：运行取放料控制器（仿真模式，跳过视觉）
python3 guji/dual_arm_pick_place.py --use_gazebo --skip_vision
```

参数说明：
- `--use_gazebo`：使用 Gazebo 仿真
- `--skip_vision`：跳过 ArUco 视觉识别（仿真中没有相机）

---

## 四、一键启动脚本

### 4.1 仿真启动脚本

```bash
# 在 ros2_ws 目录下执行
./guji/scripts/sim_start.sh
```

脚本内容：
```bash
#!/bin/bash
# 启动 Gazebo + MoveIt2 + guji 仿真适配器

source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# 后台启动 Gazebo
gnome-terminal -- bash -c "ros2 launch dual_rm_gazebo dual_rm_65b_gazebo.launch.py" &

# 等待 Gazebo 启动
sleep 5

# 后台启动 MoveIt2
gnome-terminal -- bash -c "ros2 launch dual_rm_65b_moveit_config demo.launch.py" &

# 等待 RViz 启动
sleep 3

# 启动仿真适配器
python3 guji/nodes/gazebo_arm_adapter.py
```

---

## 五、仿真模式与真实模式的对比

| 对比项 | 仿真模式 (`--use_gazebo`) | 真实模式 |
|--------|--------------------------|---------|
| 关节状态来源 | `/joint_states` (Gazebo) | `/left_arm_controller/joint_states` |
| 运动命令目标 | `/follow_joint_trajectory/goal` | `/rm_driver/movej_cmd` |
| 时间同步 | `use_sim_time=True` | `use_sim_time=False` |
| 视觉系统 | 不可用（需 Mock） | ArUco 检测可用 |
| 夹爪控制 | 不可用（需适配） | Gripperset 可用 |
| 力位混合 | 不可用 | Forcepositionmovepose 可用 |
| 物理精度 | 低（简化模型） | 高（真实机械） |

---

## 六、已知限制

1. **夹爪不仿真**：Gazebo 中的 gripper 模型较简单，夹爪命令需要额外适配
2. **力控不仿真**：力位混合控制需要 Gazebo 力和传感器插件
3. **视觉不仿真**：ArUco 检测需要相机话题输入
4. **相机话题不仿真**：需要额外的相机仿真节点发布模拟图像
5. **导航不仿真**：AGV 导航需要真实的 Woosh 底盘或模拟器

---

## 七、调试技巧

### 7.1 查看控制器状态

```bash
ros2 controller list
ros2 controller info left_arm_controller
```

### 7.2 直接发送关节角度命令

```bash
# 通过 ros2 control 发送关节命令
ros2 topic pub /left_arm_controller/joint_trajectory_controller/cmd std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" -1
```

### 7.3 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

### 7.4 查看节点关系图

```bash
ros2 run rqt_graph rqt_graph
```

---

## 八、退出仿真

```bash
# 在各终端按 Ctrl+C 正常退出
# 或强制关闭所有节点
ros2 node list
ros2 node kill /<node_name>
```
