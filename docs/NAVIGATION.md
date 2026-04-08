# AGV 导航系统集成文档

本文档说明如何将 AGV 导航功能集成到双臂取放料系统中。

---

## 一、系统架构

### 1.1 整体架构

```
┌─────────────────────────────────────────────────────────┐
│              DualArmPickPlaceController                  │
│                  (主控制器)                              │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐  │
│  │  机械臂控制  │  │  视觉识别   │  │  AGV 导航   │  │
│  └──────┬──────┘  └──────┬──────┘  └──────┬───────┘  │
└─────────┼────────────────┼────────────────┼───────────┘
          │                │                │
          ▼                ▼                ▼
   ┌────────────┐   ┌────────────┐   ┌────────────────┐
   │ rm_driver  │   │ aruco_     │   │ ros2_agv_robot │
   │            │   │ detector   │   │ /exec_task     │
   └────────────┘   └────────────┘   └───────┬────────┘
                                              │
                                              ▼
                                     ┌──────────────────┐
                                     │   Woosh AGV 底盘  │
                                     │   + wooshslam    │
                                     └──────────────────┘
```

### 1.2 TF 坐标关系

```
world ──► agv_base ──► right_base / left_base ──► right_top / left_top
                        │
                        └──► camera_right (手眼标定)
```

---

## 二、硬件准备

### 2.1 必需硬件

| 设备 | 说明 |
|------|------|
| Woosh AGV 底盘 | 支持 ROS2 接口的移动底盘 |
| 双臂机械臂 | 睿尔曼 RM-65 双臂 |
| RealSense D435 | 视觉识别相机 |

### 2.2 网络配置

| 设备 | IP 地址 |
|------|---------|
| 计算机 (Jetson) | 192.168.151.119 |
| 左臂 | 192.168.150.111 |
| 右臂 | 192.168.150.112 |
| AGV 底盘 | 169.254.128.2 |

---

## 三、软件配置

### 3.1 安装 ros2_agv_robot

参考 `ros2_ws/src/ros2_agv_robot/README.md`：

```bash
# 安装底盘 ros2 接口安装包
cd ~/ros2_agv_robot/lib
sudo ./ros-foxy-woosh-robot-agent_0.0.1-0focal_arm64.run

# 或使用预编译包
sudo apt install ros-foxy-woosh-robot-agent
```

### 3.2 配置 navigation.yaml

编辑 `guji/config/navigation.yaml`：

```yaml
agv:
  enabled: true  # 改为 true 启用导航
  navigation_timeout: 120.0

  topics:
    mark_topic: 'goto_mark/go'
    result_topic: 'goto_mark/result'
    status_topic: 'robot_status'

  marks:
    pick:
      transition: '110'   # 取料区过渡点
      main: '11'          # 取料点
    place:
      transition: '210'   # 放料区过渡点
      main: '21'          # 放料点
    charge:
      main: '120'         # 充电点

  workflow:
    pick_sequence:
      - { mark: '110', task_type: 1, description: '导航到取料区过渡点' }
      - { mark: '11',  task_type: 1, description: '导航到取料点' }

    place_sequence:
      - { mark: '110', task_type: 1, description: '返回过渡点' }
      - { mark: '210', task_type: 1, description: '导航到放料区过渡点' }
      - { mark: '21',  task_type: 1, description: '导航到放料点' }
```

### 3.3 采集导航点位

使用 Woosh Design 软件采集实际环境的导航点位：

1. 手动将 AGV 移动到各个目标位置
2. 在 Woosh Design 中记录点位名称（mark_no）
3. 将点位名称填入 `navigation.yaml`

---

## 四、启动流程

### 4.1 启动顺序

```bash
# 终端 1：启动 AGV 底盘连接节点
ros2 run woosh_robot_agent agent --ros-args \
  -r __ns:=/woosh_robot \
  -p ip:="169.254.128.2"

# 终端 2：启动机械臂驱动
ros2 launch ros2_rm_robot dual_rm_65_driver.launch.py

# 终端 3：启动相机驱动
ros2 launch realsense2_camera rs_d435.launch.py

# 终端 4：启动视觉节点
ros2 launch guji vision_pipeline.launch.py

# 终端 5：启动主控制器（含导航）
python3 guji/dual_arm_pick_place.py
```

### 4.2 验证 AGV 连接

```bash
# 查看话题
ros2 topic list | grep -E "goto_mark|robot_status"

# 测试导航（手动）
ros2 run ros2_agv_robot exectask --ros-args -p mark_no:=11
```

---

## 五、使用方法

### 5.1 API 接口

```python
from guji.dual_arm_pick_place import DualArmPickPlaceController

controller = DualArmPickPlaceController()

# ===== 导航基础操作 =====
# 导航到单个点位（组合调用）
controller.agv_navigate('11')

# 导航到单个点位（分离调用，可中间处理）
carry = controller.agv_navigate('11', task_type=1, wait=False)
# ... 中间处理，如调整升降机高度 ...
controller.wait_until_reached(carry)

# 取消导航
controller.agv_cancel()

# 立即停止
controller.agv_stop()

# ===== 序列导航 =====
# 按序列依次导航
sequence = [
    {'mark': '110', 'task_type': 1, 'description': '到过渡点'},
    {'mark': '11',  'task_type': 1, 'description': '到取料点'},
]
controller.agv_navigate_sequence(sequence)

# ===== 完整流程 =====
# 执行含导航的完整取放料流程
controller.run_with_navigation()
```

### 5.2 命令行使用

```bash
# 执行含导航的完整流程
python3 guji/dual_arm_pick_place.py

# 跳过启动检查
python3 guji/dual_arm_pick_place.py --skip-checks
```

### 5.3 单独使用导航控制器

```bash
# 连接到真实 AGV
python3 guji/nodes/agv_navigator.py --mark 11

# 使用模拟器（无需真实 AGV）
python3 guji/nodes/agv_navigator.py --sim --mark 11
```

---

## 六、导航状态与回调

### 6.1 Woosh task_state 状态值

| 值 | 状态 | 说明 |
|----|------|------|
| 0 | 空闲 | 无任务执行 |
| 1 | 运行中 | 正在移动 |
| 2 | 暂停 | 任务暂停 |
| 7 | **完成** | 到达目标点 |
| 8 | **失败** | 任务失败 |

### 6.2 导航等待机制

```python
# AGVNavigator 内部机制：
# 1. navigate() 发布目标点到 goto_mark/go
# 2. wait_until_reached() 等待以下任一条件：
#    - 收到 goto_mark/result 消息
#    - task_state == 7 (完成)
#    - task_state == 8 (失败)
#    - 超时
```

---

## 七、仿真模式

### 7.1 使用模拟器

当没有真实 AGV 底盘时，可以使用模拟器进行开发和测试：

```bash
python3 guji/nodes/agv_navigator.py --sim --mark 11 --timeout 5
```

### 7.2 配置模拟器

在 `navigation.yaml` 中启用模拟模式：

```yaml
agv:
  enabled: true

  debug:
    simulation: true   # 启用模拟器
    sim_speed: 2.0     # 模拟移动速度（秒）
```

模拟器会自动：
- 接收 `goto_mark/go` 话题的目标点
- 模拟 AGV 移动过程（线性插值）
- 在 `sim_speed` 秒后发布 `goto_mark/result`
- 发布 `task_state=7` 表示完成

---

## 八、调试与故障排除

### 8.1 查看导航状态

```bash
# 查看 AGV 话题
ros2 topic list | grep -E "goto|robot"

# 实时查看导航目标
ros2 topic echo /goto_mark/go

# 实时查看底盘状态
ros2 topic echo /robot_status

# 查看控制器列表
ros2 controller list
```

### 8.2 常见问题

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| 导航超时 | AGV 未连接 / 点位错误 | 检查网络 / 确认 mark_no |
| task_state=8 | 路径障碍 / 点位不可达 | 检查环境 / 重新采集点位 |
| 无法导入 agv_navigator | PYTHONPATH 未设置 | `source ros2_ws/install/setup.bash` |
| navigation.yaml 加载失败 | YAML 格式错误 | 检查文件语法 |

### 8.3 日志分析

```python
# 在代码中启用详细日志
self.get_logger().info(f'导航目标: mark_no={mark_no}')
self.get_logger().info(f'task_state={self._nav.get_task_state()}')
self.get_logger().info(f'robot_state={self._nav.get_robot_state()}')
```

---

## 九、与 Warehouse_handling_robot 的对比

| 对比项 | Warehouse_handling_robot | dual_arm_pick_place + AGV |
|--------|--------------------------|---------------------------|
| 平台 | ROS1 (Melodic) | ROS2 (Foxy) |
| 导航方法 | `_navigation_plan()` + `_navigation_wait()` | `agv_navigate()` + 序列 |
| 状态监听 | `task_state==7` | 同 |
| 点位管理 | Woosh Design | YAML 配置文件 |
| 模拟器 | 无 | `AGVNavigatorSimulated` |

---

## 十、扩展功能

### 10.1 多楼层支持

参考 Warehouse_handling_robot 的升降机控制：

```python
# 在 AGV 到达过渡点后，调整升降机高度
controller.agv_navigate('110')  # 到达过渡点
controller.set_lift_height(500)  # 调整升降机
controller.agv_navigate('11')    # 到达取料点
```

### 10.2 自主充电

```python
# 检测电量低时自动导航到充电桩
if self.get_battery_level() < 20:
    controller.agv_navigate('120', task_type=3)  # task_type=3 为充电
```

### 10.3 路径重规划

当检测到障碍物时：

```python
# 取消当前导航
controller.agv_cancel()

# 等待障碍物移除
time.sleep(5)

# 重新规划
controller.agv_navigate('11')
```
