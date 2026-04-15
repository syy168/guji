# ros2_navigation_topic_test

本目录提供两部分内容：

1. `Doc/Warehouse导航集成总结与ROS2迁移说明.md`：
   - 总结 Warehouse_handling_robot 的导航软件与接入方式。
   - 解释 ROS1 与 ROS2 Foxy 的不可照抄点。

2. `src/woosh_nav_feedback`：
   - 仅底盘导航（不涉及机械臂）的 ROS2 Foxy 示例包。
   - 实现“开始、进行中、结束”反馈确认。

## 1. 接口约定

节点：`exec_task_feedback_node`

输入：
- 目标点位：`/navigation/go_mark` (`std_msgs/String`)

输出：
- 统一反馈：`/navigation/feedback` (`std_msgs/String`，JSON 字符串)

反馈阶段：
- `START_REQUEST`
- `START_CONFIRMED`
- `IN_PROGRESS`
- `END_CONFIRMED`
- `TIMEOUT`

## 2. 编译

在该目录作为 ROS2 工作区根目录使用：

```bash
cd /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test
colcon build --packages-select woosh_nav_feedback
source install/setup.bash
```

## 3. 运行

先启动 Woosh ROS2 Agent（示例）：

```bash
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"
```

再启动反馈节点：

```bash
ros2 launch woosh_nav_feedback exec_task_feedback.launch.py
```

发送导航目标（示例点位名）：

```bash
ros2 topic pub --once /navigation/go_mark std_msgs/msg/String "{data: '7613B5D'}"
```

查看反馈：

```bash
ros2 topic echo /navigation/feedback
```

## 4. 常用参数

- `wait_for_taskable`：是否要求机器人处于可接任务状态。
- `task_timeout_sec`：超时秒数，超时会请求取消。
- `task_type`、`direction`、`task_type_no`：透传到底盘任务参数。
- `startup_mark_no`：若非空，节点启动后自动下发一次导航。
