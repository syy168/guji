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

Action 名称（实测确认）：
- 必须使用 `/woosh_robot/robot/ExecTask`
- 不是 `/woosh_robot/exec_task`

## 2. 编译

在该目录作为 ROS2 工作区根目录使用：

```bash
cd /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test
colcon build --packages-select woosh_nav_feedback
source install/setup.bash
```

## 3. 运行

处理导航服务前，必须先启动 Woosh ROS2 Agent：

```bash
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="192.168.150.118"
```

当前环境按以上 IP 写法直接执行即可。

确认 Action Server 已上线：

```bash
ros2 action list
```

期望输出中包含：

```text
/woosh_robot/robot/ExecTask
```

如果看不到该 Action，说明 Agent 还没成功就绪（例如 IP 未连通），此时启动客户端会停在等待服务器阶段。

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
