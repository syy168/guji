# Warehouse_handling_robot 导航集成总结与 ROS2 Foxy 迁移说明

## 1. 结论先行

- 导航软件：`Woosh` 底盘导航（README 明确写到“基于 Woosh 底盘的 SLAM 建图与导航”）。
- ROS1 项目中，导航任务由业务脚本触发，底盘接口作为独立能力接入，不依赖机械臂逻辑才能运行。
- 迁移到 ROS2 Foxy 时，不应照抄 ROS1 的 `service + topic` 形式；应改为 Woosh 在 ROS2 下提供的 `Action + Topic` 形式。

## 2. Warehouse_handling_robot 在 ROS1 中如何接入导航

## 2.1 从 README 看使用方式

README 的核心导航流程是：

1. 在 Woosh 软件中建图。
2. 在 Woosh Design 中部署点位（储位/充电点），并上传到机器人。
3. 业务侧通过点位名（如 120、101、102 等）请求导航。

这说明导航地图、点位管理由 Woosh 体系维护，ROS 程序只负责“发导航任务 + 读导航状态”。

## 2.2 代码级接入模式（ROS1）

在 `body_handling_demo/scripts/body_handling_action.py` 中，导航相关逻辑可抽象为三步：

1. 发起导航
- 通过 `rospy.ServiceProxy("/exec_task", ExecTask)` 请求底盘执行目标点位。
- 在 `_navigation_plan()` 中填写 `task_id/task_type/mark_no` 等参数。

2. 过程反馈
- 订阅 `/robot_status`（`RobotStatus`）。
- 在 `_woosh_status()` 中读取 `task_state/work_mode/robot_state`。

3. 结束确认
- 当 `task_state == 7` 视为任务完成。
- 通过 `threading.Event` + 定时器实现 `_navigation_wait()`：
  - 等待成功信号；
  - 超时则判失败。

可见它的本质是：
- “开始”：发送导航请求并检查请求是否成功；
- “进行中”：持续监听状态；
- “结束”：明确完成/失败/超时。

## 3. ROS1 -> ROS2 Foxy 的关键差异（不能照抄处）

## 3.1 通信模型变化

ROS1（Warehouse）常见写法：
- `/exec_task`：Service
- `/robot_status`：Topic

ROS2 Foxy（你当前工程已有示例）：
- `woosh_robot/robot/ExecTask`：Action（目标、反馈、结果三段式）
- `woosh_robot/robot/TaskProc`：任务进度 Topic
- `woosh_robot/robot/OperationState`：机器人是否可接任务等运行状态 Topic

因此不能简单平移 ROS1 代码，需要改为 Action Client 模式。

## 3.2 任务完成判定变化

ROS1 中你看到的是 `task_state == 7`。
ROS2 中建议用 Woosh 消息定义常量判定，例如：
- `woosh_task_msgs/msg/State::K_COMPLETED`
- `K_FAILED`
- `K_CANCELED`

## 3.3 建议保留的业务抽象

虽然接口变了，但业务抽象可保持一致：

1. `plan(mark_no)`：下发目标点位。
2. `wait_or_feedback()`：接收过程反馈。
3. `confirm_result()`：完成/失败/取消/超时。

## 4. 面向你项目的最小导航集成方案（不含机械臂）

只做底盘导航闭环，节点职责如下：

1. 订阅目标点位命令（例如 `/navigation/go_mark`，消息 `std_msgs/String`）。
2. Action 客户端连接 `woosh_robot/robot/ExecTask`。
3. 订阅 `woosh_robot/robot/OperationState`，判断是否可接任务。
4. 发布统一反馈主题（例如 `/navigation/feedback`）：
- `START_REQUEST`
- `START_CONFIRMED`
- `IN_PROGRESS`
- `END_CONFIRMED`
- `TIMEOUT` 或 `REJECTED`

这样你后续无论挂自动流程、状态机还是 UI，都能用统一反馈做联动。

## 5. 本次交付对应

已在 `workspace/ros2_navigation_topic_test` 下提供：

1. 本文档（本文件）。
2. `src/woosh_nav_feedback` ROS2 Foxy 示例包：
- 仅导航能力；
- Woosh Action 接入；
- 具备开始/进行/结束反馈确认。
