# woosh_nav_feedback

ROS2 Foxy 下的 Woosh 导航反馈最小示例。

功能：
- 订阅导航目标点位；
- 通过 `/woosh_robot/robot/ExecTask` Action 发起导航；
- 通过反馈和结果发布统一状态；
- 提供开始、进行中、结束、超时确认。

注意：
- Action 全名必须与 `ros2 action list` 一致，当前实测为 `/woosh_robot/robot/ExecTask`。
- 若 Action 不在列表中，优先检查 woosh_robot_agent 是否已成功启动并连通底盘。
- 处理导航服务前先运行：`ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="192.168.150.118"`。

主要节点：
- `exec_task_feedback_node`

主要话题：
- 输入：`/navigation/go_mark` (`std_msgs/String`)
- 输出：`/navigation/feedback` (`std_msgs/String`)
