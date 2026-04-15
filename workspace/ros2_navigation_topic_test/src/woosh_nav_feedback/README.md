# woosh_nav_feedback

ROS2 Foxy 下的 Woosh 导航反馈最小示例。

功能：
- 订阅导航目标点位；
- 通过 `woosh_robot/robot/ExecTask` Action 发起导航；
- 通过反馈和结果发布统一状态；
- 提供开始、进行中、结束、超时确认。

主要节点：
- `exec_task_feedback_node`

主要话题：
- 输入：`/navigation/go_mark` (`std_msgs/String`)
- 输出：`/navigation/feedback` (`std_msgs/String`)
