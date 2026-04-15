# demo_pick_place 导航接入总控文档

## 1. 目的

本文件用于沉淀当前导航接入工作现状。
在完成 demo_pick_place 端测试后，可直接让后续 AI 先阅读本文件，再执行代码填充接入。

## 2. 当前结论（已确认）

1. Warehouse_handling_robot 使用的是 Woosh 底盘导航。
2. ROS1 方案核心机制是：发任务、收状态、等待完成（含超时）。
3. 在 ROS2 Foxy 下不能照抄 ROS1 的 service + topic 方式，应采用 Woosh 的 Action + Topic 组合。
4. 将导航填充进 demo_pick_place 是可行且相对方便的，因为流程步骤已预留导航占位。

## 3. 已有交付物（已完成）

1. 导航分析文档：
   - /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test/Doc/Warehouse导航集成总结与ROS2迁移说明.md

2. 导航示例包（仅导航，不含机械臂）：
   - /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test/src/woosh_nav_feedback
   - 核心节点文件：
     - /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test/src/woosh_nav_feedback/woosh_nav_feedback/exec_task_feedback_node.py
   - 启动文件：
     - /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test/src/woosh_nav_feedback/launch/exec_task_feedback.launch.py

3. 示例包能力：
   - 接收目标点位
   - 发送 Woosh ExecTask Action
   - 输出开始、进行中、结束、超时反馈

## 4. demo_pick_place 侧建议接入点（已梳理）

以下是最小改造面（优先按此执行）：

1. 扩展步骤配置模型：
   - /home/feiguang/桌面/guji/workspace/demo_pick_place/demo/config_model.py
   - 增加导航相关字段（目标、超时、失败策略）

2. 扩展步骤执行分支：
   - /home/feiguang/桌面/guji/workspace/demo_pick_place/demo/sequence_runner.py
   - 在步骤执行逻辑中新增 navigate 分支

3. 扩展机器人 IO 层：
   - /home/feiguang/桌面/guji/workspace/demo_pick_place/demo/robot_io.py
   - 加入导航客户端调用、反馈处理、超时处理

4. 更新流程配置：
   - /home/feiguang/桌面/guji/workspace/demo_pick_place/config/demo_config.yaml
   - 将步骤 8/9 从 pause 占位替换为 navigate

## 5. 推荐接入方法（后续实现应遵循）

1. 采用项目内集成方式，不额外增加外部编排节点。
2. 在 demo_pick_place 内直接调用 Woosh 导航接口。
3. 保持统一反馈语义，至少包含以下阶段：
   - START_REQUEST
   - START_CONFIRMED
   - IN_PROGRESS
   - END_CONFIRMED
   - TIMEOUT
4. 出错处理必须明确：
   - 目标无效
   - 服务器不可用
   - 导航失败
   - 超时

## 6. 你测试完成后的标准交接口令（给下一个 AI）

可直接将下面这段话发给 AI：

请先阅读以下文件：
1) /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test/Doc/demo_pick_place导航接入总控文档.md
2) /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test/Doc/Warehouse导航集成总结与ROS2迁移说明.md
3) /home/feiguang/桌面/guji/workspace/ros2_navigation_topic_test/src/woosh_nav_feedback/woosh_nav_feedback/exec_task_feedback_node.py

然后把导航能力接入到 /home/feiguang/桌面/guji/workspace/demo_pick_place，要求：
- 不涉及机械臂算法改造
- 仅新增导航步骤类型与反馈闭环
- 修改尽量集中在 config_model.py、sequence_runner.py、robot_io.py、demo_config.yaml
- 完成后给出可执行测试命令与失败场景验证方法

## 7. 后续接入验收标准

1. demo_pick_place 能识别 navigate 步骤并执行。
2. 导航开始、过程、结束有清晰日志或状态输出。
3. 超时与失败分支可触发且有明确处理。
4. 原有非导航步骤不回归失败。

## 8. 备注

当前文档是总控摘要文档。
后续所有接入改动应优先遵循本文件与 Warehouse 导航迁移说明中的约束。