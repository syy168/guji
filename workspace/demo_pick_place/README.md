# Pick-Place 演示 (基于 robot_run)

本演示按项目描述 0-11 流程组织，核心执行模式为：
每个机械臂动作对应一条录制轨迹文件，执行时对轨迹分段并提取每段中点，按中点序列逐步运动。

## 设计目标

- 配置、ROS IO、流程编排模块解耦，便于排查。
- 默认安全行为：`dry_run: true`（仅日志，不发运动指令）。
- 可选人工确认模式：每次发送运动指令前在终端打印指令详情并等待确认。
- 每个步骤前后输出清晰日志。
- 每个动作前后执行力阈值/错误码安全检查。
- 支持“轨迹分段中点运动”，降低大跨度直达导致的碰撞风险。

## 项目结构

- `run_demo.py`: 入口脚本。
- `demo/config_model.py`: 运行配置数据模型。
- `demo/config_loader.py`: YAML 配置加载与校验。
- `demo/robot_io.py`: ROS 话题发布/订阅、安全检查。
- `demo/sequence_runner.py`: 工作流执行器。
- `demo/trajectory_loader.py`: 轨迹文件加载与分段中点采样。
- `config/demo_config.yaml`: 工作流与轨迹配置。
- `config/trajectories/`: 每个动作对应的轨迹文件目录。

## 快速开始

1. 启动 ROS2 与机械臂驱动。
2. 先运行 dry-run（默认）：

```bash
cd /home/feiguang/桌面/guji/workspace/demo_pick_place
python3 run_demo.py
```

3. 将每个动作对应的录制轨迹放入 `config/trajectories/`。
4. 在 `config/demo_config.yaml` 中为每个动作填写：
- `trajectory_file`: 对应轨迹文件
- `segment_count`: 分段数（每段取中点执行）
5. 验证无误后再将 `dry_run` 改为 `false`。
6. 如需每条运动指令前人工确认，可在 `config/demo_config.yaml` 设置 `confirm_before_motion: true`。

也可通过命令行覆盖配置：

```bash
python3 run_demo.py --confirm-before-motion
python3 run_demo.py --no-confirm-before-motion
```

## 轨迹格式

1. CSV：包含 `joint_1` ~ `joint_6` 列，单位为度。
2. JSON Lines/TXT：每行 JSON，格式 `{"point":[v1,v2,v3,v4,v5,v6]}`，程序会按 `/1000` 转换为度。

## 说明

- 默认话题命名适配 `left_arm_controller` / `right_arm_controller`。
- 若命名空间不同，仅需修改 YAML 配置。
- 步骤 8、9 使用 `pause` 占位，预留导航系统集成。

## 力阈值机制（已验证方案）

当前已对齐 `workspace/force_collision_monitor/run_force_monitor.py` 的阈值方法：

- 采用每臂独立阈值：`arms.<arm>.threshold_n`。
- 使用释放迟滞：`safety.release_ratio`，释放阈值为 `threshold_n * release_ratio`。
- 力数据获取改为主动查询：定时发布 `get_force_data_cmd`，订阅 `get_force_data_result`。
- 力消息字段兼容：优先读取 `force_fx/force_fy/force_fz`，同时兼容 `fx/fy/fz`。
- 阈值触发后立即 `move_stop_cmd`，并进入锁存；当力降到释放阈值以下自动解除锁存。

建议配置：

- `safety.force_mode` 使用 `query`，`safety.force_query_hz` 使用 `50.0`。
- `safety.force_query_result` 推荐 `raw`，对应 `get_force_data_result`。
- `udp_six_force` 与 `udp_six_zero_force` 在当前现场不可用，不作为数据源。
- `threshold_n` 初始可设 `35.0`，再按现场负载逐步微调。
- `release_ratio` 推荐 `0.8`，避免阈值边缘抖动反复触发。