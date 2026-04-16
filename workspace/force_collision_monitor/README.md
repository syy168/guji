# 力碰撞监控器

该目录是一个独立的力碰撞监控模块。
它只订阅力传感器话题并实时打印力值，不会发布任何机械臂运动指令。

## 功能说明

- 订阅单臂或双臂的 `udp_six_force` 话题。
- 按固定频率在终端输出 `fx`、`fy`、`fz` 与合力范数。
- 当合力范数超过阈值时触发碰撞事件。
- 触发时打印 `collision_detected=1`。
- 支持两种数据来源：
	- `udp`：订阅实时推送话题（默认）。
	- `query`：定时发布 `get_force_data_cmd`，订阅 `get_force_data_result`（UDP全0时建议使用）。

## 文件说明

- `run_force_monitor.py`：监控入口脚本。
- `config/monitor_config.yaml`：监控话题与阈值配置。

## 使用方法

在本目录下运行：

```bash
python run_force_monitor.py
```

默认行为：

- 使用 `config/monitor_config.yaml`。
- `--arm` 默认 `both`（双臂）。
- `--mode`、`--query-hz`、`--query-result` 不传时读取配置文件中的 `monitor` 配置。

常用参数示例：

```bash
python run_force_monitor.py --arm right
python run_force_monitor.py --threshold 20.0
python run_force_monitor.py --print-hz 20.0
python run_force_monitor.py --config config/monitor_config.yaml
python run_force_monitor.py --arm right --mode udp --topic-suffix udp_six_force
python run_force_monitor.py --arm right --mode udp --topic-suffix udp_six_zero_force
python run_force_monitor.py --arm right --mode query --query-hz 50 --query-result raw
python run_force_monitor.py --arm right --mode query --query-hz 50 --query-result zero
```

## 注意事项

- 该脚本不控制机械臂运动。
- 碰撞触发为“越阈值边沿触发”：力从低于阈值变为高于阈值时触发一次。
- 使用释放迟滞（`release_ratio`）避免在阈值附近反复抖动触发。
- 如果 Web 端有力反馈但 UDP 话题全 0，可优先切换到 `--mode query` 继续开发与验证。
