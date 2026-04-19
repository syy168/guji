# ArUco 两次识别验证程序

该目录是独立程序目录，满足 todo 第 3 条需求：
- 右臂不动
- 两次识别 ArUco
- 输出相机坐标系、右臂基坐标系、机器人基坐标系位置
- 输出机器人基坐标系下的平移差
- 同时记录每次识别时右臂关节角快照（来自 joint_states）
- 默认会自动探测仓库根目录，并读取 [guji_beta_v0/config/camera.yaml](../../guji_beta_v0/config/camera.yaml) 和 [joint.urdf.xacro](../../ros2_ws/src/ros2_rm_robot/dual_rm_description/dual_rm_65b_description/urdf/joint.urdf.xacro)

## 文件

- aruco_pose_validation.py: 主程序

## 依赖

- ROS2 Humble
- rclpy
- OpenCV (cv2, aruco)
- cv_bridge
- numpy
- tf2_ros

## 运行

```bash
cd <仓库根目录>/workspace/aruco_pose_validation
python3 aruco_pose_validation.py
```

如需显式覆盖路径：

```bash
python3 aruco_pose_validation.py \
  --config <仓库根目录>/guji_beta_v0/config/camera.yaml \
  --joint-xacro <仓库根目录>/ros2_ws/src/ros2_rm_robot/dual_rm_description/dual_rm_65b_description/urdf/joint.urdf.xacro
```

## 稳健参数（建议保留默认）

- `--capture-timeout`：单次识别采样窗口时长（秒），默认 `2.0`
- `--min-samples`：单次识别最低有效样本数，默认 `6`
- `--max-samples`：单次识别最多采样数，默认 `25`

程序会在一个短窗口内采集多帧有效识别，并对三套坐标结果做稳健统计（中位数 + 标准差），再用于两次结果比较。

## 说明

1. `camera -> right_base`：优先按图像时间戳查 TF，失败时自动回退 `latest TF`。
2. `right_base -> robot_base`：优先使用 TF（会真正使用 `robot_base_frame` 参数）；
   仅当 frame 组合是 `platform_base_link <- r_base_link1` 且 TF 查询失败时，才回退 `joint.urdf.xacro` 中 `r_base_joint1` 固定参数：
- xyz = `-0.1 -0.1103 0.031645`
- rpy = `0 -0.7854 0`
3. 相机 frame 会按候选链路回退：命令行覆盖 > `camera.yaml` > `CameraInfo.frame_id` > `camera_right`。
4. `camera.yaml` 中的 `topic_prefix`、`frames`、`aruco`、`hand_eye_right` 会作为默认参数自动读取。
5. 如果你的现场 frame 命名不同，只需修改启动参数覆盖默认值。
