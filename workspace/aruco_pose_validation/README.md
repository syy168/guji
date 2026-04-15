# ArUco 两次识别验证程序

该目录是独立程序目录，满足 todo 第 3 条需求：
- 右臂不动
- 两次识别 ArUco
- 输出相机坐标系、右臂基坐标系、机器人基坐标系位置
- 输出机器人基坐标系下的平移差
- 同时记录每次识别时右臂关节角快照（来自 joint_states）
- 默认读取项目中的 [guji_beta_v0/config/camera.yaml](../../guji_beta_v0/config/camera.yaml) 和 [joint.urdf.xacro](../../ros2_ws/src/ros2_rm_robot/dual_rm_description/dual_rm_65b_description/urdf/joint.urdf.xacro)

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
cd /home/feiguang/桌面/guji/workspace/aruco_pose_validation
python3 aruco_pose_validation.py \
  --config /home/feiguang/桌面/guji/guji_beta_v0/config/camera.yaml \
  --joint-xacro /home/feiguang/桌面/guji/ros2_ws/src/ros2_rm_robot/dual_rm_description/dual_rm_65b_description/urdf/joint.urdf.xacro
```

## 说明

1. 程序中 `right_base -> robot_base` 使用的是 `joint.urdf.xacro` 里 `r_base_joint1` 的固定参数：
- xyz = `-0.1 -0.1103 0.031645`
- rpy = `0 -0.7854 0`

2. `camera -> right_base` 使用 TF 查询（`right_base_frame <- camera_frame`）。
3. `camera.yaml` 中的 `topic_prefix`、`frames`、`aruco`、`hand_eye_right` 会作为默认参数自动读取。
4. 如果你的现场 frame 命名不同，只需修改启动参数覆盖默认值。
