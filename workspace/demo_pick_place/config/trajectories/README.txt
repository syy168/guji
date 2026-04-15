将每个动作对应的录制轨迹文件放到本目录。

支持格式：
1) CSV：包含 joint_1~joint_6 列（单位：度）
2) JSON Lines/TXT：每行 JSON，格式 {"point":[编码值...]}，程序会按 /1000 转为度

命名建议（可按需修改）：
- right_step_00_scan_pose.csv
- left_step_01_contact_pose.csv
- left_step_02_lift_pose.csv
- right_step_03_insert_prepare.csv
- right_step_05_rotate_horizontal.csv
- right_step_06_nav_safe_pose.csv
- left_step_07_nav_safe_pose.csv
- right_step_10_rescan_pose.csv
- right_step_11_place_limit_top.csv

注意：
- 演示程序会将每条轨迹分段并取每段中点执行，避免大跨度直达动作。
- 在 demo_config.yaml 中可通过 segment_count 控制每步取多少个中点。
