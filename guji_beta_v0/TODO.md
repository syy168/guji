# 双臂取放料系统 — TODO 修复清单

> 本文档记录 `DUAL_ARM_PICK_PLACE.md` 审查中发现的所有问题及修复状态。
> 创建时间：2026-04-08
> 更新时间：2026-04-08

---

## 图例


| 标记    | 含义          |
| ----- | ----------- |
| 🔴 P0 | 必须修复，系统无法运行 |
| 🟠 P1 | 核心功能受影响     |
| 🟡 P2 | 功能正确性隐患     |
| 🟢 P3 | 优化/文档改进     |
| ✅     | 已修复         |
| ⏳     | 待修复         |


---

## 🔴 P0 — 必须立即修复（系统无法运行）

### #1 TF 坐标链断裂（最严重）

**文件：** `nodes/tf_broadcaster.py`, `nodes/aruco_detector.py`

**问题：** Eye-in-Hand 配置下，相机安装在移动的法兰上。当前代码将相机视为固定在基座上，且缺少法兰→相机的完整变换链。

**修复方向：**

- 将 `tf_broadcaster` 的静态 TF 改为 `right_top → camera_right`
- 通过订阅 `rm_driver/udp_arm_position` 发布法兰动态 TF `right_base → right_top`
- 更新文档中的坐标系描述

**状态：** ✅ 已修复

---

### #2 `.now().now()` 类型错误

**文件：** `nodes/aruco_detector.py` 第 408 行

**问题：** `resp.header.stamp = self.get_clock().now().now().to_msg()` 连续调用 `.now()` 会报属性错误。

**修复：** 改为 `self.get_clock().now().to_msg()`

**状态：** ✅ 已修复

---

### #4 左臂点位全部为占位数据

**文件：** `config/poses.yaml`

**问题：** 左臂多个状态点位完全相同（`pick_left_push` = `pick_left_lift`，多个 `place_`* = `initial`）

**状态：** ⏳ 暂不处理（用户确认）

---

### #5 手眼标定四元数未标定

**文件：** `config/camera.yaml`

**问题：** 四元数全为零，与 tf_broadcaster 默认值不一致

**状态：** ⏳ 暂不处理（用户确认）

---

## 🟠 P1 — 核心功能受影响

### #3 camera_bridge YAML 读取路径错误

**文件：** `nodes/camera_bridge.py`

**问题：** 从 `system.yaml` 读取 `camera` 配置块，但 `camera` 在 `camera.yaml` 中

**修复：** 分别从 `camera.yaml` 和 `system.yaml` 加载

**状态：** ✅ 已修复

---

### #6 视觉偏移补偿方法不严谨

**文件：** `dual_arm_pick_place.py` 第 1022-1068 行

**问题：** 经验系数补偿偏移量，偏移 > 30mm 时失效

**修复方向：** 接入 MoveIt! IK 求解器或建立标定查找表

**状态：** ⏳ 待修复

---

### #11 缺少碰撞检测保护

**文件：** `dual_arm_pick_place.py` 整体

**问题：** 双臂协同运动中完全没有碰撞检测

**修复方向：** 定义碰撞区域，集成 MoveIt! 碰撞检测

**状态：** ⏳ 待修复

---

### #19 launch 可执行文件路径错误

**文件：** `launch/vision_pipeline.launch.py`

**问题：** `executable='camera_bridge.py'` 应为编译后的二进制名

**修复：** 创建 `package.xml` + `setup.py` 声明所有入口点，更新 launch 中的 executable 名称

**状态：** ✅ 已修复

---

## 🟡 P2 — 功能正确性隐患

### #7 TF 时间戳用法不当

**文件：** `nodes/aruco_detector.py` 第 449-453 行

**问题：** `rclpy.time.Time()` 不带参数创建零时间

**修复：** 改为 `rclpy.time.Time(seconds=0)`

**状态：** ✅ 已修复

---

### #8 文档坐标系描述矛盾

**文件：** `DUAL_ARM_PICK_PLACE.md` 第 6 章

**修复：** 更新坐标系关系图、含义表、TF 验证命令

**状态：** ✅ 已修复

---

### #9 Service 超时参数截断错误

**文件：** `dual_arm_pick_place.py` 第 972 行

**问题：** `int(req.timeout)` 将浮点数截断

**修复：** 改为 `req.timeout * 2 + 5`

**状态：** ✅ 已修复

---

### #12 block 参数不生效

**文件：** `dual_arm_pick_place.py` 第 532-563 行

**问题：** `block` 参数写入消息但无等待机制

**状态：** ⏳ 待修复

---

### #13 camera_prefix 读默认值问题

**文件：** `nodes/camera_bridge.py`

**问题：** `topic_prefix` 从 `system.yaml.camera.topic_prefix` 读取（路径错误）

**修复：** 从 `camera.yaml` 的 `topic_prefix` 字段读取（随 #3 一起修复）

**状态：** ✅ 已修复

---

### #16 关节角度越界无检查
**文件：** `dual_arm_pick_place.py`

**修复：**
- 在 `_load_config` 中加载 RM65 关节限位数据（来自 URDF）
- 新增 `_check_joint_limits(arm, joints)` 方法，逐关节检查角度是否在安全范围内
- 在 `_get_joints` 中调用限位检查，越限时跳过该步骤并报错

**限位数据（来自 URDF）：**

| 关节 | 最小值 | 最大值 |
|------|--------|--------|
| J1 | -178° | +178° |
| J2 | -130° | +130° |
| J3 | -135° | +135° |
| J4 | -178° | +178° |
| J5 | -128° | +128° |
| J6 | -360° | +360° (连续) |

**状态：** ✅ 已修复

---

### #17 流程无重试机制

**文件：** `dual_arm_pick_place.py`

**状态：** ⏳ 待修复

---

## 🟢 P3 — 优化/文档改进

### #10 夹爪语义文档不足

**状态：** ⏳ 待修复

---

### #14 rm_driver 依赖未验证

**问题：** launch 文件直接启动 `rm_driver`，参数可能与实际固件不匹配

**修复方向：** 将双臂驱动从视觉 launch 中分离

**状态：** ⏳ 待修复

---

### #15 YOLO 扩展未实现

**状态：** ⏳ 待修复

---

### #18 camera_bridge 内参检查失效代码偏移补偿

**文件：** `nodes/camera_bridge.py` 第 184-193 行

**问题：** 内层 `if` 判断 `self._camera_info` 是否已有数据，但赋值在回调开头，条件永远为假

**修复：** 删除失效的嵌套判断

**状态：** ✅ 已修复

---

### #20 未使用深度图

**文件：** `nodes/aruco_detector.py`

**问题：** 配置了 `depth_averaging_radius` 但代码未使用

**状态：** ⏳ 待修复

---

## 修复进度汇总


| #   | 问题                      | 优先级   | 状态     |
| --- | ----------------------- | ----- | ------ |
| 1   | TF 坐标链断裂                | 🔴 P0 | ✅ 已修复  |
| 2   | `.now().now()` 错误       | 🔴 P0 | ✅ 已修复  |
| 3   | camera_bridge YAML 路径错误 | 🟠 P1 | ✅ 已修复  |
| 4   | 左臂占位数据                  | 🔴 P0 | ⏳ 暂不处理 |
| 5   | 相机四元数未标定                | 🔴 P0 | ⏳ 暂不处理 |
| 6   | 偏移补偿不严谨                 | 🟠 P1 | ⏳ 待修复  |
| 7   | TF 时间戳用法不当              | 🟡 P2 | ✅ 已修复  |
| 8   | 文档坐标系矛盾                 | 🟡 P2 | ✅ 已修复  |
| 9   | Service 超时截断            | 🟡 P2 | ✅ 已修复  |
| 10  | 夹爪语义文档不足                | 🟢 P3 | ⏳ 待修复  |
| 11  | 碰撞检测缺失                  | 🟠 P1 | ⏳ 待修复  |
| 12  | block 参数不生效             | 🟡 P2 | ⏳ 待修复  |
| 13  | camera_prefix 读默认值      | 🟡 P2 | ✅ 已修复  |
| 14  | rm_driver 依赖未验证         | 🟢 P3 | ⏳ 待修复  |
| 15  | YOLO 扩展未实现              | 🟢 P3 | ⏳ 待修复  |
| 16  | 关节越界无检查                 | 🟡 P2 | ✅ 已修复  |
| 17  | 流程无重试机制                 | 🟡 P2 | ⏳ 待修复  |
| 18  | 内参检查失效代码                | 🟢 P3 | ✅ 已修复  |
| 19  | launch 可执行路径错误          | 🟠 P1 | ✅ 已修复  |
| 20  | 未使用深度图                  | 🟢 P3 | ⏳ 待修复  |


**已修复：10 项 | 待修复：8 项 | 暂不处理：2 项**

---

## 本次修复涉及的文件变更


| 文件                                 | 修复内容                            |
| ---------------------------------- | ------------------------------- |
| `nodes/aruco_detector.py`          | 修复 `.now().now()`、TF 时间戳用法      |
| `nodes/camera_bridge.py`           | 重写 `_load_config`、修复内参检查失效代码    |
| `nodes/tf_broadcaster.py`          | 重写，发布法兰动态 TF + 相机标定静态 TF        |
| `launch/vision_pipeline.launch.py` | 修复 executable 名称（去掉 `.py` 后缀）   |
| `config/camera.yaml`               | 补充 `intrinsic.fps`、完善四元数注释      |
| `DUAL_ARM_PICK_PLACE.md`           | 更新坐标系描述、关节限位数据来源说明 |
| `dual_arm_pick_place.py`           | 修复 Service 超时、添加关节限位检查 |
| `package.xml`                      | 新建，声明 ROS2 功能包元信息               |
| `setup.py`                         | 新建，声明所有可执行入口点                   |
| `TODO.md`                          | 更新修复进度                          |


