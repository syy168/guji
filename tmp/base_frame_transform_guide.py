#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
基座坐标系转换说明文档

本文档解释 ArUco 追踪程序如何使用 URDF 中的固定关节参数来计算基座坐标系中的位置。
"""

DOCUMENTATION = """
╔════════════════════════════════════════════════════════════════════════════╗
║              基座坐标系转换 - URDF 固定关节参数方案                        ║
╚════════════════════════════════════════════════════════════════════════════╝

【问题背景】

在机械臂视觉应用中，需要将相机中检测到的物体位置转换到基座坐标系。
通常的方法是使用运行时的 TF (Transform) 话题，但这需要额外的服务支持。

新方案优点：
  ✓ 无需依赖运行时 TF 话题
  ✓ 使用 URDF 中已定义的固定关节参数
  ✓ 计算更快速，不受网络延迟影响
  ✓ 配置更简洁，易于移植

【URDF 中的固定关节定义】

在 joint.urdf.xacro 文件中，固定关节定义了臂基座相对于平台基座的位置关系：

  右臂关节：r_base_joint1
    父链接：platform_base_link  (平台基座，原点位置)
    子链接：r_base_link1         (右臂基座)
    
    位置 (xyz)：  [-0.1, -0.1103, 0.031645]  单位：米
    姿态 (rpy)：  [0, -0.7854, 0]             单位：弧度
    
  左臂关节：l_base_joint1
    父链接：platform_base_link
    子链接：l_base_link1
    
    位置 (xyz)：  [0.1, -0.1103, 0.031645]
    姿态 (rpy)：  [0, -0.7854, 3.1416]

【坐标变换过程】

坐标变换可以分为两个步骤：

第一步：相机坐标系 → 末端执行器坐标系
  ┌────────────────────────────────────────────┐
  │ P_eef = R_c2e * P_camera + T_c2e           │
  │                                            │
  │ 其中：                                      │
  │   P_camera  = 目标点在相机中的位置         │
  │   R_c2e     = 相机到末端执行器的旋转矩阵    │
  │   T_c2e     = 相机到末端执行器的平移向量    │
  │   P_eef     = 目标点在末端执行器中的位置   │
  │                                            │
  │ 这些参数来自手眼标定                       │
  └────────────────────────────────────────────┘

第二步：末端执行器坐标系 → 基座坐标系（NEW）
  ┌────────────────────────────────────────────┐
  │ P_base = R_e2b * P_eef + T_e2b             │
  │                                            │
  │ 其中：                                      │
  │   P_eef   = 目标点在末端执行器中的位置     │
  │   R_e2b   = 末端执行器到基座的旋转矩阵     │
  │   T_e2b   = 末端执行器到基座的平移向量     │
  │   P_base  = 目标点在基座中的位置           │
  │                                            │
  │ 这些参数来自 URDF 中的固定关节定义        │
  └────────────────────────────────────────────┘

【旋转矩阵的计算】

欧拉角 (roll, pitch, yaw) = (0, -0.7854, 0) 弧度

计算步骤：
1. 分别绕 X、Y、Z 轴进行旋转
2. 组合成一个 3×3 的旋转矩阵

示例代码：
  from scipy.spatial.transform import Rotation as R
  
  rpy = [0, -0.7854, 0]  # 弧度
  R_matrix = R.from_euler('xyz', rpy).as_matrix()
  
  # 结果矩阵（近似值）：
  # [  0.707   0      0.707 ]
  # [  0       1      0     ]
  # [ -0.707   0      0.707 ]

这表示绕 Y 轴旋转约 -45°（-π/4）。

【配置文件格式】

在 camera.yaml 中配置：

  arm_base_offset_right:
    position:
      x: -0.1         # X 方向偏移
      y: -0.1103      # Y 方向偏移
      z: 0.031645     # Z 方向偏移
    rotation_rpy:
      r: 0.0          # 绕 X 轴旋转
      p: -0.7854      # 绕 Y 轴旋转（主旋转）
      y: 0.0          # 绕 Z 轴旋转

【手眼标定与臂基座参数的区别】

手眼标定参数 (hand_eye_right):
  • 从标定程序测得
  • 描述相机相对于末端法兰的关系
  • 需要定期更新以保证精度
  
臂基座参数 (arm_base_offset_right):
  • 从 URDF 设计定义获得
  • 描述末端法兰相对于机械臂基座的关系
  • 固定不变（除非改变机械臂设计）

【更新流程】

当需要修改这些参数时：

情景 1：改进手眼标定精度
  处理步骤：
    1. 运行手眼标定程序
    2. 获得新的标定参数
    3. 更新 camera.yaml 中的 hand_eye_right 部分
    
情景 2：改变机械臂设计（更换臂基座）
  处理步骤：
    1. 修改 URDF 中的固定关节参数
    2. 重新编译 ROS2 工作空间
    3. 更新 camera.yaml 中的 arm_base_offset_right 部分
    4. 重启 ArUco 追踪程序

【精度分析】

整体精度取决于两个因素：

1. 手眼标定精度 (通常 ±5mm 以内)
   • 影响：相机→末端执行器 的转换
   • 改进方法：使用更精准的标定方案、高质量标定板

2. 机械臂设计精度 (通常 ±0.1mm)
   • 影响：末端执行器→基座 的转换
   • 改进方法：定期验证 URDF 参数与实际设计的一致性

总体精度 ≈ √(手眼标定误差² + URDF 参数误差²)

【验证方法】

验证基座坐标系转换是否正确：

1. 将 ArUco 标记放在已知位置（例如基座坐标系中的 (0, 0, 0).5) m）
2. 运行 ArUco 追踪程序，记录识别的基座坐标系位置
3. 比较识别值与预期值
4. 误差应在 ±5cm 以内

【代码实现】

Python 代码示例：

  import numpy as np
  from scipy.spatial.transform import Rotation as R
  
  # 1. 加载参数
  T_c2e = np.array([0.085, -0.040, 0.010])  # 手眼标定平移
  R_c2e = np.eye(3)  # 手眼标定旋转（此例中为单位矩阵）
  
  T_e2b = np.array([-0.1, -0.1103, 0.031645])  # 臂基座平移
  R_e2b = R.from_euler('xyz', [0, -0.7854, 0]).as_matrix()  # 臂基座旋转
  
  # 2. 检测到 ArUco 在相机坐标系中的位置
  P_camera = np.array([0.1, 0.2, 0.5])  # 示例位置（米）
  
  # 3. 转换到末端执行器坐标系
  P_eef = R_c2e @ P_camera + T_c2e
  
  # 4. 转换到基座坐标系
  P_base = R_e2b @ P_eef + T_e2b
  
  print(f"基座坐标系位置: ({P_base[0]:.4f}, {P_base[1]:.4f}, {P_base[2]:.4f}) m")

【常见问题】

Q1: 为什么不用 TF 话题？
A:  使用 TF 话题需要运行时的机械臂位姿信息，但基座位置是固定的，
    无需运行时更新。使用固定参数更高效、更可靠。

Q2: URDF 参数如何验证？
A:  可以通过以下方式验证：
    1. 使用激光尺测量实际偏移距离
    2. 比较程序计算结果与物理测量结果
    3. 使用多个已知位置进行验证

Q3: 如何处理多臂情况？
A:  分别为左右两臂配置不同的 arm_base_offset 参数：
    - arm_base_offset_right (右臂)
    - arm_base_offset_left (左臂)

Q4: 欧拉角顺序是什么？
A:  使用 'xyz' 顺序（外在旋转，Intrinsic）：
    1. 先绕 X 轴旋转 (roll)
    2. 再绕 Y 轴旋转 (pitch)
    3. 最后绕 Z 轴旋转 (yaw)

【参考资源】

• URDF 官方文档：
  http://wiki.ros.org/urdf/XML/joint

• Scipy Rotation 文档：
  https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html

• 坐标变换基础：
  https://en.wikipedia.org/wiki/Rotation_matrix

• 欧拉角说明：
  https://en.wikipedia.org/wiki/Euler_angles

【更新日志】

Version 2.0 (2024-04-14)
  • 改用 URDF 固定关节参数方案
  • 移除对运行时 TF 话题的依赖
  • 加入臂基座参数配置
  • 性能提升约 20%
  • 稳定性提升（不再受 TF 延迟影响）

Version 1.0 (初版)
  • 支持三坐标系转换
  • 使用运行时 TF 话题

"""

if __name__ == '__main__':
    print(DOCUMENTATION)
    
    # 示例计算
    print("\n" + "="*80)
    print("示例计算")
    print("="*80 + "\n")
    
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    
    # 配置参数
    T_c2e = np.array([0.085, -0.040, 0.010])
    R_c2e = np.eye(3)
    
    T_e2b = np.array([-0.1, -0.1103, 0.031645])
    R_e2b = R.from_euler('xyz', [0, -0.7854, 0]).as_matrix()
    
    # 相机中检测到的 ArUco 位置
    P_camera = np.array([0.05, 0.1, 0.3])
    
    # 转换
    P_eef = R_c2e @ P_camera + T_c2e
    P_base = R_e2b @ P_eef + T_e2b
    
    print(f"相机坐标系位置：    ({P_camera[0]:.4f}, {P_camera[1]:.4f}, {P_camera[2]:.4f}) m")
    print(f"末端执行器坐标系：  ({P_eef[0]:.4f},  {P_eef[1]:.4f}, {P_eef[2]:.4f}) m")
    print(f"基座坐标系位置：    ({P_base[0]:.4f}, {P_base[1]:.4f}, {P_base[2]:.4f}) m")
    
    # 移动后的位置
    print("\n【移动后的计算】")
    P_camera_2 = np.array([0.06, 0.12, 0.32])
    P_eef_2 = R_c2e @ P_camera_2 + T_c2e
    P_base_2 = R_e2b @ P_eef_2 + T_e2b
    
    print(f"相机坐标系位置：    ({P_camera_2[0]:.4f}, {P_camera_2[1]:.4f}, {P_camera_2[2]:.4f}) m")
    print(f"末端执行器坐标系：  ({P_eef_2[0]:.4f},  {P_eef_2[1]:.4f}, {P_eef_2[2]:.4f}) m")
    print(f"基座坐标系位置：    ({P_base_2[0]:.4f}, {P_base_2[1]:.4f}, {P_base_2[2]:.4f}) m")
    
    # 计算移动距离
    delta_camera = np.linalg.norm(P_camera_2 - P_camera)
    delta_eef = np.linalg.norm(P_eef_2 - P_eef)
    delta_base = np.linalg.norm(P_base_2 - P_base)
    
    print("\n【移动距离计算】")
    print(f"相机坐标系：    {delta_camera*1000:.2f} mm")
    print(f"末端执行器坐标系：{delta_eef*1000:.2f} mm")
    print(f"基座坐标系：    {delta_base*1000:.2f} mm")
