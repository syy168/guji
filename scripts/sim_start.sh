#!/bin/bash
# ============================================================
# Gazebo 仿真快速启动脚本
# ============================================================
# 在 ros2_ws 目录下执行本脚本
#
# 使用方法：
#   ./guji/scripts/sim_start.sh
#
# 功能：
#   1. Source ROS2 环境
#   2. 启动 Gazebo 双臂仿真
#   3. 启动 MoveIt2 + RViz
#   4. （可选）启动 Gazebo 适配器
#   5. （可选）启动 guji 控制器

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查是否在正确目录
if [ ! -d "src/ros2_rm_robot" ]; then
    echo -e "${RED}错误：未找到 ros2_rm_robot 目录${NC}"
    echo "请在 ros2_ws 目录下执行本脚本"
    exit 1
fi

# Source ROS2 环境
echo -e "${BLUE}[1/5] Source ROS2 环境...${NC}"
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# 检查 Gazebo 是否可用
if ! command -v gazebo &> /dev/null; then
    echo -e "${RED}错误：Gazebo 未安装${NC}"
    echo "请运行：sudo apt install ros-foxy-gazebo-*"
    exit 1
fi

echo -e "${GREEN}[1/5] ROS2 环境就绪${NC}"

# 解析参数
USE_ADAPTER=false
USE_CONTROLLER=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --adapter)
            USE_ADAPTER=true
            shift
            ;;
        --controller)
            USE_CONTROLLER=true
            shift
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--adapter] [--controller]"
            exit 1
            ;;
    esac
done

# 创建临时目录用于日志
LOG_DIR="/tmp/realman_sim_logs"
mkdir -p "$LOG_DIR"

# 启动 Gazebo
echo -e "${BLUE}[2/5] 启动 Gazebo 仿真...${NC}"
echo -e "${YELLOW}    提示：Gazebo 窗口将在后台打开${NC}"
gnome-terminal --geometry=100x40 --title="Gazebo仿真" -- bash -c "
    echo 'Gazebo 仿真进程启动中...'
    ros2 launch dual_rm_gazebo dual_rm_65b_gazebo.launch.py 2>&1 | tee $LOG_DIR/gazebo.log
" &
GAZEBBO_PID=$!
sleep 3

# 启动 MoveIt2
echo -e "${BLUE}[3/5] 启动 MoveIt2 + RViz...${NC}"
gnome-terminal --geometry=100x40 --title="MoveIt2控制" -- bash -c "
    echo 'MoveIt2 启动中...'
    ros2 launch dual_rm_65b_moveit_config demo.launch.py 2>&1 | tee $LOG_DIR/moveit.log
" &
MOVEIT_PID=$!
sleep 3

echo -e "${GREEN}[3/5] MoveIt2 已启动${NC}"

# 启动 Gazebo 适配器（可选）
if [ "$USE_ADAPTER" = true ]; then
    echo -e "${BLUE}[4/5] 启动 Gazebo 适配器...${NC}"
    gnome-terminal --geometry=100x40 --title="Gazebo适配器" -- bash -c "
        echo 'Gazebo 适配器启动中...'
        cd /root
        source /opt/ros/foxy/setup.bash
        source ./ros2_ws/install/setup.bash
        python3 guji/nodes/gazebo_arm_adapter.py 2>&1 | tee $LOG_DIR/adapter.log
    " &
    ADAPTER_PID=$!
    echo -e "${GREEN}[4/5] Gazebo 适配器已启动${NC}"
else
    echo -e "${YELLOW}[4/5] 跳过 Gazebo 适配器（使用 --adapter 参数启用）${NC}"
fi

# 启动 guji 控制器（可选）
if [ "$USE_CONTROLLER" = true ]; then
    echo -e "${BLUE}[5/5] 启动 guji 双臂控制器（仿真模式）...${NC}"
    gnome-terminal --geometry=100x40 --title="guji控制器" -- bash -c "
        echo 'guji 控制器启动中...'
        cd /root
        source /opt/ros/foxy/setup.bash
        source ./ros2_ws/install/setup.bash
        python3 guji/dual_arm_controller.py --use_gazebo 2>&1 | tee $LOG_DIR/controller.log
    " &
    CONTROLLER_PID=$!
    echo -e "${GREEN}[5/5] guji 控制器已启动${NC}"
else
    echo -e "${YELLOW}[5/5] 跳过 guji 控制器（使用 --controller 参数启用）${NC}"
fi

# 显示启动信息
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Gazebo 仿真环境已启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}运行状态：${NC}"
echo -e "  Gazebo PID:     ${GAZEBBO_PID}"
echo -e "  MoveIt2 PID:    ${MOVEIT_PID}"
if [ "$USE_ADAPTER" = true ]; then
    echo -e "  适配器 PID:     ${ADAPTER_PID}"
fi
if [ "$USE_CONTROLLER" = true ]; then
    echo -e "  控制器 PID:     ${CONTROLLER_PID}"
fi
echo ""
echo -e "${YELLOW}日志文件：${NC}"
echo "  $LOG_DIR/gazebo.log"
echo "  $LOG_DIR/moveit.log"
if [ "$USE_ADAPTER" = true ]; then
    echo "  $LOG_DIR/adapter.log"
fi
if [ "$USE_CONTROLLER" = true ]; then
    echo "  $LOG_DIR/controller.log"
fi
echo ""
echo -e "${YELLOW}查看日志：${NC}"
echo "  tail -f $LOG_DIR/gazebo.log"
echo "  tail -f $LOG_DIR/moveit.log"
echo ""
echo -e "${YELLOW}停止仿真：${NC}"
echo "  kill $GAZEBBO_PID $MOVEIT_PID"
if [ "$USE_ADAPTER" = true ]; then
    echo "  kill $ADAPTER_PID"
fi
if [ "$USE_CONTROLLER" = true ]; then
    echo "  kill $CONTROLLER_PID"
fi
echo ""
echo -e "${YELLOW}查看话题：${NC}"
echo "  ros2 topic list | grep -E 'joint_states|arm_controller'"
echo ""
echo -e "${BLUE}按 Ctrl+C 停止所有进程...${NC}"
echo ""

# 等待中断
trap "echo '正在停止所有进程...'; kill $GAZEBBO_PID $MOVEIT_PID 2>/dev/null; exit" SIGINT SIGTERM

# 无限循环，等待用户中断
while true; do
    sleep 1
done
