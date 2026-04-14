// Copyright RealMan
// License: BSD/GPL/...
// Author: Starry
// Description:
// 使用 MoveIt2 提供的 C++ 接口实现左右臂的笛卡尔空间运动
// （逆运动学：给定笛卡尔空间中的某个位置，求解各个关节角度）
// 并修复 getCurrentPose() 获取不到机械臂状态的问题 —— 通过多线程执行器处理回调。

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>
#include <memory>

class MoveItIkDemo {
public:
    MoveItIkDemo()
    : node_(rclcpp::Node::make_shared("hello_moveit2_ik_demo")),
      logger_(rclcpp::get_logger("log")),
      move_group_interface_left_(node_, "left_arm"),
      move_group_interface_right_(node_, "right_arm")
    {
        // 启动多线程执行器，保证 MoveIt 内部 joint_states 回调能被正常执行
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() { executor_->spin(); });

        RCLCPP_INFO(logger_, "MoveIt2 双臂控制节点初始化中...");

        // 设置 MoveIt 参数 
        move_group_interface_left_.setGoalPositionTolerance(0.1);
        move_group_interface_left_.setGoalOrientationTolerance(0.1);
        move_group_interface_left_.setPlanningTime(30.0);

        move_group_interface_right_.setGoalPositionTolerance(0.1);
        move_group_interface_right_.setGoalOrientationTolerance(0.1);
        move_group_interface_right_.setPlanningTime(30.0);

        // 启动状态监控（等待 joint_states 数据更新） 
        move_group_interface_left_.startStateMonitor(5.0);
        move_group_interface_right_.startStateMonitor(5.0);

        // 让系统有时间接收 joint_states
        rclcpp::sleep_for(std::chrono::seconds(3));

        printCurrentPose();
        setTargetPose_left();
        setTargetPose_right();
    }

    ~MoveItIkDemo() {
        // 安全退出执行器线程
        executor_->cancel();
        if (executor_thread_.joinable())
            executor_thread_.join();
    }

    void run_left() {
        if (planAndExecute_left()) {
            RCLCPP_INFO(logger_, "左臂执行成功。");
        } else {
            RCLCPP_ERROR(logger_, "左臂规划失败！");
        }
    }

    void run_right() {
        if (planAndExecute_right()) {
            RCLCPP_INFO(logger_, "右臂执行成功。");
        } else {
            RCLCPP_ERROR(logger_, "右臂规划失败！");
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;

    moveit::planning_interface::MoveGroupInterface move_group_interface_left_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_right_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_left_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_right_;

    // 打印当前左右臂位姿
    void printCurrentPose() {
        move_group_interface_left_.setStartStateToCurrentState();
        move_group_interface_right_.setStartStateToCurrentState();

        auto current_pose_left = move_group_interface_left_.getCurrentPose();
        auto current_pose_right = move_group_interface_right_.getCurrentPose();

        if (current_pose_left.header.frame_id.empty()) {
            RCLCPP_ERROR(logger_, "左臂位姿获取失败！");
        } else {
            RCLCPP_INFO(logger_, "左臂当前位姿 - 位置: (%.3f, %.3f, %.3f)",
                        current_pose_left.pose.position.x,
                        current_pose_left.pose.position.y,
                        current_pose_left.pose.position.z);
        }

        if (current_pose_right.header.frame_id.empty()) {
            RCLCPP_ERROR(logger_, "右臂位姿获取失败！");
        } else {
            RCLCPP_INFO(logger_, "右臂当前位姿 - 位置: (%.3f, %.3f, %.3f)",
                        current_pose_right.pose.position.x,
                        current_pose_right.pose.position.y,
                        current_pose_right.pose.position.z);
        }
    }

    // 设置左臂目标位姿
    void setTargetPose_left() {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.1;
        target_pose.position.y = 0.1;
        target_pose.position.z = 0.3;
        move_group_interface_left_.setPoseTarget(target_pose);
    }

    // 设置右臂目标位姿
    void setTargetPose_right() {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = -0.71;
        target_pose.position.y = -0.31;
        target_pose.position.z = 0.92;
        move_group_interface_right_.setPoseTarget(target_pose);
    }

    // 左臂规划与执行
    bool planAndExecute_left() {
        RCLCPP_INFO(logger_, "开始规划左臂...");
        bool success = false;
        int retry = 0;

        while (!success && retry < 3) {
            success = static_cast<bool>(move_group_interface_left_.plan(plan_left_));
            RCLCPP_INFO(logger_, "规划尝试 %d 次，结果: %s", retry + 1, success ? "成功" : "失败");
            retry++;
        }

        if (success) {
            move_group_interface_left_.execute(plan_left_);
        }
        return success;
    }

    // 右臂规划与执行
    bool planAndExecute_right() {
        RCLCPP_INFO(logger_, "开始规划右臂...");
        bool success = false;
        int retry = 0;

        while (!success && retry < 3) {
            success = static_cast<bool>(move_group_interface_right_.plan(plan_right_));
            RCLCPP_INFO(logger_, "规划尝试 %d 次，结果: %s", retry + 1, success ? "成功" : "失败");
            retry++;
        }

        if (success) {
            move_group_interface_right_.execute(plan_right_);
        }
        return success;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto demo = std::make_shared<MoveItIkDemo>();
    demo->run_left();
    demo->run_right();

    rclcpp::shutdown();
    return 0;
}
