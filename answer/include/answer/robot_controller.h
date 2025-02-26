//
// Created by roc on 25-2-25.
//
#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <info_interfaces/msg/point.hpp>
#include <vector>

class RobotController {
public:
    RobotController(std::shared_ptr<rclcpp::Node> node);

    // 设置新的路径点
    void setPath(const std::vector<info_interfaces::msg::Point>& path);

    // 更新控制指令
    void update(const info_interfaces::msg::Point& current_pos);

    // 检查是否到达目标
    bool reachedGoal() const { return current_path_index_ >= path_.size(); }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    std::vector<info_interfaces::msg::Point> path_;
    size_t current_path_index_;

    // 控制参数
    static constexpr double LINEAR_SPEED = 0.5;  // 线速度
    static constexpr double ANGULAR_SPEED = 1.0; // 角速度
    static constexpr double POSITION_TOLERANCE = 0.1; // 位置容差

    // 计算下一个目标点的控制指令
    geometry_msgs::msg::Twist calculateCommand(
        const info_interfaces::msg::Point& current,
        const info_interfaces::msg::Point& target);
};

#endif