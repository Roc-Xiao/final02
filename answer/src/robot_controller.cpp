#include "answer/robot_controller.hpp"
#include <cmath>

RobotController::RobotController(std::shared_ptr<rclcpp::Node> node)
    : node_(node), current_path_index_(0) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void RobotController::setPath(const std::vector<info_interfaces::msg::Point>& path) {
    path_ = path;
    current_path_index_ = 0;
    RCLCPP_INFO(node_->get_logger(), "New path set with %zu points", path.size());
}

void RobotController::update(const info_interfaces::msg::Point& current_pos) {
    if (current_path_index_ >= path_.size()) {
        // 已到达终点，发送停止指令
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    // 获取当前目标点
    const auto& target = path_[current_path_index_];
    
    // 计算到目标点的距离
    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // 如果到达当前目标点，移动到下一个目标点
    if (distance < POSITION_TOLERANCE) {
        current_path_index_++;
        RCLCPP_INFO(node_->get_logger(), "Reached waypoint %zu", current_path_index_);
        return;
    }
    
    // 计算并发送控制指令
    auto cmd_vel = calculateCommand(current_pos, target);
    cmd_vel_pub_->publish(cmd_vel);
}

geometry_msgs::msg::Twist RobotController::calculateCommand(
    const info_interfaces::msg::Point& current,
    const info_interfaces::msg::Point& target) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // 计算方向角度
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double target_angle = std::atan2(dy, dx);
    
    // 设置线速度和角速度
    cmd_vel.linear.x = LINEAR_SPEED;
    cmd_vel.angular.z = target_angle * ANGULAR_SPEED;
    
    return cmd_vel;
}