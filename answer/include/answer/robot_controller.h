//
// Created by roc on 25-2-25.
//
#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <info_interfaces/msg/point.hpp>
#include <info_interfaces/msg/map.hpp>
#include <info_interfaces/msg/robot.hpp>
#include <vector>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <example_interfaces/msg/bool.hpp>

class RobotController {
public:
    RobotController(std::shared_ptr<rclcpp::Node> node);
    // 添加构造函数声明
    RobotController();

    // 初始化方法
    void initialize(std::shared_ptr<rclcpp::Node> node);

    // 设置新的路径点
    void setPath(const std::vector<info_interfaces::msg::Point>& path);

    // 更新控制指令
    void update(const info_interfaces::msg::Point& current_pos);

    // 检查是否到达目标
    bool reachedGoal() const { return current_path_index_ >= path_.size(); }

    // 添加 moveToTarget 函数声明
    geometry_msgs::msg::Twist moveToTarget(
        const info_interfaces::msg::Point& current,
        const info_interfaces::msg::Point& target);

    // 添加位姿发布器
    void setPosePublisher(rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher) {
        pose_publisher_ = pose_publisher;
    }

    // 添加射击发布器
    void setShootPublisher(rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr shoot_publisher) {
        shoot_publisher_ = shoot_publisher;
    }

    // 添加发布位姿信息的方法
    void publishPose(const geometry_msgs::msg::Pose2D& pose) {
        if (pose_publisher_) {
            pose_publisher_->publish(pose);
        }
    }

    // 添加发布射击命令的方法
    void publishShoot(bool shoot) {
        if (shoot_publisher_) {
            example_interfaces::msg::Bool msg;
            msg.data = shoot;
            shoot_publisher_->publish(msg);
        }
    }

    // 添加对 robot_data_ 和 map_data_ 的引用
    void setRobotData(const info_interfaces::msg::Robot::SharedPtr& robot_data) {
        robot_data_ = robot_data;
    }

    void setMapData(const info_interfaces::msg::Map::SharedPtr& map_data) {
        map_data_ = map_data;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    std::vector<info_interfaces::msg::Point> path_;
    size_t current_path_index_;

    // 控制参数
    static constexpr double LINEAR_SPEED = 0.5;  // 线速度
    static constexpr double ANGULAR_SPEED = 1.0; // 角速度
    static constexpr double POSITION_TOLERANCE = 0.1; // 位置容差

    // 添加速度常量
    static constexpr double VELOCITY_SCALE = 0.5; // 速度常量

    // 计算下一个目标点的控制指令
    geometry_msgs::msg::Twist calculateCommand(
        const info_interfaces::msg::Point& current,
        const info_interfaces::msg::Point& target);

    // 添加 rotateTurret 函数声明
    geometry_msgs::msg::Twist rotateTurret(
        const info_interfaces::msg::Point& current,
        const info_interfaces::msg::Point& target);

    // 添加 canShoot 函数声明
    bool canShoot(
        const info_interfaces::msg::Point& our_pos,
        const info_interfaces::msg::Point& enemy_pos,
        const info_interfaces::msg::Map& map,
        double turret_angle);

    // 添加 needResupply 函数声明
    bool needResupply(int current_ammo);

    // 添加 hasWallBetween 函数声明
    bool hasWallBetween(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& end,
        const info_interfaces::msg::Map& map);

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_; // 添加位姿发布器成员变量
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr shoot_publisher_; // 添加射击发布器成员变量

    // 添加 robot_data_ 和 map_data_ 成员变量
    info_interfaces::msg::Robot::SharedPtr robot_data_;
    info_interfaces::msg::Map::SharedPtr map_data_;
};

#endif