//
// Created by roc on 25-2-25.
//

#ifndef GAME_NODE_HPP
#define GAME_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "answer/game_state.h"
#include "answer/robot_controller.h"
#include "info_interfaces/msg/robot.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "example_interfaces/msg/bool.hpp"

class GameNode : public rclcpp::Node{
public:
    // 修改: 添加接受 rclcpp::NodeOptions 参数的构造函数
    GameNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("game_node", options) {
        // 初始化订阅者
        robot_sub_ = create_subscription<info_interfaces::msg::Robot>(
            "/robot_info", 10, 
            std::bind(&GameNode::robotCallback, this, std::placeholders::_1));
            
        map_sub_ = create_subscription<info_interfaces::msg::Map>(
            "/map_info", 10,
            std::bind(&GameNode::mapCallback, this, std::placeholders::_1));
            
        area_sub_ = create_subscription<info_interfaces::msg::Area>(
            "/area_info", 10,
            std::bind(&GameNode::areaCallback, this, std::placeholders::_1));

        // 初始化发布者
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        password_pub_ = create_publisher<std_msgs::msg::String>("/password", 10); // 确保使用正确的 std_msgs::msg::String
        pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10); // 添加位姿发布器
        shoot_pub_ = create_publisher<example_interfaces::msg::Bool>("/shoot", 10); // 添加射击发布器

        // 初始化控制器
        controller_ = std::make_unique<RobotController>();
        controller_->setPosePublisher(pose_pub_); // 设置位姿发布器
        controller_->setShootPublisher(shoot_pub_); // 设置射击发布器

        // 初始化 game_state_
        game_state_ = std::make_unique<GameState>();
        // 构造函数中不能使用 shared_from_this ，在对象构造时，还没有 std::shared_ptr 管理此对象
    }

    void initialize() {
        // 传递 shared_from_this() 给控制器和游戏状态
        controller_->initialize(shared_from_this());
        game_state_->initialize(shared_from_this());
    }

private:
    void robotCallback(const info_interfaces::msg::Robot::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),"Robot received and updated.");
        current_robot_ = *msg;
        game_state_->updateRobot(msg); // 更新 robot_data_
        updateGameLogic();
    }

    void mapCallback(const info_interfaces::msg::Map::SharedPtr msg) {
        current_map_ = *msg;
        game_state_->updateMap(msg); // 更新 map_data_
        RCLCPP_INFO(this->get_logger(), "Map received and updated.");
    }

    void areaCallback(const info_interfaces::msg::Area::SharedPtr msg) {
        game_area_ = *msg;
        game_state_->updateArea(msg); // 更新 area_data_
        RCLCPP_INFO(this->get_logger(), "Area received and updated.");
    }

    void updateGameLogic() {
        if (game_state_->getCurrentState() == GameState::State::SEEKING_SUPPLY) {
            // SEEKING_SUPPLY时使用A*算法规划路径返回补给点
            auto path = game_state_->planPath(game_state_->getRobotData()->our_robot, game_state_->getAreaData()->recover);
            if (!path.empty()) {
                controller_->setPath(path);
            }
        } else if (game_state_->getCurrentState() == GameState::State::HUNTING_ENEMIES) {
            // HUNTING_ENEMIES时使用A*算法规划路径接近最近的敌人
            if (!game_state_->getRobotData()->enemy.empty()) {
                auto nearest_enemy = game_state_->getRobotData()->enemy[0];
                auto path = game_state_->planPath(game_state_->getRobotData()->our_robot, nearest_enemy);
                if (!path.empty()) {
                    controller_->setPath(path);
                }

                // 检查是否可以射击
                if (controller_->canShoot(game_state_->getRobotData()->our_robot, nearest_enemy, *game_state_->getMapData())) {
                    controller_->publishShoot(true);
                } else {
                    controller_->publishShoot(false);
                }
            }
        }

        // 更新机器人位置
        controller_->update(game_state_->getRobotData()->our_robot);
    }

    // 成员变量
    std::unique_ptr<RobotController> controller_;
    std::unique_ptr<GameState> game_state_;
    
    info_interfaces::msg::Robot current_robot_;
    info_interfaces::msg::Map current_map_;
    info_interfaces::msg::Area game_area_;
    std::string final_password_;

    rclcpp::Subscription<info_interfaces::msg::Robot>::SharedPtr robot_sub_;
    rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr map_sub_;
    rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr area_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr password_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr password_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_; // 添加位姿发布器成员变量
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr shoot_pub_; // 添加射击发布器成员变量
};

#endif