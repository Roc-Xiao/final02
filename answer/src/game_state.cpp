#include "answer/game_state.h"
#include <std_msgs/msg/string.hpp> // 修改: 确保包含正确的 std_msgs::msg::String
#include <cmath>

GameState::GameState(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      current_state_(State::INIT),
      game_complete_(false),
      base_vulnerable_(false) { // 初始化 base_vulnerable_
    
    // 初始化路径规划器和机器人控制器
    path_planner_ = std::make_unique<PathPlanner>(800, 600);  // 默认地图大小
    robot_controller_ = std::make_unique<RobotController>(node);

    RCLCPP_INFO(node_->get_logger(), "Game state initialized");
}

// 添加初始化方法定义
void GameState::initialize(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    path_planner_ = std::make_unique<PathPlanner>(800, 600);  // 默认地图大小
    robot_controller_ = std::make_unique<RobotController>(node);
    password_pub_ = node_->create_publisher<std_msgs::msg::String>("/password", 10);
}

void GameState::updateRobot(const info_interfaces::msg::Robot::SharedPtr msg) {
    robot_data_ = msg;
    if (robot_controller_) {
        robot_controller_->setRobotData(robot_data_); // 设置 robot_data_
        robot_controller_->update(msg->our_robot);
    }
    handleState();
}

void GameState::updateMap(const info_interfaces::msg::Map::SharedPtr msg) {
    map_data_ = msg;
    // 更新路径规划器的地图大小
    path_planner_ = std::make_unique<PathPlanner>(msg->row, msg->col);
    if (robot_controller_) {
        robot_controller_->setMapData(map_data_); // 设置 map_data_
    }
    handleState();
}

void GameState::updateArea(const info_interfaces::msg::Area::SharedPtr msg) {
    area_data_ = msg;
    handleState();
}

void GameState::handleState() {
    if (!map_data_ || !area_data_ || !robot_data_) {
        return;  // 等待所有数据就绪
    }

    switch (current_state_) {
        case State::INIT: {
            RCLCPP_INFO(node_->get_logger(), "Initializing game...");
            current_state_ = State::SEEKING_SUPPLY;
            break;
        }

        case State::SEEKING_SUPPLY: {
            RCLCPP_INFO(node_->get_logger(), "Seeking supply...");
            auto path = planPath(robot_data_->our_robot, area_data_->recover);
            if (!path.empty()) {
                robot_controller_->setPath(path);
                // 检查是否到达补给站
                if (robot_controller_->reachedGoal()) {
                    RCLCPP_INFO(node_->get_logger(), "Reached supply point");
                    current_state_ = State::HUNTING_ENEMIES;
                }
            }
            break;
        }

        case State::HUNTING_ENEMIES: {
            if (!robot_data_->enemy.empty()) {
                RCLCPP_INFO(node_->get_logger(), "Hunting enemies...");
                // 找到最近的敌人
                auto nearest_enemy = robot_data_->enemy[0];
                auto path = planPath(robot_data_->our_robot, nearest_enemy);
                if (!path.empty()) {
                    robot_controller_->setPath(path);
                }
                
                // 检查是否获得密码片段
                if (password_fragments_.size() < 2) {

                    if (password_fragments_.size() == 2) {
                        RCLCPP_INFO(node_->get_logger(), "Got both password fragments, moving to password phase");
                        current_state_ = State::GOTO_PASSWORD_AREA;
                    }
                }
            }
            break;
        }

        case State::GOTO_PASSWORD_AREA: {
            RCLCPP_INFO(node_->get_logger(), "Going to password area...");
            auto path = planPath(robot_data_->our_robot, area_data_->password);
            if (!path.empty()) {
                robot_controller_->setPath(path);
                // 检查是否到达密码区域
                if (robot_controller_->reachedGoal()) {
                    RCLCPP_INFO(node_->get_logger(), "Reached password area");
                    current_state_ = State::ATTACKING_BASE;
                }
            }
            break;
        }

        case State::ATTACKING_BASE: {
            RCLCPP_INFO(node_->get_logger(), "Attacking base...");
            auto path = planPath(robot_data_->our_robot, area_data_->base);
            if (!path.empty()) {
                robot_controller_->setPath(path);
                // 检查是否到达并摧毁基地
                if (robot_controller_->reachedGoal() &&
                    calculateDistance(robot_data_->our_robot, area_data_->base) < 1.0) {
                    RCLCPP_INFO(node_->get_logger(), "Base destroyed! Mission complete!");
                    game_complete_ = true;
                }
            }
            break;
        }

        case State::SENDING_TO_JUDGE: {
            RCLCPP_INFO(node_->get_logger(), "Sending password to judge...");
            std_msgs::msg::String password_msg;
            password_msg.data = getCombinedPassword();
            password_pub_->publish(password_msg);
            game_complete_ = true;
            break;
        }

        default:
            RCLCPP_WARN(node_->get_logger(), "Unhandled game state: %d", static_cast<int>(current_state_));
            break;
    }
}

std::vector<info_interfaces::msg::Point> GameState::planPath(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal) {

    if (!map_data_) {
        RCLCPP_WARN(node_->get_logger(), "No map data available for path planning");
        return std::vector<info_interfaces::msg::Point>();
    }

    return path_planner_->findPath(start, goal, map_data_->mat);
}

double GameState::calculateDistance(
    const info_interfaces::msg::Point& p1,
    const info_interfaces::msg::Point& p2) {

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// 添加函数定义
void GameState::updateState(State new_state) {
    current_state_ = new_state;
}

GameState::State GameState::getCurrentState() const {
    return current_state_;
}

void GameState::addPasswordFragment(const std::string& fragment) {
    if (password_fragments_.size() < REQUIRED_FRAGMENTS) {
        password_fragments_.push_back(fragment);
    }
}

bool GameState::hasAllPasswordFragments() const {
    return password_fragments_.size() >= REQUIRED_FRAGMENTS;
}

std::string GameState::getCombinedPassword() const {
    std::string combined;
    for (const auto& fragment : password_fragments_) {
        combined += fragment;
    }
    return combined;
}

bool GameState::isBaseVulnerable() const {
    return base_vulnerable_;
}

void GameState::setBaseVulnerable(bool vulnerable) {
    base_vulnerable_ = vulnerable;
}