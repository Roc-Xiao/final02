#include "answer/game_state.hpp"
#include <std_msgs/msg/string.hpp>
#include <cmath>

GameState::GameState(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      current_state_(State::INIT),
      game_complete_(false) {
    
    // 初始化串口通信
    serial_ = std::make_unique<my_serial::MySerial>("/dev/ttyUSB0", 115200);
    if (!serial_->open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open serial port");
        throw std::runtime_error("Serial port initialization failed");
    }
    
    // 初始化路径规划器和机器人控制器
    path_planner_ = std::make_unique<PathPlanner>(800, 600);  // 默认地图大小
    robot_controller_ = std::make_unique<RobotController>(node);
    
    // 创建密码发布器
    password_pub_ = node_->create_publisher<std_msgs::msg::String>("password", 10);
    
    RCLCPP_INFO(node_->get_logger(), "Game state initialized");
}

void GameState::updateMap(const info_interfaces::msg::Map::SharedPtr msg) {
    map_data_ = msg;
    // 更新路径规划器的地图大小
    path_planner_ = std::make_unique<PathPlanner>(msg->row, msg->col);
    handleState();
}

void GameState::updateArea(const info_interfaces::msg::Area::SharedPtr msg) {
    area_data_ = msg;
    handleState();
}

void GameState::updateRobot(const info_interfaces::msg::Robot::SharedPtr msg) {
    robot_data_ = msg;
    if (robot_controller_) {
        robot_controller_->update(msg->our_robot);
    }
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
                    // 当击毁敌人时，系统会自动发送密码片段
                    auto fragment = serial_->receivePassword();
                    if (fragment != -1) {
                        password_fragments_.push_back(static_cast<uint64_t>(fragment));
                        RCLCPP_INFO(node_->get_logger(),
                            "Received password fragment %zu: 0x%lx",
                            password_fragments_.size(),
                            static_cast<uint64_t>(fragment));
                    }

                    if (password_fragments_.size() == 2) {
                        RCLCPP_INFO(node_->get_logger(), "Got both password fragments, moving to password phase");
                        current_state_ = State::SENDING_PASSWORD;
                    }
                }
            }
            break;
        }

        case State::SENDING_PASSWORD: {
            RCLCPP_INFO(node_->get_logger(), "Sending password fragments to judge...");
            if (serial_->sendPasswordFragments(password_fragments_[0], password_fragments_[1])) {
                RCLCPP_INFO(node_->get_logger(), "Waiting for complete password from judge...");
                int64_t complete_password = serial_->receivePassword();
                if (complete_password != -1) {
                    RCLCPP_INFO(node_->get_logger(),
                        "Received complete password: 0x%lx",
                        static_cast<uint64_t>(complete_password));

                    // 发送密码到指定话题
                    auto msg = std_msgs::msg::String();
                    msg.data = std::to_string(complete_password);
                    password_pub_->publish(msg);

                    // 移动到密码区域
                    auto path = planPath(robot_data_->our_robot, area_data_->password);
                    if (!path.empty()) {
                        RCLCPP_INFO(node_->get_logger(), "Moving to password zone...");
                        robot_controller_->setPath(path);
                        if (robot_controller_->reachedGoal()) {
                            RCLCPP_INFO(node_->get_logger(), "Password sent successfully");
                            current_state_ = State::ATTACKING_BASE;
                        }
                    }
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