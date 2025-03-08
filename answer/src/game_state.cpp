#include "answer/game_state.h"
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

GameState::GameState(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      robot_controller_(std::make_unique<RobotController>()),
      image_processor_(std::make_unique<ImageProcessor>()) {
    // 初始化路径规划器和机器人控制器
    robot_controller_ = std::make_unique<RobotController>(node);

    RCLCPP_INFO(node_->get_logger(), "Game state initialized");

    // 添加: 订阅图像话题
    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&GameState::imageCallback, this, std::placeholders::_1));

    // 添加: 初始化地图发布者
    map_publisher_ = node_->create_publisher<info_interfaces::msg::Map>("/map", 10);
}

// 添加: 图像处理回调函数实现
void GameState::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 将 ROS 图像消息转换为 OpenCV 图像格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cv_raw_img = cv_ptr->image;

        // 获取图像的宽度和高度
        int width = cv_raw_img.cols;
        int height = cv_raw_img.rows;

        // 定义网格数量
        int grid_rows = 10;
        int grid_cols = 10;

        // 计算每个网格的实际宽度和高度
        int grid_width = width / grid_cols;
        int grid_height = height / grid_rows;

        // 使用颜色阈值对图像进行二值化处理
        cv::Scalar lower(55, 55, 55);
        cv::Scalar upper(60, 60, 60);
        cv::Mat binary;
        cv::inRange(cv_raw_img, lower, upper, binary);

        // 对二值化后的图像进行形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::erode(binary, binary, kernel);
        cv::dilate(binary, binary, kernel);

        // 创建 info_interfaces::msg::Map 消息对象
        info_interfaces::msg::Map map_msg;
        map_msg.row = grid_rows;
        map_msg.col = grid_cols;
        map_msg.grid_width = grid_width;
        map_msg.grid_height = grid_height;

        // 根据二值化图像填充地图矩阵
        map_msg.mat.resize(grid_rows * grid_cols);
        for (int i = 0; i < grid_rows; ++i) {
            for (int j = 0; j < grid_cols; ++j) {
                int x = j * grid_width + grid_width / 2;
                int y = i * grid_height + grid_height / 2;
                if (binary.at<uchar>(y, x) == 255) {
                    map_msg.mat[i * grid_cols + j] = 1; // 墙壁
                } else {
                    map_msg.mat[i * grid_cols + j] = 0; // 路径
                }
            }
        }

        // 发布地图消息
        map_publisher_->publish(map_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

// 添加初始化方法定义
void GameState::initialize(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    path_planner_ = std::make_unique<PathPlanner>(256, 128);  // 默认地图大小
    robot_controller_ = std::make_unique<RobotController>(node);
    password_pub_ = node_->create_publisher<std_msgs::msg::String>("/password", 10);
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

void GameState::updateRobot(const info_interfaces::msg::Robot::SharedPtr msg) {
    robot_data_ = msg;
    if (robot_controller_) {
        robot_controller_->setRobotData(robot_data_); // 设置 robot_data_
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