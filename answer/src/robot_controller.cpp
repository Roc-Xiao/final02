#include "answer/robot_controller.h"
#include <cmath>

static constexpr double MOVE_SPEED = 1.0;
static constexpr double ATTACK_RANGE = 5.0;
static constexpr double ANGLE_TOLERANCE = 0.1;
static constexpr int MIN_AMMO = 10;
static constexpr double POSITION_TOLERANCE = 0.1;

RobotController::RobotController(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      cmd_vel_pub_(node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10)),
      current_path_index_(0) {}

// 添加默认构造函数定义
RobotController::RobotController()
    : node_(nullptr),
      cmd_vel_pub_(nullptr),
      current_path_index_(0) {}

// 添加初始化方法定义
void RobotController::initialize(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

geometry_msgs::msg::Twist RobotController::moveToTarget(
    const info_interfaces::msg::Point& current,
    const info_interfaces::msg::Point& target) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // 计算方向向量
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance > POSITION_TOLERANCE) {
        double angle = std::atan2(dy, dx);
        double current_angle = std::atan2(current.y, current.x);
        double angle_diff = angle - current_angle;
        
        // 限制旋转角度
        if (std::abs(angle_diff) > ANGLE_TOLERANCE) {
            cmd_vel.angular.z = ANGULAR_SPEED * angle_diff;
        } else {
            cmd_vel.linear.x = LINEAR_SPEED;
        }
    }
    
    return cmd_vel;
}

geometry_msgs::msg::Twist RobotController::rotateTurret(
    const info_interfaces::msg::Point& current,
    const info_interfaces::msg::Point& target) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // 计算目标角度
    double target_angle = std::atan2(target.y - current.y, target.x - current.x);
    double current_angle = std::atan2(current.y, current.x);
    double angle_diff = target_angle - current_angle;
    
    // 限制旋转角度
    if (std::abs(angle_diff) > ANGLE_TOLERANCE) {
        cmd_vel.angular.z = ANGULAR_SPEED * angle_diff;
    }
    
    return cmd_vel;
}

bool RobotController::canShoot(
    const info_interfaces::msg::Point& our_pos,
    const info_interfaces::msg::Point& enemy_pos,
    const info_interfaces::msg::Map& map,
    double turret_angle) {
    
    // 检查距离
    double dx = enemy_pos.x - our_pos.x;
    double dy = enemy_pos.y - our_pos.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance > ATTACK_RANGE) {
        return false;
    }
    
    // 检查角度
    double target_angle = std::atan2(dy, dx);
    if (std::abs(target_angle - turret_angle) > ANGLE_TOLERANCE) {
        return false;
    }
    
    // 检查墙体
    return !hasWallBetween(our_pos, enemy_pos, map);
}

bool RobotController::needResupply(int current_ammo) {
    return current_ammo <= MIN_AMMO;
}

bool RobotController::hasWallBetween(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& end,
    const info_interfaces::msg::Map& map) {
    
    // 使用Bresenham算法检查两点之间是否有墙
    int x1 = start.x;
    int y1 = start.y;
    int x2 = end.x;
    int y2 = end.y;
    
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int x = x1;
    int y = y1;
    
    int step_x = (x1 < x2) ? 1 : -1;
    int step_y = (y1 < y2) ? 1 : -1;
    int error = dx - dy;
    
    while (x != x2 || y != y2) {
        // 检查当前点是否是墙
        std::vector<unsigned char>::size_type index = y * map.col + x;
        if (index < map.mat.size() && map.mat[index] == 1) {
            return true;
        }
        
        int error2 = error * 2;
        if (error2 > -dy) {
            error -= dy;
            x += step_x;
        }
        if (error2 < dx) {
            error += dx;
            y += step_y;
        }
    }
    
    return false;
}

void RobotController::update(const info_interfaces::msg::Point& current_pos) {
    if (current_path_index_ >= path_.size()) {
        return; // 如果路径已经走完，直接返回
    }

    // 计算下一个目标点
    const auto& target = path_[current_path_index_];

    // 计算当前位置与目标位置的距离
    double distance = std::sqrt(std::pow(target.x - current_pos.x, 2) + std::pow(target.y - current_pos.y, 2));

    if (distance <= POSITION_TOLERANCE) {
        // 如果到达目标点，移动到下一个目标点
        current_path_index_++;
        return;
    }

    // 计算控制指令
    geometry_msgs::msg::Twist cmd_vel = calculateCommand(current_pos, target);

    // 发布控制指令
    cmd_vel_pub_->publish(cmd_vel);

    // 发布位姿信息
    geometry_msgs::msg::Pose2D pose;
    pose.x = current_pos.x;
    pose.y = current_pos.y;
    pose.theta = std::atan2(target.y - current_pos.y, target.x - current_pos.x);
    publishPose(pose);

    // 检查是否需要射击
    if (robot_data_ && !robot_data_->enemy.empty()) {
        bool shoot = canShoot(current_pos, robot_data_->enemy[0], *map_data_, pose.theta);
        publishShoot(shoot);
    }
}

void RobotController::setPath(const std::vector<info_interfaces::msg::Point>& path) {
    path_ = path;
    current_path_index_ = 0; // 重置路径索引
}

// 添加 calculateCommand 函数定义
geometry_msgs::msg::Twist RobotController::calculateCommand(
    const info_interfaces::msg::Point& current,
    const info_interfaces::msg::Point& target) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // 计算方向向量
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance > POSITION_TOLERANCE) {
        double angle = std::atan2(dy, dx);
        double current_angle = std::atan2(current.y, current.x);
        double angle_diff = angle - current_angle;
        
        // 限制旋转角度
        if (std::abs(angle_diff) > ANGLE_TOLERANCE) {
            cmd_vel.angular.z = ANGULAR_SPEED * angle_diff;
        } else {
            cmd_vel.linear.x = LINEAR_SPEED;
        }
    }
    
    return cmd_vel;
}