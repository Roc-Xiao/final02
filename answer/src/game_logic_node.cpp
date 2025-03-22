#include "answer/game_logic_node.h"  // 确保包含了 PathPlanner 的定义

namespace game_logic {

GameLogicNode::GameLogicNode()
    : Node("game_logic_node"),
      rows_(256),
      cols_(128) {
    // 订阅地图话题
    map_subscription_ = this->create_subscription<info_interfaces::msg::Map>(
        topic_name::map, 10, std::bind(&GameLogicNode::updateMap, this, std::placeholders::_1));

    // 订阅区域话题
    area_subscription_ = this->create_subscription<info_interfaces::msg::Area>(
        topic_name::area, 10, std::bind(&GameLogicNode::updateArea, this, std::placeholders::_1));

    // 订阅机器人话题
    robot_subscription_ = this->create_subscription<info_interfaces::msg::Robot>(
        topic_name::robot, 10, std::bind(&GameLogicNode::updateRobot, this, std::placeholders::_1));

    // 初始化 pose 话题发布者
    auto pose_pub = this->create_publisher<geometry_msgs::msg::Pose2D>(topic_name::pose, 10);
    setPosePublisher(pose_pub);

    // 初始化 shoot 话题发布者
    auto shoot_pub = this->create_publisher<std_msgs::msg::String>(topic_name::shoot, 10);
    setShootPublisher(shoot_pub);

    // 初始化密码发布者
    password_pub_ = this->create_publisher<std_msgs::msg::String>(topic_name::password, 10);
}

void GameLogicNode::initialize(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(topic_name::pose, 10);
    shoot_pub_ = node_->create_publisher<std_msgs::msg::String>(topic_name::shoot, 10);
}

geometry_msgs::msg::Pose2D GameLogicNode::moveToTarget(
    const info_interfaces::msg::Point& current,
    const info_interfaces::msg::Point& target) {
    geometry_msgs::msg::Pose2D cmd_vel;
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance > POSITION_TOLERANCE) {
        double angle = std::atan2(dy, dx);
        double current_angle = std::atan2(current.y, current.x);
        double angle_diff = angle - current_angle;
        
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        if (std::abs(angle_diff) > ANGLE_TOLERANCE) {
            cmd_vel.theta = ANGULAR_SPEED * angle_diff * VELOCITY_SCALE;
        } else {
            cmd_vel.x = LINEAR_SPEED;
        }
    }
    
    return cmd_vel;
}

geometry_msgs::msg::Pose2D GameLogicNode::rotateTurret(
    const info_interfaces::msg::Point& current,
    const info_interfaces::msg::Point& target) {
    geometry_msgs::msg::Pose2D cmd_vel;
    double target_angle = std::atan2(target.y - current.y, target.x - current.x);
    double current_angle = std::atan2(current.y, current.x);
    double angle_diff = target_angle - current_angle;
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
    if (std::abs(angle_diff) > ANGLE_TOLERANCE) {
        cmd_vel.theta = ANGULAR_SPEED * angle_diff * VELOCITY_SCALE;
    }
    return cmd_vel;
}

bool GameLogicNode::canShoot(
    const info_interfaces::msg::Point& our_pos,
    const info_interfaces::msg::Point& enemy_pos,
    const info_interfaces::msg::Map& map) {
    double distance = std::sqrt((our_pos.x - enemy_pos.x)*(our_pos.x - enemy_pos.x) + (our_pos.y - enemy_pos.y)*(our_pos.y - enemy_pos.y));
    return distance <= ATTACK_RANGE && !hasWallBetween(our_pos, enemy_pos, map);
}

bool GameLogicNode::hasWallBetween(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& end,
    const info_interfaces::msg::Map& map) {
    int x1 = start.x, y1 = start.y, x2 = end.x, y2 = end.y;
    int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
    int x = x1, y = y1;
    int step_x = (x1 < x2) ? 1 : -1;
    int step_y = (y1 < y2) ? 1 : -1;
    int error = dx - dy;
    while (x != x2 || y != y2) {
        std::size_t index = y * map.col + x;
        if (index < map.mat.size() && map.mat[index] == 1) return true;
        int error2 = error * 2;
        if (error2 > -dy) { error -= dy; x += step_x; }
        if (error2 < dx) { error += dx; y += step_y; }
    }

    return false;
}

void GameLogicNode::update(const info_interfaces::msg::Point& current_pos) {
    if (current_path_.empty()) {
        // 修正 calculatePath() 方法调用，提供正确参数
        current_path_ = calculatePath(robot_data_->our_robot, getNearestEnemy(current_pos));
    }

    if (!current_path_.empty()) {
        auto target = current_path_[current_path_index_];
        auto cmd_vel = calculateCommand(current_pos, target);
        publishPose(cmd_vel);
        if ((current_pos.x > target.x ? (current_pos.x - target.x) : (target.x - current_pos.x)) < POSITION_TOLERANCE &&
            (current_pos.y > target.y ? (current_pos.y - target.y) : (target.y - current_pos.y)) < POSITION_TOLERANCE) {
            current_path_index_++;
            if (static_cast<std::size_t>(current_path_index_) >= current_path_.size()) { // 将 int 转换为 std::size_t
                current_path_.clear();
                current_path_index_ = 0;
            }
        }
    }

    if (canShoot(robot_data_->our_robot, getNearestEnemy(current_pos), *map_data_)) { // 解引用 map_data_
        publishShoot(true); // 修改: 添加布尔参数 true
    }
}

std::vector<info_interfaces::msg::Point> GameLogicNode::calculatePath(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal) {
    return findPath(start, goal, map_data_->mat);
}

geometry_msgs::msg::Pose2D GameLogicNode::calculateCommand(
    const info_interfaces::msg::Point& current,
    const info_interfaces::msg::Point& target) {
    auto move_cmd = moveToTarget(current, target);
    auto rotate_cmd = rotateTurret(current, target);
    move_cmd.theta += rotate_cmd.theta;
    return move_cmd;
}

void GameLogicNode::publishPose(const geometry_msgs::msg::Pose2D& cmd_vel) {
    cmd_vel_pub_->publish(cmd_vel);
}

void GameLogicNode::publishShoot(bool shoot) {
    std_msgs::msg::String msg;
    msg.data = shoot ? "shoot" : "";
    shoot_pub_->publish(msg);
}

void GameLogicNode::setPosePublisher(rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub) {
    cmd_vel_pub_ = pub;
}

void GameLogicNode::setShootPublisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) {
    shoot_pub_ = pub;
}

void GameLogicNode::setMapData(const info_interfaces::msg::Map::SharedPtr map_data) {
    // 直接将传入的 shared_ptr 赋值给成员变量
    map_data_ = map_data;
}

void GameLogicNode::setRobotData(const info_interfaces::msg::Robot::SharedPtr robot_data) {
    // 直接将传入的 shared_ptr 赋值给成员变量
    robot_data_ = robot_data;
}

info_interfaces::msg::Point GameLogicNode::getNearestEnemy(const info_interfaces::msg::Point& our_pos) {
    info_interfaces::msg::Point nearest_enemy;
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& enemy : robot_data_->enemy) { // 使用 -> 解引用
        double distance = std::sqrt((our_pos.x - enemy.x)*(our_pos.x - enemy.x) + (our_pos.y - enemy.y)*(our_pos.y - enemy.y)); // 使用 our_pos 而不是 robot_data_->our_robot
        if (distance < min_distance) {
            min_distance = distance;
            nearest_enemy = enemy;
        }
    }
    return nearest_enemy;
}

std::vector<info_interfaces::msg::Point> GameLogicNode::findPath(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal,
    const std::vector<uint8_t>& map_data) {
    if (!isValid(cv::Point(start.x, start.y), map_data) || !isValid(cv::Point(goal.x, goal.y), map_data)) {
        return {};
    }

    std::priority_queue<
        std::pair<double, cv::Point>,
        std::vector<std::pair<double, cv::Point>>,
        PointCompare> open_set;

    std::unordered_map<cv::Point, double, PointHash> g_costs;
    std::unordered_map<cv::Point, cv::Point, PointHash> came_from;

    cv::Point start_cv(start.x, start.y);
    cv::Point goal_cv(goal.x, goal.y);
    open_set.emplace(0, start_cv);
    g_costs[start_cv] = 0;

    while (!open_set.empty()) {
        auto current = open_set.top().second;
        open_set.pop();

        if (current == goal_cv) {
            std::vector<info_interfaces::msg::Point> path;
            while (current != start_cv) {
                info_interfaces::msg::Point point;
                point.x = static_cast<float>(current.x);
                point.y = static_cast<float>(current.y);
                path.emplace_back(point);
                current = came_from[current];
            }
            // 修改: 直接赋值 x 和 y 字段，而不是使用花括号初始化列表
            info_interfaces::msg::Point start_point;
            start_point.x = start.x;
            start_point.y = start.y;
            path.push_back(start_point);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& neighbor : getNeighbors(current)) {
            if (!isValid(neighbor, map_data)) continue;

            double tentative_g_cost = g_costs[current] + calculateDistance(current, neighbor);
            if (g_costs.find(neighbor) == g_costs.end() || tentative_g_cost < g_costs[neighbor]) {
                came_from[neighbor] = current;
                g_costs[neighbor] = tentative_g_cost;
                double f_cost = tentative_g_cost + calculateHeuristic(neighbor, goal_cv);
                open_set.emplace(f_cost, neighbor);
            }
        }
    }
    return {};
}

std::vector<cv::Point> GameLogicNode::getNeighbors(const cv::Point& p) const {
    std::vector<cv::Point> neighbors;
    neighbors.emplace_back(p.x + 1, p.y);
    neighbors.emplace_back(p.x - 1, p.y);
    neighbors.emplace_back(p.x, p.y + 1);
    neighbors.emplace_back(p.x, p.y - 1);
    return neighbors;
}

bool GameLogicNode::isValid(const cv::Point& p, const std::vector<uint8_t>& map_data) const {
    if (p.x < 0 || p.y < 0 || static_cast<uint32_t>(p.x) >= cols_ || static_cast<uint32_t>(p.y) >= rows_)
        return false;
    return map_data[p.y * cols_ + p.x] == 0;
}

double GameLogicNode::calculateHeuristic(const cv::Point& a, const cv::Point& b) const {
    return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

double GameLogicNode::calculateDistance(const cv::Point& p1, const cv::Point& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
}

void GameLogicNode::updateMap(const info_interfaces::msg::Map::SharedPtr msg) {
    // 更新地图数据
    setMapData(msg);
}

void GameLogicNode::updateArea(const info_interfaces::msg::Area::SharedPtr msg) {
    // 更新区域数据
    current_area_ = msg;
}

void GameLogicNode::updateRobot(const info_interfaces::msg::Robot::SharedPtr msg) {
    // 更新机器人数据
    setRobotData(msg);
}

} // namespace game_logic

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto game_logic_node = std::make_shared<game_logic::GameLogicNode>();
    rclcpp::spin(game_logic_node);
    rclcpp::shutdown();
    return 0;
}