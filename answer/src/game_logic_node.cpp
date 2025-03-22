#include "answer/game_logic_node.h"  // 确保包含了 PathPlanner 的定义

namespace game_logic {

GameLogicNode::GameLogicNode()
    : Node("game_logic_node"),
      rows_(256),
      cols_(128),
      node_(std::make_shared<rclcpp::Node>("game_logic_node")),
      shoot_pub_(node_->create_publisher<std_msgs::msg::String>(topic_name::shoot, 10)),
      pose_pub_(node_->create_publisher<geometry_msgs::msg::Pose2D>(topic_name::pose, 10)),
      password_pub_(node_->create_publisher<std_msgs::msg::String>(topic_name::password, 10)) {

    map_data_ = std::make_shared<info_interfaces::msg::Map>();
    robot_data_ = std::make_shared<info_interfaces::msg::Robot>();
    current_area_ = std::make_shared<info_interfaces::msg::Area>();
    // 订阅地图话题
    map_subscription_ = this->create_subscription<info_interfaces::msg::Map>(
        topic_name::map, 10, std::bind(&GameLogicNode::updateMap, this, std::placeholders::_1));

    // 订阅区域话题
    area_subscription_ = this->create_subscription<info_interfaces::msg::Area>(
        topic_name::area, 10, std::bind(&GameLogicNode::updateArea, this, std::placeholders::_1));

    // 订阅机器人话题
    robot_subscription_ = this->create_subscription<info_interfaces::msg::Robot>(
        topic_name::robot, 10, std::bind(&GameLogicNode::updateRobot, this, std::placeholders::_1));
}

void GameLogicNode::updateMap(const info_interfaces::msg::Map::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received map message with grid size: %d x %d", msg->grid_width, msg->grid_height);
    map_data_ = msg;

    // 根据地图数据构建可通行路径网络
    buildPathNetwork();
}

// 添加 buildPathNetwork 方法实现
void GameLogicNode::buildPathNetwork() {
    // 使用A*算法构建可通行路径网络
    path_network_.clear(); // 清空之前的路径网络

    // 获取地图的实际行数和列数
    rows_ = map_data_->row;
    cols_ = map_data_->col;

    for (uint32_t i = 0; i < rows_; ++i) {
        for (uint32_t j = 0; j < cols_; ++j) {
            addNodeToPathNetwork(i, j);
        }
    }
}

void GameLogicNode::addNodeToPathNetwork(int row, int col) {
    // 添加节点到路径网络的具体实现
    // 使用邻接表表示路径网络
    cv::Point node(col, row);

    if (isValid(node, map_data_->mat)) {
        std::vector<cv::Point> neighbors = getNeighbors(node);
        std::vector<cv::Point> valid_neighbors;

        for (const auto& neighbor : neighbors) {
            if (isValid(neighbor, map_data_->mat)) {
                valid_neighbors.push_back(neighbor);
            }
        }

        path_network_[node] = valid_neighbors; // 将有效的邻居节点添加到路径网络中
    }
}

void GameLogicNode::updateArea(const info_interfaces::msg::Area::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received area message with base position: (%u, %u)", msg->base.x, msg->base.y);
    current_area_ = msg;
}

void GameLogicNode::updateRobot(const info_interfaces::msg::Robot::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received robot message with our robot position: (%u, %u)", msg->our_robot.x, msg->our_robot.y);
    robot_data_ = msg;
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
            if (static_cast<std::size_t>(current_path_index_) >= current_path_.size()) {
                current_path_.clear();
                current_path_index_ = 0;
            }
        }
    }

    if (canShoot(robot_data_->our_robot, getNearestEnemy(current_pos), *map_data_)) { // 解引用 map_data_
        publishShoot(true);
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
    pose_pub_->publish(cmd_vel);
}

void GameLogicNode::publishShoot(bool shoot) {
    std_msgs::msg::String msg;
    msg.data = shoot ? "shoot" : "";
    shoot_pub_->publish(msg);
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
    return aStarSearch(start, goal, map_data);
}

std::vector<info_interfaces::msg::Point> GameLogicNode::aStarSearch(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal,
    const std::vector<uint8_t>& map_data) {
    // A*算法的具体实现
    std::unordered_set<cv::Point, PointHash> closed_set;
    std::priority_queue<std::pair<double, cv::Point>, std::vector<std::pair<double, cv::Point>>, PointCompare> open_set;

    std::unordered_map<cv::Point, cv::Point, PointHash> came_from;
    std::unordered_map<cv::Point, double, PointHash> g_score;
    std::unordered_map<cv::Point, double, PointHash> f_score;

    cv::Point start_point(start.x, start.y);
    cv::Point goal_point(goal.x, goal.y);

    open_set.push({0.0, start_point});
    g_score[start_point] = 0.0;
    f_score[start_point] = calculateHeuristic(start_point, goal_point);

    while (!open_set.empty()) {
        auto current = open_set.top().second;
        open_set.pop();

        if (current == goal_point) {
            return reconstructPath(came_from, start_point, goal_point);
        }

        closed_set.insert(current);

        for (const auto& neighbor : getNeighbors(current)) {
            if (!isValid(neighbor, map_data) || closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            double tentative_g_score = g_score[current] + calculateDistance(current, neighbor);
            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = tentative_g_score + calculateHeuristic(neighbor, goal_point);
                open_set.push({f_score[neighbor], neighbor});
            }
        }
    }

    return {}; // 如果没有找到路径，返回空向量
}

std::vector<info_interfaces::msg::Point> GameLogicNode::reconstructPath(
    const std::unordered_map<cv::Point, cv::Point, PointHash>& came_from,
    const cv::Point& start,
    const cv::Point& goal) {
    std::vector<info_interfaces::msg::Point> path;
    cv::Point current = goal;
    while (current != start) {

        info_interfaces::msg::Point point;
        point.x = static_cast<float>(current.x);
        point.y = static_cast<float>(current.y);
        path.push_back(point);
        current = came_from.at(current);
    }
    // 添加起始点
    info_interfaces::msg::Point startPoint;
    startPoint.x = static_cast<float>(start.x);
    startPoint.y = static_cast<float>(start.y);
    path.push_back(startPoint);
    std::reverse(path.begin(), path.end());
    return path;
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

} // namespace game_logic

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto game_logic_node = std::make_shared<game_logic::GameLogicNode>();
    rclcpp::spin(game_logic_node);
    rclcpp::shutdown();
    return 0;
}