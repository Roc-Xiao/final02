#ifndef ANSWER_GAME_LOGIC_NODE_H
#define ANSWER_GAME_LOGIC_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"
#include "std_msgs/msg/string.hpp"
#include "answer/topic_name.h"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <stack>
#include <unordered_set>
#include <memory>

namespace game_logic {

struct PointCompare {
    bool operator()(const std::pair<double, cv::Point>& a, const std::pair<double, cv::Point>& b) const {
        return a.first > b.first || (a.first == b.first && (a.second.x > b.second.x || (a.second.x == b.second.x && a.second.y > b.second.y)));
    }
};

struct PointHash {
    std::size_t operator()(const cv::Point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

class GameLogicNode : public rclcpp::Node {
public:
    GameLogicNode();

private:

    std::unordered_map<cv::Point, std::vector<cv::Point>, PointHash> path_network_;

    void updateMap(const info_interfaces::msg::Map::SharedPtr msg);
    void updateArea(const info_interfaces::msg::Area::SharedPtr msg);
    void updateRobot(const info_interfaces::msg::Robot::SharedPtr msg);

    bool canShoot(
        const info_interfaces::msg::Point& our_pos,
        const info_interfaces::msg::Point& enemy_pos,
        const info_interfaces::msg::Map& map);

    // 原PathPlanner类部分
    uint32_t rows_;
    uint32_t cols_;

    std::vector<info_interfaces::msg::Point> findPath(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& goal,
        const std::vector<uint8_t>& map_data);

    bool isValid(const cv::Point& p, const std::vector<uint8_t>& map_data) const;
    std::vector<cv::Point> getNeighbors(const cv::Point& p) const;
    double calculateHeuristic(const cv::Point& a, const cv::Point& b) const;
    double calculateDistance(const cv::Point& p1, const cv::Point& p2) const;

    // 原RobotController类部分
    geometry_msgs::msg::Pose2D moveToTarget(
        const info_interfaces::msg::Point& current,
        const info_interfaces::msg::Point& target);

    geometry_msgs::msg::Pose2D rotateTurret(
        const info_interfaces::msg::Point& current,
        const info_interfaces::msg::Point& target);

    bool needResupply(int current_ammo);

    bool hasWallBetween(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& end,
        const info_interfaces::msg::Map& map);

    void update(const info_interfaces::msg::Point& current_pos);

    info_interfaces::msg::Point getNearestEnemy();

    info_interfaces::msg::Point getNearestEnemy(const info_interfaces::msg::Point& our_pos);

    geometry_msgs::msg::Pose2D calculateCommand(
        const info_interfaces::msg::Point& current,
        const info_interfaces::msg::Point& target);

    void publishPose(const geometry_msgs::msg::Pose2D& pose);

    void publishShoot(bool shoot);

    std::vector<info_interfaces::msg::Point> calculatePath(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& goal);

    std::vector<info_interfaces::msg::Point> current_path_;

    // Other RobotController members
    // std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shoot_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    std::vector<info_interfaces::msg::Point> path_;
    int current_path_index_;
    info_interfaces::msg::Area::SharedPtr current_area_;
    info_interfaces::msg::Robot::SharedPtr robot_data_;
    info_interfaces::msg::Map::SharedPtr map_data_;
    info_interfaces::msg::Point last_robot_pose_;
    info_interfaces::msg::Point target_pose_;

    rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr map_subscription_;
    rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr area_subscription_;
    rclcpp::Subscription<info_interfaces::msg::Robot>::SharedPtr robot_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr password_pub_;

    // 添加常量定义
    static constexpr double POSITION_TOLERANCE = 0.1;
    static constexpr double ANGLE_TOLERANCE = 0.1;
    static constexpr double ANGULAR_SPEED = 0.5;
    static constexpr double VELOCITY_SCALE = 1.0;
    static constexpr double LINEAR_SPEED = 0.5;
    static constexpr double ATTACK_RANGE = 5.0;

    void buildPathNetwork();

    void addNodeToPathNetwork(int row, int col);

    std::vector<info_interfaces::msg::Point> aStarSearch(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& goal,
        const std::vector<uint8_t>& map_data);

    std::vector<info_interfaces::msg::Point> reconstructPath(
        const std::unordered_map<cv::Point, cv::Point, PointHash>& came_from,
        const cv::Point& start,
        const cv::Point& goal);
};

} // namespace game_logic

#endif // ANSWER_GAME_LOGIC_NODE_H