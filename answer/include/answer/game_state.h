//
// Created by roc on 25-2-25.
//

#ifndef GAME_STATE_HPP
#define GAME_STATE_HPP

#include <info_interfaces/msg/detail/area__struct.hpp>
#include <info_interfaces/msg/detail/robot__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> // 修改: 确保包含正确的 std_msgs::msg::String
#include "example_interfaces/msg/bool.hpp"
#include "info_interfaces/msg/area.h"
#include "info_interfaces/msg/map.h"
#include "info_interfaces/msg/robot.h"
#include "info_interfaces/msg/point.h"
#include "path_planner.h"
#include "robot_controller.h"

class GameState {
public:
    GameState(std::shared_ptr<rclcpp::Node> node);

    // 初始化方法
    void initialize(std::shared_ptr<rclcpp::Node> node);

    void updateMap(const info_interfaces::msg::Map::SharedPtr msg);
    void updateArea(const info_interfaces::msg::Area::SharedPtr msg);
    void updateRobot(const info_interfaces::msg::Robot::SharedPtr msg);
    bool isGameComplete() const { return game_complete_; }

public: // 修改: 将 private 改为 public
    enum class State {
        INIT,
        SEEKING_SUPPLY,
        HUNTING_ENEMIES,
        GOTO_PASSWORD_AREA,
        ATTACKING_BASE,
        SENDING_TO_JUDGE
    };

    // 添加函数声明
    void updateState(GameState::State new_state);
    GameState::State getCurrentState() const;
    void addPasswordFragment(const std::string& fragment);
    bool hasAllPasswordFragments() const;
    std::string getCombinedPassword() const;
    bool isBaseVulnerable() const;
    void setBaseVulnerable(bool vulnerable);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<RobotController> robot_controller_;

    State current_state_;
    bool game_complete_;
    bool base_vulnerable_;

    info_interfaces::msg::Map::SharedPtr map_data_;
    info_interfaces::msg::Area::SharedPtr area_data_;
    info_interfaces::msg::Robot::SharedPtr robot_data_;

    std::vector<std::string> password_fragments_; // 修改: 将类型改为 std::vector<std::string>

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr password_pub_;

    void handleState();
    std::vector<info_interfaces::msg::Point> planPath(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& goal);

    double calculateDistance(
        const info_interfaces::msg::Point& p1,
        const info_interfaces::msg::Point& p2);

    static constexpr size_t REQUIRED_FRAGMENTS = 2; // 定义 REQUIRED_FRAGMENTS 常量
};

#endif