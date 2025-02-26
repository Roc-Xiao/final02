//
// Created by roc on 25-2-25.
//

#ifndef GAME_STATE_HPP
#define GAME_STATE_HPP

#include <rclcpp/rclcpp.hpp>
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/robot.hpp"
#include "info_interfaces/msg/point.hpp"
#include "my_serial.hpp"
#include "path_planner.hpp"
#include "robot_controller.hpp"

class GameState {
public:
    GameState(std::shared_ptr<rclcpp::Node> node);

    void updateMap(const info_interfaces::msg::Map::SharedPtr msg);
    void updateArea(const info_interfaces::msg::Area::SharedPtr msg);
    void updateRobot(const info_interfaces::msg::Robot::SharedPtr msg);
    bool isGameComplete() const { return game_complete_; }

private:
    enum class State {
        INIT,
        SEEKING_SUPPLY,
        HUNTING_ENEMIES,
        SENDING_PASSWORD,
        ATTACKING_BASE
    };

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<my_serial::MySerial> serial_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<RobotController> robot_controller_;

    State current_state_;
    bool game_complete_;

    info_interfaces::msg::Map::SharedPtr map_data_;
    info_interfaces::msg::Area::SharedPtr area_data_;
    info_interfaces::msg::Robot::SharedPtr robot_data_;

    std::vector<uint64_t> password_fragments_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr password_pub_;

    void handleState();
    std::vector<info_interfaces::msg::Point> planPath(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& goal);
};

#endif