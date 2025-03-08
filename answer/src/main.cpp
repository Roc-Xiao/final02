#include "rclcpp/rclcpp.hpp"
#include "answer/game_state.h"
#include "answer/robot_controller.h"
#include "answer/game_node.h" // 添加: 包含 GameNode 头文件
#include "info_interfaces/msg/robot.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp" // 确保包含正确的 std_msgs::msg::String

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 修改: 使用 rclcpp::NodeOptions 创建 GameNode
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GameNode>(options);
    node->initialize(); // 初始化控制器和游戏状态
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}