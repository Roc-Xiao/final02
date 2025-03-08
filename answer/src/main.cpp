#include "rclcpp/rclcpp.hpp"
#include "answer/game_state.h"
#include "answer/robot_controller.h"
#include "info_interfaces/msg/robot.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp" // 确保包含正确的 std_msgs::msg::String

class GameNode : public rclcpp::Node {
public:
    GameNode() : Node("game_node") {
        // 初始化订阅者
        robot_sub_ = create_subscription<info_interfaces::msg::Robot>(
            "/robot_info", 10, 
            std::bind(&GameNode::robotCallback, this, std::placeholders::_1));
            
        map_sub_ = create_subscription<info_interfaces::msg::Map>(
            "/map_info", 10,
            std::bind(&GameNode::mapCallback, this, std::placeholders::_1));
            
        area_sub_ = create_subscription<info_interfaces::msg::Area>(
            "/area_info", 10,
            std::bind(&GameNode::areaCallback, this, std::placeholders::_1));

        // 初始化发布者
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        password_pub_ = create_publisher<std_msgs::msg::String>("/password", 10); // 确保使用正确的 std_msgs::msg::String
        pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10); // 添加位姿发布器
        shoot_pub_ = create_publisher<example_interfaces::msg::Bool>("/shoot", 10); // 添加射击发布器

        // 初始化控制器
        controller_ = std::make_unique<RobotController>();
        controller_->setPosePublisher(pose_pub_); // 设置位姿发布器
        controller_->setShootPublisher(shoot_pub_); // 设置射击发布器
    }

    void initialize() {
        // 传递 shared_from_this() 给控制器和游戏状态
        controller_->initialize(shared_from_this());
        game_state_ = std::make_unique<GameState>(shared_from_this()); // 添加: 初始化 game_state_
    }

private:
    void robotCallback(const info_interfaces::msg::Robot::SharedPtr msg) {
        current_robot_ = *msg;
        game_state_->updateRobot(msg); // 更新 robot_data_
        updateGameLogic();
    }

    void mapCallback(const info_interfaces::msg::Map::SharedPtr msg) {
        current_map_ = *msg;
        game_state_->updateMap(msg); // 更新 map_data_
    }

    void areaCallback(const info_interfaces::msg::Area::SharedPtr msg) {
        game_area_ = *msg;
    }

    void updateGameLogic() {
        switch (game_state_->getCurrentState()) {
            case GameState::State::HUNTING_ENEMIES:
                handleHuntingState();
                break;
            case GameState::State::GOTO_PASSWORD_AREA:
                handleGotoPasswordArea();
                break;
            case GameState::State::ATTACKING_BASE:
                handleAttackingBase();
                break;
            default:
                break;
        }
    }

    void handleHuntingState() {
        if (!current_robot_.enemy.empty()) {
            auto cmd = controller_->moveToTarget(
                current_robot_.our_robot,
                current_robot_.enemy[0]);
            cmd_pub_->publish(cmd);
        }
    }

    void handleGotoPasswordArea() {
        auto cmd = controller_->moveToTarget(
            current_robot_.our_robot,
            game_area_.password);
        cmd_pub_->publish(cmd);
    }

    void handleAttackingBase() {
        if (game_state_->isBaseVulnerable()) {
            auto cmd = controller_->moveToTarget(
                current_robot_.our_robot,
                game_area_.base);
            cmd_pub_->publish(cmd);
        }
    }

    // 成员变量
    std::unique_ptr<RobotController> controller_;
    std::unique_ptr<GameState> game_state_;
    
    info_interfaces::msg::Robot current_robot_;
    info_interfaces::msg::Map current_map_;
    info_interfaces::msg::Area game_area_;
    std::string final_password_;

    rclcpp::Subscription<info_interfaces::msg::Robot>::SharedPtr robot_sub_;
    rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr map_sub_;
    rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr area_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr password_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr password_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_; // 添加位姿发布器成员变量
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr shoot_pub_; // 添加射击发布器成员变量
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GameNode>();
    node->initialize(); // 初始化控制器和游戏状态
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
