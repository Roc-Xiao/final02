//
// Created by roc on 25-2-25.
//

#include "rclcpp/rclcpp.hpp"
#include "answer/game_state.hpp"
#include "answer/image_processor.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>

class MainNode : public rclcpp::Node {
public:
    MainNode() : Node("answer_node") {
        // 声明参数
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("map_topic", "map");
        this->declare_parameter("area_topic", "area");
        this->declare_parameter("robot_topic", "robot");
        this->declare_parameter("image_topic", "game_image");
        this->declare_parameter("linear_speed", 0.5);
        this->declare_parameter("angular_speed", 1.0);
        this->declare_parameter("position_tolerance", 0.1);

        // 获取参数值
        std::string serial_port = this->get_parameter("serial_port").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        std::string area_topic = this->get_parameter("area_topic").as_string();
        std::string robot_topic = this->get_parameter("robot_topic").as_string();
        std::string image_topic = this->get_parameter("image_topic").as_string();
        double linear_speed = this->get_parameter("linear_speed").as_double();
        double angular_speed = this->get_parameter("angular_speed").as_double();
        double position_tolerance = this->get_parameter("position_tolerance").as_double();

        RCLCPP_INFO(this->get_logger(), "Initializing with parameters:");
        RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port.c_str());
        RCLCPP_INFO(this->get_logger(), "Map topic: %s", map_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Area topic: %s", area_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Robot topic: %s", robot_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic.c_str());

        try {
            // 创建串口处理器
            serial_ = std::make_shared<my_serial::MySerial>(serial_port, 115200);
            if (!serial_->open()) {
                throw std::runtime_error("Failed to open serial port: " + serial_port);
            }

            // 创建图像处理器
            image_processor_ = std::make_unique<ImageProcessor>();

            // 创建游戏状态控制器
            game_state_ = std::make_unique<GameState>(
                std::shared_ptr<rclcpp::Node>(this), serial_);

            // 创建订阅者
            map_sub_ = this->create_subscription<info_interfaces::msg::Map>(
                map_topic, 10,
                std::bind(&MainNode::mapCallback, this, std::placeholders::_1));

            area_sub_ = this->create_subscription<info_interfaces::msg::Area>(
                area_topic, 10,
                std::bind(&MainNode::areaCallback, this, std::placeholders::_1));

            robot_sub_ = this->create_subscription<info_interfaces::msg::Robot>(
                robot_topic, 10,
                std::bind(&MainNode::robotCallback, this, std::placeholders::_1));

            // 创建图像订阅者
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic, 10,
                std::bind(&MainNode::imageCallback, this, std::placeholders::_1));

            // 创建定时器用于检查游戏状态
            using namespace std::chrono_literals;
            timer_ = this->create_wall_timer(
                100ms, std::bind(&MainNode::checkGameState, this));

            RCLCPP_INFO(this->get_logger(), "Main node initialized successfully");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
            throw;
        }
    }

private:
    void mapCallback(const info_interfaces::msg::Map::SharedPtr msg) {
        try {
            game_state_->updateMap(msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in map callback: %s", e.what());
        }
    }

    void areaCallback(const info_interfaces::msg::Area::SharedPtr msg) {
        try {
            game_state_->updateArea(msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in area callback: %s", e.what());
        }
    }

    void robotCallback(const info_interfaces::msg::Robot::SharedPtr msg) {
        try {
            game_state_->updateRobot(msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in robot callback: %s", e.what());
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 处理图像并获取数据
            auto map_data = image_processor_->processMap(cv_ptr->image);
            auto area_data = image_processor_->processAreas(cv_ptr->image);
            auto robot_data = image_processor_->processRobots(cv_ptr->image);

            // 更新游戏状态
            game_state_->updateMap(std::make_shared<info_interfaces::msg::Map>(map_data));
            game_state_->updateArea(std::make_shared<info_interfaces::msg::Area>(area_data));
            game_state_->updateRobot(std::make_shared<info_interfaces::msg::Robot>(robot_data));

            // 可选：显示处理后的图像（用于调试）
            if (this->get_parameter("debug_view").as_bool()) {
                cv::imshow("Processed Image", cv_ptr->image);
                cv::waitKey(1);
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }

    void checkGameState() {
        try {
            if (game_state_->isGameComplete()) {
                RCLCPP_INFO(this->get_logger(), "Game completed successfully!");
                rclcpp::shutdown();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in game state check: %s", e.what());
        }
    }

    std::shared_ptr<my_serial::MySerial> serial_;
    std::unique_ptr<GameState> game_state_;
    std::unique_ptr<ImageProcessor> image_processor_;

    rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr map_sub_;
    rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr area_sub_;
    rclcpp::Subscription<info_interfaces::msg::Robot>::SharedPtr robot_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<MainNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Program terminated with error: " << e.what() << std::endl;
        return 1;
    }
}