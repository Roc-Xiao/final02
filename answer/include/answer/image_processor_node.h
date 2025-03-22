#ifndef ANSWER_IMAGE_PROCESSOR_NODE_H
#define ANSWER_IMAGE_PROCESSOR_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"
#include "cv_bridge/cv_bridge.h"
#include "answer/topic_name.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <stack>
#include <unordered_set>
#include <memory>

namespace image_processor {

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

    class ImageProcessorNode : public rclcpp::Node {
    public:
        ImageProcessorNode(): Node("image_processor_node") {
            // 订阅图像话题
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                topic_name::image_raw, 10, std::bind(&ImageProcessorNode::imageCallback, this, std::placeholders::_1));

            // 初始化地图发布者
            map_publisher_ = this->create_publisher<info_interfaces::msg::Map>(topic_name::map, 10);

            // 初始化区域发布者
            area_publisher_ = this->create_publisher<info_interfaces::msg::Area>(topic_name::area, 10);

            // 初始化机器人发布者
            robot_publisher_ = this->create_publisher<info_interfaces::msg::Robot>(topic_name::robot, 10);
        }

    private:
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        info_interfaces::msg::Map processMap(const cv::Mat& image);
        std::vector<cv::Point> findColorArea(const cv::Mat& hsv, cv::Scalar lower, cv::Scalar upper, int min_x, int max_x, int min_y, int max_y);
        std::pair<cv::Point, cv::Point> findTeleportPoints(const cv::Mat& hsv, cv::Scalar lower, cv::Scalar upper, int min_x, int max_x, int min_y, int max_y);

        // 添加以下两个方法声明
        info_interfaces::msg::Area processAreas(const cv::Mat& image);
        info_interfaces::msg::Robot processRobots(const cv::Mat& image);

        // 新增方法声明：坐标抽象化
        info_interfaces::msg::Point transformAbstractPoint(const cv::Point& point);

        // 新增方法声明：通用轮廓检测
        std::vector<cv::Point> findContours(const cv::Mat& hsv, cv::Scalar lower, cv::Scalar upper, int min_x, int max_x, int min_y, int max_y);

        const cv::Scalar BACE_LOWER{165, 115, 145};
        const cv::Scalar BACE_UPPER{175, 125, 155};
        const cv::Scalar ROBOT_LOWER{85, 165, 235};
        const cv::Scalar ROBOT_UPPER{95, 175, 245};
        const cv::Scalar RECOVER_LOWER{55, 80, 105};
        const cv::Scalar RECOVER_UPPER{65, 90, 115};
        const cv::Scalar ENEMY_LOWER{250, 100, 100};
        const cv::Scalar ENEMY_UPPER{255, 110, 110};
        const cv::Scalar GREEN_LOWER{25, 195, 110};
        const cv::Scalar GREEN_UPPER{35, 205, 120};
        const cv::Scalar PURPLE_LOWER{190, 95, 210};
        const cv::Scalar PURPLE_UPPER{200, 105, 220};
        const cv::Scalar PASSWORD_LOWER{90, 95, 250};
        const cv::Scalar PASSWORD_UPPER{100, 105, 255};

        cv::Mat preprocess(const cv::Mat& image);
        cv::Mat detectEdges(const cv::Mat& image);
        cv::Mat morphologyProcess(const cv::Mat& image);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
        rclcpp::Publisher<info_interfaces::msg::Map>::SharedPtr map_publisher_;
        rclcpp::Publisher<info_interfaces::msg::Area>::SharedPtr area_publisher_;
        rclcpp::Publisher<info_interfaces::msg::Robot>::SharedPtr robot_publisher_;
    };

} // namespace image_processor

#endif // ANSWER_IMAGE_PROCESSOR_NODE_H