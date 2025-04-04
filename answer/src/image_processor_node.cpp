#include <chrono>
#include "answer/image_processor_node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace image_processor {

    ImageProcessorNode::ImageProcessorNode()
        : Node("image_processor_node"),
          image_width_(0),
          image_height_(0)
    {
        // 订阅图像话题
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name::image_raw, 10, std::bind(&ImageProcessorNode::imageCallback, this, std::placeholders::_1));

        // 初始化地图发布者
        map_publisher_ = this->create_publisher<info_interfaces::msg::Map>(topic_name::map, 10);

        // 初始化区域发布者
        area_publisher_ = this->create_publisher<info_interfaces::msg::Area>(topic_name::area, 10);

        // 初始化机器人发布者
        robot_publisher_ = this->create_publisher<info_interfaces::msg::Robot>(topic_name::robot, 10);

        // 创建定时器，每500毫秒触发一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&ImageProcessorNode::timerCallback, this));
    }

    void ImageProcessorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 检查图像消息是否为空
            if (!msg || msg->data.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty image message");
                return;
            }

            // 设置图像宽度和高度
            image_width_ = msg->width;
            image_height_ = msg->height;

            // 检查图像编码是否为BGR8或RGB8
            if (msg->encoding != sensor_msgs::image_encodings::BGR8 && msg->encoding != sensor_msgs::image_encodings::RGB8) {
                RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
                return;
            }

            // 将 ROS 图像消息转换为 OpenCV 图像格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat cv_raw_img = cv_ptr->image;

            // 检查图像是否为空
            if (cv_raw_img.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty image");
                return;
            }

            // 确认图像数据类型
            if (cv_raw_img.type() != CV_8UC3) {
                RCLCPP_ERROR(this->get_logger(), "Invalid image type: %d", cv_raw_img.type());
                return;
            }

            latest_image_ = cv_raw_img.clone();

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    void ImageProcessorNode::timerCallback() {
        if (!latest_image_.empty()) {

            // 处理地图数据
            info_interfaces::msg::Map map_msg = processMap(latest_image_);
            if (map_publisher_->get_subscription_count() > 0) {
                map_publisher_->publish(map_msg);
                RCLCPP_INFO(this->get_logger(), "Map message published successfully with grid size: %d x %d", map_msg.grid_width, map_msg.grid_height);
            } else {
                RCLCPP_WARN(this->get_logger(), "No subscribers for map topic");
            }

            // 处理区域信息
            info_interfaces::msg::Area area_msg = processAreas(latest_image_);
            if (area_publisher_->get_subscription_count() > 0) {
                area_publisher_->publish(area_msg);
                RCLCPP_INFO(this->get_logger(), "Area message published successfully with base position: (%u, %u)", area_msg.base.x, area_msg.base.y);
            } else {
                RCLCPP_WARN(this->get_logger(), "No subscribers for area topic");
            }

            // 处理机器人信息
            info_interfaces::msg::Robot robot_msg = processRobots(latest_image_);
            if (robot_publisher_->get_subscription_count() > 0) {
                robot_publisher_->publish(robot_msg);
                RCLCPP_INFO(this->get_logger(), "Robot message published successfully with our robot position: (%u, %u)", robot_msg.our_robot.x, robot_msg.our_robot.y);
            } else {
                RCLCPP_WARN(this->get_logger(), "No subscribers for robot topic");
            }

            // 添加日志记录
            RCLCPP_INFO(this->get_logger(), "Processed and published image data");
        }
    }

    info_interfaces::msg::Map ImageProcessorNode::processMap(const cv::Mat& image) {
        info_interfaces::msg::Map map_msg;

        // 获取迷宫区域
        cv::Mat binary;
        cv::Scalar lower = cv::Scalar(55, 55, 55);
        cv::Scalar upper = cv::Scalar(60, 60, 60);
        cv::inRange(image, lower, upper, binary);

        // 对图像进行去噪操作
        cv::Mat cleaned = morphologyProcess(binary);

        // 设置地图的基本信息
        map_msg.row = cleaned.rows;
        map_msg.col = cleaned.cols;
        map_msg.grid_width = cleaned.cols;
        map_msg.grid_height = cleaned.rows;

        // 初始化map.mat数组
        map_msg.mat.resize(cleaned.total());

        // 将处理后的图像数据转换为网格图
        for (int j = 0; j < cleaned.rows; ++j) {
            for (int i = 0; i < cleaned.cols; ++i) {
                map_msg.mat[i + j * cleaned.cols] = cleaned.at<uchar>(j, i) > 0 ? 1 : 0;
            }
        }

        return map_msg;
    }

    info_interfaces::msg::Area ImageProcessorNode::processAreas(const cv::Mat& image) {
        info_interfaces::msg::Area area_msg;

        // 确保输入图像不是灰度图
        if (image.channels() == 1) {
            RCLCPP_ERROR(this->get_logger(), "Input image is grayscale, cannot process areas");
            return area_msg;
        }

        // 查找基地位置
        cv::Mat base_mask;
        cv::inRange(image, BACE_LOWER, BACE_UPPER, base_mask);
        auto base_points = findContours(base_mask, BACE_LOWER, BACE_UPPER, 20, 20, 100, 100); // 添加颜色范围参数
        if (!base_points.empty()) {
            area_msg.base = transformAbstractPoint(base_points[0]);
        }

        // 查找补给站位置
        cv::Mat recover_mask;
        cv::inRange(image, RECOVER_LOWER, RECOVER_UPPER, recover_mask);
        auto recover_points = findContours(recover_mask, RECOVER_LOWER, RECOVER_UPPER, 20, 20, 100, 100); // 添加颜色范围参数
        if (!recover_points.empty()) {
            area_msg.recover = transformAbstractPoint(recover_points[0]);
        }

        // 查找密码区域
        cv::Mat password_mask;
        cv::inRange(image, PASSWORD_LOWER, PASSWORD_UPPER, password_mask);
        auto password_points = findContours(password_mask, PASSWORD_LOWER, PASSWORD_UPPER, 20, 20, 100, 100); // 添加颜色范围参数
        if (!password_points.empty()) {
            area_msg.password = transformAbstractPoint(password_points[0]);
        }

        // 查找绿色传送点
        cv::Mat green_mask;
        cv::inRange(image, GREEN_LOWER, GREEN_UPPER, green_mask);
        auto green_teleport = findTeleportPoints(green_mask, GREEN_LOWER, GREEN_UPPER, 0, 0, 100, 100); // 添加颜色范围参数
        area_msg.green_in = transformAbstractPoint(green_teleport.first);
        area_msg.green_out = transformAbstractPoint(green_teleport.second);

        // 查找紫色传送点
        cv::Mat purple_mask;
        cv::inRange(image, PURPLE_LOWER, PURPLE_UPPER, purple_mask);
        auto purple_teleport = findTeleportPoints(purple_mask, PURPLE_LOWER, PURPLE_UPPER, 0, 0, 100, 100); // 添加颜色范围参数
        area_msg.purple_in = transformAbstractPoint(purple_teleport.first);
        area_msg.purple_out = transformAbstractPoint(purple_teleport.second);

        return area_msg;
    }

    info_interfaces::msg::Robot ImageProcessorNode::processRobots(const cv::Mat& image) {
        info_interfaces::msg::Robot robot_msg;

        // 确保输入图像不是灰度图
        if (image.channels() == 1) {
            RCLCPP_ERROR(this->get_logger(), "Input image is grayscale, cannot process robots");
            return robot_msg;
        }

        // 查找我方机器人
        cv::Mat our_robot_mask;
        cv::inRange(image, ROBOT_LOWER, ROBOT_UPPER, our_robot_mask);
        auto our_robot = findContours(our_robot_mask, ROBOT_LOWER, ROBOT_UPPER, 20, 20, 100, 100); // 添加颜色范围参数
        if (!our_robot.empty()) {
            robot_msg.our_robot = transformAbstractPoint(our_robot[0]);
        }

        // 查找敌方机器人
        cv::Mat enemy_mask;
        cv::inRange(image, ENEMY_LOWER, ENEMY_UPPER, enemy_mask);
        auto enemy_robots = findContours(enemy_mask, ENEMY_LOWER, ENEMY_UPPER, 20, 20, 100, 100); // 添加颜色范围参数
        for (const auto& enemy : enemy_robots) {
            info_interfaces::msg::Point enemy_point = transformAbstractPoint(enemy);
            robot_msg.enemy.push_back(enemy_point);
        }

        return robot_msg;
    }

    std::vector<cv::Point> ImageProcessorNode::findContours(const cv::Mat& image, cv::Scalar lower, cv::Scalar upper, int min_x, int max_x, int min_y, int max_y) {
        cv::Mat mask;
        cv::inRange(image, lower, upper, mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> points;
        for (const auto& contour : contours) {
            cv::Rect rect = cv::boundingRect(contour);
            if (rect.x >= min_x && rect.x + rect.width <= max_x && rect.y >= min_y && rect.y + rect.height <= max_y) {
                cv::Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
                points.push_back(center);
            }
        }
        return points;
    }

    std::pair<cv::Point, cv::Point> ImageProcessorNode::findTeleportPoints(const cv::Mat& image, cv::Scalar lower, cv::Scalar upper, int min_x, int max_x, int min_y, int max_y) {
        cv::Mat mask;
        cv::inRange(image, lower, upper, mask);  // 直接在RGB图像上进行颜色匹配

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> points;
        for (const auto& contour : contours) {
            cv::Rect rect = cv::boundingRect(contour);
            if (rect.x >= min_x && rect.x + rect.width <= max_x && rect.y >= min_y && rect.y + rect.height <= max_y) {
                cv::Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
                points.push_back(center);
            }
        }

        if (points.size() < 2) {
            return {{0, 0}, {0, 0}};
        }
        // 假设第一个点是入口，第二个点是出口
        return {points[0], points[1]};
    }

    cv::Mat ImageProcessorNode::morphologyProcess(const cv::Mat& image) {
        cv::Mat result;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        // 开运算去除小噪点
        cv::morphologyEx(image, result, cv::MORPH_OPEN, kernel);

        // 闭运算填充小孔
        cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);

        return result;
    }

    info_interfaces::msg::Point ImageProcessorNode::transformAbstractPoint(const cv::Point& point) {
        // 将像素坐标转换为256*128的网格坐标
        info_interfaces::msg::Point abstract_point;
        abstract_point.x = static_cast<float>(point.x * 256 / image_width_);
        abstract_point.y = static_cast<float>(point.y * 128 / image_height_);
        return abstract_point;
    }

}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto image_processor_node = std::make_shared<image_processor::ImageProcessorNode>();
    rclcpp::spin(image_processor_node);
    rclcpp::shutdown();
    return 0;
}