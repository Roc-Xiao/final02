#include "answer/image_processor_node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace image_processor {

void ImageProcessorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 检查图像消息是否为空
        if (!msg || msg->data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received empty image message");
            return;
        }

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

        // 预处理图像
        cv::Mat processed = preprocess(cv_raw_img);

        // 处理地图数据
        info_interfaces::msg::Map map_msg = processMap(processed);  // 使用预处理后的图像
        map_publisher_->publish(map_msg);

        // 处理区域信息
        info_interfaces::msg::Area area_msg = processAreas(processed);  // 使用预处理后的图像
        area_publisher_->publish(area_msg);

        // 处理机器人信息
        info_interfaces::msg::Robot robot_msg = processRobots(processed);  // 使用预处理后的图像
        robot_publisher_->publish(robot_msg);

        // 添加日志记录
        RCLCPP_INFO(this->get_logger(), "Processed and published image data");

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

cv::Mat ImageProcessorNode::preprocess(const cv::Mat& image) {
    // 不再直接转换为灰度图，保持原样返回
    return image.clone();
}

info_interfaces::msg::Map ImageProcessorNode::processMap(const cv::Mat& image) {
    info_interfaces::msg::Map map_msg;

    // 边缘检测前先转换为灰度图
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat edges = detectEdges(gray);

    // 形态学操作，清理噪声
    cv::Mat cleaned = morphologyProcess(edges);

    // 转换为地图数据
    map_msg.row = cleaned.rows;
    map_msg.col = cleaned.cols;
    map_msg.grid_width = 256;  // 网格宽度
    map_msg.grid_height = 128; // 网格高度

    // 将图像数据转换为一维数组
    map_msg.mat.resize(cleaned.total());
    for (int i = 0; i < cleaned.rows; ++i) {
        for (int j = 0; j < cleaned.cols; ++j) {
            map_msg.mat[i * cleaned.cols + j] = cleaned.at<uchar>(i, j) > 0 ? 1 : 0;
        }
    }

    return map_msg;
}

info_interfaces::msg::Area ImageProcessorNode::processAreas(const cv::Mat& image) {
    info_interfaces::msg::Area area_msg;
    cv::Mat hsv;

    // 确保输入图像不是灰度图
    if (image.channels() == 1) {
        RCLCPP_ERROR(this->get_logger(), "Input image is grayscale, cannot process areas");
        return area_msg;
    }

    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 查找基地位置
    auto base_points = findContours(hsv, BACE_LOWER, BACE_UPPER, 20, 20, 100, 100);
    if (!base_points.empty()) {
        area_msg.base.x = base_points[0].x;
        area_msg.base.y = base_points[0].y;
    }

    // 查找补给站位置
    auto recover_points = findContours(hsv, RECOVER_LOWER, RECOVER_UPPER, 20, 20, 100, 100);
    if (!recover_points.empty()) {
        area_msg.recover.x = recover_points[0].x;
        area_msg.recover.y = recover_points[0].y;
    }

    // 查找密码区域
    auto password_points = findContours(hsv, PASSWORD_LOWER, PASSWORD_UPPER, 20, 20, 100, 100);
    if (!password_points.empty()) {
        area_msg.password.x = password_points[0].x;
        area_msg.password.y = password_points[0].y;
    }

    // 查找绿色传送点
    auto green_teleport = findTeleportPoints(hsv, GREEN_LOWER, GREEN_UPPER, 0, 0, 100, 100);
    area_msg.green_in.x = green_teleport.first.x;
    area_msg.green_in.y = green_teleport.first.y;
    area_msg.green_out.x = green_teleport.second.x;
    area_msg.green_out.y = green_teleport.second.y;

    // 查找紫色传送点
    auto purple_teleport = findTeleportPoints(hsv, PURPLE_LOWER, PURPLE_UPPER, 0, 0, 100, 100);
    area_msg.purple_in.x = purple_teleport.first.x;
    area_msg.purple_in.y = purple_teleport.first.y;
    area_msg.purple_out.x = purple_teleport.second.x;
    area_msg.purple_out.y = purple_teleport.second.y;

    return area_msg;
}

info_interfaces::msg::Robot ImageProcessorNode::processRobots(const cv::Mat& image) {
    info_interfaces::msg::Robot robot_msg;
    cv::Mat hsv;

    // 确保输入图像不是灰度图
    if (image.channels() == 1) {
        RCLCPP_ERROR(this->get_logger(), "Input image is grayscale, cannot process robots");
        return robot_msg;
    }

    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 查找我方机器人
    auto our_robot = findContours(hsv, ROBOT_LOWER, ROBOT_UPPER, 20, 20, 100, 100);
    if (!our_robot.empty()) {
        robot_msg.our_robot.x = our_robot[0].x;
        robot_msg.our_robot.y = our_robot[0].y;
    }

    // 查找敌方机器人
    auto enemy_robots = findContours(hsv, ENEMY_LOWER, ENEMY_UPPER, 20, 20, 100, 100);
    for (const auto& enemy : enemy_robots) {
        info_interfaces::msg::Point enemy_point;
        enemy_point.x = enemy.x;
        enemy_point.y = enemy.y;
        robot_msg.enemy.push_back(enemy_point);
    }

    return robot_msg;
}

std::vector<cv::Point> ImageProcessorNode::findContours(const cv::Mat& hsv, cv::Scalar lower, cv::Scalar upper, int min_x, int max_x, int min_y, int max_y) {
    cv::Mat mask;
    cv::inRange(hsv, lower, upper, mask);
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

std::pair<cv::Point, cv::Point> ImageProcessorNode::findTeleportPoints(const cv::Mat& hsv, cv::Scalar lower, cv::Scalar upper, int min_x, int max_x, int min_y, int max_y) {
    std::vector<cv::Point> points = findContours(hsv, lower, upper, min_x, max_x, min_y, max_y);
    if (points.size() < 2) {
        return {{0, 0}, {0, 0}};
    }
    // 假设第一个点是入口，第二个点是出口
    return {points[0], points[1]};
}

cv::Mat ImageProcessorNode::detectEdges(const cv::Mat& image) {
    cv::Mat result;
    cv::Canny(image, result, 50, 150);
    return result;
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
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto image_processor_node = std::make_shared<image_processor::ImageProcessorNode>();
    rclcpp::spin(image_processor_node);
    rclcpp::shutdown();
    return 0;
}