//
// Created by roc on 25-2-26.
//

#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <info_interfaces/msg/area.hpp>
#include <info_interfaces/msg/map.hpp>
#include <info_interfaces/msg/robot.hpp>

class ImageProcessor {
public:
    ImageProcessor();

    // 处理地图图像，返回地图数据
    info_interfaces::msg::Map processMap(const cv::Mat& image);

    // 处理区域图像，返回区域数据
    info_interfaces::msg::Area processAreas(const cv::Mat& image);

    // 处理机器人图像，返回机器人位置数据
    info_interfaces::msg::Robot processRobots(const cv::Mat& image);

private:
    // 图像预处理
    cv::Mat preprocess(const cv::Mat& image);

    // 边缘检测
    cv::Mat detectEdges(const cv::Mat& image);

    // 查找特定颜色区域
    std::vector<cv::Point> findColorArea(const cv::Mat& image, const cv::Scalar& lower, const cv::Scalar& upper, int min_width, int min_height, int max_width, int max_height);

    // 查找传送点
    std::pair<cv::Point, cv::Point> findTeleportPoints(const cv::Mat& image, const cv::Scalar& color_lower, const cv::Scalar& color_upper, int min_width, int min_height, int max_width, int max_height);

    // 形态学操作
    cv::Mat morphologyProcess(const cv::Mat& image);

    // 颜色阈值
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
};

#endif