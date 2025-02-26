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
    std::vector<cv::Point> findColorArea(const cv::Mat& image, const cv::Scalar& lower, const cv::Scalar& upper);

    // 查找传送点
    std::pair<cv::Point, cv::Point> findTeleportPoints(const cv::Mat& image, const cv::Scalar& color);

    // 形态学操作
    cv::Mat morphologyProcess(const cv::Mat& image);

    // 颜色阈值
    const cv::Scalar BLUE_LOWER{100, 50, 50};
    const cv::Scalar BLUE_UPPER{140, 255, 255};
    const cv::Scalar RED_LOWER{0, 50, 50};
    const cv::Scalar RED_UPPER{10, 255, 255};
    const cv::Scalar GREEN_LOWER{35, 50, 50};
    const cv::Scalar GREEN_UPPER{85, 255, 255};
    const cv::Scalar PURPLE_LOWER{140, 50, 50};
    const cv::Scalar PURPLE_UPPER{170, 255, 255};
};

#endif