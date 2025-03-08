//
// Created by roc on 25-2-26.
//

#include "answer/image_processor.h"

ImageProcessor::ImageProcessor() {}

info_interfaces::msg::Map ImageProcessor::processMap(const cv::Mat& image) {
    info_interfaces::msg::Map map_msg;

    // 预处理图像
    cv::Mat processed = preprocess(image);

    // 边缘检测
    cv::Mat edges = detectEdges(processed);

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

info_interfaces::msg::Area ImageProcessor::processAreas(const cv::Mat& image) {
    info_interfaces::msg::Area area_msg;
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 查找基地位置
    auto base_points = findColorArea(hsv, BACE_LOWER, BACE_UPPER, 20, 20, 100, 100);
    if (!base_points.empty()) {
        area_msg.base.x = base_points[0].x;
        area_msg.base.y = base_points[0].y;
    }

    // 查找补给站位置
    auto recover_points = findColorArea(hsv, RECOVER_LOWER, RECOVER_UPPER, 20, 20, 100, 100);
    if (!recover_points.empty()) {
        area_msg.recover.x = recover_points[0].x;
        area_msg.recover.y = recover_points[0].y;
    }

    // 查找密码区域
    auto password_points = findColorArea(hsv, PASSWORD_LOWER, PASSWORD_UPPER, 20, 20, 100, 100);
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

info_interfaces::msg::Robot ImageProcessor::processRobots(const cv::Mat& image) {
    info_interfaces::msg::Robot robot_msg;
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 查找我方机器人
    auto our_robot = findColorArea(hsv, ROBOT_LOWER, ROBOT_UPPER, 20, 20, 100, 100);
    if (!our_robot.empty()) {
        robot_msg.our_robot.x = our_robot[0].x;
        robot_msg.our_robot.y = our_robot[0].y;
    }

    // 查找敌方机器人
    auto enemy_robots = findColorArea(hsv, ENEMY_LOWER, ENEMY_UPPER, 20, 20, 100, 100);
    for (const auto& enemy : enemy_robots) {
        info_interfaces::msg::Point enemy_point;
        enemy_point.x = enemy.x;
        enemy_point.y = enemy.y;
        robot_msg.enemy.push_back(enemy_point);
    }

    return robot_msg;
}

cv::Mat ImageProcessor::preprocess(const cv::Mat& image) {
    cv::Mat gray, blurred;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    return blurred;
}

cv::Mat ImageProcessor::detectEdges(const cv::Mat& image) {
    cv::Mat edges;
    cv::Canny(image, edges, 50, 150);
    return edges;
}

std::vector<cv::Point> ImageProcessor::findColorArea(
    const cv::Mat& image,
    const cv::Scalar& lower,
    const cv::Scalar& upper,
    int min_width,
    int min_height,
    int max_width,
    int max_height) {

    cv::Mat mask;
    cv::inRange(image, lower, upper, mask);

    // 形态学操作清理噪声
    mask = morphologyProcess(mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> centers;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > 100) {  // 面积阈值(存疑）
            cv::Moments m = cv::moments(contour);
            if (m.m00 != 0) {
                cv::Rect bounding_box = cv::boundingRect(contour);
                if (bounding_box.width >= min_width && bounding_box.width <= max_width &&
                    bounding_box.height >= min_height && bounding_box.height <= max_height) {
                    cv::Point center(m.m10/m.m00, m.m01/m.m00);
                    centers.push_back(center);
                }
            }
        }
    }

    return centers;
}

std::pair<cv::Point, cv::Point> ImageProcessor::findTeleportPoints(
    const cv::Mat& image,
    const cv::Scalar& color_lower,
    const cv::Scalar& color_upper,
    int min_width,
    int min_height,
    int max_width,
    int max_height) {

    cv::Mat mask;
    cv::inRange(image, color_lower, color_upper, mask);

    mask = morphologyProcess(mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> centers;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > 100) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 != 0) {
                cv::Rect bounding_box = cv::boundingRect(contour);
                if (bounding_box.width >= min_width && bounding_box.width <= max_width &&
                    bounding_box.height >= min_height && bounding_box.height <= max_height) {
                    centers.push_back(cv::Point(m.m10/m.m00, m.m01/m.m00));
                }
            }
        }
    }

    if (centers.size() >= 2) {
        return {centers[0], centers[1]};
    }
    return {cv::Point(0,0), cv::Point(0,0)};
}

cv::Mat ImageProcessor::morphologyProcess(const cv::Mat& image) {
    cv::Mat result;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // 开运算去除小噪点
    cv::morphologyEx(image, result, cv::MORPH_OPEN, kernel);

    // 闭运算填充小孔
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);

    return result;
}
