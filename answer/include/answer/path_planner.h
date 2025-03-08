//
// Created by roc on 25-2-25.
//

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <stack>
#include "info_interfaces/msg/point.hpp"
#include "info_interfaces/msg/map.hpp"

// 添加 PointCompare 结构体
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

class PathPlanner {
public:
    PathPlanner(uint32_t rows, uint32_t cols);

    std::vector<info_interfaces::msg::Point> findPath(
        const info_interfaces::msg::Point& start,
        const info_interfaces::msg::Point& goal,
        const std::vector<uint8_t>& map_data);

private:
    uint32_t rows_;
    uint32_t cols_;

    bool isValid(const cv::Point& p, const std::vector<uint8_t>& map_data) const;
    std::vector<cv::Point> getNeighbors(const cv::Point& p) const;
    double calculateHeuristic(const cv::Point& a, const cv::Point& b) const;
    double calculateDistance(const cv::Point& p1, const cv::Point& p2) const;
};

#endif