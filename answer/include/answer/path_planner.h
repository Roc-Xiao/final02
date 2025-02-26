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

    // DFS搜索状态
    struct SearchState {
        cv::Point pos;
        std::vector<cv::Point> path;
        std::vector<bool> visited;

        SearchState(const cv::Point& p, uint32_t size)
            : pos(p), visited(size, false) {}
    };

    bool isValid(const cv::Point& p, const std::vector<uint8_t>& map_data) const;
    std::vector<cv::Point> getNeighbors(const cv::Point& p) const;
    double calculateHeuristic(const cv::Point& a, const cv::Point& b) const;
};

#endif