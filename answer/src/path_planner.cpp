//
// Created by roc on 25-2-25.
//
#include "answer/path_planner.h"

#include <queue>
#include <unordered_map>
#include <functional>

// 添加 PointCompare 结构体
struct PointCompare {
    bool operator()(const std::pair<double, cv::Point>& a, const std::pair<double, cv::Point>& b) const {
        return a.first > b.first; // 优先队列按 f 值从小到大排序
    }
};

PathPlanner::PathPlanner(uint32_t rows, uint32_t cols)
    : rows_(rows), cols_(cols) {}

std::vector<info_interfaces::msg::Point> PathPlanner::findPath(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal,
    const std::vector<uint8_t>& map_data) {

    cv::Point start_point(start.x, start.y);
    cv::Point goal_point(goal.x, goal.y);

    // 使用优先队列存储待探索节点，添加 PointCompare 比较函数
    std::priority_queue<
        std::pair<double, cv::Point>,
        std::vector<std::pair<double, cv::Point>>,
        PointCompare
    > open_set;

    // 记录每个节点的g值（从起点到该节点的实际代价）
    std::unordered_map<cv::Point, double, PointHash> g_values;
    g_values[start_point] = 0.0;

    // 记录每个节点的f值（g值加上启发式代价）
    std::unordered_map<cv::Point, double, PointHash> f_values;
    f_values[start_point] = calculateHeuristic(start_point, goal_point);

    // 记录每个节点的父节点
    std::unordered_map<cv::Point, cv::Point, PointHash> came_from;

    // 将起点加入开放列表
    open_set.emplace(f_values[start_point], start_point);

    while (!open_set.empty()) {
        cv::Point current = open_set.top().second;
        open_set.pop();

        // 如果到达目标点，重建路径并返回
        if (current == goal_point) {
            std::vector<cv::Point> path;
            while (current != start_point) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start_point);
            std::reverse(path.begin(), path.end());

            // 转换为ROS消息格式
            std::vector<info_interfaces::msg::Point> result;
            for (const auto& p : path) {
                info_interfaces::msg::Point point;
                point.x = p.x;
                point.y = p.y;
                result.push_back(point);
            }

            return result;
        }

        // 获取相邻点
        auto neighbors = getNeighbors(current);

        for (const auto& neighbor : neighbors) {
            if (!isValid(neighbor, map_data)) continue;

            // 计算从起点经过当前节点到邻居节点的代价
            double tentative_g_value = g_values[current] + 1.0;

            // 如果找到更优路径，更新代价和父节点
            if (!g_values.count(neighbor) || tentative_g_value < g_values[neighbor]) {
                came_from[neighbor] = current;
                g_values[neighbor] = tentative_g_value;
                f_values[neighbor] = tentative_g_value + calculateHeuristic(neighbor, goal_point);
                open_set.emplace(f_values[neighbor], neighbor);
            }
        }
    }

    // 如果没有找到路径，返回空路径
    return std::vector<info_interfaces::msg::Point>();
}

bool PathPlanner::isValid(const cv::Point& p, const std::vector<uint8_t>& map_data) const {
    if (p.x < 0 || p.x >= static_cast<int>(cols_) ||
        p.y < 0 || p.y >= static_cast<int>(rows_)) {
        return false;
    }
    return map_data[p.y * cols_ + p.x] == 0; // 假设0表示可通行
}

std::vector<cv::Point> PathPlanner::getNeighbors(const cv::Point& p) const {
    std::vector<cv::Point> neighbors;
    const int dx[] = {-1, 0, 1, 0};
    const int dy[] = {0, 1, 0, -1};

    for (int i = 0; i < 4; ++i) {
        neighbors.emplace_back(p.x + dx[i], p.y + dy[i]);
    }

    return neighbors;
}

double PathPlanner::calculateHeuristic(const cv::Point& a, const cv::Point& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}