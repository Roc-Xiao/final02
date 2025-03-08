//
// Created by roc on 25-2-25.
//
#include "answer/path_planner.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath> // 添加: 包含 cmath 头文件

PathPlanner::PathPlanner(uint32_t rows, uint32_t cols)
    : rows_(rows), cols_(cols) {}

std::vector<info_interfaces::msg::Point> PathPlanner::findPath(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal,
    const std::vector<uint8_t>& map_data) {

    // 定义优先队列，使用 PointCompare 作为比较函数
    std::priority_queue<
        std::pair<double, cv::Point>,
        std::vector<std::pair<double, cv::Point>>,
        PointCompare> open_set;

    // 定义成本和前驱映射
    std::unordered_map<cv::Point, double, PointHash> g_costs;
    std::unordered_map<cv::Point, cv::Point, PointHash> came_from;

    // 初始化起点
    cv::Point start_cv(start.x, start.y);
    cv::Point goal_cv(goal.x, goal.y);
    open_set.emplace(0, start_cv);
    g_costs[start_cv] = 0;

    while (!open_set.empty()) {
        auto current = open_set.top().second;
        open_set.pop();

        if (current == goal_cv) {
            // 重建路径
            std::vector<info_interfaces::msg::Point> path;
            while (current != start_cv) {
                // 修改: 使用 x 和 y 成员变量设置点的坐标
                info_interfaces::msg::Point point;
                point.x = static_cast<float>(current.x);
                point.y = static_cast<float>(current.y);
                path.emplace_back(point);
                current = came_from[current];
            }
            // 修改: 使用 x 和 y 成员变量设置点的坐标
            info_interfaces::msg::Point point;
            point.x = static_cast<float>(start.x);
            point.y = static_cast<float>(start.y);
            path.emplace_back(point);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& neighbor : getNeighbors(current)) {
            if (!isValid(neighbor, map_data)) {
                continue;
            }

            double tentative_g_cost = g_costs[current] + calculateDistance(current, neighbor);
            if (g_costs.find(neighbor) == g_costs.end() || tentative_g_cost < g_costs[neighbor]) {
                came_from[neighbor] = current;
                g_costs[neighbor] = tentative_g_cost;
                double f_cost = tentative_g_cost + calculateHeuristic(neighbor, goal_cv);
                open_set.emplace(f_cost, neighbor);
            }
        }
    }

    // 如果没有找到路径，返回空路径
    return {};
}

bool PathPlanner::isValid(const cv::Point& p, const std::vector<uint8_t>& map_data) const {
    // 修改: 先检查是否小于0，避免无意义的 unsigned 比较
    if (p.x < 0 || p.y < 0 || static_cast<uint32_t>(p.x) >= cols_ || static_cast<uint32_t>(p.y) >= rows_) {
        return false;
    }
    return map_data[p.y * cols_ + p.x] == 0; // 假设0表示可通行，1表示障碍物
}

std::vector<cv::Point> PathPlanner::getNeighbors(const cv::Point& p) const {
    std::vector<cv::Point> neighbors;
    neighbors.emplace_back(p.x + 1, p.y);
    neighbors.emplace_back(p.x - 1, p.y);
    neighbors.emplace_back(p.x, p.y + 1);
    neighbors.emplace_back(p.x, p.y - 1);
    return neighbors;
}

double PathPlanner::calculateHeuristic(const cv::Point& a, const cv::Point& b) const {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// 添加: calculateDistance 函数定义
double PathPlanner::calculateDistance(const cv::Point& p1, const cv::Point& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}
