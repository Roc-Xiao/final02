//
// Created by roc on 25-2-25.
//
#include "answer/path_planner.h"

PathPlanner::PathPlanner(uint32_t rows, uint32_t cols)
    : rows_(rows), cols_(cols) {}

std::vector<info_interfaces::msg::Point> PathPlanner::findPath(
    const info_interfaces::msg::Point& start,
    const info_interfaces::msg::Point& goal,
    const std::vector<uint8_t>& map_data) {

    cv::Point start_point(start.x, start.y);
    cv::Point goal_point(goal.x, goal.y);

    // 初始化搜索状态
    std::stack<SearchState> stack;
    SearchState initial_state(start_point, rows_ * cols_);
    initial_state.path.push_back(start_point);
    stack.push(initial_state);

    // 记录最佳路径
    std::vector<cv::Point> best_path;
    double best_cost = std::numeric_limits<double>::infinity();

    while (!stack.empty()) {
        SearchState current = stack.top();
        stack.pop();

        // 标记当前位置为已访问
        current.visited[current.pos.y * cols_ + current.pos.x] = true;

        // 检查是否到达目标
        if (current.pos == goal_point) {
            double current_cost = current.path.size();
            if (current_cost < best_cost) {
                best_cost = current_cost;
                best_path = current.path;
            }
            continue;
        }

        // 获取相邻点
        auto neighbors = getNeighbors(current.pos);

        // 按照到目标点的启发式距离排序邻居节点
        std::sort(neighbors.begin(), neighbors.end(),
            [this, goal_point](const cv::Point& a, const cv::Point& b) {
                return calculateHeuristic(a, goal_point) < calculateHeuristic(b, goal_point);
            });

        // 遍历邻居节点
        for (const auto& next : neighbors) {
            if (!isValid(next, map_data)) continue;
            if (current.visited[next.y * cols_ + next.x]) continue;

            // 创建新的搜索状态
            SearchState next_state = current;
            next_state.pos = next;
            next_state.path.push_back(next);
            stack.push(next_state);
        }
    }

    // 转换为ROS消息格式
    std::vector<info_interfaces::msg::Point> result;
    for (const auto& p : best_path) {
        info_interfaces::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        result.push_back(point);
    }

    return result;
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