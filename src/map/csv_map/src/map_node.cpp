#include "csv_map/map_node.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <pwd.h>
#include <sstream>
#include <unistd.h>

double NormalizeAngle(double angle) {
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

namespace map {

// 添加一个辅助函数来展开波浪号
std::string expandTilde(const std::string &path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    // 获取当前用户的主目录
    const char *home = getenv("HOME");
    if (home == nullptr) {
        struct passwd *pwd = getpwuid(getuid());
        if (pwd) {
            home = pwd->pw_dir;
        }
    }

    if (home == nullptr) {
        return path; // 如果无法获取主目录，返回原始路径
    }

    // 替换波浪号
    if (path.length() == 1) { // 仅有 "~"
        return home;
    }
    if (path[1] == '/') { // "~/xxx"
        return std::string(home) + path.substr(1);
    }
    return path; // "~xxx" 其他情况返回原始路径
}

MapNode::MapNode() : Node("map_node") {
    // 声明并获取参数
    this->declare_parameter("map_files_dir", "~/auto_clean_bot/map_files");
    this->declare_parameter("left_boundary_file", "local_record_4_left_boundary.csv");
    this->declare_parameter("right_boundary_file", "local_record_4_right_boundary.csv");
    this->declare_parameter("left_boundary_name", "left_boundary");
    this->declare_parameter("right_boundary_name", "right_boundary");
    this->declare_parameter("boundary_length", 50.0);
    this->declare_parameter("publish_frequency", 10.0);

    // 方向稳定性参数
    this->declare_parameter("direction_stability_weight", 2.0);
    this->declare_parameter("max_index_jump", 30.0);
    this->declare_parameter("yaw_weight", 3.0);

    map_files_dir_ = this->get_parameter("map_files_dir").as_string();
    std::string left_boundary_file = this->get_parameter("left_boundary_file").as_string();
    std::string right_boundary_file = this->get_parameter("right_boundary_file").as_string();
    left_boundary_name_ = this->get_parameter("left_boundary_name").as_string();
    right_boundary_name_ = this->get_parameter("right_boundary_name").as_string();
    boundary_length_ = this->get_parameter("boundary_length").as_double();
    publish_frequency_ = this->get_parameter("publish_frequency").as_double();

    // 获取方向稳定性参数
    direction_stability_weight_ = this->get_parameter("direction_stability_weight").as_double();
    max_index_jump_ = this->get_parameter("max_index_jump").as_double();
    yaw_weight_ = this->get_parameter("yaw_weight").as_double();

    // 构建边界文件的完整路径
    left_boundary_file_path_ = map_files_dir_ + "/" + left_boundary_file;
    right_boundary_file_path_ = map_files_dir_ + "/" + right_boundary_file;

    // 加载边界文件
    bool left_loaded = loadBoundaryFile(left_boundary_file_path_, left_boundary_points_);
    bool right_loaded = loadBoundaryFile(right_boundary_file_path_, right_boundary_points_);

    if (!left_loaded) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load left boundary file: %s", left_boundary_file_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Loaded left boundary file with %zu points", left_boundary_points_.size());
    }

    if (!right_loaded) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load right boundary file: %s", right_boundary_file_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Loaded right boundary file with %zu points", right_boundary_points_.size());
    }

    // 创建发布者和订阅者
    left_boundary_pub_ = this->create_publisher<bot_msg::msg::Boundary>("/map/left_boundary", 10);
    right_boundary_pub_ = this->create_publisher<bot_msg::msg::Boundary>("/map/right_boundary", 10);

    localization_sub_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
        "/localization/rtk_info", 10, std::bind(&MapNode::localizationCallback, this, std::placeholders::_1));

    // 创建定时器
    double timer_period = 1.0 / publish_frequency_;
    timer_ =
        this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&MapNode::timerCallback, this));

    // 输出方向稳定性参数
    RCLCPP_INFO(this->get_logger(), "Direction stability parameters:");
    RCLCPP_INFO(this->get_logger(), "  direction_stability_weight: %f", direction_stability_weight_);
    RCLCPP_INFO(this->get_logger(), "  max_index_jump: %f", max_index_jump_);
    RCLCPP_INFO(this->get_logger(), "  yaw_weight: %f", yaw_weight_);

    RCLCPP_INFO(this->get_logger(), "Map node initialized");
}

MapNode::~MapNode() { RCLCPP_INFO(this->get_logger(), "Map node shutting down"); }

void MapNode::localizationCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    current_east_ = msg->east;
    current_north_ = msg->north;
    current_yaw_ = msg->yaw;
    localization_received_ = true;
}

void MapNode::timerCallback() {
    if (!localization_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No localization data received yet");
        return;
    }

    if (left_boundary_points_.empty() || right_boundary_points_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Boundary points not loaded or empty");
        return;
    }

    // 查找当前位置最近的边界点
    size_t left_closest_idx = findClosestPointIndex(left_boundary_points_, current_east_, current_north_);
    size_t right_closest_idx = findClosestPointIndex(right_boundary_points_, current_east_, current_north_);

    // 创建边界消息
    auto left_boundary_msg = std::make_unique<bot_msg::msg::Boundary>();
    auto right_boundary_msg = std::make_unique<bot_msg::msg::Boundary>();

    // 设置消息头
    left_boundary_msg->header.stamp = this->now();
    left_boundary_msg->header.frame_id = "map";
    right_boundary_msg->header.stamp = this->now();
    right_boundary_msg->header.frame_id = "map";

    // 设置边界名称和类型
    left_boundary_msg->boundary_name = left_boundary_name_;
    left_boundary_msg->boundary_type = 0; // 左边界
    right_boundary_msg->boundary_name = right_boundary_name_;
    right_boundary_msg->boundary_type = 1; // 右边界

    // 计算边界段
    calculateBoundarySegment(left_boundary_points_, left_closest_idx, boundary_length_, left_boundary_msg->points);
    calculateBoundarySegment(right_boundary_points_, right_closest_idx, boundary_length_, right_boundary_msg->points);

    // 发布边界消息
    left_boundary_pub_->publish(std::move(left_boundary_msg));
    right_boundary_pub_->publish(std::move(right_boundary_msg));
}

bool MapNode::loadBoundaryFile(const std::string &file_path, std::vector<BoundaryPoint> &boundary_points) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
        return false;
    }

    boundary_points.clear();

    std::string line;
    bool header_skipped = false;

    while (std::getline(file, line)) {
        if (!header_skipped) {
            header_skipped = true;
            continue;
        }

        std::stringstream ss(line);
        std::string token;

        BoundaryPoint point;

        if (std::getline(ss, token, ',')) {
            point.east = std::stod(token);
        } else {
            continue;
        }

        if (std::getline(ss, token, ',')) {
            point.north = std::stod(token);
        } else {
            continue;
        }

        if (std::getline(ss, token, ',')) {
            point.yaw = std::stod(token);
        } else {
            point.yaw = 0.0; // 默认值
        }

        boundary_points.push_back(point);
    }

    file.close();
    return !boundary_points.empty();
}

size_t MapNode::findClosestPointIndex(const std::vector<BoundaryPoint> &boundary_points, double east, double north) {
    size_t closest_idx = 0;
    double min_combined_cost = std::numeric_limits<double>::max();

    // 方向稳定性参数 - 为左右边界分别维护历史
    static std::map<const std::vector<BoundaryPoint> *, size_t> last_closest_indices;
    static std::map<const std::vector<BoundaryPoint> *, bool> first_runs;

    // 获取或初始化当前边界的历史信息
    auto &last_closest_idx = last_closest_indices[&boundary_points];
    auto &first_run = first_runs[&boundary_points];

    double cur_yaw_rad = current_yaw_ * M_PI / 180.0; // 当前车辆航向(弧度)

    // 优化搜索策略：局部搜索 + 全局备份
    size_t search_start = 0;
    size_t search_end = boundary_points.size();

    if (!first_run && last_closest_idx < boundary_points.size()) {
        // 局部搜索范围：以上次最近点为中心的邻域
        size_t local_search_radius = static_cast<size_t>(max_index_jump_ * 1.5);

        search_start = (last_closest_idx > local_search_radius) ? (last_closest_idx - local_search_radius) : 0;
        search_end = std::min(last_closest_idx + local_search_radius + 1, boundary_points.size());

        // RCLCPP_DEBUG(this->get_logger(), "Local search range: [%zu, %zu) around last_idx: %zu",
        //              search_start, search_end, last_closest_idx);
    }

    // 局部搜索
    for (std::size_t i = search_start; i < search_end; i++) {
        double combined_cost = calculatePointCost(boundary_points, i, cur_yaw_rad, last_closest_idx, first_run);

        if (combined_cost < min_combined_cost) {
            min_combined_cost = combined_cost;
            closest_idx = i;
        }
    }

    // 如果局部搜索没有找到足够好的结果，进行全局搜索
    bool need_global_search = false;
    if (!first_run) {
        double distance_to_found = std::sqrt(std::pow(boundary_points[closest_idx].east - current_east_, 2) +
                                             std::pow(boundary_points[closest_idx].north - current_north_, 2));

        // 如果找到的点距离太远，可能需要全局搜索
        if (distance_to_found > 10.0) { // 10米阈值，可配置
            need_global_search = true;
            RCLCPP_WARN(this->get_logger(), "Local search result too far (%.2fm), performing global search",
                        distance_to_found);
        }
    }

    // 全局搜索（首次运行或局部搜索失败时）
    if (first_run || need_global_search) {
        double global_min_cost = min_combined_cost;
        size_t global_closest_idx = closest_idx;

        // 跳跃式搜索：每隔几个点采样，然后在最佳区域细化
        size_t step_size = std::max(1UL, boundary_points.size() / 100); // 最多检查100个采样点

        for (std::size_t i = 0; i < boundary_points.size(); i += step_size) {
            double combined_cost = calculatePointCost(boundary_points, i, cur_yaw_rad, last_closest_idx, first_run);

            if (combined_cost < global_min_cost) {
                global_min_cost = combined_cost;
                global_closest_idx = i;
            }
        }

        // 在最佳采样点周围进行细化搜索
        if (global_closest_idx != closest_idx) {
            size_t refine_start = (global_closest_idx > step_size) ? (global_closest_idx - step_size) : 0;
            size_t refine_end = std::min(global_closest_idx + step_size + 1, boundary_points.size());

            for (std::size_t i = refine_start; i < refine_end; i++) {
                double combined_cost = calculatePointCost(boundary_points, i, cur_yaw_rad, last_closest_idx, first_run);

                if (combined_cost < global_min_cost) {
                    global_min_cost = combined_cost;
                    global_closest_idx = i;
                }
            }

            min_combined_cost = global_min_cost;
            closest_idx = global_closest_idx;
        }
    }

    // 添加调试日志监控方向稳定性
    if (!first_run) {
        double index_change = static_cast<double>(closest_idx) - static_cast<double>(last_closest_idx);
        if (std::abs(index_change) > max_index_jump_) {
            RCLCPP_WARN(this->get_logger(),
                        "Large boundary index jump detected: from %zu to %zu (change: %.1f), "
                        "vehicle pos: (%.2f, %.2f), yaw: %.1f°",
                        last_closest_idx, closest_idx, index_change, current_east_, current_north_, current_yaw_);
        }
    }

    // 更新历史信息
    last_closest_idx = closest_idx;
    first_run = false;

    return closest_idx;
}

double MapNode::calculatePointCost(const std::vector<BoundaryPoint> &boundary_points, size_t index, double cur_yaw_rad,
                                   size_t last_closest_idx, bool first_run) {
    if (index >= boundary_points.size()) {
        return std::numeric_limits<double>::max();
    }

    // 计算距离成本
    double dist = std::sqrt(std::pow(boundary_points[index].east - current_east_, 2) +
                            std::pow(boundary_points[index].north - current_north_, 2));

    // 计算航向成本 - 将边界点航向转为弧度并计算差值
    double path_yaw_rad = boundary_points[index].yaw * M_PI / 180.0;
    double yaw_diff = std::abs(NormalizeAngle(path_yaw_rad) - NormalizeAngle(cur_yaw_rad));

    // 计算方向稳定性成本
    double stability_cost = 0.0;
    if (!first_run) {
        // 计算与上次最近点的索引差异
        double index_diff = std::abs(static_cast<double>(index) - static_cast<double>(last_closest_idx));

        // 如果索引跳跃过大，增加惩罚
        if (index_diff > max_index_jump_) {
            stability_cost = direction_stability_weight_ * (index_diff - max_index_jump_);
        }

        // 检查是否可能发生方向反转
        if (index_diff > boundary_points.size() / 4) {
            // 计算边界方向变化
            if (last_closest_idx < boundary_points.size() && index < boundary_points.size()) {
                double last_path_yaw = boundary_points[last_closest_idx].yaw * M_PI / 180.0;
                double current_path_yaw = boundary_points[index].yaw * M_PI / 180.0;
                double direction_change = std::abs(NormalizeAngle(current_path_yaw - last_path_yaw));

                // 如果方向变化超过90度，可能是反向，增加大的惩罚
                if (direction_change > M_PI / 2) {
                    stability_cost += direction_stability_weight_ * 10.0;
                }
            }
        }
    }

    // 组合成本：距离 + 航向差异权重 + 方向稳定性成本
    return dist + yaw_weight_ * yaw_diff + stability_cost;
}

void MapNode::calculateBoundarySegment(const std::vector<BoundaryPoint> &boundary_points, size_t start_index,
                                       double length, std::vector<bot_msg::msg::BoundaryPoint> &segment_points) {
    const double previous_distance = 2.0;
    segment_points.clear();

    if (boundary_points.empty() || start_index >= boundary_points.size()) {
        return;
    }

    double accumulated_distance = 0.0;
    double previous_east = boundary_points[start_index].east;
    double previous_north = boundary_points[start_index].north;

    // 向后添加点，直到达到previous_distance或边界开始
    for (int i = start_index - 1; i >= 0; --i) {
        double dx = boundary_points[i].east - previous_east;
        double dy = boundary_points[i].north - previous_north;
        double segment_distance = std::sqrt(dx * dx + dy * dy);

        if (accumulated_distance + segment_distance > previous_distance) {
            break;
        }

        accumulated_distance += segment_distance;
        previous_east = boundary_points[i].east;
        previous_north = boundary_points[i].north;

        bot_msg::msg::BoundaryPoint point;
        point.east = boundary_points[i].east;
        point.north = boundary_points[i].north;
        point.up = 0.0; // 默认高度为0
        point.distance = accumulated_distance;
        segment_points.push_back(point);
    }
    std::reverse(segment_points.begin(), segment_points.end());

    // 添加起始点
    bot_msg::msg::BoundaryPoint start_point;
    start_point.east = boundary_points[start_index].east;
    start_point.north = boundary_points[start_index].north;
    start_point.up = 0.0; // 默认高度为0
    start_point.distance = 0.0;
    segment_points.push_back(start_point);

    // 向前添加点，直到达到指定长度或边界结束
    for (size_t i = start_index + 1; i < boundary_points.size(); ++i) {
        double dx = boundary_points[i].east - previous_east;
        double dy = boundary_points[i].north - previous_north;
        double segment_distance = std::sqrt(dx * dx + dy * dy);

        accumulated_distance += segment_distance;

        bot_msg::msg::BoundaryPoint point;
        point.east = boundary_points[i].east;
        point.north = boundary_points[i].north;
        point.up = 0.0; // 默认高度为0
        point.distance = accumulated_distance;
        segment_points.push_back(point);

        previous_east = boundary_points[i].east;
        previous_north = boundary_points[i].north;

        if (accumulated_distance >= length) {
            break;
        }
    }
}

} // namespace map

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<map::MapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}