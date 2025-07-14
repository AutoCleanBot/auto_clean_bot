#include "csv_map/map_node.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pwd.h>
#include <sstream>
#include <unistd.h>

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

    map_files_dir_ = this->get_parameter("map_files_dir").as_string();
    std::string left_boundary_file = this->get_parameter("left_boundary_file").as_string();
    std::string right_boundary_file = this->get_parameter("right_boundary_file").as_string();
    left_boundary_name_ = this->get_parameter("left_boundary_name").as_string();
    right_boundary_name_ = this->get_parameter("right_boundary_name").as_string();
    boundary_length_ = this->get_parameter("boundary_length").as_double();
    publish_frequency_ = this->get_parameter("publish_frequency").as_double();

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

    RCLCPP_INFO(this->get_logger(), "Map node initialized");
}

MapNode::~MapNode() { RCLCPP_INFO(this->get_logger(), "Map node shutting down"); }

void MapNode::localizationCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    current_east_ = msg->east;
    current_north_ = msg->north;
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
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < boundary_points.size(); ++i) {
        double dx = boundary_points[i].east - east;
        double dy = boundary_points[i].north - north;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }

    return closest_idx;
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