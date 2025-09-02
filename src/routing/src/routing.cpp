#include "routing/routing.h"

#include <pwd.h>
#include <unistd.h>

#include <fstream>
#include <string>

#include "bot_msg/msg/adc_trajectory.hpp"
#include "bot_msg/msg/trajectory_point.hpp"

namespace routing {

// 添加一个辅助函数来展开波浪号
std::string expandTilde(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    // 获取当前用户的主目录
    const char* home = getenv("HOME");
    if (home == nullptr) {
        struct passwd* pwd = getpwuid(getuid());
        if (pwd) {
            home = pwd->pw_dir;
        }
    }

    if (home == nullptr) {
        return path;  // 如果无法获取主目录，返回原始路径
    }

    // 替换波浪号
    if (path.length() == 1) {  // 仅有 "~"
        return home;
    }
    if (path[1] == '/') {  // "~/xxx"
        return std::string(home) + path.substr(1);
    }
    return path;  // "~xxx" 其他情况返回原始路径
}

RoutingNode::RoutingNode() : Node("routing") {
    // Initialize subscribers and publishers
    InitParams();
    m_service = this->create_service<bot_msg::srv::Routing>(
        "/routing_service", std::bind(&RoutingNode::HandleRoutingRequest, this,
                                      std::placeholders::_1, std::placeholders::_2));

    // 添加服务创建成功的日志
    RCLCPP_INFO(this->get_logger(), "Routing service '%s' is ready", "/routing_service");
}
void RoutingNode::HandleRoutingRequest(const bot_msg::srv::Routing::Request::SharedPtr request,
                                       bot_msg::srv::Routing::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Routing request received");

    const std::string csv_path_file_prefix = "~/auto_clean_bot/path/local_record_";
    const std::string csv_path_file_suffix = ".csv";
    std::string csv_path =
        csv_path_file_prefix + std::to_string(request->path_type) + csv_path_file_suffix;
    csv_path = expandTilde(csv_path);
    std::ifstream file(csv_path);

    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", csv_path.c_str());
        return;
    }

    std::string line;
    bot_msg::msg::ADCTrajectory trajectory;
    bot_msg::msg::TrajectoryPoint point;

    // 读取表头
    std::getline(file, line);
    // 可以在此处解析表头以确保字段顺序正确

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        double tmp;
        std::getline(ss, value, ',');
        point.longtitude = std::stod(value);
        std::getline(ss, value, ',');
        point.latitude = std::stod(value);
        std::getline(ss, value, ',');
        point.altitude = std::stod(value);
        std::getline(ss, value, ',');
        point.north = std::stod(value);
        std::getline(ss, value, ',');
        point.east = std::stod(value);
        std::getline(ss, value, ',');
        point.up = std::stod(value);
        std::getline(ss, value, ',');
        point.yaw = std::stof(value);
        std::getline(ss, value, ',');
        point.pitch = std::stof(value);
        std::getline(ss, value, ',');
        point.roll = std::stof(value);
        std::getline(ss, value, ',');
        point.vel_speed = std::stof(value);
        std::getline(ss, value, ',');
        point.north_speed = std::stof(value);
        std::getline(ss, value, ',');
        point.east_speed = std::stof(value);
        std::getline(ss, value, ',');
        tmp = std::stof(value);
        std::getline(ss, value, ',');
        point.acceleration_x = std::stof(value);
        std::getline(ss, value, ',');
        point.acceleration_y = std::stof(value);
        std::getline(ss, value, ',');
        point.acceleration_z = std::stof(value);
        std::getline(ss, value, ',');
        tmp = std::stof(value);
        std::getline(ss, value, ',');
        tmp = std::stof(value);
        std::getline(ss, value, ',');
        tmp = std::stof(value);
        std::getline(ss, value, ',');
        tmp = static_cast<uint8_t>(std::stoi(value));

        trajectory.points.push_back(point);
    }
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->now();
    response->path = trajectory;
    RCLCPP_INFO(this->get_logger(), "Routing response sent path type %d, points: %d",
                request->path_type, trajectory.points.size());
    file.close();
}
void RoutingNode::InitParams() {}
}  // namespace routing

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<routing::RoutingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
