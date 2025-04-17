#include "routing/routing.h"
#include "bot_msg/msg/adc_trajectory.hpp"
#include "bot_msg/msg/trajectory_point.hpp"
#include <fstream>
namespace routing {

RoutingNode::RoutingNode() : Node("routing") {
    // Initialize subscribers and publishers
    InitParams();
    m_service =
        this->create_service<bot_msg::srv::Routing>(
            "/routing_service",
            std::bind(&RoutingNode::HandleRoutingRequest, this,
                      std::placeholders::_1,
                      std::placeholders::_2));
    
    // 添加服务创建成功的日志
    RCLCPP_INFO(this->get_logger(), "Routing service '%s' is ready", "/routing_service");
}
void RoutingNode::HandleRoutingRequest(
    const bot_msg::srv::Routing::Request::SharedPtr request,
    bot_msg::srv::Routing::Response::SharedPtr response) {  
    
    RCLCPP_INFO(this->get_logger(), "Routing request received");

    auto csv_path = m_map_names[request->path_type];
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
        std::getline(ss, value, ','); point.longtitude = std::stod(value);
        std::getline(ss, value, ','); point.latitude = std::stod(value);
        std::getline(ss, value, ','); point.altitude = std::stod(value);
        std::getline(ss, value, ','); point.north = std::stod(value);
        std::getline(ss, value, ','); point.east = std::stod(value);
        std::getline(ss, value, ','); point.up = std::stod(value);
        std::getline(ss, value, ','); point.yaw = std::stof(value);
        std::getline(ss, value, ','); point.pitch = std::stof(value);
        std::getline(ss, value, ','); point.roll = std::stof(value);
        std::getline(ss, value, ','); point.vel_speed = std::stof(value);
        std::getline(ss, value, ','); point.north_speed = std::stof(value);
        std::getline(ss, value, ','); point.east_speed = std::stof(value);
        std::getline(ss, value, ','); tmp = std::stof(value);
        std::getline(ss, value, ','); point.acceleration_x = std::stof(value);
        std::getline(ss, value, ','); point.acceleration_y = std::stof(value);
        std::getline(ss, value, ','); point.acceleration_z = std::stof(value);
        std::getline(ss, value, ','); tmp = std::stof(value);
        std::getline(ss, value, ','); tmp = std::stof(value);
        std::getline(ss, value, ','); tmp = std::stof(value);
        std::getline(ss, value, ','); tmp = static_cast<uint8_t>(std::stoi(value));

        trajectory.points.push_back(point);
    }
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->now();
    response->path = trajectory;
    RCLCPP_INFO(this->get_logger(), "Routing response sent");
    file.close();
}
void RoutingNode::InitParams() {
    m_map_names.resize(10);
    // Initialize parameters
    this->declare_parameter("1", "/");
    this->declare_parameter("2", "/");
    this->declare_parameter("3", "/");
    this->declare_parameter("4", "/");
    this->declare_parameter("5", "/");
    this->declare_parameter("6", "/");
    this->declare_parameter("7", "/");
    this->declare_parameter("8", "/");
    this->declare_parameter("9", "/");

    RCLCPP_INFO(this->get_logger(), "Init parameters");
    // Get parameters
    for(size_t i = 1; i < 10; i++){
        std::string csv_path = this->get_parameter(std::to_string(i)).get_value<std::string>();
        m_map_names[i] = csv_path;
    }
}
}  // namespace routing


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<routing::RoutingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


