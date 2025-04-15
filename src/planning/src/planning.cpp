#include "planning/planning.h"
#include "bot_msg/srv/routing.hpp"
#include <chrono>

namespace planning {
PlanningNode::PlanningNode() : Node("planning_node") {
    InitParams();
    InitGlobalPath();
    // Initialize subscribers and publishers
    double timer_interval = 1.0;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(timer_interval * 1000.0)),
        std::bind(&PlanningNode::TimerCallback, this));
    sub_localization_info_ = this->create_subscription<
        bot_msg::msg::LocalizationInfo>(
        local_topic_name_, 10,
        std::bind(&PlanningNode::LocalizationInfoCallback,
                  this, std::placeholders::_1));
}
void PlanningNode::InitParams() {
    this->declare_parameter("local_topic_name", "/localization_info");
    this->declare_parameter("process_frq", 10.0);
    this->declare_parameter("service_name", "/service_name");
    this->declare_parameter("path_type", 0);

    local_topic_name_ = this->get_parameter("local_topic_name").as_string();
    service_name_ = this->get_parameter("service_name").as_string();
    process_frq_ = this->get_parameter("process_frq").as_double();
    path_type_ = this->get_parameter("path_type").as_int();

    RCLCPP_INFO(this->get_logger(),
                "local_topic_name: %s", local_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "service_name: %s", service_name_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "process_frq: %f", process_frq_);
    RCLCPP_INFO(this->get_logger(),
                "path_type: %d", path_type_);
}
void PlanningNode::InitGlobalPath() {
    auto client = this->create_client<bot_msg::srv::Routing>(service_name_);
    
    // 等待服务可用
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for service %s to appear...", service_name_.c_str());
    }
    
    // 创建请求
    auto request = std::make_shared<bot_msg::srv::Routing::Request>();
    request->path_type = path_type_;

    RCLCPP_INFO(this->get_logger(), "Sending request with path_type: %d", path_type_);

    // 发送异步请求并添加回调
    auto future_result = client->async_send_request(
        request,
        [this](rclcpp::Client<bot_msg::srv::Routing>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response) {
                    g_traj_ = response->path;
                    RCLCPP_INFO(this->get_logger(),
                              "Global path is received, size: %d",
                              response->path.points.size());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Received null response");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                         "Service call failed: %s", e.what());
            }
        });

    RCLCPP_INFO(this->get_logger(), "Waiting for global path...");

    // 等待响应（可选，设置超时时间）
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get response from service");
        return;
    }
    // 打印接收的前10个点
    for (int i = 0; i < 10; i++) {
        RCLCPP_INFO(this->get_logger(), "point %d: x: %f, y: %f, z: %f", i,
                    g_traj_.points[i].east,
                    g_traj_.points[i].north,
                    g_traj_.points[i].up);
    }
}


/**
 * @brief 定位信息回调函数
 */
void PlanningNode::LocalizationInfoCallback(
    const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    cur_local_ = *msg;
}
void PlanningNode::TimerCallback() {
    // 基于当前的当前定位信息, 找到当前位置在全局路径上的最近点
    double min_dist = 1000000.0;
    std::size_t min_idx = 0;
    for (std::size_t i = 0; i < g_traj_.points.size(); i++) {
        double dist = std::sqrt(std::pow(g_traj_.points[i].east - cur_local_.east, 2) +
                                std::pow(g_traj_.points[i].north - cur_local_.north, 2));
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    RCLCPP_INFO(this->get_logger(), "min_dist: %f, min_idx: %ld", min_dist, min_idx);
    // 根据min_idx, 找到min_idx的前5m和后20m
    const double preview_dist = 20.0;
    double cur_dis_cnt = 0.0;
    std::size_t start_idx = min_idx;
    while (cur_dis_cnt < 5.0 && start_idx >= 1) {
        cur_dis_cnt += std::sqrt(std::pow(g_traj_.points[start_idx].east - g_traj_.points[start_idx - 1].east, 2) +
                                  std::pow(g_traj_.points[start_idx].north - g_traj_.points[start_idx - 1].north, 2));
        start_idx--;
    }
    cur_dis_cnt = 0.0;
    std::size_t preview_idx = min_idx;
    while(cur_dis_cnt < preview_dist && preview_idx + 1 < g_traj_.points.size()) {
        cur_dis_cnt += std::sqrt(std::pow(g_traj_.points[preview_idx].east - g_traj_.points[preview_idx + 1].east, 2) +
                                  std::pow(g_traj_.points[preview_idx].north - g_traj_.points[preview_idx + 1].north, 2));
        preview_idx++;
    }
    // 根据start_idx和preview_idx, 找到start_idx和preview_idx之间的路径
    bot_msg::msg::ADCTrajectory pub_traj;
    for (std::size_t i = start_idx; i <= preview_idx; i++) {
        pub_traj.points.push_back(g_traj_.points[i]);
    }
    RCLCPP_INFO(this->get_logger(), "cur_dis_cnt: %f, start_idx: %ld, preview_idx: %ld", cur_dis_cnt, start_idx, preview_idx);
    // 发布路径
    pub_traj.header.stamp = this->now();
    pub_traj.header.frame_id = "map";
    this->pub_traj_->publish(pub_traj);
}
PlanningNode::~PlanningNode() {
    RCLCPP_INFO(this->get_logger(), "planning node stopped");
}
}  // namespace planning

// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning::PlanningNode>();
    RCLCPP_INFO(node->get_logger(), "planning node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
