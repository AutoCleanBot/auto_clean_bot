#include "planning/planning.h"
#include "bot_msg/srv/routing.hpp"
#include <chrono>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

namespace planning {
PlanningNode::PlanningNode() : Node("planning_node"), timer_cnt_(0) {
    InitParams();

    // 使用明确的 QoS 设置创建发布者
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    pub_traj_ = this->create_publisher<bot_msg::msg::ADCTrajectory>(traj_topic_name_, qos);
    // 立即发布一个空的轨迹消息，确保话题被注册
    bot_msg::msg::ADCTrajectory empty_traj;
    empty_traj.header.stamp = this->now();
    empty_traj.header.frame_id = "map";
    pub_traj_->publish(empty_traj);
    RCLCPP_INFO(this->get_logger(), "Published initial empty trajectory message");

    // 等待一小段时间确保消息被发布
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    InitGlobalPath();
    // Initialize subscribers
    int32_t timer_interval = static_cast<int32_t>(1.0 / process_frq_ * 1000);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval),
                                     std::bind(&PlanningNode::TimerCallback, this));
    sub_localization_info_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
        local_topic_name_, 10, std::bind(&PlanningNode::LocalizationInfoCallback, this, std::placeholders::_1));
    sub_perc_ = this->create_subscription<bot_msg::msg::Obstacles>(
        perc_topic_name_, 10, std::bind(&PlanningNode::ObstaclesCallback, this, std::placeholders::_1));

    planning_status_ = PlanningStatus::Planning;
}
void PlanningNode::InitParams() {
    this->declare_parameter("local_topic_name", "/localization_info");
    this->declare_parameter("process_frq", 10.0);
    this->declare_parameter("service_name", "/service_name");
    this->declare_parameter("traj_topic_name", "/planning/trajectory");
    this->declare_parameter("perc_topic_name", "/planning/perception");
    this->declare_parameter("path_type", 0);
    this->declare_parameter("preview_dist", 20.0);
    this->declare_parameter("start_dist", 5.0);
    this->declare_parameter("traj_pub_interval", 0.1);
    this->declare_parameter("planning_spd", 2.0);
    local_topic_name_ = this->get_parameter("local_topic_name").as_string();
    service_name_ = this->get_parameter("service_name").as_string();
    process_frq_ = this->get_parameter("process_frq").as_double();
    path_type_ = this->get_parameter("path_type").as_int();
    preview_dist_ = this->get_parameter("preview_dist").as_double();
    start_dist_ = this->get_parameter("start_dist").as_double();
    traj_pub_interval_ = this->get_parameter("traj_pub_interval").as_double();
    traj_topic_name_ = this->get_parameter("traj_topic_name").as_string();
    perc_topic_name_ = this->get_parameter("perc_topic_name").as_string();
    planning_spd_ = this->get_parameter("planning_spd").as_double();
    traj_pub_cnt_ = static_cast<int32_t>(traj_pub_interval_ * process_frq_);

    RCLCPP_INFO(this->get_logger(), "local_topic_name: %s", local_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "service_name: %s", service_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "traj_topic_name: %s", traj_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "process_frq: %f", process_frq_);
    RCLCPP_INFO(this->get_logger(), "path_type: %d", path_type_);
    RCLCPP_INFO(this->get_logger(), "preview_dist: %f", preview_dist_);
    RCLCPP_INFO(this->get_logger(), "start_dist: %f", start_dist_);
    RCLCPP_INFO(this->get_logger(), "traj_pub_interval: %f", traj_pub_interval_);
    RCLCPP_INFO(this->get_logger(), "planning_spd_: %f", planning_spd_);
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
    auto future_result =
        client->async_send_request(request, [this](rclcpp::Client<bot_msg::srv::Routing>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response) {
                    g_traj_ = response->path;
                    RCLCPP_INFO(this->get_logger(), "Global path is received, size: %d", response->path.points.size());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Received null response");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
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
    for (size_t i = 0; i < 10; i++) {
        RCLCPP_INFO(this->get_logger(), "point %d: x: %f, y: %f, z: %f", i, g_traj_.points[i].east,
                    g_traj_.points[i].north, g_traj_.points[i].up);
    }
}

// 障碍物回调函数
void PlanningNode::ObstaclesCallback(const bot_msg::msg::Obstacles::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "ObstaclesCallback, size: %d", msg->obstacles.size());
    obstacles_ = *msg;
}

/**
 * @brief 定位信息回调函数
 */
void PlanningNode::LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) { cur_local_ = *msg; }

bool PlanningNode::IsPathTail() {
    auto path_len = g_traj_.points.size();
    auto tail_east = g_traj_.points[path_len - 1].east;
    auto tail_north = g_traj_.points[path_len - 1].north;
    auto cur_east = cur_local_.east;
    auto cur_north = cur_local_.north;
    double dis = sqrt(pow(tail_east - cur_east, 2) + pow(tail_north - cur_north, 2));
    if (dis < 1 || closet_idx_ == path_len - 1) {
        return true;
    }
    return false;
}

void PlanningNode::FillPubTraj(bot_msg::msg::ADCTrajectory &pub_traj) {
    double min_dist = 1000000.0;

    // 检查是否有路径数据
    if (g_traj_.points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No trajectory points available");

        // 即使没有路径点，也发布一个空轨迹以保持话题活跃
        bot_msg::msg::ADCTrajectory empty_traj;
        empty_traj.header.stamp = this->now();
        empty_traj.header.frame_id = "map";
        pub_traj_->publish(empty_traj);
        return;
    }

    for (std::size_t i = 0; i < g_traj_.points.size(); i++) {
        double dist = std::sqrt(std::pow(g_traj_.points[i].east - cur_local_.east, 2) +
                                std::pow(g_traj_.points[i].north - cur_local_.north, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closet_idx_ = i;
        }
    }
    RCLCPP_INFO(this->get_logger(), "min_dist: %f, min_idx: %ld", min_dist, closet_idx_);
    // 根据min_idx, 找到min_idx的前5m和后20m

    double cur_dis_cnt = 0.0;
    std::size_t start_idx = closet_idx_;
    while (cur_dis_cnt < start_dist_ && start_idx >= 1) {
        cur_dis_cnt += std::sqrt(std::pow(g_traj_.points[start_idx].east - g_traj_.points[start_idx - 1].east, 2) +
                                 std::pow(g_traj_.points[start_idx].north - g_traj_.points[start_idx - 1].north, 2));
        start_idx--;
    }
    cur_dis_cnt = 0.0;
    std::size_t preview_idx = closet_idx_;
    while (cur_dis_cnt < preview_dist_ && preview_idx + 1 < g_traj_.points.size()) {
        cur_dis_cnt +=
            std::sqrt(std::pow(g_traj_.points[preview_idx].east - g_traj_.points[preview_idx + 1].east, 2) +
                      std::pow(g_traj_.points[preview_idx].north - g_traj_.points[preview_idx + 1].north, 2));
        preview_idx++;
    }
    // 根据start_idx和preview_idx, 找到start_idx和preview_idx之间的路径
    for (std::size_t i = start_idx; i <= preview_idx; i++) {
        pub_traj.points.push_back(g_traj_.points[i]);
    }
    RCLCPP_INFO(this->get_logger(), "cur_dis_cnt: %f, start_idx: %ld, preview_idx: %ld", cur_dis_cnt, start_idx,
                preview_idx);
}

// TODO 待验证,更新机制是有有问题
// TODO 路径终点的处理机制
void PlanningNode::TimerCallback() {
    // 基于当前的当前定位信息, 找到当前位置在全局路径上的最近点
    UpdateObstacleInfo();
    UpdatePlanningStatus();

    bot_msg::msg::ADCTrajectory pub_traj_path;
    FillPubTraj(pub_traj_path);
    // 填充速度
    for (std::size_t i = 0; i < pub_traj_path.points.size(); i++) {
        if (planning_status_ == PlanningStatus::Stop) {
            pub_traj_path.points[i].vel_speed = 0.0;
        } else {
            pub_traj_path.points[i].vel_speed = planning_spd_; // 速度2m/s
        }
    }

    // 发布路径
    pub_traj_path.header.stamp = this->now();
    pub_traj_path.header.frame_id = "map";
    this->pub_traj_->publish(pub_traj_path);
}

void PlanningNode::UpdateObstacleInfo() {
    obstacle_info_.fill(-1);
    for (std::size_t i = 0; i < obstacles_.obstacles.size(); i++) {
        auto &&obstacle = obstacles_.obstacles[i];
        // 障碍物在当前车辆正前方
        if (obstacle.position_y < 1.0 && obstacle.position_y > -1.0) {
            if (obstacle.position_x < 10.0 && obstacle.position_x > 0.0) {
                obstacle_info_[1] = i;
            }
        }
        // 障碍物在当前车辆左侧
        if (obstacle.position_y > 2.0 && obstacle.position_y < 10.0) {
            if (obstacle.position_x < 10.0 && obstacle.position_x > 0.0) {
                obstacle_info_[0] = i;
            }
        }
        // 障碍物在当前车辆右侧
        if (obstacle.position_y < -2.0 && obstacle.position_y > -10.0) {
            if (obstacle.position_x < 10.0 && obstacle.position_x > 0.0) {
                obstacle_info_[2] = i;
            }
        }
    }
}

void PlanningNode::UpdatePlanningStatus() {
    bool is_path_tail = IsPathTail();
    if (obstacle_info_[1] != -1 || is_path_tail) {
        planning_status_ = PlanningStatus::Stop;
    } else {
        planning_status_ = PlanningStatus::Planning;
    }
}
PlanningNode::~PlanningNode() { RCLCPP_INFO(this->get_logger(), "planning node stopped"); }
} // namespace planning

// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // 使用更明确的节点选项
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.allow_undeclared_parameters(true);
    options.use_intra_process_comms(false);

    auto node = std::make_shared<planning::PlanningNode>();
    RCLCPP_INFO(node->get_logger(), "planning node started");

    // 使用多线程执行器以提高节点的响应性
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Spinning planning node");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
