#include "planning/planning.h"
#include "bot_msg/srv/routing.hpp"
#include <chrono>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

double NormalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

/*
 * *TODO 边界点的接受和处理
 */

namespace planning {
PlanningNode::PlanningNode() : Node("planning_node"), timer_cnt_(0) {
    InitParams();

    // 使用明确的 QoS 设置创建发布者
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    pub_traj_ = this->create_publisher<bot_msg::msg::ADCTrajectory>(traj_topic_name_, qos);
    pub_visualization_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic_name_, qos);
    // 立即发布一个空的轨迹消息，确保话题被注册
    bot_msg::msg::ADCTrajectory empty_traj;
    empty_traj.header.stamp = this->now();
    empty_traj.header.frame_id = "map";
    pub_traj_->publish(empty_traj);
    RCLCPP_INFO(this->get_logger(), "Published initial empty trajectory message");

    // 等待一小段时间确保消息被发布
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 如果不是测试模式，则初始化全局路径
    if (!test_mode_) {
        InitGlobalPath();
    } else {
        RCLCPP_INFO(this->get_logger(), "Test mode enabled, skipping global path initialization");
        // 在测试模式下创建一个简单的直线路径
        g_traj_.points.clear();
        for (int i = 0; i < 20; ++i) {
            bot_msg::msg::TrajectoryPoint point;
            point.east = static_cast<double>(i);
            point.north = 0.0;
            point.up = 0.0;
            point.vel_speed = 3.0;
            g_traj_.points.push_back(point);
        }
        RCLCPP_INFO(this->get_logger(), "Created test trajectory with %zu points", g_traj_.points.size());
    }
    // Initialize subscribers
    int32_t timer_interval = static_cast<int32_t>(1.0 / process_frq_ * 1000);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval),
                                     std::bind(&PlanningNode::TimerCallback, this));
    sub_localization_info_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
        local_topic_name_, 10, std::bind(&PlanningNode::LocalizationInfoCallback, this, std::placeholders::_1));
    sub_perc_ = this->create_subscription<bot_msg::msg::Obstacles>(
        perc_topic_name_, 10, std::bind(&PlanningNode::ObstaclesCallback, this, std::placeholders::_1));

    // 添加边界订阅
    sub_left_boundary_ = this->create_subscription<bot_msg::msg::Boundary>(
        left_boundary_topic_name_, 10, std::bind(&PlanningNode::LeftBoundaryCallback, this, std::placeholders::_1));
    sub_right_boundary_ = this->create_subscription<bot_msg::msg::Boundary>(
        right_boundary_topic_name_, 10, std::bind(&PlanningNode::RightBoundaryCallback, this, std::placeholders::_1));

    // 添加占用栅格地图订阅
    sub_occupancy_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_grid_topic_name_, 10, std::bind(&PlanningNode::OccupancyGridCallback, this, std::placeholders::_1));
    sub_remote_control_ = this->create_subscription<std_msgs::msg::Int32>(
        remote_control_topic_name_, 10, std::bind(&PlanningNode::RemoteControlCallback, this, std::placeholders::_1));
    if (!remote_control_enabled_) {
        planning_status_ = PlanningStatus::Planning;
        key_stop_ = false;
    } else {
        planning_status_ = PlanningStatus::Stop;
        key_stop_ = true;
    }
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
    this->declare_parameter("path_end_dist", 2.0);
    this->declare_parameter("reverse_moving", false);
    this->declare_parameter("left_boundary_topic_name", "/map/left_boundary");
    this->declare_parameter("right_boundary_topic_name", "/map/right_boundary");
    this->declare_parameter("occupancy_grid_topic_name", "/occupancy_grid");
    this->declare_parameter("use_occupancy_grid", true);
    this->declare_parameter("test_mode", false);
    this->declare_parameter("visualization_topic_name", "/planning/visualization");
    this->declare_parameter("remote_control_topic_name", "/remote_control/cmd");
    this->declare_parameter("remote_control_enabled", false);

    // 占用栅格地图障碍物检测参数
    this->declare_parameter("min_obstacle_distance", 10.0);
    this->declare_parameter("front_obstacle_width", 1.0);
    this->declare_parameter("side_obstacle_width", 2.0);
    this->declare_parameter("occupied_threshold", 50);

    // 方向稳定性参数
    this->declare_parameter("direction_stability_weight", 2.0);
    this->declare_parameter("max_index_jump", 50.0);

    // 性能统计参数
    this->declare_parameter("enable_timing_logs", true);
    this->declare_parameter("timing_log_interval", 10);
    this->declare_parameter("enable_detailed_timing", false);
    this->declare_parameter("enable_zero_copy", true);

    // 栅格地图优化参数
    this->declare_parameter("max_obstacles_to_check", 50);
    this->declare_parameter("grid_sampling_resolution", 0.1);
    this->declare_parameter("skip_boundary_check", true);
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
    reverse_moving_ = this->get_parameter("reverse_moving").as_bool();
    path_end_dist_ = this->get_parameter("path_end_dist").as_double();
    left_boundary_topic_name_ = this->get_parameter("left_boundary_topic_name").as_string();
    right_boundary_topic_name_ = this->get_parameter("right_boundary_topic_name").as_string();
    occupancy_grid_topic_name_ = this->get_parameter("occupancy_grid_topic_name").as_string();
    remote_control_topic_name_ = this->get_parameter("remote_control_topic_name").as_string();
    use_occupancy_grid_ = this->get_parameter("use_occupancy_grid").as_bool();
    test_mode_ = this->get_parameter("test_mode").as_bool();
    visualization_topic_name_ = this->get_parameter("visualization_topic_name").as_string();
    remote_control_enabled_ = this->get_parameter("remote_control_enabled").as_bool();

    // 获取占用栅格地图障碍物检测参数
    min_obstacle_distance_ = this->get_parameter("min_obstacle_distance").as_double();
    front_obstacle_width_ = this->get_parameter("front_obstacle_width").as_double();
    side_obstacle_width_ = this->get_parameter("side_obstacle_width").as_double();
    occupied_threshold_ = this->get_parameter("occupied_threshold").as_int();

    // 获取方向稳定性参数
    direction_stability_weight_ = this->get_parameter("direction_stability_weight").as_double();
    max_index_jump_ = this->get_parameter("max_index_jump").as_double();

    // 获取性能统计参数
    enable_timing_logs_ = this->get_parameter("enable_timing_logs").as_bool();
    timing_log_interval_ = this->get_parameter("timing_log_interval").as_int();
    enable_detailed_timing_ = this->get_parameter("enable_detailed_timing").as_bool();
    enable_zero_copy_ = this->get_parameter("enable_zero_copy").as_bool();

    // 获取栅格地图优化参数
    max_obstacles_to_check_ = this->get_parameter("max_obstacles_to_check").as_int();
    grid_sampling_resolution_ = this->get_parameter("grid_sampling_resolution").as_double();
    skip_boundary_check_ = this->get_parameter("skip_boundary_check").as_bool();

    // 初始化性能统计变量
    frame_count_ = 0;
    total_processing_time_ = 0.0;
    total_obstacle_detection_time_ = 0.0;
    total_trajectory_planning_time_ = 0.0;
    total_visualization_time_ = 0.0;
    total_occupancy_grid_time_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "local_topic_name: %s", local_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "service_name: %s", service_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "traj_topic_name: %s", traj_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "perc_topic_name: %s", perc_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "remote_control_topic_name: %s", remote_control_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "remote_control_enabled: %d", remote_control_enabled_);
    RCLCPP_INFO(this->get_logger(), "process_frq: %f", process_frq_);
    RCLCPP_INFO(this->get_logger(), "path_type: %d", path_type_);
    RCLCPP_INFO(this->get_logger(), "preview_dist: %f", preview_dist_);
    RCLCPP_INFO(this->get_logger(), "start_dist: %f", start_dist_);
    RCLCPP_INFO(this->get_logger(), "traj_pub_interval: %f", traj_pub_interval_);
    RCLCPP_INFO(this->get_logger(), "planning_spd_: %f", planning_spd_);
    RCLCPP_INFO(this->get_logger(), "reverse_moving: %d", reverse_moving_);
    RCLCPP_INFO(this->get_logger(), "path_end_dist: %f", path_end_dist_);
    RCLCPP_INFO(this->get_logger(), "left_boundary_topic_name: %s", left_boundary_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "right_boundary_topic_name: %s", right_boundary_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "direction_stability_weight: %f", direction_stability_weight_);
    RCLCPP_INFO(this->get_logger(), "max_index_jump: %f", max_index_jump_);

    // 性能统计配置信息
    RCLCPP_INFO(this->get_logger(), "Timing logs enabled: %s", enable_timing_logs_ ? "true" : "false");
    if (enable_timing_logs_) {
        RCLCPP_INFO(this->get_logger(), "Timing log interval: every %d frames", timing_log_interval_);
        RCLCPP_INFO(this->get_logger(), "Detailed timing enabled: %s", enable_detailed_timing_ ? "true" : "false");
    }
    RCLCPP_INFO(this->get_logger(), "Zero-copy optimization enabled: %s", enable_zero_copy_ ? "true" : "false");

    // 栅格地图优化配置信息
    RCLCPP_INFO(this->get_logger(), "=== Grid Map Optimization Settings ===");
    RCLCPP_INFO(this->get_logger(), "Max obstacles to check: %d", max_obstacles_to_check_);
    RCLCPP_INFO(this->get_logger(), "Grid sampling resolution: %.2f m", grid_sampling_resolution_);
    RCLCPP_INFO(this->get_logger(), "Skip boundary check: %s", skip_boundary_check_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "======================================");
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

void PlanningNode::RemoteControlCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    static int pre_key_value = 0;
    int key_value = msg->data;
    if (key_value != pre_key_value && key_value != 0) {
        remote_control_cmd_ = key_value;
    }
    pre_key_value = key_value;
    // RCLCPP_INFO(this->get_logger(), "RemoteControlCallback, cmd: %d", remote_control_cmd_);
    if (remote_control_cmd_ == 2) {
        key_stop_ = true;
    }
}

// 占用栅格地图回调函数
void PlanningNode::OccupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "OccupancyGridCallback, width: %d, height: %d, resolution: %f", msg->info.width,
                 msg->info.height, msg->info.resolution);
    occupancy_grid_ = *msg;
}

/**
 * @brief 定位信息回调函数
 */
void PlanningNode::LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) { cur_local_ = *msg; }

/**
 *
 * @brief 左边界回调函数
 */
void PlanningNode::LeftBoundaryCallback(const bot_msg::msg::Boundary::SharedPtr msg) { left_boundary_ = *msg; }

/**
 *
 * @brief 右边界回调函数
 */
void PlanningNode::RightBoundaryCallback(const bot_msg::msg::Boundary::SharedPtr msg) { right_boundary_ = *msg; }

bool PlanningNode::IsPathTail() {
    double path_end_dist = planning_spd_ * 4.0;
    auto path_len = g_traj_.points.size();
    auto tail_east = g_traj_.points[path_len - 1].east;
    auto tail_north = g_traj_.points[path_len - 1].north;
    auto cur_east = cur_local_.east;
    auto cur_north = cur_local_.north;
    double dis = sqrt(pow(tail_east - cur_east, 2) + pow(tail_north - cur_north, 2));
    
    // 简化判断：只基于距离或接近路径末尾的索引
    if (dis < path_end_dist || closet_idx_ > path_len - 20) {
        return true;
    }
    return false;
}

void PlanningNode::FillPubTraj(bot_msg::msg::ADCTrajectory &pub_traj) {
    if (g_traj_.points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No trajectory points available");

        // 检查是否有路径数据
        // 即使没有路径点，也发布一个空轨迹以保持话题活跃
        bot_msg::msg::ADCTrajectory empty_traj;
        empty_traj.header.stamp = this->now();
        empty_traj.header.frame_id = "map";
        pub_traj_->publish(empty_traj);
        return;
    }

    // 简化的最近点搜索：只考虑距离和连续性
    static std::size_t last_closest_idx = 0;
    static bool first_run = true;

    size_t closest_idx = 0;
    double min_cost = std::numeric_limits<double>::max();

    // 优化搜索策略：局部搜索 + 全局备份
    size_t search_start = 0;
    size_t search_end = g_traj_.points.size();

    if (!first_run && last_closest_idx < g_traj_.points.size()) {
        // 局部搜索范围：以上次最近点为中心的邻域
        size_t local_search_radius = static_cast<size_t>(max_index_jump_ * 1.5);

        search_start = (last_closest_idx > local_search_radius) ? (last_closest_idx - local_search_radius) : 0;
        search_end = std::min(last_closest_idx + local_search_radius + 1, g_traj_.points.size());

        RCLCPP_DEBUG(this->get_logger(), "局部搜索范围: [%zu, %zu) 围绕上次索引: %zu",
                     search_start, search_end, last_closest_idx);
    }

    // 局部搜索
    for (size_t i = search_start; i < search_end; i++) {
        double distance = std::sqrt(std::pow(g_traj_.points[i].east - cur_local_.east, 2) +
                                    std::pow(g_traj_.points[i].north - cur_local_.north, 2));

        // 计算连续性成本（避免大幅跳跃）
        double continuity_cost = 0.0;
        if (!first_run) {
            double index_diff = std::abs(static_cast<double>(i) - static_cast<double>(last_closest_idx));
            if (index_diff > max_index_jump_) {
                continuity_cost = direction_stability_weight_ * (index_diff - max_index_jump_);
            }
        }

        // 总成本 = 距离 + 连续性成本
        double total_cost = distance + continuity_cost;

        if (total_cost < min_cost) {
            min_cost = total_cost;
            closest_idx = i;
        }
    }

    // 如果局部搜索没有找到足够好的结果，进行全局搜索
    bool need_global_search = false;
    if (!first_run) {
        double distance_to_found = std::sqrt(std::pow(g_traj_.points[closest_idx].east - cur_local_.east, 2) +
                                             std::pow(g_traj_.points[closest_idx].north - cur_local_.north, 2));

        // 如果找到的点距离太远，可能需要全局搜索
        if (distance_to_found > 15.0) { // 15米阈值
            need_global_search = true;
            RCLCPP_WARN(this->get_logger(),
                        "当前位置: (%.2f, %.2f), 局部搜索结果距离过远 (%.2fm), 执行全局搜索",
                        cur_local_.east, cur_local_.north, distance_to_found);
        }
    }

    // 全局搜索（首次运行或局部搜索失败时）
    if (first_run || need_global_search) {
        double global_min_cost = min_cost;
        size_t global_closest_idx = closest_idx;

        // 跳跃式搜索：每隔几个点采样，然后在最佳区域细化
        size_t step_size = std::max(1UL, g_traj_.points.size() / 200); // 最多检查200个采样点

        for (size_t i = 0; i < g_traj_.points.size(); i += step_size) {
            double distance = std::sqrt(std::pow(g_traj_.points[i].east - cur_local_.east, 2) +
                                        std::pow(g_traj_.points[i].north - cur_local_.north, 2));

            // 对于全局搜索，连续性成本权重较小
            double continuity_cost = 0.0;
            if (!first_run) {
                double index_diff = std::abs(static_cast<double>(i) - static_cast<double>(last_closest_idx));
                if (index_diff > max_index_jump_) {
                    continuity_cost = direction_stability_weight_ * 0.5 * (index_diff - max_index_jump_);
                }
            }

            double total_cost = distance + continuity_cost;

            if (total_cost < global_min_cost) {
                global_min_cost = total_cost;
                global_closest_idx = i;
            }
        }

        // 在最佳采样点周围进行细化搜索
        if (global_closest_idx != closest_idx) {
            size_t refine_start = (global_closest_idx > step_size) ? (global_closest_idx - step_size) : 0;
            size_t refine_end = std::min(global_closest_idx + step_size + 1, g_traj_.points.size());

            for (size_t i = refine_start; i < refine_end; i++) {
                double distance = std::sqrt(std::pow(g_traj_.points[i].east - cur_local_.east, 2) +
                                            std::pow(g_traj_.points[i].north - cur_local_.north, 2));

                double continuity_cost = 0.0;
                if (!first_run) {
                    double index_diff = std::abs(static_cast<double>(i) - static_cast<double>(last_closest_idx));
                    if (index_diff > max_index_jump_) {
                        continuity_cost = direction_stability_weight_ * 0.5 * (index_diff - max_index_jump_);
                    }
                }

                double total_cost = distance + continuity_cost;

                if (total_cost < global_min_cost) {
                    global_min_cost = total_cost;
                    global_closest_idx = i;
                }
            }

            min_cost = global_min_cost;
            closest_idx = global_closest_idx;
        }
    }

    // 添加调试日志监控连续性
    if (!first_run) {
        double index_change = static_cast<double>(closest_idx) - static_cast<double>(last_closest_idx);
        if (std::abs(index_change) > max_index_jump_) {
            RCLCPP_WARN(this->get_logger(),
                        "检测到轨迹索引大幅跳跃: 从 %zu 到 %zu (变化: %.1f), "
                        "车辆位置: (%.2f, %.2f), 距离: %.2fm",
                        last_closest_idx, closest_idx, index_change, cur_local_.east, cur_local_.north,
                        std::sqrt(std::pow(g_traj_.points[closest_idx].east - cur_local_.east, 2) +
                                  std::pow(g_traj_.points[closest_idx].north - cur_local_.north, 2)));
        }
    }

    // 更新历史信息
    last_closest_idx = closest_idx;
    first_run = false;
    closet_idx_ = closest_idx;

    RCLCPP_DEBUG(this->get_logger(), "选择轨迹点 %zu, 距离: %.2fm, 总成本: %.2f", 
                 closest_idx, 
                 std::sqrt(std::pow(g_traj_.points[closest_idx].east - cur_local_.east, 2) +
                           std::pow(g_traj_.points[closest_idx].north - cur_local_.north, 2)), 
                 min_cost);

    // 其余代码保持不变
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
    if (timer_cnt_ % 10 == 0)
        RCLCPP_INFO(this->get_logger(),
                    "cur_east: %f, cur_north: %f,closet_idx: %ld,cur_dis_cnt: %f, start_idx: %ld, preview_idx: %ld",
                    cur_local_.east, cur_local_.north, closet_idx_, cur_dis_cnt, start_idx, preview_idx);
}

// TODO 待验证,更新机制是有有问题
// TODO 路径终点的处理机制
void PlanningNode::TimerCallback() {
    // 开始计时
    double start_time = getCurrentTimeMs();

    // 障碍物检测
    double obstacle_start_time = getCurrentTimeMs();
    if (use_occupancy_grid_) {
        if (enable_zero_copy_) {
            updateObstacleInfoFromOccupancyGridZeroCopy();
        } else {
            UpdateObstacleInfoFromOccupancyGrid();
        }
    } else {
        UpdateObstacleInfo();
    }
    double obstacle_end_time = getCurrentTimeMs();
    total_obstacle_detection_time_ += (obstacle_end_time - obstacle_start_time);

    UpdatePlanningStatus();

    // 轨迹规划
    double trajectory_start_time = getCurrentTimeMs();
    bot_msg::msg::ADCTrajectory pub_traj_path;
    FillPubTraj(pub_traj_path);
    // 填充速度
    for (std::size_t i = 0; i < pub_traj_path.points.size(); i++) {
        if (planning_status_ == PlanningStatus::Stop) {
            pub_traj_path.points[i].vel_speed = 0.0;
        } else {
            pub_traj_path.points[i].vel_speed = planning_spd_;
        }
    }

    // 发布路径
    pub_traj_path.header.stamp = this->now();
    pub_traj_path.header.frame_id = "map";
    pub_traj_path.direction = reverse_moving_ ? 1 : 0;
    this->pub_traj_->publish(pub_traj_path);
    double trajectory_end_time = getCurrentTimeMs();
    total_trajectory_planning_time_ += (trajectory_end_time - trajectory_start_time);

    // 可视化
    double visualization_start_time = getCurrentTimeMs();
    PublishVisualization(pub_traj_path);
    double visualization_end_time = getCurrentTimeMs();
    total_visualization_time_ += (visualization_end_time - visualization_start_time);

    // 结束计时并计算总耗时
    double end_time = getCurrentTimeMs();
    double process_time = end_time - start_time;

    // 更新统计信息
    frame_count_++;
    total_processing_time_ += process_time;

    // 根据配置输出耗时日志
    if (enable_timing_logs_ && (frame_count_ % timing_log_interval_ == 0)) {
        if (enable_detailed_timing_) {
            logTimingStatistics();
        } else {
            double avg_time = total_processing_time_ / frame_count_;
            RCLCPP_INFO(this->get_logger(), "Frame %d: Current=%.2fms, Average=%.2fms", frame_count_, process_time,
                        avg_time);
        }
    }

    // 防止计数器溢出，每10000帧重置一次统计
    if (frame_count_ >= 10000) {
        resetTimingStatistics();
    }

    ++timer_cnt_;
    if (timer_cnt_ > 99) {
        timer_cnt_ = 1;
    }
}

/**
 * @brief 更新障碍物信息
 */
void PlanningNode::UpdateObstacleInfo() {
    obstacle_info_.fill(-1);

    const double MIN_OBSTACLE_DISTANCE = 10.0; // 最小障碍物距离阈值
    const double FRONT_OBSTACLE_WIDTH = 1.0;   // 前方障碍物区域宽度(±1.0米)
    const double SIDE_OBSTACLE_WIDTH = 2.0;    // 侧方障碍物区域距离(±2.0米外)
    const double SAFETY_MARGIN = 0.5;          // 安全裕度，比实际障碍物大0.5米
    const double DEFAULT_OBSTACLE_SIZE = 0.5;  // 默认障碍物尺寸

    // 遍历所有障碍物
    for (std::size_t i = 0; i < obstacles_.obstacles.size(); i++) {
        auto &&obstacle = obstacles_.obstacles[i];

        // 检查障碍物是否在边界内
        bool is_in_boundary = IsObstacleInBoundary(obstacle);

        // 如果障碍物不在边界内，跳过
        if (!is_in_boundary) {
            continue;
        }

        // 计算障碍物距离
        double obstacle_distance = std::sqrt(std::pow(obstacle.position_x, 2) + std::pow(obstacle.position_y, 2));

        // 使用障碍物尺寸或默认值
        double obstacle_width = (obstacle.width > 0.01) ? obstacle.width : DEFAULT_OBSTACLE_SIZE;
        double obstacle_length = (obstacle.length > 0.01) ? obstacle.length : DEFAULT_OBSTACLE_SIZE;

        // 计算最小安全距离：考虑障碍物尺寸和安全裕度
        double obstacle_radius = std::sqrt(std::pow(obstacle_width / 2.0, 2) + std::pow(obstacle_length / 2.0, 2));
        double safe_distance = obstacle_distance - obstacle_radius - SAFETY_MARGIN;

        // 距离超过阈值，跳过
        if (safe_distance > MIN_OBSTACLE_DISTANCE) {
            continue;
        }

        // 根据行驶方向检测障碍物
        if (reverse_moving_) {
            // 倒车模式：检测车辆后方障碍物
            if (obstacle.position_y < FRONT_OBSTACLE_WIDTH && obstacle.position_y > -FRONT_OBSTACLE_WIDTH) {
                if (obstacle.position_x > -MIN_OBSTACLE_DISTANCE && obstacle.position_x < 0.0) {
                    obstacle_info_[1] = i;
                    RCLCPP_INFO(this->get_logger(),
                                "倒车模式-后方发现障碍物，距离：%.2f 米，安全距离：%.2f 米，尺寸：%.2f x %.2f",
                                std::abs(obstacle.position_x), safe_distance, obstacle_length, obstacle_width);
                }
            }
        } else {
            // 前进模式：检测车辆前方障碍物
            if (obstacle.position_y < FRONT_OBSTACLE_WIDTH && obstacle.position_y > -FRONT_OBSTACLE_WIDTH) {
                if (obstacle.position_x < MIN_OBSTACLE_DISTANCE && obstacle.position_x > 0.0) {
                    obstacle_info_[1] = i;
                    RCLCPP_INFO(this->get_logger(),
                                "前进模式-前方发现障碍物，距离：%.2f 米，安全距离：%.2f 米，尺寸：%.2f x %.2f",
                                obstacle.position_x, safe_distance, obstacle_length, obstacle_width);
                }
            }
        }
        // 障碍物在当前车辆左侧
        if (obstacle.position_y > SIDE_OBSTACLE_WIDTH && obstacle.position_y < MIN_OBSTACLE_DISTANCE) {
            if (obstacle.position_x < MIN_OBSTACLE_DISTANCE && obstacle.position_x > 0.0) {
                obstacle_info_[0] = i;
                RCLCPP_INFO(this->get_logger(), "左侧发现障碍物，距离：%.2f 米，安全距离：%.2f 米，尺寸：%.2f x %.2f",
                            obstacle_distance, safe_distance, obstacle_length, obstacle_width);
            }
        }
        // 障碍物在当前车辆右侧
        if (obstacle.position_y < -SIDE_OBSTACLE_WIDTH && obstacle.position_y > -MIN_OBSTACLE_DISTANCE) {
            if (obstacle.position_x < MIN_OBSTACLE_DISTANCE && obstacle.position_x > 0.0) {
                obstacle_info_[2] = i;
                RCLCPP_INFO(this->get_logger(), "右侧发现障碍物，距离：%.2f 米，安全距离：%.2f 米，尺寸：%.2f x %.2f",
                            obstacle_distance, safe_distance, obstacle_length, obstacle_width);
            }
        }
    }
}

/**
 * @brief 判断障碍物是否在边界内
 * @param obstacle 障碍物信息
 * @return 是否在边界内
 */
bool PlanningNode::IsObstacleInBoundary(const bot_msg::msg::ObstacleInfo &obstacle) {
    // 检查左右边界是否有数据
    if (left_boundary_.points.empty() || right_boundary_.points.empty()) {
        return false;
    }

    // 直接使用障碍物坐标（假设已经在全局坐标系中）
    double global_east = obstacle.position_x;
    double global_north = obstacle.position_y;

    // 考虑障碍物的大小，创建多个检测点
    // 提供默认尺寸，防止障碍物消息中没有尺寸信息
    const double DEFAULT_OBSTACLE_WIDTH = 0.5;  // 默认宽度0.5米
    const double DEFAULT_OBSTACLE_LENGTH = 0.5; // 默认长度0.5米

    // 使用障碍物自身尺寸或默认尺寸
    double obstacle_width = (obstacle.width > 0.01) ? obstacle.width : DEFAULT_OBSTACLE_WIDTH;
    double obstacle_length = (obstacle.length > 0.01) ? obstacle.length : DEFAULT_OBSTACLE_LENGTH;

    const double obstacle_half_width = obstacle_width / 2.0;
    const double obstacle_half_length = obstacle_length / 2.0;

    RCLCPP_DEBUG(this->get_logger(), "障碍物尺寸: %.2f x %.2f 米", obstacle_length, obstacle_width);

    // 创建障碍物四角的检测点
    std::vector<std::pair<double, double>> check_points = {
        {global_east, global_north},                                              // 中心点
        {global_east + obstacle_half_length, global_north + obstacle_half_width}, // 右上角
        {global_east + obstacle_half_length, global_north - obstacle_half_width}, // 右下角
        {global_east - obstacle_half_length, global_north + obstacle_half_width}, // 左上角
        {global_east - obstacle_half_length, global_north - obstacle_half_width}  // 左下角
    };

    // 检查所有点是否在边界内，只要有一个点在边界内，就认为障碍物在边界内
    for (const auto &point : check_points) {
        double east = point.first;
        double north = point.second;

        // 找到最近的边界点
        int left_idx = FindNearestBoundaryPoint(left_boundary_, east, north);
        int right_idx = FindNearestBoundaryPoint(right_boundary_, east, north);

        if (left_idx < 0 || right_idx < 0) {
            continue;
        }

        // 计算点到边界的距离
        double dist_to_left = CalculatePointToBoundaryDistance(east, north, left_boundary_.points[left_idx].east,
                                                               left_boundary_.points[left_idx].north);

        double dist_to_right = CalculatePointToBoundaryDistance(east, north, right_boundary_.points[right_idx].east,
                                                                right_boundary_.points[right_idx].north);

        // 如果点在两条边界线之间，就认为这个点在边界内
        if (dist_to_left > 0 && dist_to_right > 0) {
            return true; // 只要有一个点在边界内，就返回true
        }
    }

    // 所有点都不在边界内，返回false
    return false;
}

/**
 * @brief 寻找最近的边界点
 * @param boundary 边界线
 * @param east 东向坐标
 * @param north 北向坐标
 * @return 最近边界点索引，失败返回-1
 */
int PlanningNode::FindNearestBoundaryPoint(const bot_msg::msg::Boundary &boundary, double east, double north) {
    if (boundary.points.empty()) {
        return -1;
    }

    int nearest_idx = -1;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < boundary.points.size(); i++) {
        double dist =
            std::sqrt(std::pow(boundary.points[i].east - east, 2) + std::pow(boundary.points[i].north - north, 2));
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    return nearest_idx;
}

/**
 * @brief 计算点到边界的距离
 * @param east 东向坐标
 * @param north 北向坐标
 * @param bound_east 边界点东向坐标
 * @param bound_north 边界点北向坐标
 * @return 点到边界的距离，正值表示在边界内，负值表示在边界外
 */
double PlanningNode::CalculatePointToBoundaryDistance(double east, double north, double bound_east,
                                                      double bound_north) {
    // 计算边界向量（相对于车辆中心）
    double boundary_vector_east = bound_east - cur_local_.east;
    double boundary_vector_north = bound_north - cur_local_.north;

    // 计算障碍物向量（相对于车辆中心）
    double obstacle_vector_east = east - cur_local_.east;
    double obstacle_vector_north = north - cur_local_.north;

    // 计算边界向量和障碍物向量的叉积，判断障碍物在边界的哪一侧
    double cross_product = boundary_vector_east * obstacle_vector_north - boundary_vector_north * obstacle_vector_east;

    return cross_product;
}

/**
 * @brief 基于占用栅格地图更新障碍物信息
 */
void PlanningNode::UpdateObstacleInfoFromOccupancyGrid() {
    double grid_start_time = getCurrentTimeMs();

    obstacle_info_.fill(-1);

    // 清空之前的障碍物点
    detected_obstacle_points_.clear();

    // 检查占用栅格地图是否有效
    if (occupancy_grid_.data.empty() || occupancy_grid_.info.width == 0 || occupancy_grid_.info.height == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Occupancy grid is empty or invalid");
        return;
    }

    // 检查边界线是否有效
    if (left_boundary_.points.empty() || right_boundary_.points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Boundary lines are empty");
        return;
    }

    // 获取占用栅格地图信息
    const auto &info = occupancy_grid_.info;
    const double resolution = info.resolution;
    const int width = info.width;
    const int height = info.height;

    // 获取地图原点在全局坐标系中的位置
    double origin_x = info.origin.position.x;
    double origin_y = info.origin.position.y;

    // 获取栅格地图的旋转信息
    double grid_yaw = tf2::getYaw(info.origin.orientation);

    // RCLCPP_INFO(this->get_logger(), "Occupancy grid size: %dx%d, resolution: %.2f, origin: (%.2f, %.2f), yaw:
    // %.2f",
    //             width, height, resolution, origin_x, origin_y, grid_yaw);

    // 遍历车辆前方区域，检查障碍物
    for (int grid_x = 0; grid_x < width; ++grid_x) {
        for (int grid_y = 0; grid_y < height; ++grid_y) {
            int index = grid_y * width + grid_x;

            // 检查栅格是否被占用
            if (occupancy_grid_.data[index] < occupied_threshold_) {
                continue;
            }

            // 将栅格坐标转换为相对于栅格地图原点的坐标（考虑分辨率）
            double local_x = grid_x * resolution;
            double local_y = grid_y * resolution;

            // 应用旋转变换，将相对坐标转换为考虑航向的坐标
            double rotated_x = local_x * cos(grid_yaw) - local_y * sin(grid_yaw);
            double rotated_y = local_x * sin(grid_yaw) + local_y * cos(grid_yaw);

            // 计算全局坐标（地图坐标系）
            double global_x = origin_x + rotated_x;
            double global_y = origin_y + rotated_y;

            // 计算全局坐标差
            double dx = global_x - cur_local_.east;
            double dy = global_y - cur_local_.north;

            // 获取车辆航向角（顺时针为正，单位：弧度）
            double vehicle_heading = cur_local_.yaw * M_PI / 180.0;

            // 为提高效率和可读性，预先计算 sin 和 cos 值
            double cos_heading = cos(vehicle_heading);
            double sin_heading = sin(vehicle_heading);

            // 将全局坐标差转换到车辆坐标系下
            // 车前方为y轴正方向，车右侧为x轴正方向
            //
            // 根据向量投影公式：
            // relative_x = dx * cos(h) - dy * sin(h)
            // relative_y = dx * sin(h) + dy * cos(h)
            // h 为顺时针为正的航向角

            double relative_x = dx * cos_heading - dy * sin_heading; // 车右侧为x轴正方向
            double relative_y = dx * sin_heading + dy * cos_heading; // 车前方为y轴正方向
            // 计算距离
            double distance = std::sqrt(relative_x * relative_x + relative_y * relative_y);

            // 距离超过阈值，跳过
            if (distance > min_obstacle_distance_) {
                continue;
            }

            // 检查障碍物是否在边界内
            bool is_in_boundary = IsObstacleInBoundaryByPosition(global_x, global_y);

            // 添加调试信息，记录边界判断结果
            if (is_in_boundary) {
                RCLCPP_INFO(this->get_logger(),
                            "车辆当前位置：(%.2f, %.2f, %.0f deg),点 (%.2f, %.2f) 在边界内，相对位置：(%.2f, %.2f)",
                            cur_local_.east, cur_local_.north, cur_local_.yaw, global_x, global_y, relative_x,
                            relative_y);
            }

            // 存储检测到的障碍物点
            if (is_in_boundary) {
                ObstaclePoint obstacle_point;
                obstacle_point.x = global_x;
                obstacle_point.y = global_y;
                obstacle_point.in_boundary = is_in_boundary;
                detected_obstacle_points_.push_back(obstacle_point);
            }

            if (!is_in_boundary) {
                continue;
            }

            // 根据车辆坐标系和行驶方向判断障碍物位置
            bool obstacle_detected = false;
            if (reverse_moving_) {
                // 倒车模式：检测车辆后方障碍物
                if (relative_y < 0.0 && relative_y > -min_obstacle_distance_) {
                    obstacle_detected = true;
                    RCLCPP_INFO(this->get_logger(), "倒车模式-后方发现障碍物，距离：%.2f 米，相对位置：(%.2f, %.2f)",
                                distance, relative_x, relative_y);
                }
            } else {
                // 前进模式：检测车辆前方障碍物
                if (relative_y > 0.0 && relative_y < min_obstacle_distance_) {
                    obstacle_detected = true;
                    RCLCPP_INFO(this->get_logger(), "前进模式-前方发现障碍物，距离：%.2f 米，相对位置：(%.2f, %.2f)",
                                distance, relative_x, relative_y);
                }
            }

            if (obstacle_detected) {
                obstacle_info_[1] = 1; // 使用1表示检测到障碍物
            }
        }
    }

    // 记录占用栅格地图处理耗时
    double grid_end_time = getCurrentTimeMs();
    total_occupancy_grid_time_ += (grid_end_time - grid_start_time);
}

/**
 * @brief 根据位置判断障碍物是否在边界内
 * @param global_x 障碍物全局X坐标
 * @param global_y 障碍物全局Y坐标
 * @return 是否在边界内
 */
bool PlanningNode::IsObstacleInBoundaryByPosition(double global_x, double global_y) {
    // 找到最近的边界点
    int left_idx = FindNearestBoundaryPoint(left_boundary_, global_x, global_y);
    int right_idx = FindNearestBoundaryPoint(right_boundary_, global_x, global_y);

    // 检查索引是否有效
    if (left_idx < 0 || right_idx < 0 || left_idx >= left_boundary_.points.size() ||
        right_idx >= right_boundary_.points.size()) {
        return false;
    }

    // 获取左右边界点
    const auto &left_point = left_boundary_.points[left_idx];
    const auto &right_point = right_boundary_.points[right_idx];

    // 计算障碍物到车辆的向量
    double obs_vec_x = global_x - cur_local_.east;
    double obs_vec_y = global_y - cur_local_.north;

    // 计算左边界点到车辆的向量
    double left_vec_x = left_point.east - cur_local_.east;
    double left_vec_y = left_point.north - cur_local_.north;

    // 计算右边界点到车辆的向量
    double right_vec_x = right_point.east - cur_local_.east;
    double right_vec_y = right_point.north - cur_local_.north;

    // 计算叉积，判断障碍物相对于边界的位置
    // 对于左边界，障碍物应该在边界的右侧（叉积为负）
    double left_cross = left_vec_x * obs_vec_y - left_vec_y * obs_vec_x;

    // 对于右边界，障碍物应该在边界的左侧（叉积为正）
    double right_cross = right_vec_x * obs_vec_y - right_vec_y * obs_vec_x;

    // 计算点到左右边界的距离
    double dist_to_left = std::sqrt(std::pow(global_x - left_point.east, 2) + std::pow(global_y - left_point.north, 2));
    double dist_to_right =
        std::sqrt(std::pow(global_x - right_point.east, 2) + std::pow(global_y - right_point.north, 2));

    // 计算左右边界之间的距离
    double boundary_width =
        std::sqrt(std::pow(left_point.east - right_point.east, 2) + std::pow(left_point.north - right_point.north, 2));

    // 修改：使用更宽松的条件判断点是否在边界内
    // 1. 如果点到任一边界的距离超过边界宽度的0.9倍，认为它在边界外
    if (dist_to_left > boundary_width * 0.9 || dist_to_right > boundary_width * 0.9) {
        return false;
    }

    // 2. 使用叉积判断点是否在边界内
    // 障碍物在左边界右侧且在右边界左侧，则在边界内
    return (left_cross < 0 && right_cross > 0);
}

void PlanningNode::UpdatePlanningStatus() {
    bool is_path_tail = IsPathTail();
    bool has_front_obstacle = (obstacle_info_[1] != -1);

    // 非对称状态稳定性计数器
    static int clear_stable_count = 0;
    const int CLEAR_STABILITY_THRESHOLD = 3; // 无障碍物需要连续3次确认才恢复运行

    // 更新稳定性计数器
    if (has_front_obstacle) {
        // 有障碍物立即重置清除计数器
        clear_stable_count = 0;
    } else {
        // 无障碍物时累计清除计数
        clear_stable_count++;
    }

    // 按键1是启动,按键2是停止
    if (planning_status_ == PlanningStatus::Stop) {
        if (remote_control_cmd_ == 1 && !is_path_tail && clear_stable_count >= CLEAR_STABILITY_THRESHOLD) {
            planning_status_ = PlanningStatus::Planning;
            key_stop_ = false;
            clear_stable_count = 0;
            RCLCPP_INFO(this->get_logger(), "Manual start: Planning resumed");
        }
        // 只有在连续检测到无障碍物时才切换到PLANNING
        if (!key_stop_ && clear_stable_count >= CLEAR_STABILITY_THRESHOLD && !is_path_tail) {
            planning_status_ = PlanningStatus::Planning;
            RCLCPP_INFO(this->get_logger(), "Auto resume: Clear for %d frames", clear_stable_count);
        }
    } else if (planning_status_ == PlanningStatus::Planning) {
        // 有障碍物立即停车，或手动停止，或到达路径终点
        if (remote_control_cmd_ == 2) {
            planning_status_ = PlanningStatus::Stop;
            RCLCPP_INFO(this->get_logger(), "Manual stop: Planning stopped");
        } else if (has_front_obstacle || is_path_tail) {
            planning_status_ = PlanningStatus::Stop;
            if (is_path_tail) {
                RCLCPP_INFO(this->get_logger(), "Auto stop: Reached path tail");
            } else {
                RCLCPP_INFO(this->get_logger(), "Auto stop: Obstacle detected - immediate stop");
            }
        }
    }

    if (timer_cnt_ % 10 == 0) {
        std::string status_text = "Planning status: ";
        if (planning_status_ == PlanningStatus::Stop) {
            if (remote_control_cmd_ == 2) {
                status_text += "STOP (by remote control)";
            } else if (has_front_obstacle) {
                status_text += "STOP (by obstacle)";
            } else if (is_path_tail) {
                status_text += "STOP (reached path tail)";
            } else {
                status_text += "STOP (unknown reason)";
            }
        } else {
            status_text += "PLANNING";
        }
        status_text += " [has_obstacle:" + std::string(has_front_obstacle ? "YES" : "NO") +
                       ", clear_count:" + std::to_string(clear_stable_count) + "]";
        RCLCPP_INFO(this->get_logger(), "%s", status_text.c_str());
    }
}

/**
 * @brief 发布可视化信息
 */
void PlanningNode::PublishVisualization(const bot_msg::msg::ADCTrajectory &pub_traj) {
    visualization_msgs::msg::MarkerArray marker_array;

    // 统一的时间戳，确保所有marker同步
    auto current_time = this->now();
    const std::string frame_id = "map";

    // 创建轨迹可视化
    auto trajectory_marker = CreateTrajectoryMarker(pub_traj);
    // RCLCPP_INFO(this->get_logger(), "轨迹点数: %ld", trajectory_marker.points.size());
    if (trajectory_marker.points.size() > 0) {
        trajectory_marker.header.stamp = current_time;
        trajectory_marker.header.frame_id = frame_id;
        marker_array.markers.push_back(trajectory_marker);
    }

    // 添加障碍物点可视化
    auto obstacle_points_marker = CreateObstaclePointsMarker();
    if (obstacle_points_marker.points.size() > 0) {
        obstacle_points_marker.header.stamp = current_time;
        obstacle_points_marker.header.frame_id = frame_id;
        marker_array.markers.push_back(obstacle_points_marker);
    }

    // 创建车辆位置可视化
    auto vehicle_marker = CreateVehicleMarker();
    vehicle_marker.header.stamp = current_time;
    vehicle_marker.header.frame_id = frame_id;
    marker_array.markers.push_back(vehicle_marker);

    // 创建障碍物状态可视化
    auto obstacle_status_marker = CreateObstacleStatusMarker();
    obstacle_status_marker.header.stamp = current_time;
    obstacle_status_marker.header.frame_id = frame_id;
    marker_array.markers.push_back(obstacle_status_marker);

    // 创建边界线可视化
    if (!left_boundary_.points.empty()) {
        std_msgs::msg::ColorRGBA left_color;
        left_color.r = 0.0;
        left_color.g = 1.0;
        left_color.b = 0.0;
        left_color.a = 0.8;
        auto left_boundary_marker = CreateBoundaryMarker(left_boundary_, "left_boundary", left_color);
        left_boundary_marker.header.stamp = current_time;
        left_boundary_marker.header.frame_id = frame_id;
        marker_array.markers.push_back(left_boundary_marker);
    }

    if (!right_boundary_.points.empty()) {
        std_msgs::msg::ColorRGBA right_color;
        right_color.r = 1.0;
        right_color.g = 0.0;
        right_color.b = 0.0;
        right_color.a = 0.8;
        auto right_boundary_marker = CreateBoundaryMarker(right_boundary_, "right_boundary", right_color);
        right_boundary_marker.header.stamp = current_time;
        right_boundary_marker.header.frame_id = frame_id;
        marker_array.markers.push_back(right_boundary_marker);
    }

    pub_visualization_->publish(marker_array);
}

/**
 * @brief 创建轨迹可视化标记
 */
visualization_msgs::msg::Marker PlanningNode::CreateTrajectoryMarker(const bot_msg::msg::ADCTrajectory &pub_traj) {
    visualization_msgs::msg::Marker marker;
    // header会在PublishVisualization中统一设置
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.2; // 线宽
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // 添加轨迹点
    for (const auto &point : pub_traj.points) {
        geometry_msgs::msg::Point p;
        p.x = point.east;
        p.y = point.north;
        p.z = 0;
        marker.points.push_back(p);
    }

    return marker;
}

/**
 * @brief 创建车辆位置可视化标记
 */
visualization_msgs::msg::Marker PlanningNode::CreateVehicleMarker() {
    visualization_msgs::msg::Marker marker;
    // header会在PublishVisualization中统一设置
    marker.ns = "vehicle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = cur_local_.east;
    marker.pose.position.y = cur_local_.north;
    marker.pose.position.z = 0.0;

    // 根据yaw角度设置朝向
    double yaw = -cur_local_.yaw * M_PI / 180.0 + M_PI / 2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sin(yaw / 2.0);
    marker.pose.orientation.w = cos(yaw / 2.0);

    marker.scale.x = 0.4; // 箭头长度
    marker.scale.y = 0.1; // 箭头宽度
    marker.scale.z = 0.1; // 箭头高度

    // 根据规划状态设置颜色
    switch (planning_status_) {
    case PlanningStatus::Stop:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0; // 红色
        break;
    case PlanningStatus::Planning:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0; // 绿色
        break;
    default:
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0; // 黄色
        break;
    }
    marker.color.a = 1.0;

    return marker;
}

/**
 * @brief 创建障碍物状态可视化标记
 */
visualization_msgs::msg::Marker PlanningNode::CreateObstacleStatusMarker() {
    visualization_msgs::msg::Marker marker;
    // header会在PublishVisualization中统一设置
    marker.ns = "obstacle_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = cur_local_.east;
    marker.pose.position.y = cur_local_.north + 1.0; // 在车辆上方3米显示
    marker.pose.position.z = 1.0;

    marker.scale.z = 0.4; // 文字大小
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // 根据障碍物检测结果设置文本
    std::string status_text = "Status: ";
    if (planning_status_ == PlanningStatus::Stop) {
        status_text += "STOP";
    } else {
        status_text += "PLANNING";
    }

    status_text += "\nObstacles: ";
    if (obstacle_info_[0] != -1)
        status_text += "LEFT ";
    if (obstacle_info_[1] != -1)
        status_text += "FRONT ";
    if (obstacle_info_[2] != -1)
        status_text += "RIGHT ";
    if (obstacle_info_[0] == -1 && obstacle_info_[1] == -1 && obstacle_info_[2] == -1) {
        status_text += "NONE";
    }

    status_text += "\nMode: " + std::string(use_occupancy_grid_ ? "OccupancyGrid" : "Traditional");

    marker.text = status_text;

    return marker;
}

/**
 * @brief 创建边界线可视化标记
 */
visualization_msgs::msg::Marker PlanningNode::CreateBoundaryMarker(const bot_msg::msg::Boundary &boundary,
                                                                   const std::string &ns,
                                                                   const std_msgs::msg::ColorRGBA &color) {
    visualization_msgs::msg::Marker marker;
    // header会在PublishVisualization中统一设置
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1; // 线宽
    marker.color = color;

    // 添加边界点
    for (const auto &point : boundary.points) {
        geometry_msgs::msg::Point p;
        p.x = point.east;
        p.y = point.north;
        p.z = 0;
        marker.points.push_back(p);
    }

    return marker;
}

/**
 * @brief 创建障碍物点可视化标记
 */
visualization_msgs::msg::Marker PlanningNode::CreateObstaclePointsMarker() {
    visualization_msgs::msg::Marker marker;
    // header会在PublishVisualization中统一设置
    marker.ns = "obstacle_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1; // 点的大小
    marker.scale.y = 0.1;

    // 添加所有检测到的障碍物点
    for (const auto &point : detected_obstacle_points_) {
        // 只显示边界内的点
        if (point.in_boundary) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.1; // 稍微抬高一点，以便更容易看到

            marker.points.push_back(p);

            // 设置点的颜色为红色
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
            marker.colors.push_back(color);
        }
    }

    return marker;
}

double PlanningNode::calculateTrajectoryPointCost(size_t index, double cur_yaw_rad, size_t last_closest_idx,
                                                  bool first_run) {
    if (index >= g_traj_.points.size()) {
        return std::numeric_limits<double>::max();
    }

    // 计算距离成本
    double dist = std::sqrt(std::pow(g_traj_.points[index].east - cur_local_.east, 2) +
                            std::pow(g_traj_.points[index].north - cur_local_.north, 2));

    // 计算连续性成本（避免大幅跳跃）
    double stability_cost = 0.0;
    if (!first_run) {
        // 计算与上次最近点的索引差异
        double index_diff = std::abs(static_cast<double>(index) - static_cast<double>(last_closest_idx));

        // 如果索引跳跃过大，增加惩罚
        if (index_diff > max_index_jump_) {
            stability_cost = direction_stability_weight_ * (index_diff - max_index_jump_);
        }
    }

    // 组合成本：距离 + 连续性成本
    return dist + stability_cost;
}

double PlanningNode::getCurrentTimeMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
}

void PlanningNode::logTimingStatistics() const {
    if (frame_count_ == 0)
        return;

    double avg_total = total_processing_time_ / frame_count_;
    double avg_obstacle = total_obstacle_detection_time_ / frame_count_;
    double avg_trajectory = total_trajectory_planning_time_ / frame_count_;
    double avg_visualization = total_visualization_time_ / frame_count_;
    double avg_occupancy = total_occupancy_grid_time_ / frame_count_;

    RCLCPP_INFO(get_logger(), "=== Planning Processing Statistics (Frame %d) ===", frame_count_);
    RCLCPP_INFO(get_logger(), "Total Processing:      %.2f ms (avg)", avg_total);
    RCLCPP_INFO(get_logger(), "  - Obstacle Detection: %.2f ms (avg)", avg_obstacle);
    RCLCPP_INFO(get_logger(), "  - Trajectory Planning:%.2f ms (avg)", avg_trajectory);
    RCLCPP_INFO(get_logger(), "  - Visualization:      %.2f ms (avg)", avg_visualization);
    RCLCPP_INFO(get_logger(), "  - Occupancy Grid:     %.2f ms (avg)", avg_occupancy);

    // 计算各步骤占总时间的百分比
    if (avg_total > 0) {
        RCLCPP_INFO(get_logger(), "Time Distribution:");
        RCLCPP_INFO(get_logger(), "  - Obstacle Detection: %.1f%%", (avg_obstacle / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Trajectory Planning:%.1f%%", (avg_trajectory / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Visualization:      %.1f%%", (avg_visualization / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Occupancy Grid:     %.1f%%", (avg_occupancy / avg_total) * 100.0);
    }
    RCLCPP_INFO(get_logger(), "Zero-copy mode: %s", enable_zero_copy_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(get_logger(), "================================================");
}

void PlanningNode::updateObstacleInfoFromOccupancyGridZeroCopy() {
    double grid_start_time = getCurrentTimeMs();

    obstacle_info_.fill(-1);
    detected_obstacle_points_.clear();

    // 检查占用栅格地图是否有效
    if (occupancy_grid_.data.empty() || occupancy_grid_.info.width == 0 || occupancy_grid_.info.height == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Occupancy grid is empty or invalid");
        return;
    }

    // 检查边界线是否有效
    if (left_boundary_.points.empty() || right_boundary_.points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Boundary lines are empty");
        return;
    }

    // 零拷贝优化：直接访问栅格数据，避免不必要的拷贝
    const auto &info = occupancy_grid_.info;
    const auto &data = occupancy_grid_.data; // 直接引用，避免拷贝

    const double resolution = info.resolution;
    const int width = info.width;
    const int height = info.height;
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const double grid_yaw = tf2::getYaw(info.origin.orientation);

    // 预计算三角函数值，避免重复计算
    const double cos_grid_yaw = cos(grid_yaw);
    const double sin_grid_yaw = sin(grid_yaw);
    const double vehicle_heading = cur_local_.yaw * M_PI / 180.0;
    const double cos_heading = cos(vehicle_heading);
    const double sin_heading = sin(vehicle_heading);

    // 预分配障碍物点容器，避免频繁内存分配
    detected_obstacle_points_.reserve(1000); // 预估容量

    // 优化的栅格遍历：只检查车辆前方的相关区域
    const double max_check_distance = min_obstacle_distance_ + 2.0; // 减少缓冲区，从5米减到2米

    // 计算需要检查的栅格范围（在车辆坐标系下）
    const int check_range_cells = static_cast<int>(max_check_distance / resolution) + 1;

    // 采样优化：根据配置的采样分辨率调整采样步长
    int sampling_step = 1;
    if (resolution < grid_sampling_resolution_) {
        sampling_step = static_cast<int>(grid_sampling_resolution_ / resolution);
    }

    // 将车辆位置转换到栅格坐标系
    double vehicle_local_x = (cur_local_.east - origin_x) * cos_grid_yaw + (cur_local_.north - origin_y) * sin_grid_yaw;
    double vehicle_local_y =
        -(cur_local_.east - origin_x) * sin_grid_yaw + (cur_local_.north - origin_y) * cos_grid_yaw;
    int vehicle_grid_x = static_cast<int>(vehicle_local_x / resolution);
    int vehicle_grid_y = static_cast<int>(vehicle_local_y / resolution);

    // 限制搜索范围，避免越界
    int start_x = std::max(0, vehicle_grid_x - check_range_cells);
    int end_x = std::min(width, vehicle_grid_x + check_range_cells);
    int start_y = std::max(0, vehicle_grid_y - check_range_cells);
    int end_y = std::min(height, vehicle_grid_y + check_range_cells);

    // 遍历限定区域内的栅格（使用采样步长优化）
    int obstacle_count = 0;

    for (int grid_x = start_x; grid_x < end_x; grid_x += sampling_step) {
        for (int grid_y = start_y; grid_y < end_y; grid_y += sampling_step) {
            // 早期退出：如果已经检查了足够多的障碍物点，就停止
            if (obstacle_count >= max_obstacles_to_check_) {
                break;
            }
            int index = grid_y * width + grid_x;

            // 检查栅格是否被占用（直接访问数据，无拷贝）
            if (data[index] < occupied_threshold_) {
                continue;
            }

            // 快速计算全局坐标（减少重复计算）
            double local_x = grid_x * resolution;
            double local_y = grid_y * resolution;
            double rotated_x = local_x * cos_grid_yaw - local_y * sin_grid_yaw;
            double rotated_y = local_x * sin_grid_yaw + local_y * cos_grid_yaw;
            double global_x = origin_x + rotated_x;
            double global_y = origin_y + rotated_y;

            // 快速距离检查（避免sqrt计算）
            double dx = global_x - cur_local_.east;
            double dy = global_y - cur_local_.north;
            double distance_squared = dx * dx + dy * dy;
            double max_distance_squared = min_obstacle_distance_ * min_obstacle_distance_;

            if (distance_squared > max_distance_squared) {
                continue;
            }

            // 转换到车辆坐标系（使用预计算的三角函数值）
            double relative_x = dx * cos_heading - dy * sin_heading;
            double relative_y = dx * sin_heading + dy * cos_heading;

            // 恢复完整的边界检查（未优化版本）
            bool is_in_boundary = IsObstacleInBoundaryByPosition(global_x, global_y);

            if (is_in_boundary) {
                // 使用emplace_back避免临时对象创建
                detected_obstacle_points_.emplace_back(ObstaclePoint{global_x, global_y, true});
                obstacle_count++;

                // 根据车辆坐标系和行驶方向判断障碍物位置
                bool obstacle_detected = false;
                double distance = sqrt(distance_squared);
                if (reverse_moving_) {
                    // 倒车模式：检测车辆后方障碍物
                    if (relative_y < 0.0 && relative_y > -min_obstacle_distance_) {
                        obstacle_detected = true;
                        RCLCPP_INFO(this->get_logger(),
                                    "倒车模式-后方发现障碍物，距离：%.2f 米，相对位置：(%.2f, %.2f)", distance,
                                    relative_x, relative_y);
                    }
                } else {
                    // 前进模式：检测车辆前方障碍物
                    if (relative_y > 0.0 && relative_y < min_obstacle_distance_) {
                        obstacle_detected = true;
                        RCLCPP_INFO(this->get_logger(),
                                    "前进模式-前方发现障碍物，距离：%.2f 米，相对位置：(%.2f, %.2f)", distance,
                                    relative_x, relative_y);
                    }
                }

                if (obstacle_detected) {
                    obstacle_info_[1] = 1; // 使用1表示检测到障碍物
                }
            }
        }
    }

    // 记录占用栅格地图处理耗时
    double grid_end_time = getCurrentTimeMs();
    total_occupancy_grid_time_ += (grid_end_time - grid_start_time);
}

void PlanningNode::resetTimingStatistics() const {
    RCLCPP_INFO(get_logger(), "Resetting timing statistics after %d frames", frame_count_);
    frame_count_ = 0;
    total_processing_time_ = 0.0;
    total_obstacle_detection_time_ = 0.0;
    total_trajectory_planning_time_ = 0.0;
    total_visualization_time_ = 0.0;
    total_occupancy_grid_time_ = 0.0;
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
