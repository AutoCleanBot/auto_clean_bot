#include "planning/planning.h"
#include "bot_msg/srv/routing.hpp"
#include <chrono>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

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

    planning_status_ = PlanningStatus::Planning;
    reverse_moving_ = false; // 如果未设置,则默认是前进
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

    // 占用栅格地图障碍物检测参数
    this->declare_parameter("min_obstacle_distance", 10.0);
    this->declare_parameter("front_obstacle_width", 1.0);
    this->declare_parameter("side_obstacle_width", 2.0);
    this->declare_parameter("occupied_threshold", 50);
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
    use_occupancy_grid_ = this->get_parameter("use_occupancy_grid").as_bool();
    test_mode_ = this->get_parameter("test_mode").as_bool();
    visualization_topic_name_ = this->get_parameter("visualization_topic_name").as_string();

    // 获取占用栅格地图障碍物检测参数
    min_obstacle_distance_ = this->get_parameter("min_obstacle_distance").as_double();
    front_obstacle_width_ = this->get_parameter("front_obstacle_width").as_double();
    side_obstacle_width_ = this->get_parameter("side_obstacle_width").as_double();
    occupied_threshold_ = this->get_parameter("occupied_threshold").as_int();

    RCLCPP_INFO(this->get_logger(), "local_topic_name: %s", local_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "service_name: %s", service_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "traj_topic_name: %s", traj_topic_name_.c_str());
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
    auto path_len = g_traj_.points.size();
    auto tail_east = g_traj_.points[path_len - 1].east;
    auto tail_north = g_traj_.points[path_len - 1].north;
    auto cur_east = cur_local_.east;
    auto cur_north = cur_local_.north;
    double dis = sqrt(pow(tail_east - cur_east, 2) + pow(tail_north - cur_north, 2));
    if (dis < path_end_dist_ || closet_idx_ == path_len - 1) {
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
    // RCLCPP_INFO(this->get_logger(), "min_dist: %f, min_idx: %ld", min_dist, closet_idx_);
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
    // RCLCPP_INFO(this->get_logger(), "cur_dis_cnt: %f, start_idx: %ld, preview_idx: %ld", cur_dis_cnt, start_idx,
    //             preview_idx);
}

// TODO 待验证,更新机制是有有问题
// TODO 路径终点的处理机制
void PlanningNode::TimerCallback() {
    // 基于当前的当前定位信息, 找到当前位置在全局路径上的最近点
    // 根据配置选择障碍物检测方法
    if (use_occupancy_grid_) {
        UpdateObstacleInfoFromOccupancyGrid();
    } else {
        UpdateObstacleInfo();
    }
    UpdatePlanningStatus();

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

    // 发布可视化信息
    PublishVisualization(pub_traj_path);
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

        // 障碍物在当前车辆正前方
        if (obstacle.position_y < FRONT_OBSTACLE_WIDTH && obstacle.position_y > -FRONT_OBSTACLE_WIDTH) {
            if (obstacle.position_x < MIN_OBSTACLE_DISTANCE && obstacle.position_x > 0.0) {
                obstacle_info_[1] = i;
                RCLCPP_INFO(this->get_logger(), "前方发现障碍物，距离：%.2f 米，安全距离：%.2f 米，尺寸：%.2f x %.2f",
                            obstacle.position_x, safe_distance, obstacle_length, obstacle_width);
            }
        }
        // 障碍物在当前车辆左侧
        else if (obstacle.position_y > SIDE_OBSTACLE_WIDTH && obstacle.position_y < MIN_OBSTACLE_DISTANCE) {
            if (obstacle.position_x < MIN_OBSTACLE_DISTANCE && obstacle.position_x > 0.0) {
                obstacle_info_[0] = i;
                RCLCPP_INFO(this->get_logger(), "左侧发现障碍物，距离：%.2f 米，安全距离：%.2f 米，尺寸：%.2f x %.2f",
                            obstacle_distance, safe_distance, obstacle_length, obstacle_width);
            }
        }
        // 障碍物在当前车辆右侧
        else if (obstacle.position_y < -SIDE_OBSTACLE_WIDTH && obstacle.position_y > -MIN_OBSTACLE_DISTANCE) {
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
    obstacle_info_.fill(-1);

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

    RCLCPP_INFO(this->get_logger(), "Occupancy grid size: %dx%d, resolution: %.2f, origin: (%.2f, %.2f)", width, height,
                resolution, origin_x, origin_y);

    // 遍历车辆前方区域，检查障碍物
    for (int grid_x = 0; grid_x < width; ++grid_x) {
        for (int grid_y = 0; grid_y < height; ++grid_y) {
            int index = grid_y * width + grid_x;

            // 检查栅格是否被占用
            if (occupancy_grid_.data[index] < occupied_threshold_) {
                continue;
            }

            // 将栅格坐标转换为全局坐标
            double global_x = origin_x + grid_x * resolution;
            double global_y = origin_y + grid_y * resolution;

            // 转换为相对于车辆的坐标
            double relative_x = global_x - cur_local_.east;
            double relative_y = global_y - cur_local_.north;

            // 计算距离
            double distance = std::sqrt(relative_x * relative_x + relative_y * relative_y);

            // 距离超过阈值，跳过
            if (distance > min_obstacle_distance_) {
                continue;
            }

            // 检查障碍物是否在边界内
            bool is_in_boundary = IsObstacleInBoundaryByPosition(global_x, global_y);
            if (!is_in_boundary) {
                continue;
            }

            // 根据相对位置分类障碍物
            if (relative_y > 0.0 && relative_y < min_obstacle_distance_) { // 车辆前方
                if (relative_x < front_obstacle_width_ && relative_x > -front_obstacle_width_) {
                    // 正前方障碍物
                    obstacle_info_[1] = 1; // 使用1表示检测到障碍物
                    RCLCPP_INFO(this->get_logger(), "前方发现障碍物，距离：%.2f 米，位置：(%.2f, %.2f)", distance,
                                relative_x, relative_y);
                } else if (relative_x > side_obstacle_width_) {
                    // 左前方障碍物
                    obstacle_info_[0] = 1;
                    RCLCPP_INFO(this->get_logger(), "左前方发现障碍物，距离：%.2f 米", distance);
                } else if (relative_x < -side_obstacle_width_) {
                    // 右前方障碍物
                    obstacle_info_[2] = 1;
                    RCLCPP_INFO(this->get_logger(), "右前方发现障碍物，距离：%.2f 米", distance);
                }
            }
        }
    }
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

    // 检查索引是否有效, 为下述值时说明障碍物在边界外
    if (left_idx <= 0 || right_idx <= 0 || left_idx >= left_boundary_.points.size() - 1 ||
        right_idx >= right_boundary_.points.size() - 1) {
        return false;
    }

    // 计算点到边界的距离
    double dist_to_left = CalculatePointToBoundaryDistance(global_x, global_y, left_boundary_.points[left_idx].east,
                                                           left_boundary_.points[left_idx].north);

    double dist_to_right = CalculatePointToBoundaryDistance(global_x, global_y, right_boundary_.points[right_idx].east,
                                                            right_boundary_.points[right_idx].north);

    // 如果点在两条边界线之间，就认为这个点在边界内
    return (dist_to_left > 0 && dist_to_right > 0);
}

void PlanningNode::UpdatePlanningStatus() {
    bool is_path_tail = IsPathTail();
    if (obstacle_info_[1] != -1 || is_path_tail) {
        planning_status_ = PlanningStatus::Stop;
    } else {
        planning_status_ = PlanningStatus::Planning;
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
    if (trajectory_marker.points.size() > 0) {
        trajectory_marker.header.stamp = current_time;
        trajectory_marker.header.frame_id = frame_id;
        marker_array.markers.push_back(trajectory_marker);
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
