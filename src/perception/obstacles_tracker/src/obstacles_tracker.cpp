#include "obstacles_tracker/obstacles_tracker.h"
#include <limits>
#include <algorithm>
#include <cmath>

namespace obstacles_tracker {

// ====================== ObstacleTrack 类实现 ======================

ObstacleTrack::ObstacleTrack(uint32_t id, const bot_msg::msg::ObstacleInfo& obs, 
                             const rclcpp::Time& timestamp,
                             const std::vector<double>& kf_params)
    : id_(id), type_(obs.type), status_(obs.status), 
      last_update_time_(timestamp), age_(1), coasting_count_(0) {
    
    // 创建卡尔曼滤波器并初始化
    kf_ = std::make_unique<KalmanFilter>(
        kf_params[0],  // dt
        kf_params[1],  // process_noise_pos
        kf_params[2],  // process_noise_vel
        kf_params[3],  // measurement_noise_pos
        kf_params[4]   // measurement_noise_size
    );
    
    // 初始化状态
    // 障碍物初始速度设为0，后续会通过多次测量更新估计速度
    kf_->initState(
        obs.position_x,  // x
        obs.position_y,  // y
        obs.velocity_x,  // vx
        obs.velocity_y,  // vy
        obs.width,       // width
        obs.length,      // length
        obs.height       // height
    );
}

void ObstacleTrack::predict(const rclcpp::Time& timestamp) {
    // 计算时间间隔
    double dt = (timestamp - last_update_time_).seconds();
    if (dt <= 0) {
        dt = kf_->getDt(); // 使用默认dt
    }
    
    // 预测下一个状态
    kf_->predict(dt);
    
    // 增加年龄
    age_++;
}

void ObstacleTrack::update(const bot_msg::msg::ObstacleInfo& obs, const rclcpp::Time& timestamp) {
    // 创建测量向量
    Eigen::VectorXd z(5);
    z << obs.position_x, obs.position_y, obs.width, obs.length, obs.height;
    
    // 更新卡尔曼滤波器
    kf_->update(z);
    
    // 更新时间戳
    last_update_time_ = timestamp;
    
    // 更新类型和状态
    type_ = obs.type;
    status_ = obs.status;
    
    // 重置未匹配计数
    coasting_count_ = 0;
}

bot_msg::msg::ObstacleInfo ObstacleTrack::getObstacleInfo() const {
    bot_msg::msg::ObstacleInfo obs;
    
    // 设置ID
    obs.id = id_;
    
    // 设置位置和尺寸（从卡尔曼滤波器获取）
    Eigen::Vector2d position = kf_->getPosition();
    obs.position_x = position(0);
    obs.position_y = position(1);
    
    Eigen::Vector2d velocity = kf_->getVelocity();
    obs.velocity_x = velocity(0);
    obs.velocity_y = velocity(1);
    obs.velocity = std::sqrt(velocity(0) * velocity(0) + velocity(1) * velocity(1));
    
    Eigen::Vector3d size = kf_->getSize();
    obs.width = size(0);
    obs.length = size(1);
    obs.height = size(2);
    
    // 计算朝向（根据速度方向）
    if (obs.velocity > 0.5) { // 如果速度足够大，朝向与速度方向一致
        obs.heading = std::atan2(velocity(1), velocity(0));
    } else {
        obs.heading = 0.0; // 静止障碍物的朝向默认为0
    }
    
    // 设置类型和状态
    obs.type = type_;
    obs.status = status_;
    
    return obs;
}

double ObstacleTrack::calculateIOU(const bot_msg::msg::ObstacleInfo& obs) const {
    // 获取跟踪器的当前位置和尺寸
    Eigen::Vector2d position = kf_->getPosition();
    Eigen::Vector3d size = kf_->getSize();
    
    // 计算跟踪器的边界框
    double track_x_min = position(0) - size(1) / 2.0;
    double track_x_max = position(0) + size(1) / 2.0;
    double track_y_min = position(1) - size(0) / 2.0;
    double track_y_max = position(1) + size(0) / 2.0;
    
    // 计算障碍物的边界框
    double obs_x_min = obs.position_x - obs.length / 2.0;
    double obs_x_max = obs.position_x + obs.length / 2.0;
    double obs_y_min = obs.position_y - obs.width / 2.0;
    double obs_y_max = obs.position_y + obs.width / 2.0;
    
    // 计算交集区域
    double x_overlap = std::max(0.0, std::min(track_x_max, obs_x_max) - std::max(track_x_min, obs_x_min));
    double y_overlap = std::max(0.0, std::min(track_y_max, obs_y_max) - std::max(track_y_min, obs_y_min));
    double intersection = x_overlap * y_overlap;
    
    // 计算并集区域
    double track_area = (track_x_max - track_x_min) * (track_y_max - track_y_min);
    double obs_area = (obs_x_max - obs_x_min) * (obs_y_max - obs_y_min);
    double union_area = track_area + obs_area - intersection;
    
    // 计算IOU
    if (union_area > 0) {
        return intersection / union_area;
    } else {
        return 0.0;
    }
}

double ObstacleTrack::calculateMahalanobisDistance(const bot_msg::msg::ObstacleInfo& obs) const {
    // 创建测量向量
    Eigen::VectorXd z(5);
    z << obs.position_x, obs.position_y, obs.width, obs.length, obs.height;
    
    // 计算马氏距离
    return kf_->calculateMahalanobisDistance(z);
}

// ====================== ObstaclesTracker 类实现 ======================

ObstaclesTracker::ObstaclesTracker(int max_coasting_count,
                                 double position_gate,
                                 double size_gate,
                                 double iou_threshold,
                                 double mahalanobis_threshold)
    : max_coasting_count_(max_coasting_count),
      position_gate_(position_gate),
      size_gate_(size_gate),
      iou_threshold_(iou_threshold),
      mahalanobis_threshold_(mahalanobis_threshold),
      next_track_id_(1) {
    // 默认卡尔曼滤波器参数
    kf_params_ = {0.1, 0.1, 0.2, 0.5, 0.2};
}

bot_msg::msg::Obstacles ObstaclesTracker::update(const bot_msg::msg::Obstacles& obstacles_msg) {
    // 获取时间戳
    rclcpp::Time timestamp = rclcpp::Time(obstacles_msg.header.stamp);
    
    // 为现有轨迹预测新状态
    for (auto& track_pair : tracks_) {
        track_pair.second->predict(timestamp);
    }
    
    // 数据关联
    std::vector<std::pair<uint32_t, size_t>> matched_tracks;
    std::vector<bool> matched_detections(obstacles_msg.obstacles.size(), false);
    
    associateDetections(obstacles_msg, timestamp, matched_tracks, matched_detections);
    
    // 更新现有轨迹
    updateTracks(obstacles_msg, timestamp, matched_tracks, matched_detections);
    
    // 处理未匹配的轨迹
    handleUnmatchedTracks(timestamp, matched_tracks);
    
    // 处理未匹配的检测
    handleUnmatchedDetections(obstacles_msg, timestamp, matched_detections);
    
    // 创建输出消息
    bot_msg::msg::Obstacles output_msg;
    output_msg.header = obstacles_msg.header;
    
    // 添加所有轨迹到输出消息
    for (const auto& track_pair : tracks_) {
        bot_msg::msg::ObstacleInfo obstacle = track_pair.second->getObstacleInfo();
        output_msg.obstacles.push_back(obstacle);
    }
    
    return output_msg;
}

void ObstaclesTracker::associateDetections(
    const bot_msg::msg::Obstacles& obstacles_msg,
    const rclcpp::Time& timestamp,
    std::vector<std::pair<uint32_t, size_t>>& matched_tracks,
    std::vector<bool>& matched_detections) {
    
    // 如果没有轨迹或没有检测，直接返回
    if (tracks_.empty() || obstacles_msg.obstacles.empty()) {
        return;
    }
    
    // 创建代价矩阵（越小越好）
    std::vector<std::vector<double>> cost_matrix(tracks_.size(), 
                                               std::vector<double>(obstacles_msg.obstacles.size(), 
                                                                  std::numeric_limits<double>::max()));
    
    // 填充代价矩阵
    size_t track_idx = 0;
    for (const auto& track_pair : tracks_) {
        const auto& track = track_pair.second;
        
        size_t detection_idx = 0;
        for (const auto& obs : obstacles_msg.obstacles) {
            // 获取轨迹和检测的位置
            Eigen::Vector2d track_pos = track->getPosition();
            double track_x = track_pos(0);
            double track_y = track_pos(1);
            
            double obs_x = obs.position_x;
            double obs_y = obs.position_y;
            
            // 计算欧几里得距离
            double distance = std::hypot(track_x - obs_x, track_y - obs_y);
            
            // 应用位置门限
            if (distance <= position_gate_) {
                // 获取轨迹和检测的尺寸
                Eigen::Vector3d track_size = track->getSize();
                double track_width = track_size(0);
                double track_length = track_size(1);
                double track_height = track_size(2);
                
                double obs_width = obs.width;
                double obs_length = obs.length;
                double obs_height = obs.height;
                
                // 计算尺寸比例
                double width_ratio = track_width > 0 ? obs_width / track_width : 0;
                double length_ratio = track_length > 0 ? obs_length / track_length : 0;
                double height_ratio = track_height > 0 ? obs_height / track_height : 0;
                
                // 应用尺寸门限
                if (width_ratio >= 1.0 - size_gate_ && width_ratio <= 1.0 + size_gate_ &&
                    length_ratio >= 1.0 - size_gate_ && length_ratio <= 1.0 + size_gate_ &&
                    height_ratio >= 1.0 - size_gate_ && height_ratio <= 1.0 + size_gate_) {
                    
                    // 计算IOU
                    double iou = track->calculateIOU(obs);
                    
                    // 应用IOU阈值
                    if (iou >= iou_threshold_) {
                        // 计算马氏距离
                        double mahalanobis_distance = track->calculateMahalanobisDistance(obs);
                        
                        // 应用马氏距离阈值
                        if (mahalanobis_distance <= mahalanobis_threshold_) {
                            // 使用负的IOU值作为代价（越大越好，因此取负数）
                            cost_matrix[track_idx][detection_idx] = -iou;
                        }
                    }
                }
            }
            
            detection_idx++;
        }
        
        track_idx++;
    }
    
    // 使用贪婪算法进行匹配（当然，更好的方法是使用匈牙利算法）
    std::vector<bool> assigned_tracks(tracks_.size(), false);
    std::vector<bool> assigned_detections(obstacles_msg.obstacles.size(), false);
    
    // 找到最小代价的匹配
    while (true) {
        double min_cost = std::numeric_limits<double>::max();
        int best_track_idx = -1;
        int best_detection_idx = -1;
        
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (assigned_tracks[i]) continue;
            
            for (size_t j = 0; j < obstacles_msg.obstacles.size(); ++j) {
                if (assigned_detections[j]) continue;
                
                if (cost_matrix[i][j] < min_cost) {
                    min_cost = cost_matrix[i][j];
                    best_track_idx = i;
                    best_detection_idx = j;
                }
            }
        }
        
        // 如果没有找到有效匹配，结束匹配
        if (best_track_idx == -1 || best_detection_idx == -1 || 
            min_cost == std::numeric_limits<double>::max()) {
            break;
        }
        
        // 标记为已分配
        assigned_tracks[best_track_idx] = true;
        assigned_detections[best_detection_idx] = true;
        
        // 将track_idx转换为track_id
        auto track_iter = tracks_.begin();
        std::advance(track_iter, best_track_idx);
        uint32_t track_id = track_iter->first;
        
        // 添加到匹配列表
        matched_tracks.push_back({track_id, static_cast<size_t>(best_detection_idx)});
        matched_detections[best_detection_idx] = true;
    }
}

void ObstaclesTracker::updateTracks(
    const bot_msg::msg::Obstacles& obstacles_msg,
    const rclcpp::Time& timestamp,
    const std::vector<std::pair<uint32_t, size_t>>& matched_tracks,
    const std::vector<bool>& matched_detections) {
    
    // 更新匹配的轨迹
    for (const auto& match : matched_tracks) {
        uint32_t track_id = match.first;
        size_t detection_idx = match.second;
        
        auto track_iter = tracks_.find(track_id);
        if (track_iter != tracks_.end()) {
            track_iter->second->update(obstacles_msg.obstacles[detection_idx], timestamp);
        }
    }
}

void ObstaclesTracker::handleUnmatchedTracks(
    const rclcpp::Time& timestamp,
    const std::vector<std::pair<uint32_t, size_t>>& matched_tracks) {
    
    // 收集所有已匹配的轨迹ID
    std::set<uint32_t> matched_track_ids;
    for (const auto& match : matched_tracks) {
        matched_track_ids.insert(match.first);
    }
    
    // 处理未匹配的轨迹
    std::vector<uint32_t> tracks_to_remove;
    for (const auto& track_pair : tracks_) {
        uint32_t track_id = track_pair.first;
        
        // 如果轨迹未匹配
        if (matched_track_ids.find(track_id) == matched_track_ids.end()) {
            // 增加未匹配计数
            track_pair.second->incrementCoastingCount();
            
            // 如果未匹配计数超过阈值，标记为删除
            if (track_pair.second->getCoastingCount() > max_coasting_count_) {
                tracks_to_remove.push_back(track_id);
            }
        }
    }
    
    // 删除标记的轨迹
    for (uint32_t track_id : tracks_to_remove) {
        tracks_.erase(track_id);
    }
}

void ObstaclesTracker::handleUnmatchedDetections(
    const bot_msg::msg::Obstacles& obstacles_msg,
    const rclcpp::Time& timestamp,
    const std::vector<bool>& matched_detections) {
    
    // 处理未匹配的检测
    for (size_t i = 0; i < obstacles_msg.obstacles.size(); ++i) {
        if (!matched_detections[i]) {
            // 创建新轨迹
            auto track = std::make_shared<ObstacleTrack>(
                next_track_id_,
                obstacles_msg.obstacles[i],
                timestamp,
                kf_params_
            );
            
            // 添加到轨迹映射
            tracks_[next_track_id_] = track;
            
            // 增加下一个轨迹ID
            next_track_id_++;
        }
    }
}

visualization_msgs::msg::MarkerArray ObstaclesTracker::getMarkers() const {
    visualization_msgs::msg::MarkerArray markers;
    
    for (const auto& track_pair : tracks_) {
        const auto& track = track_pair.second;
        const auto& obstacle = track->getObstacleInfo();
        
        // 创建边界框标记
        visualization_msgs::msg::Marker box_marker;
        box_marker.header.frame_id = "base_link"; // 假设是base_link坐标系
        box_marker.header.stamp = rclcpp::Clock().now();
        box_marker.ns = "obstacle_boxes";
        box_marker.id = track->getId();
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位置
        box_marker.pose.position.x = obstacle.position_x;
        box_marker.pose.position.y = obstacle.position_y;
        box_marker.pose.position.z = obstacle.position_z;
        
        // 设置朝向
        tf2::Quaternion q;
        q.setRPY(0, 0, obstacle.heading);
        box_marker.pose.orientation.x = q.x();
        box_marker.pose.orientation.y = q.y();
        box_marker.pose.orientation.z = q.z();
        box_marker.pose.orientation.w = q.w();
        
        // 设置尺寸
        box_marker.scale.x = obstacle.length;
        box_marker.scale.y = obstacle.width;
        box_marker.scale.z = obstacle.height;
        
        // 根据轨迹ID设置颜色
        box_marker.color.r = static_cast<float>(track->getId() % 3 == 0);
        box_marker.color.g = static_cast<float>(track->getId() % 3 == 1);
        box_marker.color.b = static_cast<float>(track->getId() % 3 == 2);
        box_marker.color.a = 0.5;
        
        // 设置生命周期
        box_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        
        markers.markers.push_back(box_marker);
        
        // 创建ID文本标记
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = box_marker.header;
        text_marker.ns = "obstacle_ids";
        text_marker.id = track->getId();
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位置，略高于障碍物
        text_marker.pose.position.x = obstacle.position_x;
        text_marker.pose.position.y = obstacle.position_y;
        text_marker.pose.position.z = obstacle.position_z + obstacle.height + 0.5;
        
        // 设置文本
        text_marker.text = "ID: " + std::to_string(track->getId());
        
        // 设置尺寸
        text_marker.scale.z = 0.5; // 文本高度
        
        // 设置颜色
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        // 设置生命周期
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        
        markers.markers.push_back(text_marker);
    }
    
    return markers;
}

// ====================== ObstaclesTrackerNode 类实现 ======================

ObstaclesTrackerNode::ObstaclesTrackerNode()
    : Node("obstacles_tracker_node") {
    // 初始化参数
    initParameters();
    
    // 创建障碍物跟踪器
    tracker_ = std::make_unique<ObstaclesTracker>(
        this->get_parameter("max_coasting_count").as_int(),
        this->get_parameter("position_gate").as_double(),
        this->get_parameter("size_gate").as_double(),
        this->get_parameter("iou_threshold").as_double(),
        this->get_parameter("mahalanobis_threshold").as_double()
    );
    
    // 创建障碍物订阅者
    obstacles_sub_ = this->create_subscription<bot_msg::msg::Obstacles>(
        this->get_parameter("input_topic").as_string(),
        10,
        std::bind(&ObstaclesTrackerNode::obstaclesCallback, this, std::placeholders::_1)
    );
    
    // 创建障碍物发布者
    obstacles_pub_ = this->create_publisher<bot_msg::msg::Obstacles>(
        this->get_parameter("output_topic").as_string(),
        10
    );
    
    // 创建可视化标记发布者
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        this->get_parameter("marker_topic").as_string(),
        10
    );
    
    RCLCPP_INFO(this->get_logger(), "障碍物跟踪器节点已启动");
}

void ObstaclesTrackerNode::initParameters() {
    // 声明参数
    this->declare_parameter<std::string>("input_topic", "/perception/obstacles");
    this->declare_parameter<std::string>("output_topic", "/perception/tracked_obstacles");
    this->declare_parameter<std::string>("marker_topic", "/perception/tracked_obstacles_markers");
    this->declare_parameter<int>("max_coasting_count", 5);
    this->declare_parameter<double>("position_gate", 2.0);
    this->declare_parameter<double>("size_gate", 0.5);
    this->declare_parameter<double>("iou_threshold", 0.1);
    this->declare_parameter<double>("mahalanobis_threshold", 5.0);
    this->declare_parameter<double>("kalman_dt", 0.1);
    this->declare_parameter<double>("process_noise_pos", 0.1);
    this->declare_parameter<double>("process_noise_vel", 0.2);
    this->declare_parameter<double>("measurement_noise_pos", 0.5);
    this->declare_parameter<double>("measurement_noise_size", 0.2);
}

void ObstaclesTrackerNode::obstaclesCallback(const bot_msg::msg::Obstacles::SharedPtr msg) {
    if (msg->obstacles.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "收到空的障碍物消息");
        return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "收到障碍物消息，共有 %zu 个障碍物", msg->obstacles.size());
    
    // 更新障碍物跟踪
    bot_msg::msg::Obstacles tracked_obstacles = tracker_->update(*msg);
    
    // 发布跟踪的障碍物
    obstacles_pub_->publish(tracked_obstacles);
    
    // 发布可视化标记
    visualization_msgs::msg::MarkerArray markers = tracker_->getMarkers();
    marker_pub_->publish(markers);
    
    RCLCPP_DEBUG(this->get_logger(), "已跟踪 %zu 个障碍物", tracker_->getTrackCount());
}

} // namespace obstacles_tracker 