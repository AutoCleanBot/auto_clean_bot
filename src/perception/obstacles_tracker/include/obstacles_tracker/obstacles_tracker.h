#pragma once

#include <rclcpp/rclcpp.hpp>
#include <bot_msg/msg/obstacles.hpp>
#include <bot_msg/msg/obstacle_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>

#include "obstacles_tracker/kalman_filter.h"

namespace obstacles_tracker {

/**
 * @brief 障碍物轨迹类，表示一个被追踪的障碍物
 */
class ObstacleTrack {
public:
    /**
     * @brief 构造函数
     * @param id 轨迹ID
     * @param obs 初始障碍物信息
     * @param timestamp 时间戳
     * @param kf_params 卡尔曼滤波器参数 [dt, process_noise_pos, process_noise_vel, measurement_noise_pos, measurement_noise_size]
     */
    ObstacleTrack(uint32_t id, const bot_msg::msg::ObstacleInfo& obs, 
                  const rclcpp::Time& timestamp,
                  const std::vector<double>& kf_params = {0.1, 0.1, 0.2, 0.5, 0.2});

    /**
     * @brief 预测障碍物状态
     * @param timestamp 当前时间戳
     */
    void predict(const rclcpp::Time& timestamp);

    /**
     * @brief 用新的测量值更新障碍物状态
     * @param obs 障碍物测量
     * @param timestamp 时间戳
     */
    void update(const bot_msg::msg::ObstacleInfo& obs, const rclcpp::Time& timestamp);

    /**
     * @brief 获取最后一次更新的时间戳
     * @return 最后一次更新的时间戳
     */
    rclcpp::Time getLastUpdateTime() const { return last_update_time_; }

    /**
     * @brief 获取轨迹ID
     * @return 轨迹ID
     */
    uint32_t getId() const { return id_; }

    /**
     * @brief 获取当前障碍物信息
     * @return 障碍物信息
     */
    bot_msg::msg::ObstacleInfo getObstacleInfo() const;

    /**
     * @brief 获取障碍物出现的帧数
     * @return 帧数
     */
    int getAge() const { return age_; }

    /**
     * @brief 获取未匹配的帧数
     * @return 未匹配的帧数
     */
    int getCoastingCount() const { return coasting_count_; }

    /**
     * @brief 增加未匹配的帧数
     */
    void incrementCoastingCount() { coasting_count_++; }

    /**
     * @brief 重置未匹配的帧数
     */
    void resetCoastingCount() { coasting_count_ = 0; }

    /**
     * @brief 计算与另一个障碍物的IOU（交并比）
     * @param obs 另一个障碍物信息
     * @return IOU值，范围[0, 1]
     */
    double calculateIOU(const bot_msg::msg::ObstacleInfo& obs) const;

    /**
     * @brief 计算与另一个障碍物的马氏距离
     * @param obs 另一个障碍物信息
     * @return 马氏距离
     */
    double calculateMahalanobisDistance(const bot_msg::msg::ObstacleInfo& obs) const;
    
    /**
     * @brief 获取卡尔曼滤波器的位置
     * @return 位置向量 [x, y]
     */
    Eigen::Vector2d getPosition() const { return kf_->getPosition(); }
    
    /**
     * @brief 获取卡尔曼滤波器的速度
     * @return 速度向量 [vx, vy]
     */
    Eigen::Vector2d getVelocity() const { return kf_->getVelocity(); }
    
    /**
     * @brief 获取卡尔曼滤波器的尺寸
     * @return 尺寸向量 [width, length, height]
     */
    Eigen::Vector3d getSize() const { return kf_->getSize(); }

private:
    // 轨迹ID
    uint32_t id_;
    // 卡尔曼滤波器
    std::unique_ptr<KalmanFilter> kf_;
    // 最后一次障碍物类型
    uint8_t type_;
    // 最后一次障碍物状态
    uint8_t status_;
    // 最后一次更新的时间戳
    rclcpp::Time last_update_time_;
    // 障碍物出现的帧数
    int age_;
    // 未匹配的帧数（用于判断轨迹是否丢失）
    int coasting_count_;
};

/**
 * @brief 障碍物跟踪器类，管理所有障碍物轨迹
 */
class ObstaclesTracker {
public:
    /**
     * @brief 构造函数
     * @param max_coasting_count 最大未匹配帧数
     * @param position_gate 位置门限（米）
     * @param size_gate 大小门限（比例）
     * @param iou_threshold IOU阈值
     * @param mahalanobis_threshold 马氏距离阈值
     */
    ObstaclesTracker(int max_coasting_count = 5,
                     double position_gate = 2.0,
                     double size_gate = 0.5,
                     double iou_threshold = 0.1,
                     double mahalanobis_threshold = 5.0);

    /**
     * @brief 处理障碍物消息
     * @param obstacles_msg 障碍物消息
     * @return 跟踪后的障碍物消息
     */
    bot_msg::msg::Obstacles update(const bot_msg::msg::Obstacles& obstacles_msg);

    /**
     * @brief 获取用于可视化的标记
     * @return 标记数组
     */
    visualization_msgs::msg::MarkerArray getMarkers() const;

    /**
     * @brief 获取当前跟踪的障碍物数量
     * @return 障碍物数量
     */
    size_t getTrackCount() const { return tracks_.size(); }

private:
    // 最大未匹配帧数（超过此帧数的轨迹将被删除）
    int max_coasting_count_;
    // 位置门限（米）
    double position_gate_;
    // 大小门限（比例）
    double size_gate_;
    // IOU阈值
    double iou_threshold_;
    // 马氏距离阈值
    double mahalanobis_threshold_;
    // 下一个轨迹ID
    uint32_t next_track_id_;
    // 轨迹映射（ID -> 轨迹）
    std::map<uint32_t, std::shared_ptr<ObstacleTrack>> tracks_;
    // 卡尔曼滤波器参数 [dt, process_noise_pos, process_noise_vel, measurement_noise_pos, measurement_noise_size]
    std::vector<double> kf_params_;

    /**
     * @brief 对障碍物进行数据关联
     * @param obstacles_msg 障碍物消息
     * @param timestamp 时间戳
     * @param matched_tracks 匹配的轨迹
     * @param matched_detections 匹配的检测
     */
    void associateDetections(
        const bot_msg::msg::Obstacles& obstacles_msg,
        const rclcpp::Time& timestamp,
        std::vector<std::pair<uint32_t, size_t>>& matched_tracks,
        std::vector<bool>& matched_detections);

    /**
     * @brief 更新现有轨迹
     * @param obstacles_msg 障碍物消息
     * @param timestamp 时间戳
     * @param matched_tracks 匹配的轨迹
     * @param matched_detections 匹配的检测
     */
    void updateTracks(
        const bot_msg::msg::Obstacles& obstacles_msg,
        const rclcpp::Time& timestamp,
        const std::vector<std::pair<uint32_t, size_t>>& matched_tracks,
        const std::vector<bool>& matched_detections);

    /**
     * @brief 处理未匹配的轨迹
     * @param timestamp 时间戳
     * @param matched_tracks 匹配的轨迹
     */
    void handleUnmatchedTracks(
        const rclcpp::Time& timestamp,
        const std::vector<std::pair<uint32_t, size_t>>& matched_tracks);

    /**
     * @brief 处理未匹配的检测
     * @param obstacles_msg 障碍物消息
     * @param timestamp 时间戳
     * @param matched_detections 匹配的检测
     */
    void handleUnmatchedDetections(
        const bot_msg::msg::Obstacles& obstacles_msg,
        const rclcpp::Time& timestamp,
        const std::vector<bool>& matched_detections);
};

/**
 * @brief 障碍物跟踪器节点类
 */
class ObstaclesTrackerNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    explicit ObstaclesTrackerNode();

private:
    /**
     * @brief 初始化参数
     */
    void initParameters();

    /**
     * @brief 订阅障碍物消息的回调函数
     * @param msg 障碍物消息
     */
    void obstaclesCallback(const bot_msg::msg::Obstacles::SharedPtr msg);

    // 障碍物跟踪器
    std::unique_ptr<ObstaclesTracker> tracker_;

    // 障碍物订阅者
    rclcpp::Subscription<bot_msg::msg::Obstacles>::SharedPtr obstacles_sub_;
    // 障碍物发布者
    rclcpp::Publisher<bot_msg::msg::Obstacles>::SharedPtr obstacles_pub_;
    // 可视化标记发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

} // namespace obstacles_tracker 