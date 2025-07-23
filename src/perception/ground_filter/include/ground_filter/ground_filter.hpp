#ifndef GROUND_FILTER__GROUND_FILTER_HPP_
#define GROUND_FILTER__GROUND_FILTER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "ground_filter/height_filter.hpp"
#include "ground_filter/visibility_control.hpp"

namespace ground_filter {

class GroundFilterNode : public rclcpp::Node {
  public:
    GROUND_FILTER_PUBLIC
    explicit GroundFilterNode(const rclcpp::NodeOptions &options);

  private:
    // 参数
    double update_rate_;
    double min_height_;
    double max_height_;
    std::string base_frame_;
    std::string target_frame_;
    bool use_sensor_frame_;

    // 话题参数
    std::string input_topic_;
    std::string ground_points_topic_;
    std::string no_ground_points_topic_;

    // 性能统计参数
    bool enable_timing_logs_;     // 是否启用耗时日志
    int timing_log_interval_;     // 耗时日志输出间隔（每N帧输出一次）
    bool enable_detailed_timing_; // 是否启用详细的分步耗时统计
    bool enable_zero_copy_;       // 是否启用零拷贝优化

    // 订阅和发布
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_points_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_points_pub_;

    // TF监听
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 高度滤波器
    std::unique_ptr<HeightFilter> height_filter_;

    // 性能统计变量
    mutable int frame_count_;              // 处理的帧数计数
    mutable double total_processing_time_; // 总处理时间（毫秒）
    mutable double total_transform_time_;  // 坐标变换总时间（毫秒）
    mutable double total_conversion_time_; // 格式转换总时间（毫秒）
    mutable double total_filtering_time_;  // 地面过滤总时间（毫秒）
    mutable double total_publish_time_;    // 发布总时间（毫秒）

    // 回调函数
    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void timerCallback();

    // 处理点云
    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud);

    // 性能统计辅助函数
    void logTimingStatistics() const;
    void resetTimingStatistics() const;
    double getCurrentTimeMs() const;

    // 零拷贝优化函数
    void processPointCloudZeroCopy(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud);
};

} // namespace ground_filter

#endif // GROUND_FILTER__GROUND_FILTER_HPP_