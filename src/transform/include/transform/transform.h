#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace transform {
class Transform : public rclcpp::Node {
  public:
    Transform();

  private:
    void InitParams();
    void BroadcastTransform();
    void GnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void RepublishLatestTransform();

    // 参数
    std::string base_frame_id_;
    std::string lidar_frame_id_;
    std::string map_frame_id_;
    std::string rtk_gnss_pose_topic_;
    bool enable_map_transform_;

    // 广播器
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_subscription_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr transform_refresh_timer_;
    
    // 最新转换数据
    geometry_msgs::msg::TransformStamped latest_transform_;
    bool has_latest_transform_ = false;
};
} // namespace transform

#endif // TRANSFORM_H_