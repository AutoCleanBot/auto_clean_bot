#ifndef TRANSFORM_TRANSFORM_H
#define TRANSFORM_TRANSFORM_H

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace transform {
class Transform : public rclcpp::Node {
  public:
    Transform();

  private:
    // 静态坐标转换广播器
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // 参数
    std::string base_frame_id_;
    std::string lidar_frame_id_;

    // 初始化参数
    void InitParams();

    // 广播静态坐标转换
    void BroadcastTransform();
};
} // namespace transform

#endif // TRANSFORM_TRANSFORM_H