#ifndef TRANSFORM_BROADCAST_NODE_HPP
#define TRANSFORM_BROADCAST_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class TransformBroadcast : public rclcpp::Node
{
public:
    TransformBroadcast();

private:
    void declare_transform_parameters();
    void publish_static_transforms();
    void publish_world_transforms();
    void imu_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void vehicle_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_sub;

    geometry_msgs::msg::Transform current_vehicle_pose;
    geometry_msgs::msg::Transform current_imu_pose;
};

#endif  // TRANSFORM_BROADCAST_NODE_HPP