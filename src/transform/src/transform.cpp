#include "transform/transform.h"
#include <tf2/LinearMath/Quaternion.h>

namespace transform {
Transform::Transform() : Node("transform") {
    // 初始化参数
    InitParams();

    // 创建静态坐标转换广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 广播静态坐标转换
    BroadcastTransform();

    RCLCPP_INFO(this->get_logger(), "Transform node initialized");
    RCLCPP_INFO(this->get_logger(), "Broadcasting static transform from %s to %s", base_front_id_.c_str(),
                lidar_frame_id_.c_str());
}

void Transform::InitParams() {
    // 声明参数
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("lidar_frame_id", "front_lidar");

    // 声明平移参数
    this->declare_parameter("transform_translation.x", 0.0);
    this->declare_parameter("transform_translation.y", 0.0);
    this->declare_parameter("transform_translation.z", 0.0);

    // 声明旋转参数
    this->declare_parameter("transform_rotation.roll", 0.0);
    this->declare_parameter("transform_rotation.pitch", 0.0);
    this->declare_parameter("transform_rotation.yaw", 0.0);

    // 获取参数
    base_front_id_ = this->get_parameter("base_front_id").as_string();
    lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();

    // 打印参数
    RCLCPP_INFO(this->get_logger(), "base_front_id: %s", base_front_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "lidar_frame_id: %s", lidar_frame_id_.c_str());
}

void Transform::BroadcastTransform() {
    geometry_msgs::msg::TransformStamped static_transform;

    // 设置时间戳和坐标系
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = base_front_id_;
    static_transform.child_frame_id = lidar_frame_id_;

    // 获取并设置平移参数
    static_transform.transform.translation.x = this->get_parameter("transform_translation.x").as_double();
    static_transform.transform.translation.y = this->get_parameter("transform_translation.y").as_double();
    static_transform.transform.translation.z = this->get_parameter("transform_translation.z").as_double();

    // 获取旋转参数
    double roll = this->get_parameter("transform_rotation.roll").as_double();
    double pitch = this->get_parameter("transform_rotation.pitch").as_double();
    double yaw = this->get_parameter("transform_rotation.yaw").as_double();

    // 设置旋转
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    static_transform.transform.rotation.x = q.x();
    static_transform.transform.rotation.y = q.y();
    static_transform.transform.rotation.z = q.z();
    static_transform.transform.rotation.w = q.w();

    // 打印转换信息
    RCLCPP_INFO(this->get_logger(), "Translation: x=%.3f, y=%.3f, z=%.3f", static_transform.transform.translation.x,
                static_transform.transform.translation.y, static_transform.transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "Rotation: roll=%.3f, pitch=%.3f, yaw=%.3f", roll, pitch, yaw);

    // 广播静态坐标转换
    tf_broadcaster_->sendTransform(static_transform);
}
} // namespace transform

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<transform::Transform>());
    rclcpp::shutdown();
    return 0;
}