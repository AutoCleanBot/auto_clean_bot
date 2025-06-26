#include "transform/transform.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace transform {
Transform::Transform() : Node("transform") {
    // 初始化参数
    InitParams();

    // 创建静态坐标转换广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 创建动态坐标转换广播器
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 广播静态坐标转换
    BroadcastTransform();

    // 如果启用map坐标转换，创建RTK GNSS位姿订阅
    if (enable_map_transform_) {
        gnss_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            rtk_gnss_pose_topic_, 10, 
            std::bind(&Transform::GnssPoseCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to RTK GNSS pose topic: %s", rtk_gnss_pose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Will broadcast dynamic transform from %s to %s", map_frame_id_.c_str(), base_frame_id_.c_str());

        // 添加一个计时器，定期重发最新的转换，确保转换数据始终可用
        transform_refresh_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz刷新率
            std::bind(&Transform::RepublishLatestTransform, this));
    }

    RCLCPP_INFO(this->get_logger(), "Transform node initialized");
    RCLCPP_INFO(this->get_logger(), "Broadcasting static transform from %s to %s", base_frame_id_.c_str(),
                lidar_frame_id_.c_str());
}

void Transform::InitParams() {
    // 声明参数
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("lidar_frame_id", "lidar_link");
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("rtk_gnss_pose_topic", "gnss/pose");
    this->declare_parameter("enable_map_transform", true);

    // 声明平移参数
    this->declare_parameter("transform_translation.x", 0.0);
    this->declare_parameter("transform_translation.y", 0.0);
    this->declare_parameter("transform_translation.z", 0.0);

    // 声明旋转参数
    this->declare_parameter("transform_rotation.roll", 0.0);
    this->declare_parameter("transform_rotation.pitch", 0.0);
    this->declare_parameter("transform_rotation.yaw", 0.0);

    // 获取参数
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
    map_frame_id_ = this->get_parameter("map_frame_id").as_string();
    rtk_gnss_pose_topic_ = this->get_parameter("rtk_gnss_pose_topic").as_string();
    enable_map_transform_ = this->get_parameter("enable_map_transform").as_bool();

    // 打印参数
    RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "lidar_frame_id: %s", lidar_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "rtk_gnss_pose_topic: %s", rtk_gnss_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "enable_map_transform: %s", enable_map_transform_ ? "true" : "false");
}

void Transform::BroadcastTransform() {
    geometry_msgs::msg::TransformStamped static_transform;

    // 设置时间戳和坐标系
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = base_frame_id_;
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

// 定期重新发布最新的转换数据
void Transform::RepublishLatestTransform() {
    if (has_latest_transform_) {
        // 更新时间戳到当前时间
        latest_transform_.header.stamp = this->now();
        
        // 重新广播转换
        dynamic_tf_broadcaster_->sendTransform(latest_transform_);
    }
}

// RTK GNSS位姿回调函数 - 处理从RTK接收的位姿数据并发布map到base_link的转换
void Transform::GnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 创建转换消息
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    // 设置消息头 - 使用当前时间而不是消息时间，减少时间戳不同步问题
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = map_frame_id_;      // 父坐标系：map
    transform_stamped.child_frame_id = base_frame_id_;      // 子坐标系：base_link
    
    // 设置位置（从RTK的ENU坐标）
    transform_stamped.transform.translation.x = msg->pose.position.x;  // 东向位置
    transform_stamped.transform.translation.y = msg->pose.position.y;  // 北向位置
    transform_stamped.transform.translation.z = 0;  // 上方位置
    
    // 设置姿态（从RTK的四元数）
    transform_stamped.transform.rotation = msg->pose.orientation;
    
    // 发布动态坐标转换
    dynamic_tf_broadcaster_->sendTransform(transform_stamped);
    
    // 保存最近的转换数据用于定期重发
    latest_transform_ = transform_stamped;
    has_latest_transform_ = true;
}
} // namespace transform

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<transform::Transform>());
    rclcpp::shutdown();
    return 0;
}