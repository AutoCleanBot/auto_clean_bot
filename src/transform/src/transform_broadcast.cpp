#include "transform_broadcast.hpp"

TransformBroadcast::TransformBroadcast() 
    : Node("node_transform_broadcast")
{
    // 创建动态和静态TF广播器
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    // 声明参数
    declare_transform_parameters();

    // 订阅组合惯导的里程计数据
    ins_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "imu_odom", 10,
        std::bind(&TransformBroadcast::imu_odom_callback, this, std::placeholders::_1));

    // 订阅车辆的里程计数据
    vehicle_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "vehicle_odom", 10,
        std::bind(&TransformBroadcast::vehicle_odom_callback, this, std::placeholders::_1));

    // 发布静态转换关系
    publish_static_transforms();

    // 创建定时器，定期发布动态转换
    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TransformBroadcast::publish_world_transforms, this));

    RCLCPP_INFO(this->get_logger(), "Transform broadcaster initialized");
}

void TransformBroadcast::declare_transform_parameters()
{
    // 声明激光雷达到车身坐标系的转换参数
    declare_parameter("lidar_to_vehicle.translation.x", 0.0);
    declare_parameter("lidar_to_vehicle.translation.y", 0.0);
    declare_parameter("lidar_to_vehicle.translation.z", 0.0);
    declare_parameter("lidar_to_vehicle.rotation.x", 0.0);
    declare_parameter("lidar_to_vehicle.rotation.y", 0.0);
    declare_parameter("lidar_to_vehicle.rotation.z", 0.0);
    declare_parameter("lidar_to_vehicle.rotation.w", 1.0);

    // 声明相机到激光雷达的转换参数
    declare_parameter("camera_to_lidar.translation.x", 0.0);
    declare_parameter("camera_to_lidar.translation.y", 0.0);
    declare_parameter("camera_to_lidar.translation.z", 0.0);
    declare_parameter("camera_to_lidar.rotation.x", 0.0);
    declare_parameter("camera_to_lidar.rotation.y", 0.0);
    declare_parameter("camera_to_lidar.rotation.z", 0.0);
    declare_parameter("camera_to_lidar.rotation.w", 1.0);

    // 声明组合惯导到车身坐标系的转换参数
    declare_parameter("imu_to_vehicle.translation.x", 0.0);
    declare_parameter("imu_to_vehicle.translation.y", 0.0);
    declare_parameter("imu_to_vehicle.translation.z", 0.0);
    declare_parameter("imu_to_vehicle.rotation.x", 0.0);
    declare_parameter("imu_to_vehicle.rotation.y", 0.0);
    declare_parameter("imu_to_vehicle.rotation.z", 0.0);
    declare_parameter("imu_to_vehicle.rotation.w", 1.0);
}

void TransformBroadcast::publish_static_transforms()
{
    // 激光雷达到车身坐标系的静态转换
    geometry_msgs::msg::TransformStamped t_lidar_vehicle;
    t_lidar_vehicle.header.stamp = now();
    t_lidar_vehicle.header.frame_id = "vehicle";
    t_lidar_vehicle.child_frame_id = "lidar";
    
    t_lidar_vehicle.transform.translation.x = get_parameter("lidar_to_vehicle.translation.x").as_double();
    t_lidar_vehicle.transform.translation.y = get_parameter("lidar_to_vehicle.translation.y").as_double();
    t_lidar_vehicle.transform.translation.z = get_parameter("lidar_to_vehicle.translation.z").as_double();
    
    t_lidar_vehicle.transform.rotation.x = get_parameter("lidar_to_vehicle.rotation.x").as_double();
    t_lidar_vehicle.transform.rotation.y = get_parameter("lidar_to_vehicle.rotation.y").as_double();
    t_lidar_vehicle.transform.rotation.z = get_parameter("lidar_to_vehicle.rotation.z").as_double();
    t_lidar_vehicle.transform.rotation.w = get_parameter("lidar_to_vehicle.rotation.w").as_double();

    static_broadcaster->sendTransform(t_lidar_vehicle);

    // 2. 发布相机到激光雷达的静态转换
    geometry_msgs::msg::TransformStamped t_camera_lidar;
    t_camera_lidar.header.stamp = now();
    t_camera_lidar.header.frame_id = "lidar";
    t_camera_lidar.child_frame_id = "camera";
    
    t_camera_lidar.transform.translation.x = get_parameter("camera_to_lidar.translation.x").as_double();
    t_camera_lidar.transform.translation.y = get_parameter("camera_to_lidar.translation.y").as_double();
    t_camera_lidar.transform.translation.z = get_parameter("camera_to_lidar.translation.z").as_double();
    
    t_camera_lidar.transform.rotation.x = get_parameter("camera_to_lidar.rotation.x").as_double();
    t_camera_lidar.transform.rotation.y = get_parameter("camera_to_lidar.rotation.y").as_double();
    t_camera_lidar.transform.rotation.z = get_parameter("camera_to_lidar.rotation.z").as_double();
    t_camera_lidar.transform.rotation.w = get_parameter("camera_to_lidar.rotation.w").as_double();

    static_broadcaster->sendTransform(t_camera_lidar);

    // 3. 发布组合惯导到车身坐标系的静态转换
    geometry_msgs::msg::TransformStamped t_imu_vehicle;
    t_imu_vehicle.header.stamp = now();
    t_imu_vehicle.header.frame_id = "vehicle";
    t_imu_vehicle.child_frame_id = "imu";
    
    t_imu_vehicle.transform.translation.x = get_parameter("imu_to_vehicle.translation.x").as_double();
    t_imu_vehicle.transform.translation.y = get_parameter("imu_to_vehicle.translation.y").as_double();
    t_imu_vehicle.transform.translation.z = get_parameter("imu_to_vehicle.translation.z").as_double();
    
    t_imu_vehicle.transform.rotation.x = get_parameter("imu_to_vehicle.rotation.x").as_double();
    t_imu_vehicle.transform.rotation.y = get_parameter("imu_to_vehicle.rotation.y").as_double();
    t_imu_vehicle.transform.rotation.z = get_parameter("imu_to_vehicle.rotation.z").as_double();
    t_imu_vehicle.transform.rotation.w = get_parameter("imu_to_vehicle.rotation.w").as_double();

    static_broadcaster->sendTransform(t_imu_vehicle);

    RCLCPP_INFO(this->get_logger(), "Published all static transforms");
}

void TransformBroadcast::vehicle_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_vehicle_pose.translation.x = msg->pose.pose.position.x;
    current_vehicle_pose.translation.y = msg->pose.pose.position.y;
    current_vehicle_pose.translation.z = msg->pose.pose.position.z;
    
    current_vehicle_pose.rotation = msg->pose.pose.orientation;
    
    RCLCPP_DEBUG(this->get_logger(), "Received vehicle odometry update");
}

void TransformBroadcast::imu_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_imu_pose.translation.x = msg->pose.pose.position.x;
    current_imu_pose.translation.y = msg->pose.pose.position.y;
    current_imu_pose.translation.z = msg->pose.pose.position.z;
    
    current_imu_pose.rotation = msg->pose.pose.orientation;
    
    RCLCPP_DEBUG(this->get_logger(), "Received imu odometry update");
}

void TransformBroadcast::publish_world_transforms()
{
    auto current_time = now();

    // 1. 发布组合惯导到世界坐标系的动态转换
    geometry_msgs::msg::TransformStamped t_imu_world;
    t_imu_world.header.stamp = current_time;
    t_imu_world.header.frame_id = "world";
    t_imu_world.child_frame_id = "imu";
    t_imu_world.transform = current_imu_pose;

    if (std::isfinite(t_imu_world.transform.translation.x) &&
        std::isfinite(t_imu_world.transform.translation.y) &&
        std::isfinite(t_imu_world.transform.translation.z) &&
        std::isfinite(t_imu_world.transform.rotation.w)) {
        tf_broadcaster->sendTransform(t_imu_world);
        RCLCPP_DEBUG(this->get_logger(), "Published imu to world transform");
    }

    // 2. 发布车身坐标系到世界坐标系的动态转换
    geometry_msgs::msg::TransformStamped t_vehicle_world;
    t_vehicle_world.header.stamp = current_time;
    t_vehicle_world.header.frame_id = "world";
    t_vehicle_world.child_frame_id = "base_link";
    t_vehicle_world.transform = current_vehicle_pose;

    if (std::isfinite(t_vehicle_world.transform.translation.x) &&
        std::isfinite(t_vehicle_world.transform.translation.y) &&
        std::isfinite(t_vehicle_world.transform.translation.z) &&
        std::isfinite(t_vehicle_world.transform.rotation.w)) {
        tf_broadcaster->sendTransform(t_vehicle_world);
        RCLCPP_DEBUG(this->get_logger(), "Published vehicle to world transform");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformBroadcast>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}