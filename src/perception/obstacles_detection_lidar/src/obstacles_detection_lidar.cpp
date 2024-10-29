

#include <geometry_msgs/msg/point.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h> // Ensure you have this header included
// #include <pcl/segmentation/dbscan.h>
#include <obstacles_detection_lidar/obstacles_detection_lidar.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ObstaclesDetectionLidarNode::ObstaclesDetectionLidarNode() : Node("perception_node") {
    // 加载yaml配置参数
    InitParameters();
    if (is_use_front_lidar_)
        front_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            front_lidar_topic_, 10,
            std::bind(&ObstaclesDetectionLidarNode::PointClould2Callback, this, std::placeholders::_1));
    // TODO:针对于左向和右向的激光雷达，需要设计不同的处理函数
    if (is_use_left_lidar_)
        left_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            left_lidar_topic_, 10,
            std::bind(&ObstaclesDetectionLidarNode::PointClould2Callback, this, std::placeholders::_1));
    if (is_use_right_lidar_)
        right_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            right_lidar_topic_, 10,
            std::bind(&ObstaclesDetectionLidarNode::PointClould2Callback, this, std::placeholders::_1));

    obstacle_pub_ = this->create_publisher<bot_msg::msg::Obstacles>("/perception/obstacles", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/perception/marker", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

#if DEBUG_PUBLISH_POINT_CLOUD
    // 在构造函数中初始化发布器
    original_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/original_cloud", 10);
    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/filtered_cloud", 10);
    ground_seg_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_seg_cloud", 10);
    clustered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/clustered_cloud", 10);
#endif
    // 发布静态变换
    InitStaticTransformBroadcaster();
}

// TODO：静态变换的发布器，放置到专门的包中
void ObstaclesDetectionLidarNode::InitStaticTransformBroadcaster() {
    // 设置默认值并声明参数
    this->declare_parameter<double>("lidar_base_x", 0.0);
    this->declare_parameter<double>("lidar_base_y", 0.0);
    this->declare_parameter<double>("lidar_base_z", 0.0);
    this->declare_parameter<double>("lidar_base_yaw", 0.0);
    this->declare_parameter<double>("lidar_base_pitch", 0.0);
    this->declare_parameter<double>("lidar_base_roll", 0.0);
    this->declare_parameter<double>("radar_base_x", 0.0);
    this->declare_parameter<double>("radar_base_y", 0.0);
    this->declare_parameter<double>("radar_base_z", 0.0);
    this->declare_parameter<double>("radar_base_yaw", 0.0);
    this->declare_parameter<double>("radar_base_pitch", 0.0);
    this->declare_parameter<double>("radar_base_roll", 0.0);

    // 获取参数值
    lidar_base_x_ = this->get_parameter("lidar_base_x").as_double();
    lidar_base_y_ = this->get_parameter("lidar_base_y").as_double();
    lidar_base_z_ = this->get_parameter("lidar_base_z").as_double();
    lidar_base_yaw_ = DEG2RAD(this->get_parameter("lidar_base_yaw").as_double());
    lidar_base_pitch_ = DEG2RAD(this->get_parameter("lidar_base_pitch").as_double());
    lidar_base_roll_ = DEG2RAD(this->get_parameter("lidar_base_roll").as_double());
    radar_base_x_ = this->get_parameter("radar_base_x").as_double();
    radar_base_y_ = this->get_parameter("radar_base_y").as_double();
    radar_base_z_ = this->get_parameter("radar_base_z").as_double();
    radar_base_yaw_ = DEG2RAD(this->get_parameter("radar_base_yaw").as_double());
    radar_base_pitch_ = DEG2RAD(this->get_parameter("radar_base_pitch").as_double());
    radar_base_roll_ = DEG2RAD(this->get_parameter("radar_base_roll").as_double());

    // 打印输出
    RCLCPP_INFO(this->get_logger(), "Lidar base transform: x=%f, y=%f, z=%f, yaw=%f, pitch=%f, roll=%f", lidar_base_x_,
                lidar_base_y_, lidar_base_z_, lidar_base_yaw_, lidar_base_pitch_, lidar_base_roll_);
    RCLCPP_INFO(this->get_logger(), "Radar base transform: x=%f, y=%f, z=%f, yaw=%f, pitch=%f, roll=%f", radar_base_x_,
                radar_base_y_, radar_base_z_, radar_base_yaw_, radar_base_pitch_, radar_base_roll_);

    // 初始化静态变换发布器

    geometry_msgs::msg::TransformStamped lidar_to_base, radar_to_base;

    // 假设雷达的坐标系与车辆基准坐标系（base_link）的相对变换
    radar_to_base.header.stamp = this->now();
    radar_to_base.header.frame_id = "base_link";
    radar_to_base.child_frame_id = "os_radar";
    radar_to_base.transform.translation.x = radar_base_x_; // 在 x 轴上的平移
    radar_to_base.transform.translation.y = radar_base_y_; // 在 y 轴上的平移
    radar_to_base.transform.translation.z = radar_base_z_; // 在 z 轴上的平移
    tf2::Quaternion radar_q;
    radar_q.setRPY(radar_base_roll_, radar_base_pitch_, radar_base_yaw_); // 无旋转
    radar_to_base.transform.rotation.x = radar_q.x();
    radar_to_base.transform.rotation.y = radar_q.y();
    radar_to_base.transform.rotation.z = radar_q.z();
    radar_to_base.transform.rotation.w = radar_q.w();

    // 同样定义激光雷达到车辆基准坐标系的转换
    lidar_to_base.header.stamp = this->now();
    lidar_to_base.header.frame_id = "base_link";
    lidar_to_base.child_frame_id = "os_lidar";
    lidar_to_base.transform.translation.x = lidar_base_x_; // 激光雷达的 x 坐标
    lidar_to_base.transform.translation.y = lidar_base_y_; // 激光雷达的 y 坐标
    lidar_to_base.transform.translation.z = lidar_base_z_; // 激光雷达的 z 坐标
    tf2::Quaternion lidar_q;
    lidar_q.setRPY(lidar_base_roll_, lidar_base_pitch_, lidar_base_yaw_); // 假设无旋转
    lidar_to_base.transform.rotation.x = lidar_q.x();
    lidar_to_base.transform.rotation.y = lidar_q.y();
    lidar_to_base.transform.rotation.z = lidar_q.z();
    lidar_to_base.transform.rotation.w = lidar_q.w();

    // 广播两个静态转换
    static_broadcaster_->sendTransform(radar_to_base);
    static_broadcaster_->sendTransform(lidar_to_base);

    RCLCPP_INFO(this->get_logger(), "Published static transforms for radar and lidar");
}

void ObstaclesDetectionLidarNode::InitParameters() {
    // 设置默认值并声明参数
    this->declare_parameter<double>("max_height", 1.5);
    this->declare_parameter<double>("min_height", 0.2);
    this->declare_parameter<double>("vehicle_height", 1.5);
    this->declare_parameter<double>("vehicle_width", 2.0);
    this->declare_parameter<double>("vehicle_length", 4.0);
    this->declare_parameter<double>("radar_height", 1.0);
    this->declare_parameter<double>("cluster_tolerance", 0.1);
    this->declare_parameter<int>("min_cluster_size", 30);
    this->declare_parameter<int>("max_cluster_size", 20000);
    this->declare_parameter<float>("leaf_size", 0.05);
    this->declare_parameter<float>("roi_width", 1.0);
    this->declare_parameter<bool>("enable_visualization", true);
    this->declare_parameter<bool>("enable_calculate_process_time", false);
    this->declare_parameter<bool>("enable_use_roi", true);
    this->declare_parameter<bool>("enable_downsample", false);
    this->declare_parameter<int>("segment_ground_type", 1);
    this->declare_parameter<float>("plane_point_percent", 0.5);
    this->declare_parameter<bool>("is_use_front_lidar", false);
    this->declare_parameter<bool>("is_use_right_lidar", false);
    this->declare_parameter<bool>("is_use_left_lidar", false);
    this->declare_parameter<bool>("is_use_front_camera", false);
    this->declare_parameter<std::string>("front_lidar_topic", "drivers/front_lidar");
    this->declare_parameter<std::string>("left_lidar_topic", "drivers/left_lidar");
    this->declare_parameter<std::string>("right_lidar_topic", "drivers/right_lidar");
    this->declare_parameter<std::string>("front_camera_topic", "drivers/front_camera");
    this->declare_parameter<std::string>("frame_id", "base_link");

    // 获取参数值

    if (this->get_parameter("max_height", max_height_)) {
        RCLCPP_INFO(this->get_logger(), "use configed max_height: %f", this->get_parameter("max_height").as_double());
    } else {
        RCLCPP_INFO(this->get_logger(), " use default max_height: %f", 1.5);
    }

    max_height_ = this->get_parameter("max_height").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    vehicle_height_ = this->get_parameter("vehicle_height").as_double();
    vehicle_width_ = this->get_parameter("vehicle_width").as_double();
    vehicle_length_ = this->get_parameter("vehicle_length").as_double();
    radar_height_ = this->get_parameter("radar_height").as_double();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    cluster_min_size_ = this->get_parameter("min_cluster_size").as_int();
    cluster_max_size_ = this->get_parameter("max_cluster_size").as_int();
    leaf_size_ = this->get_parameter("leaf_size").as_double();
    roi_width_ = this->get_parameter("roi_width").as_double();
    enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
    enable_calculate_process_time_ = this->get_parameter("enable_calculate_process_time").as_bool();
    enable_use_roi_ = this->get_parameter("enable_use_roi").as_bool();
    enable_downsample_ = this->get_parameter("enable_downsample").as_bool();
    segment_ground_type_ = this->get_parameter("segment_ground_type").as_int();
    plane_point_percent_ = this->get_parameter("plane_point_percent").as_double();
    is_use_front_lidar_ = this->get_parameter("is_use_front_lidar").as_bool();
    is_use_right_lidar_ = this->get_parameter("is_use_right_lidar").as_bool();
    is_use_left_lidar_ = this->get_parameter("is_use_left_lidar").as_bool();
    is_use_front_camera_ = this->get_parameter("is_use_front_camera").as_bool();
    front_lidar_topic_ = this->get_parameter("front_lidar_topic").as_string();
    left_lidar_topic_ = this->get_parameter("left_lidar_topic").as_string();
    right_lidar_topic_ = this->get_parameter("right_lidar_topic").as_string();
    front_camera_topic_ = this->get_parameter("front_camera_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // 打印参数值
    RCLCPP_INFO(this->get_logger(), "max_height: %f", max_height_);
    RCLCPP_INFO(this->get_logger(), "min_height: %f", min_height_);
    RCLCPP_INFO(this->get_logger(), "vehicle_height: %f", vehicle_height_);
    RCLCPP_INFO(this->get_logger(), "vehicle_width: %f", vehicle_width_);
    RCLCPP_INFO(this->get_logger(), "vehicle_length: %f", vehicle_length_);
    RCLCPP_INFO(this->get_logger(), "radar_height: %f", radar_height_);
    RCLCPP_INFO(this->get_logger(), "cluster_tolerance: %f", cluster_tolerance_);
    RCLCPP_INFO(this->get_logger(), "cluster_min_size: %d", cluster_min_size_);
    RCLCPP_INFO(this->get_logger(), "cluster_max_size: %d", cluster_max_size_);
    RCLCPP_INFO(this->get_logger(), "leaf_size: %f", leaf_size_);
    RCLCPP_INFO(this->get_logger(), "roi_width: %f", roi_width_);
    RCLCPP_INFO(this->get_logger(), "enable_visualization: %d", enable_visualization_);
    RCLCPP_INFO(this->get_logger(), "enable_calculate_process_time: %d", enable_calculate_process_time_);
    RCLCPP_INFO(this->get_logger(), "enable_use_roi: %d", enable_use_roi_);
    RCLCPP_INFO(this->get_logger(), "enable_downsample: %d", enable_downsample_);
    RCLCPP_INFO(this->get_logger(), "segment_ground_type: %d", segment_ground_type_);
    RCLCPP_INFO(this->get_logger(), "plane_point_percent: %f", plane_point_percent_);
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());

    RCLCPP_INFO(this->get_logger(), "is_use_front_lidar: %d", is_use_front_lidar_);
    if (is_use_front_lidar_)
        RCLCPP_INFO(this->get_logger(), "front_lidar_topic: %s", front_lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "is_use_right_lidar: %d", is_use_right_lidar_);
    if (is_use_right_lidar_)
        RCLCPP_INFO(this->get_logger(), "right_lidar_topic: %s", right_lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "is_use_left_lidar: %d", is_use_left_lidar_);
    if (is_use_left_lidar_)
        RCLCPP_INFO(this->get_logger(), "left_lidar_topic: %s", left_lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "is_use_front_camera: %d", is_use_front_camera_);
    if (is_use_front_camera_)
        RCLCPP_INFO(this->get_logger(), "front_camera_topic: %s", front_camera_topic_.c_str());
}

/**
 * @brief 移除无效点
 *
 * @param cloud 点云数据
 */
void ObstaclesDetectionLidarNode::RemoveInvalidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    for (size_t i = 0; i < cloud->points.size(); i++) {
        auto &&pnt = cloud->points[i];
        bool is_invalid = std::isnan(pnt.x) || std::isnan(pnt.y) || std::isnan(pnt.z);
        if (!is_invalid) {
            indices.push_back(i);
        }
    }
    pcl::copyPointCloud(*cloud, indices, *cloud_filtered);
    *cloud = *cloud_filtered;
}

void ObstaclesDetectionLidarNode::FillAndPublishObstacleMarker(const bot_msg::msg::Obstacles &obstacle_array_msg,
                                                               int obstacles_type) {
    for (const auto &obstacle : obstacle_array_msg.obstacles) {
        auto &&marker = MakeObstacleMarker(obstacle, obstacles_type);
        marker_pub_->publish(marker);
    }
}

visualization_msgs::msg::Marker ObstaclesDetectionLidarNode::MakeObstacleMarker(int x, int y, int z, int width,
                                                                                int length, int height,
                                                                                int obstacles_type) {
    static int marker_id = 0;
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->get_clock()->now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0; // 无旋转
    marker.id = marker_id++;
    marker.ns = "obstacles";
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.lifetime = rclcpp::Duration(0.1);
    marker.scale.x = 0.02;
    if (obstacles_type == 1) {
        marker.color.r = 1.0;
        marker.color.b = 0.0;
    } else if (obstacles_type == 2) {
        marker.color.r = 0.0;
        marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    // 设定位置和大小
    // marker.pose.position.x = obstacle.position.x;
    // marker.pose.position.y = obstacle.position.y;
    // marker.pose.position.z = obstacle.position.z;

    double ctr_x = x;
    double ctr_y = y;
    double ctr_z = z;
    double scale_x = length;
    double scale_y = width;
    double scale_z = height;

    geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;
    // inten_position是目标物的坐标
    p1.x = ctr_x + scale_x / 2;
    p1.y = ctr_y - scale_y / 2;
    p1.z = ctr_z + scale_z / 2;
    p2.x = ctr_x + scale_x / 2;
    p2.y = ctr_y + scale_y / 2;
    p2.z = ctr_z + scale_z / 2;
    p3.x = ctr_x - scale_x / 2;
    p3.y = ctr_y + scale_y / 2;
    p3.z = ctr_z + scale_z / 2;
    p4.x = ctr_x - scale_x / 2;
    p4.y = ctr_y - scale_y / 2;
    p4.z = ctr_z + scale_z / 2;
    p5.x = ctr_x + scale_x / 2;
    p5.y = ctr_y - scale_y / 2;
    p5.z = ctr_z - scale_z / 2;
    p6.x = ctr_x + scale_x / 2;
    p6.y = ctr_y + scale_y / 2;
    p6.z = ctr_z - scale_z / 2;
    p7.x = ctr_x - scale_x / 2;
    p7.y = ctr_y + scale_y / 2;
    p7.z = ctr_z - scale_z / 2;
    p8.x = ctr_x - scale_x / 2;
    p8.y = ctr_y - scale_y / 2;
    p8.z = ctr_z - scale_z / 2;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p4);
    marker.points.push_back(p1);
    marker.points.push_back(p5);
    marker.points.push_back(p6);
    marker.points.push_back(p6);
    marker.points.push_back(p7);
    marker.points.push_back(p7);
    marker.points.push_back(p8);
    marker.points.push_back(p8);
    marker.points.push_back(p5);
    marker.points.push_back(p1);
    marker.points.push_back(p5);
    marker.points.push_back(p2);
    marker.points.push_back(p6);
    marker.points.push_back(p3);
    marker.points.push_back(p7);
    marker.points.push_back(p4);
    marker.points.push_back(p8);

    return marker;
}

visualization_msgs::msg::Marker
ObstaclesDetectionLidarNode::MakeObstacleMarker(const bot_msg::msg::ObstacleInfo &obstacle, int obstacles_type) {
    static int marker_id = 0;
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->get_clock()->now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0; // 无旋转
    marker.id = marker_id++;
    marker.ns = "obstacles";
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.lifetime = rclcpp::Duration(1);
    marker.scale.x = 0.02;
    if (obstacles_type == 1) {
        marker.color.r = 1.0;
        marker.color.b = 0.0;
    } else if (obstacles_type == 2) {
        marker.color.r = 0.0;
        marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    // 设定位置和大小
    marker.pose.position.x = obstacle.position_x;
    marker.pose.position.y = obstacle.position_y;
    marker.pose.position.z = obstacle.position_z;

    double ctr_x = obstacle.position_x;
    double ctr_y = obstacle.position_y;
    double ctr_z = obstacle.position_z;
    double scale_x = obstacle.length;
    double scale_y = obstacle.width;
    double scale_z = obstacle.height;

    geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;
    // inten_position是目标物的坐标
    p1.x = ctr_x + scale_x / 2;
    p1.y = ctr_y - scale_y / 2;
    p1.z = ctr_z + scale_z / 2;
    p2.x = ctr_x + scale_x / 2;
    p2.y = ctr_y + scale_y / 2;
    p2.z = ctr_z + scale_z / 2;
    p3.x = ctr_x - scale_x / 2;
    p3.y = ctr_y + scale_y / 2;
    p3.z = ctr_z + scale_z / 2;
    p4.x = ctr_x - scale_x / 2;
    p4.y = ctr_y - scale_y / 2;
    p4.z = ctr_z + scale_z / 2;
    p5.x = ctr_x + scale_x / 2;
    p5.y = ctr_y - scale_y / 2;
    p5.z = ctr_z - scale_z / 2;
    p6.x = ctr_x + scale_x / 2;
    p6.y = ctr_y + scale_y / 2;
    p6.z = ctr_z - scale_z / 2;
    p7.x = ctr_x - scale_x / 2;
    p7.y = ctr_y + scale_y / 2;
    p7.z = ctr_z - scale_z / 2;
    p8.x = ctr_x - scale_x / 2;
    p8.y = ctr_y - scale_y / 2;
    p8.z = ctr_z - scale_z / 2;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p4);
    marker.points.push_back(p1);
    marker.points.push_back(p5);
    marker.points.push_back(p6);
    marker.points.push_back(p6);
    marker.points.push_back(p7);
    marker.points.push_back(p7);
    marker.points.push_back(p8);
    marker.points.push_back(p8);
    marker.points.push_back(p5);
    marker.points.push_back(p1);
    marker.points.push_back(p5);
    marker.points.push_back(p2);
    marker.points.push_back(p6);
    marker.points.push_back(p3);
    marker.points.push_back(p7);
    marker.points.push_back(p4);
    marker.points.push_back(p8);

    return marker;
}

/**
 * @brief 显示点云的辅助函数
 *
 * @param cloud
 * @param title
 */
void ObstaclesDetectionLidarNode::VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                      const std::string &title) {
    pcl::visualization::CloudViewer viewer(title); // 为每个步骤创建一个新的 CloudViewer 实例
    viewer.showCloud(cloud);

    while (!viewer.wasStopped()) {
        // 阻塞，直到用户关闭窗口
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
/**
 * @brief 显示点云辅助函数, 不同阶段有不同颜色,仅在最后阶段显示窗口
 *
 * @param cloud
 * @param title
 * @param stage
 */
void ObstaclesDetectionLidarNode::VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                      const std::string &title, int stage) {
    // 使用静态指针使得所有阶段的点云都显示在同一个窗口中
    static pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
    viewer->setBackgroundColor(0.1, 0.1, 0.1); // 设置背景颜色

    // 根据不同的阶段设置不同颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 255); // 默认白色
    switch (stage) {
    case 0:
        color_handler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 255, 0, 0); // 红色
        break;
    case 1:
        color_handler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 0); // 绿色
        break;
    case 2:
        color_handler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 0, 255); // 蓝色
        break;
    case 3:
        color_handler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 255, 255, 0); // 黄色
        break;
    case 4:
        color_handler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 255); // 青色
        break;
    case 5:
        color_handler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 255, 0, 255); // 紫色
        break;
    }

    // 为每个阶段生成唯一的点云 ID
    std::string cloud_id = "cloud_" + std::to_string(stage);
    if (!viewer->updatePointCloud(cloud, color_handler, cloud_id)) {
        // 如果点云不存在，则添加新的点云到可视化器中
        viewer->addPointCloud(cloud, color_handler, cloud_id);
    }

    // 设置点云渲染属性，例如点大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id);

    // 刷新视图
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 防止刷新过快

    if (stage == 5) {
        // 阻塞，直到用户关闭窗口 (仅在最后一个阶段)
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

// 发布点云的辅助函数
void ObstaclesDetectionLidarNode::PublishPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher) {
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);
    output_cloud.header.stamp = this->get_clock()->now();
    output_cloud.header.frame_id = frame_id_; // 根据实际的坐标系设置
    publisher->publish(output_cloud);
}

void ObstaclesDetectionLidarNode::PointClould2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pnt_cloud) {

    auto node_timestamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "time stamp , %.2f", node_timestamp.seconds());
    auto cur_tt = node_timestamp;
    auto time_diff = cur_tt - node_timestamp;
    // 查询转换坐标关系
    geometry_msgs::msg::TransformStamped transformStamped;
    // try {
    //     // 查找 os_lidar 到 base_link 的变换
    //     transformStamped = tf_buffer_->lookupTransform("base_link", "os_lidar", tf2::TimePointZero);

    //     // 如果需要，将某些点云从 os_lidar 坐标系转换到 base_link 坐标系
    //     // 例如，可以使用 tf2::doTransform 来转换一个点
    //     // tf2::doTransform(original_point, transformed_point, transformStamped);
    // } catch (tf2::TransformException &ex) {
    //     RCLCPP_WARN(this->get_logger(), "Could not transform os_radar to base_link: %s", ex.what());
    //     return;
    // }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pnt_cloud, *cloud);
    if (enable_visualization_) {
        VisualizePointCloud(cloud, "Original Point Cloud");
    }
#if DEBUG_PUBLISH_POINT_CLOUD
    PublishPointCloud(cloud, original_cloud_pub_);
#endif
    // Apply transform to the point cloud
    // pcl_ros::transformPointCloud(*cloud, *cloud, transform_stamped);
    // 清理无效点
    RemoveInvalidPoints(cloud);

    // 1. Downsampling the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (enable_downsample_) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vg.filter(*cloud_filtered);
    } else {
        *cloud_filtered = *cloud;
    }

    if (enable_calculate_process_time_) {
        // 计算处理时间
        cur_tt = this->get_clock()->now();
        time_diff = cur_tt - node_timestamp;
        node_timestamp = cur_tt;
        RCLCPP_INFO(this->get_logger(), "downsampling process time , %.2f", time_diff.seconds());
    }

    if (enable_visualization_) {
        VisualizePointCloud(cloud_filtered, "Downsampled Point Cloud"); // 显示降采样后的点云
    }
#if DEBUG_PUBLISH_POINT_CLOUD
    PublishPointCloud(cloud_filtered, filtered_cloud_pub_);
#endif
    // 2. Clipping point cloud
    // 滤除车内点云
    std::vector<int> indices;
    for (size_t i = 0; i < cloud_filtered->points.size(); i++) {
        // 这里可以设置条件判断，判断某个点是否在障碍物的范围内
        bool is_in_vehicle = (abs(cloud_filtered->points[i].x) < vehicle_length_ / 2.0 &&
                              abs(cloud_filtered->points[i].y) < vehicle_width_ / 2.0);
        bool is_out_height = cloud_filtered->points[i].z > max_height_;
        if (!is_in_vehicle && !is_out_height) {
            indices.push_back(i);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clipped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_filtered, indices, *cloud_clipped);
    *cloud_filtered = *cloud_clipped;
    // ROI区域筛选
    if (enable_use_roi_) {
        indices.clear();
        for (size_t i = 0; i < cloud_filtered->points.size(); i++) {
            // 这里可以设置条件判断，判断某个点是否在障碍物的范围内
            bool is_in_roi = (abs(cloud_filtered->points[i].y) < (vehicle_width_ / 2.0 + roi_width_ / 2.0));
            if (is_in_roi) {
                indices.push_back(i);
            }
        }
        pcl::copyPointCloud(*cloud_filtered, indices, *cloud_clipped);
        *cloud_filtered = *cloud_clipped;
    }

    if (enable_calculate_process_time_) {
        // 计算处理时间
        cur_tt = this->get_clock()->now();
        time_diff = cur_tt - node_timestamp;
        node_timestamp = cur_tt;
        RCLCPP_INFO(this->get_logger(), "clipping process time , %.2f", time_diff.seconds());
    }

    if (enable_visualization_) {
        VisualizePointCloud(cloud_clipped, "Clipped Point Cloud"); // 显示裁剪后的点云
    }

    // 3. Segmenting the ground plane
    if (segment_ground_type_ == 1) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.08);

        int nr_points = (int)cloud_filtered->points.size();
        double not_plane_point_percent = 1 - plane_point_percent_;
        while (cloud_filtered->points.size() > not_plane_point_percent * nr_points) {
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_plane);

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_filtered);
        }
    } else { /// 使用 PassThrough 滤波器
        // 1. 创建 PassThrough 滤波器
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);

        // 2. 设置滤波范围，过滤掉 z 轴小于指定高度的点
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-vehicle_height_, std::numeric_limits<float>::max()); // 设置下限为 1.0，上限为无穷大

        // 3. 应用滤波器
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pass(new pcl::PointCloud<pcl::PointXYZ>());
        pass.filter(*cloud_filtered_pass);
        *cloud_filtered = *cloud_filtered_pass;
    }
    if (enable_calculate_process_time_) {
        cur_tt = this->get_clock()->now();
        time_diff = cur_tt - node_timestamp;
        node_timestamp = cur_tt;
        RCLCPP_INFO(this->get_logger(), "Segmenting process time , %.2f", time_diff.seconds());
    }
    if (enable_visualization_) {
        VisualizePointCloud(cloud_filtered, "Segmented Ground Point Cloud"); // 显示地面分割后的点云
    }
#if DEBUG_PUBLISH_POINT_CLOUD
    PublishPointCloud(cloud_filtered, filtered_cloud_pub_);
#endif
    // 4. Clustering to find obstacles
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); // 10 cm
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    // 创建一个新的点云，用于存储所有障碍物集群
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    // 5. Filling obstacle array message
    auto obstacle_array_msg = bot_msg::msg::Obstacles();
    int j = 0;
    for (const auto &cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices) {
            cloud_cluster->points.push_back((*cloud_filtered)[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // 将这个集群的点添加到 cloud_obstacles 中
        *cloud_obstacles += *cloud_cluster;
        bot_msg::msg::ObstacleInfo obstacle;
        // 计算质心点
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        // obstacle.position.x = centroid[0]; // Default values; you should compute them
        // obstacle.position.y = centroid[1];
        // obstacle.position.z = centroid[2];

        // 计算障碍物的长宽高
        pcl::PointXYZ min_point, max_point;
        pcl::getMinMax3D(*cloud_cluster, min_point, max_point);
        // obstacle.dimensions.x = max_point.x - min_point.x;
        // obstacle.dimensions.y = max_point.y - min_point.y;
        // obstacle.dimensions.z = max_point.z - min_point.z;

        // 判断障碍物类型
        // if (obstacle.dimensions.x > 5)
        //     obstacle.type = perception::msg::Obstacle::WALL;
        // else if (obstacle.dimensions.z > 0.1 && obstacle.dimensions.z < 0.5)
        //     obstacle.type = perception::msg::Obstacle::CONES;
        // else if (obstacle.dimensions.y > 0.1 && obstacle.dimensions.y < 1)
        //     obstacle.type = perception::msg::Obstacle::PEDESTRIAN;
        // else if (obstacle.dimensions.y > 1.5)
        //     obstacle.type = perception::msg::Obstacle::VEHICLE;
        // else
        //     obstacle.type = perception::msg::Obstacle::OTHERS;

        // 计算障碍物距离被测车辆的最近点
        pcl::PointXYZ closest_point;
        float min_distance = std::numeric_limits<float>::max();
        for (const auto &idx : cluster.indices) {
            pcl::PointXYZ point = (*cloud_filtered)[idx];
            float distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
            }
        }
        // obstacle.closest_point.x = closest_point.x;
        // obstacle.closest_point.y = closest_point.y;
        // obstacle.closest_point.z = closest_point.z;

        // 计算危险系数
        // float distance = std::sqrt(std::pow(obstacle.closest_point.x, 2) + std::pow(obstacle.closest_point.y, 2) +
        //                            std::pow(obstacle.closest_point.z, 2));
        // obstacle.danger_level = 1.0 / (distance + 0.1); // 距离越近，危险系数越大
        // // Add the obstacle to the array
        // obstacle_array_msg.obstacles.push_back(obstacle);

        j++;
        // RCLCPP_INFO(this->get_logger(),
        //             "Lidar object,position,x,%.2f,y,%.2f,z,%.2f,closest_point,x,%.2f,y,%.2f,z,%.2f,dimensions,"
        //             "x,%.2f,y,%.2f,z,%.2f,type,%d,danger_level,%.2f,count,%d",
        //             obstacle.position.x, obstacle.position.y, obstacle.position.z, obstacle.closest_point.x,
        //             obstacle.closest_point.y, obstacle.closest_point.z, obstacle.dimensions.x, obstacle.dimensions.y,
        //             obstacle.dimensions.z, obstacle.type, obstacle.danger_level, j);
    }
    if (enable_visualization_) {
        // 显示所有障碍物点云
        VisualizePointCloud(cloud_obstacles, "Clustered Obstacles Point Cloud");
    }
#if DEBUG_PUBLISH_POINT_CLOUD
    PublishPointCloud(cloud_obstacles, clustered_cloud_pub_);
#endif
    RCLCPP_INFO(this->get_logger(), "Lidar:Number of obstacles detected: %d", j);
    if (obstacle_array_msg.obstacles.size() > 0) {
        obstacle_array_msg.header.stamp = this->get_clock()->now();
        obstacle_array_msg.header.frame_id = frame_id_;
        obstacle_pub_->publish(obstacle_array_msg);
        FillAndPublishObstacleMarker(obstacle_array_msg, 1);
    }
    // 计算点云单次处理时间
    cur_tt = this->get_clock()->now();
    time_diff = cur_tt - node_timestamp;
    RCLCPP_INFO(this->get_logger(), "process time , %.2f", time_diff.seconds());
    return;
}

// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstaclesDetectionLidarNode>();
    RCLCPP_INFO(node->get_logger(), "Perception node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
