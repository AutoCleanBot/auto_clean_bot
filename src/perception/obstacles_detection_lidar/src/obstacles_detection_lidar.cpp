#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h> // Ensure you have this header included
// #include <pcl/segmentation/dbscan.h>
#include <chrono>
#include <obstacles_detection_lidar/obstacles_detection_lidar.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

ObstaclesDetectionLidarNode::ObstaclesDetectionLidarNode() : Node("perception_node"), last_marker_count_(0) {
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
    if (is_use_gnss_)
        gnss_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            gnss_topic_, 10, std::bind(&ObstaclesDetectionLidarNode::GNSSCallback, this, std::placeholders::_1));

    obstacle_pub_ = this->create_publisher<bot_msg::msg::Obstacles>("/perception/obstacles", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/perception/marker", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

#if DEBUG_PUBLISH_POINT_CLOUD
    // 在构造函数中初始化发布器
    original_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/original_cloud", 10);
    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/filtered_cloud", 10);
    ground_seg_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_seg_cloud", 10);
    clustered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/clustered_cloud", 10);
#endif


}

/**
 * @brief GNSS设备回调函数,注意GNSS设备消息的坐标系为东北天坐标系
 *
 * @param gnss_msg
 */
void ObstaclesDetectionLidarNode::GNSSCallback(const geometry_msgs::msg::PoseStamped::SharedPtr gnss_msg) {
    is_gnss_msg_received_ = true;
    gnss_msg_ = *gnss_msg;
}
/**
 * @brief 将障碍物的坐标从激光雷达坐标系转换到base坐标系
 *
 * @param obstacle
 */
void ObstaclesDetectionLidarNode::Obstacle2Base(bot_msg::msg::ObstacleInfo &obstacle) {
    RCLCPP_INFO(this->get_logger(), "Obstacle2Base: Starting coordinate transformation to base frame");

    // 将激光雷达坐标系的坐标数据转换到base坐标系下
    geometry_msgs::msg::PointStamped point_in_lidar;
    point_in_lidar.header.frame_id = front_lidar_frame_id_;
    point_in_lidar.point.x = obstacle.position_x;
    point_in_lidar.point.y = obstacle.position_y;
    point_in_lidar.point.z = obstacle.position_z;

    RCLCPP_INFO(this->get_logger(), "Obstacle2Base: Input point in lidar frame: (%.3f, %.3f, %.3f)",
                point_in_lidar.point.x, point_in_lidar.point.y, point_in_lidar.point.z);

    geometry_msgs::msg::PointStamped point_in_base;
    try {
        // 注意这里的 base_link 表示的是车辆的相对原点坐标系
        RCLCPP_INFO(this->get_logger(), "Obstacle2Base: Transforming from %s to %s", front_lidar_frame_id_.c_str(),
                    base_frame_id_.c_str());
        point_in_base = tf_buffer_->transform(point_in_lidar, base_frame_id_);
        RCLCPP_INFO(this->get_logger(), "Obstacle2Base: Transformed point in base frame: (%.3f, %.3f, %.3f)",
                    point_in_base.point.x, point_in_base.point.y, point_in_base.point.z);


        // 更新障碍物坐标为base坐标系下的坐标
        obstacle.position_x = point_in_base.point.x;
        obstacle.position_y = point_in_base.point.y;
        obstacle.position_z = point_in_base.point.z;

        RCLCPP_INFO(this->get_logger(), "Obstacle2Base: Coordinate transformation completed successfully");
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Obstacle2Base: Transform error: %s", ex.what());
        // 转换失败时保持原坐标不变
        RCLCPP_WARN(this->get_logger(), "Obstacle2Base: Keeping original coordinates due to transform failure");
    }
}

/**
 * @brief 将障碍物的坐标系转换到ENU坐标系,也即MAP坐标系
 *
 * @param obstacle
 */
void ObstaclesDetectionLidarNode::Obstacle2ENU(bot_msg::msg::ObstacleInfo &obstacle) {

    // 1. 将base坐标系下的坐标数据转换到map坐标系下
    geometry_msgs::msg::PointStamped point_in_base;
    point_in_base.header.frame_id = base_frame_id_;
    point_in_base.header.stamp = this->now();
    point_in_base.point.x = obstacle.position_x;
    point_in_base.point.y = obstacle.position_y;
    point_in_base.point.z = obstacle.position_z;

    RCLCPP_INFO(this->get_logger(), "Obstacle2ENU: Input point in base frame: (%.3f, %.3f, %.3f)",
                point_in_base.point.x, point_in_base.point.y, point_in_base.point.z);

    // 目标坐标系为map（与ENU一致）
    const std::string target_frame = "map";
    
    try {
        // 使用tf2进行坐标转换，从base_link到map
        RCLCPP_INFO(this->get_logger(), "Obstacle2ENU: Transforming from %s to %s", 
                    base_frame_id_.c_str(), target_frame.c_str());
        
        // 检查变换是否可用
        if (!tf_buffer_->canTransform(target_frame, base_frame_id_, tf2::TimePointZero)) {
            RCLCPP_WARN(this->get_logger(), "Obstacle2ENU: Transform from %s to %s not available yet. Waiting...",
                        base_frame_id_.c_str(), target_frame.c_str());
            
            // 尝试等待变换可用（最多等待1秒）
            if (!tf_buffer_->canTransform(target_frame, base_frame_id_, tf2::TimePointZero, 
                                         tf2::durationFromSec(1.0))) {
                RCLCPP_ERROR(this->get_logger(), "Obstacle2ENU: Transform not available after waiting. Keeping base coordinates.");
                return;  // 保持base坐标系下的坐标
            }
        }
        
        // 执行坐标变换
        geometry_msgs::msg::PointStamped point_in_map;
        point_in_map = tf_buffer_->transform(point_in_base, target_frame);
        
        RCLCPP_INFO(this->get_logger(), "Obstacle2ENU: Transformed point in map frame: (%.3f, %.3f, %.3f)",
                    point_in_map.point.x, point_in_map.point.y, point_in_map.point.z);

        // 更新障碍物坐标为map坐标系下的坐标
        obstacle.position_x = point_in_map.point.x;
        obstacle.position_y = point_in_map.point.y;
        obstacle.position_z = point_in_map.point.z;

        RCLCPP_INFO(this->get_logger(), "Obstacle2ENU: Coordinate transformation completed successfully");
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Obstacle2ENU: Transform error: %s", ex.what());
        // 转换失败时保持base坐标系下的坐标不变
        RCLCPP_WARN(this->get_logger(), "Obstacle2ENU: Keeping base coordinates due to transform failure");
    }
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
    this->declare_parameter<float>("leaf_size_x", 0.05);
    this->declare_parameter<float>("leaf_size_y", 0.05);
    this->declare_parameter<float>("leaf_size_z", 0.05);
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
    this->declare_parameter<bool>("is_use_gnss", false);
    this->declare_parameter<std::string>("front_lidar_topic", "drivers/front_lidar");
    this->declare_parameter<std::string>("left_lidar_topic", "drivers/left_lidar");
    this->declare_parameter<std::string>("right_lidar_topic", "drivers/right_lidar");
    this->declare_parameter<std::string>("gnss_topic", "gnss/pose");
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<std::string>("gnss_frame_id", "gnss_link");
    this->declare_parameter<std::string>("front_lidar_frame_id", "front_lidar");
    this->declare_parameter<std::string>("left_lidar_frame_id", "left_lidar");
    this->declare_parameter<std::string>("right_lidar_frame_id", "right_lidar");
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<std::string>("map_frame_id", "map");

    // 获取参数值

    max_height_ = this->get_parameter("max_height").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    vehicle_height_ = this->get_parameter("vehicle_height").as_double();
    vehicle_width_ = this->get_parameter("vehicle_width").as_double();
    vehicle_length_ = this->get_parameter("vehicle_length").as_double();
    radar_height_ = this->get_parameter("radar_height").as_double();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    cluster_min_size_ = this->get_parameter("min_cluster_size").as_int();
    cluster_max_size_ = this->get_parameter("max_cluster_size").as_int();
    leaf_size_x_ = this->get_parameter("leaf_size_x").as_double();
    leaf_size_y_ = this->get_parameter("leaf_size_y").as_double();
    leaf_size_z_ = this->get_parameter("leaf_size_z").as_double();
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
    front_lidar_topic_ = this->get_parameter("front_lidar_topic").as_string();
    left_lidar_topic_ = this->get_parameter("left_lidar_topic").as_string();
    right_lidar_topic_ = this->get_parameter("right_lidar_topic").as_string();

    gnss_frame_id_ = this->get_parameter("gnss_frame_id").as_string();
    front_lidar_frame_id_ = this->get_parameter("front_lidar_frame_id").as_string();
    left_lidar_frame_id_ = this->get_parameter("left_lidar_frame_id").as_string();
    right_lidar_frame_id_ = this->get_parameter("right_lidar_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    map_frame_id_ = this->get_parameter("map_frame_id").as_string();
    is_use_gnss_ = this->get_parameter("is_use_gnss").as_bool();
    gnss_topic_ = this->get_parameter("gnss_topic").as_string();
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
    RCLCPP_INFO(this->get_logger(), "leaf_size: %f", leaf_size_x_);
    RCLCPP_INFO(this->get_logger(), "leaf_size: %f", leaf_size_y_);
    RCLCPP_INFO(this->get_logger(), "leaf_size: %f", leaf_size_z_);
    RCLCPP_INFO(this->get_logger(), "roi_width: %f", roi_width_);
    RCLCPP_INFO(this->get_logger(), "enable_visualization: %d", enable_visualization_);
    RCLCPP_INFO(this->get_logger(), "enable_calculate_process_time: %d", enable_calculate_process_time_);
    RCLCPP_INFO(this->get_logger(), "enable_use_roi: %d", enable_use_roi_);
    RCLCPP_INFO(this->get_logger(), "enable_downsample: %d", enable_downsample_);
    RCLCPP_INFO(this->get_logger(), "segment_ground_type: %d", segment_ground_type_);
    RCLCPP_INFO(this->get_logger(), "plane_point_percent: %f", plane_point_percent_);
    RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "is_use_gnss: %d", is_use_gnss_);
    RCLCPP_INFO(this->get_logger(), "is_use_front_lidar: %d", is_use_front_lidar_);
    if (is_use_front_lidar_)
        RCLCPP_INFO(this->get_logger(), "front_lidar_topic: %s", front_lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "is_use_right_lidar: %d", is_use_right_lidar_);
    if (is_use_right_lidar_)
        RCLCPP_INFO(this->get_logger(), "right_lidar_topic: %s", right_lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "is_use_left_lidar: %d", is_use_left_lidar_);
    if (is_use_left_lidar_)
        RCLCPP_INFO(this->get_logger(), "left_lidar_topic: %s", left_lidar_topic_.c_str());
    if (is_use_gnss_)
        RCLCPP_INFO(this->get_logger(), "gnss_topic: %s", gnss_topic_.c_str());
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

/**
 * @brief 移除车辆范围内的点
 *
 * @param cloud 输入输出点云
 */
void ObstaclesDetectionLidarNode::RemoveVehiclePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    RCLCPP_INFO(this->get_logger(), "RemoveVehiclePoints: Starting vehicle points removal with %zu input points",
                cloud->points.size());

    if (cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "RemoveVehiclePoints: Input cloud is empty, skipping");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "RemoveVehiclePoints: Using vehicle dimensions - length: %.2f, width: %.2f",
                vehicle_length_, vehicle_width_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 激光雷达不在车辆正中心
    const double lidar_vehicle_front_offset_x = 2.7;
    const double lidar_vehicle_back_offset_x = vehicle_length_ - lidar_vehicle_front_offset_x;

    // 假设车辆中心在原点，车辆的长度方向沿X轴，宽度方向沿Y轴
    // double half_length = vehicle_length_ / 2.0;
    double half_width = vehicle_width_ / 2.0;

    for (const auto &point : cloud->points) {
        // 检查点是否在车辆范围内

        
        bool is_inside_vehicle = (std::abs(point.y) <= half_width) && (
            point.x <= lidar_vehicle_front_offset_x || point.x >= -lidar_vehicle_back_offset_x
        );

        // 如果点不在车辆范围内，则保留
        if (!is_inside_vehicle) {
            filtered_cloud->points.push_back(point);
        }
    }

    // 更新点云属性
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    size_t original_size = cloud->points.size();
    size_t removed_count = original_size - filtered_cloud->points.size();
    *cloud = *filtered_cloud;

    RCLCPP_INFO(this->get_logger(), "RemoveVehiclePoints: Completed - removed %zu vehicle points, %zu points remaining",
                removed_count, cloud->points.size());
}

void ObstaclesDetectionLidarNode::FillAndPublishObstacleMarker(const bot_msg::msg::Obstacles &obstacle_array_msg,
                                                               int obstacles_type) {


    // 然后发布新的marker，使用固定的ID
    int marker_id = 0;
    for (const auto &obstacle : obstacle_array_msg.obstacles) {
        auto &&marker = MakeObstacleMarker(obstacle, obstacles_type, marker_id);
        marker_pub_->publish(marker);
        marker_id++;
    }

    // 更新marker计数
    last_marker_count_ = obstacle_array_msg.obstacles.size();
}

void ObstaclesDetectionLidarNode::ClearAllObstacleMarkers() {
    // 发送DELETE action来清除所有之前的marker
    for (int i = 0; i < last_marker_count_; ++i) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = base_frame_id_.empty() ? "base_link" : base_frame_id_;
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.ns = "obstacles";
        delete_marker.id = i;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_pub_->publish(delete_marker);
    }
}

visualization_msgs::msg::Marker ObstaclesDetectionLidarNode::MakeObstacleMarker(int x, int y, int z, int width,
                                                                                int length, int height,
                                                                                int obstacles_type) {
    try {
        // 创建线段标记
        auto marker = visualization_msgs::msg::Marker();

        // 设置基本属性
        marker.header.frame_id = base_frame_id_.empty() ? "base_link" : base_frame_id_;
        marker.header.stamp = this->get_clock()->now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0; // 无旋转
        marker.id = 0;
        marker.ns = "obstacles";
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.lifetime = rclcpp::Duration(0.1);
        marker.scale.x = 0.02;

        // 设置颜色
        marker.color.a = 0.7; // 半透明
        if (obstacles_type == 1) {
            // LiDAR检测 - 红色
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (obstacles_type == 2) {
            // Camera检测 - 蓝色
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else {
            // Radar检测 - 绿色
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        // 计算中心点和尺寸
        double ctr_x = static_cast<double>(x);
        double ctr_y = static_cast<double>(y);
        double ctr_z = static_cast<double>(z);
        double scale_x = std::max(0.1, static_cast<double>(length));
        double scale_y = std::max(0.1, static_cast<double>(width));
        double scale_z = std::max(0.1, static_cast<double>(height));

        // 创建8个顶点
        geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;

        // 定义顶点坐标
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

        // 预先分配空间
        marker.points.reserve(24);

        // 添加线段 (按照边的顺序添加点)
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

        RCLCPP_INFO(this->get_logger(), "Successfully created marker");
        return marker;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error in MakeObstacleMarker: %s", e.what());

        // 返回一个简单的默认标记
        visualization_msgs::msg::Marker default_marker;
        default_marker.header.frame_id = base_frame_id_.empty() ? "base_link" : base_frame_id_;
        default_marker.header.stamp = this->get_clock()->now();
        default_marker.id = 0;
        default_marker.ns = "error";
        default_marker.type = visualization_msgs::msg::Marker::SPHERE;
        default_marker.action = visualization_msgs::msg::Marker::ADD;
        default_marker.pose.position.x = 0;
        default_marker.pose.position.y = 0;
        default_marker.pose.position.z = 0;
        default_marker.pose.orientation.w = 1.0;
        default_marker.scale.x = 0.2;
        default_marker.scale.y = 0.2;
        default_marker.scale.z = 0.2;
        default_marker.color.r = 1.0;
        default_marker.color.a = 1.0;

        return default_marker;
    }
}

visualization_msgs::msg::Marker
ObstaclesDetectionLidarNode::MakeObstacleMarker(const bot_msg::msg::ObstacleInfo &obstacle, int obstacles_type,
                                                int marker_id) {
    // 创建立方体标记
    auto marker = visualization_msgs::msg::Marker();
    if(is_use_gnss_){
        marker.header.frame_id = map_frame_id_;
    }else{
        marker.header.frame_id = base_frame_id_;
    }
    marker.header.stamp = this->get_clock()->now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0; // 无旋转
    marker.id = marker_id;
    marker.ns = "obstacles";
    marker.type = visualization_msgs::msg::Marker::CUBE; // 使用立方体类型
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);               // 设置较短的生命周期

    // 设置颜色
    marker.color.a = 0.7; // 半透明
    if (obstacles_type == 1) {
        // LiDAR检测 - 蓝色
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    } else if (obstacles_type == 2) {
        // Camera检测 - 绿色
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    } else if (obstacles_type == 3) {
        // Radar检测 - 黄色
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    } else {
        // 默认 - 红色
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }

    // 设置立方体中心位置
    marker.pose.position.x = obstacle.position_x;
    marker.pose.position.y = obstacle.position_y;
    marker.pose.position.z = obstacle.position_z + obstacle.height / 2.0; // 立方体中心位置

    // 设置立方体尺寸
    marker.scale.x = obstacle.length;
    marker.scale.y = obstacle.width;
    marker.scale.z = obstacle.height;

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
    
    // 将frame_id设置为map而不是frame_id_，使点云在RViz中显示在map坐标系下
    output_cloud.header.frame_id = "map";
    
    publisher->publish(output_cloud);
}

/**
 * @brief 核心的回调函数, 目前的是接受到就处理
 *
 * @param pnt_cloud
 */
void ObstaclesDetectionLidarNode::PointClould2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pnt_cloud) {
    RCLCPP_INFO(this->get_logger(), "PointClould2Callback entered.");

    // 处理点云数据
    bot_msg::msg::Obstacles obstacles_msg = ProcessPointCloud(pnt_cloud);

    // 发布障碍物消息
    obstacle_pub_->publish(obstacles_msg);
    // 发布障碍物可视化标记
    FillAndPublishObstacleMarker(obstacles_msg, 2);

    RCLCPP_INFO(this->get_logger(), "PointClould2Callback exiting safely.");
    return;
}

/**
 * @brief 地面滤除主函数，根据配置选择不同的算法
 *
 * @param cloud 输入点云
 * @param ground_cloud 输出地面点云
 * @param non_ground_cloud 输出非地面点云
 */
void ObstaclesDetectionLidarNode::FilterGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud) {
    RCLCPP_INFO(this->get_logger(), "FilterGroundPoints: Starting ground filtering with %zu input points",
                cloud->points.size());
    RCLCPP_INFO(this->get_logger(), "FilterGroundPoints: Using ground segmentation type: %d", segment_ground_type_);

    if (segment_ground_type_ == 1) {
        // 使用RANSAC算法进行地面分割
        RCLCPP_INFO(this->get_logger(), "FilterGroundPoints: Using RANSAC-based ground filtering");
        FilterGroundByRANSAC(cloud, ground_cloud, non_ground_cloud);
    } else {
        // 使用基于高度的简单地面分割
        RCLCPP_INFO(this->get_logger(), "FilterGroundPoints: Using height-based ground filtering");
        FilterGroundByHeight(cloud, ground_cloud, non_ground_cloud);
    }

    RCLCPP_INFO(this->get_logger(), "FilterGroundPoints: Completed - Ground: %zu, Non-ground: %zu",
                ground_cloud->points.size(), non_ground_cloud->points.size());
}

/**
 * @brief 基于高度的地面滤除算法
 *
 * @param cloud 输入点云
 * @param ground_cloud 输出地面点云
 * @param non_ground_cloud 输出非地面点云
 */
void ObstaclesDetectionLidarNode::FilterGroundByHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud) {
    ground_cloud->clear();
    non_ground_cloud->clear();

    for (const auto &point : cloud->points) {
        if (point.z < min_height_) {
            ground_cloud->points.push_back(point);
        } else {
            non_ground_cloud->points.push_back(point);
        }
    }

    // 设置点云属性
    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    non_ground_cloud->width = non_ground_cloud->points.size();
    non_ground_cloud->height = 1;
    non_ground_cloud->is_dense = true;

    RCLCPP_DEBUG(this->get_logger(), "Height-based ground filtering: %zu ground points, %zu non-ground points",
                 ground_cloud->points.size(), non_ground_cloud->points.size());
}

/**
 * @brief 基于RANSAC的地面滤除算法
 *
 * @param cloud 输入点云
 * @param ground_cloud 输出地面点云
 * @param non_ground_cloud 输出非地面点云
 */
void ObstaclesDetectionLidarNode::FilterGroundByRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud) {
    ground_cloud->clear();
    non_ground_cloud->clear();

    if (cloud->points.empty()) {
        return;
    }

    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // 设置分割参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.2); // 距离阈值，可以作为参数配置

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
        *non_ground_cloud = *cloud;
        return;
    }

    // 检查平面是否接近水平（地面）
    // 地面法向量应该接近 (0, 0, 1)
    double normal_z = coefficients->values[2];
    if (std::abs(normal_z) < 0.8) { // 法向量z分量应该接近1
        RCLCPP_DEBUG(this->get_logger(), "Detected plane is not horizontal enough (normal_z: %f)", normal_z);
        *non_ground_cloud = *cloud;
        return;
    }

    // 检查内点比例
    double inlier_ratio = static_cast<double>(inliers->indices.size()) / cloud->points.size();
    if (inlier_ratio < plane_point_percent_) {
        RCLCPP_DEBUG(this->get_logger(), "Ground plane inlier ratio too low: %f", inlier_ratio);
        *non_ground_cloud = *cloud;
        return;
    }

    // 提取地面点和非地面点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // 提取地面点
    extract.setNegative(false);
    extract.filter(*ground_cloud);

    // 提取非地面点
    extract.setNegative(true);
    extract.filter(*non_ground_cloud);

    RCLCPP_DEBUG(this->get_logger(), "RANSAC ground filtering: %zu ground points, %zu non-ground points",
                 ground_cloud->points.size(), non_ground_cloud->points.size());
}

/**
 * @brief ROI区域滤波，过滤掉感兴趣区域外的点
 *
 * @param cloud 输入输出点云
 */
void ObstaclesDetectionLidarNode::FilterROI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    RCLCPP_INFO(this->get_logger(), "FilterROI: Starting ROI filtering with %zu input points", cloud->points.size());

    if (!enable_use_roi_) {
        RCLCPP_INFO(this->get_logger(), "FilterROI: ROI filtering disabled, skipping");
        return;
    }

    if (cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "FilterROI: Input cloud is empty, skipping");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "FilterROI: Using ROI width: %f, height range: [%f, %f]", roi_width_, min_height_,
                max_height_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &point : cloud->points) {
        // 检查点是否在ROI范围内
        // 假设车辆在原点，ROI为以车辆为中心的矩形区域
        if (std::abs(point.x) <= roi_width_ && std::abs(point.y) <= roi_width_) {
            filtered_cloud->points.push_back(point);
        }
    }

    // 更新点云属性
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    size_t original_size = cloud->points.size();
    *cloud = *filtered_cloud;

    RCLCPP_INFO(this->get_logger(), "FilterROI: Completed - %zu -> %zu points remaining", original_size,
                cloud->points.size());
}

/**
 * @brief 体素下采样，减少点云密度
 *
 * @param cloud 输入输出点云
 */
void ObstaclesDetectionLidarNode::DownsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    RCLCPP_INFO(this->get_logger(), "DownsampleCloud: Starting downsampling with %zu input points",
                cloud->points.size());

    if (!enable_downsample_) {
        RCLCPP_INFO(this->get_logger(), "DownsampleCloud: Downsampling disabled, skipping");
        return;
    }

    if (cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "DownsampleCloud: Input cloud is empty, skipping");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "DownsampleCloud: Using leaf size: %f, %f, %f", leaf_size_x_, leaf_size_y_,
                leaf_size_z_);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*downsampled_cloud);

    size_t original_size = cloud->points.size();
    *cloud = *downsampled_cloud;

    RCLCPP_INFO(this->get_logger(), "DownsampleCloud: Completed - %zu -> %zu points", original_size,
                cloud->points.size());
}

/**
 * @brief 欧几里得聚类算法，识别独立的障碍物
 *
 * @param cloud 输入点云
 * @return std::vector<pcl::PointIndices> 聚类结果
 */
std::vector<pcl::PointIndices> ObstaclesDetectionLidarNode::ClusterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    RCLCPP_INFO(this->get_logger(), "ClusterPoints: Starting clustering with %zu input points", cloud->points.size());

    std::vector<pcl::PointIndices> cluster_indices;

    if (cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "ClusterPoints: Input cloud is empty, returning empty clusters");
        return cluster_indices;
    }

    RCLCPP_INFO(this->get_logger(), "ClusterPoints: Using parameters - tolerance: %f, min_size: %d, max_size: %d",
                cluster_tolerance_, cluster_min_size_, cluster_max_size_);

    // 创建KdTree对象用于搜索
    RCLCPP_INFO(this->get_logger(), "ClusterPoints: Creating KdTree for search...");
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // 创建欧几里得聚类提取对象
    RCLCPP_INFO(this->get_logger(), "ClusterPoints: Setting up EuclideanClusterExtraction...");
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); // 聚类距离阈值
    ec.setMinClusterSize(cluster_min_size_);    // 最小聚类点数
    ec.setMaxClusterSize(cluster_max_size_);    // 最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // 执行聚类
    RCLCPP_INFO(this->get_logger(), "ClusterPoints: Executing clustering...");
    ec.extract(cluster_indices);

    RCLCPP_INFO(this->get_logger(), "ClusterPoints: Completed - found %zu clusters", cluster_indices.size());

    return cluster_indices;
}

/**
 * @brief 从聚类中提取障碍物信息
 *
 * @param cluster 聚类点云
 * @param id 障碍物ID
 * @return bot_msg::msg::ObstacleInfo 障碍物信息
 */
bot_msg::msg::ObstacleInfo
ObstaclesDetectionLidarNode::ExtractObstacleInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, uint32_t id) {
    RCLCPP_INFO(this->get_logger(), "ExtractObstacleInfo: Starting extraction for obstacle %u with %zu points", id,
                cluster->points.size());

    bot_msg::msg::ObstacleInfo obstacle;

    if (cluster->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "ExtractObstacleInfo: Cluster is empty for obstacle %u", id);
        return obstacle;
    }

    // 计算边界框
    RCLCPP_INFO(this->get_logger(), "ExtractObstacleInfo: Computing bounding box for obstacle %u...", id);
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cluster, min_point, max_point);

    // 计算中心点
    RCLCPP_INFO(this->get_logger(), "ExtractObstacleInfo: Computing centroid for obstacle %u...", id);
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (const auto &point : cluster->points) {
        centroid.add(point);
    }
    pcl::PointXYZ center;
    centroid.get(center);

    // 设置障碍物信息
    obstacle.id = id;
    obstacle.position_x = center.x;
    obstacle.position_y = center.y;
    obstacle.position_z = center.z;

    // 计算尺寸
    obstacle.length = max_point.x - min_point.x;
    obstacle.width = max_point.y - min_point.y;
    obstacle.height = max_point.z - min_point.z;

    // 初始化速度信息（静态障碍物）
    obstacle.velocity_x = 0.0;
    obstacle.velocity_y = 0.0;
    obstacle.velocity = 0.0;
    obstacle.heading = 0.0;

    // 设置障碍物类型和状态
    obstacle.type = 1;   // 默认类型
    obstacle.status = 1; // 活跃状态

    // 转换坐标系
    RCLCPP_INFO(this->get_logger(), "ExtractObstacleInfo: Converting coordinates for obstacle %u...", id);
    // 首先转换到base坐标系
    RCLCPP_INFO(this->get_logger(), "ExtractObstacleInfo: Converting to base coordinates for obstacle %u", id);
    Obstacle2Base(obstacle);
    if (is_use_gnss_ ) {
        // 如果启用了GNSS，转换到ENU坐标系
        RCLCPP_INFO(this->get_logger(), "ExtractObstacleInfo: Converting to ENU coordinates for obstacle %u", id);
        Obstacle2ENU(obstacle);
    }

    RCLCPP_INFO(this->get_logger(),
                "ExtractObstacleInfo: Completed obstacle %u: pos(%.2f, %.2f, %.2f), size(%.2f, %.2f, %.2f)", id,
                obstacle.position_x, obstacle.position_y, obstacle.position_z, obstacle.length, obstacle.width,
                obstacle.height);

    return obstacle;
}

/**
 * @brief 完整的点云处理流程
 *
 * @param pnt_cloud 输入点云消息
 * @return bot_msg::msg::Obstacles 障碍物消息
 */
bot_msg::msg::Obstacles
ObstaclesDetectionLidarNode::ProcessPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pnt_cloud) {
    bot_msg::msg::Obstacles obstacles_msg;
    obstacles_msg.header = pnt_cloud->header;
    if(is_use_gnss_){
        obstacles_msg.header.frame_id = map_frame_id_;
    }else{
        obstacles_msg.header.frame_id = base_frame_id_;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "=== Starting point cloud processing ===");

    // 1. 转换ROS消息到PCL点云
    RCLCPP_INFO(this->get_logger(), "Step 1: Converting ROS message to PCL point cloud...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pnt_cloud, *cloud);

    RCLCPP_INFO(this->get_logger(), "Step 1 completed: Original cloud size: %zu points", cloud->points.size());

    // 发布原始点云（调试用）
#if DEBUG_PUBLISH_POINT_CLOUD
    PublishPointCloud(cloud, original_cloud_pub_);
    RCLCPP_INFO(this->get_logger(), "Publishing original point cloud for debugging...");
    RCLCPP_INFO(this->get_logger(), "Original point cloud published successfully");
#endif

    // 2. 移除无效点
    RCLCPP_INFO(this->get_logger(), "Step 2: Removing invalid points...");
    RemoveInvalidPoints(cloud);
    RCLCPP_INFO(this->get_logger(), "Step 2 completed: Cloud size after removing invalid points: %zu",
                cloud->points.size());

    // 3. 移除车辆范围内的点
    RCLCPP_INFO(this->get_logger(), "Step 3: Removing vehicle points...");
    RemoveVehiclePoints(cloud);
    RCLCPP_INFO(this->get_logger(), "Step 3 completed: Cloud size after removing vehicle points: %zu",
                cloud->points.size());

    // 可视化
    if (enable_visualization_) {
        RCLCPP_INFO(this->get_logger(), "Starting visualization...");
        VisualizePointCloud(cloud, "Vehicle removed Point Cloud");
        RCLCPP_INFO(this->get_logger(), "Visualization completed");
    }
    

    // 4. ROI滤波
    RCLCPP_INFO(this->get_logger(), "Step 4: Applying ROI filtering...");
    FilterROI(cloud);
    RCLCPP_INFO(this->get_logger(), "Step 4 completed: Cloud size after ROI filtering: %zu", cloud->points.size());

    // 5. 下采样
    RCLCPP_INFO(this->get_logger(), "Step 5: Applying downsampling...");
    DownsampleCloud(cloud);
    RCLCPP_INFO(this->get_logger(), "Step 5 completed: Cloud size after downsampling: %zu", cloud->points.size());

    // 发布滤波后的点云（调试用）
#if DEBUG_PUBLISH_POINT_CLOUD
    RCLCPP_INFO(this->get_logger(), "Publishing filtered point cloud for debugging...");
    PublishPointCloud(cloud, filtered_cloud_pub_);
    RCLCPP_INFO(this->get_logger(), "Filtered point cloud published successfully");
#endif

    // 6. 地面滤除
    RCLCPP_INFO(this->get_logger(), "Step 6: Applying ground filtering...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    FilterGroundPoints(cloud, ground_cloud, non_ground_cloud);
    RCLCPP_INFO(this->get_logger(), "Step 6 completed: Ground points: %zu, Non-ground points: %zu",
                ground_cloud->points.size(), non_ground_cloud->points.size());

    // 发布地面分割结果（调试用）
#if DEBUG_PUBLISH_POINT_CLOUD
    RCLCPP_INFO(this->get_logger(), "Publishing ground segmentation result for debugging...");
    PublishPointCloud(ground_cloud, ground_seg_cloud_pub_);
    RCLCPP_INFO(this->get_logger(), "Ground segmentation result published successfully");
#endif

    if (non_ground_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No non-ground points found, returning empty obstacles message");
        return obstacles_msg;
    }

    // 7. 聚类
    RCLCPP_INFO(this->get_logger(), "Step 7: Applying clustering...");
    std::vector<pcl::PointIndices> cluster_indices = ClusterPoints(non_ground_cloud);
    RCLCPP_INFO(this->get_logger(), "Step 7 completed: Found %zu clusters", cluster_indices.size());

    // 8. 提取障碍物信息
    RCLCPP_INFO(this->get_logger(), "Step 8: Extracting obstacle information...");
    uint32_t obstacle_id = 0;
    for (const auto &cluster_idx : cluster_indices) {
        RCLCPP_INFO(this->get_logger(), "Processing cluster %u with %zu points", obstacle_id,
                    cluster_idx.indices.size());

        // 创建聚类点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*non_ground_cloud, cluster_idx, *cluster_cloud);

        // 提取障碍物信息
        RCLCPP_INFO(this->get_logger(), "Extracting obstacle info for cluster %u...", obstacle_id);
        bot_msg::msg::ObstacleInfo obstacle = ExtractObstacleInfo(cluster_cloud, obstacle_id++);
        obstacles_msg.obstacles.push_back(obstacle);
        RCLCPP_INFO(this->get_logger(), "Obstacle %u extracted successfully", obstacle_id - 1);
    }
    RCLCPP_INFO(this->get_logger(), "Step 8 completed: Extracted %zu obstacles", obstacles_msg.obstacles.size());

    // 发布聚类结果（调试用）
#if DEBUG_PUBLISH_POINT_CLOUD
    RCLCPP_INFO(this->get_logger(), "Publishing clustered point cloud for debugging...");
    PublishPointCloud(non_ground_cloud, clustered_cloud_pub_);
    RCLCPP_INFO(this->get_logger(), "Clustered point cloud published successfully");
#endif

    // 计算处理时间
    if (enable_calculate_process_time_) {
        RCLCPP_INFO(this->get_logger(), "Calculating processing time...");
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(this->get_logger(), "Point cloud processing time: %ld ms, found %zu obstacles", duration.count(),
                    obstacles_msg.obstacles.size());
    }

    // 可视化
    if (enable_visualization_) {
        RCLCPP_INFO(this->get_logger(), "Starting visualization...");
        VisualizePointCloud(non_ground_cloud, "Processed Point Cloud");
        RCLCPP_INFO(this->get_logger(), "Visualization completed");
    }

    RCLCPP_INFO(this->get_logger(), "=== Point cloud processing completed successfully ===");
    return obstacles_msg;
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
