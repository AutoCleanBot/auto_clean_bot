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
    // 新增OBB和体素滤波相关参数
    this->declare_parameter<bool>("use_obb", true);
    this->declare_parameter<int>("min_points_per_voxel", 1);
    this->declare_parameter<int>("max_points_per_voxel_in_large_cluster", 5);
    this->declare_parameter<int>("min_voxel_cluster_size_for_filtering", 50);
    // 原有参数继续
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
    // 新增参数获取
    use_obb_ = this->get_parameter("use_obb").as_bool();
    min_points_per_voxel_ = this->get_parameter("min_points_per_voxel").as_int();
    max_points_per_voxel_in_large_cluster_ = this->get_parameter("max_points_per_voxel_in_large_cluster").as_int();
    min_voxel_cluster_size_for_filtering_ = this->get_parameter("min_voxel_cluster_size_for_filtering").as_int();
    // 初始化随机数引擎
    random_engine_ = std::default_random_engine(42); // 使用固定种子以确保可重复性
    // 原有参数获取继续
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
    RCLCPP_INFO(this->get_logger(), "leaf_size_x: %f", leaf_size_x_);
    RCLCPP_INFO(this->get_logger(), "leaf_size_y: %f", leaf_size_y_);
    RCLCPP_INFO(this->get_logger(), "leaf_size_z: %f", leaf_size_z_);
    RCLCPP_INFO(this->get_logger(), "roi_width: %f", roi_width_);
    RCLCPP_INFO(this->get_logger(), "enable_visualization: %d", enable_visualization_);
    RCLCPP_INFO(this->get_logger(), "enable_calculate_process_time: %d", enable_calculate_process_time_);
    RCLCPP_INFO(this->get_logger(), "enable_use_roi: %d", enable_use_roi_);
    RCLCPP_INFO(this->get_logger(), "enable_downsample: %d", enable_downsample_);
    RCLCPP_INFO(this->get_logger(), "segment_ground_type: %d", segment_ground_type_);
    RCLCPP_INFO(this->get_logger(), "plane_point_percent: %f", plane_point_percent_);
    RCLCPP_INFO(this->get_logger(), "use_obb: %d", use_obb_);
    RCLCPP_INFO(this->get_logger(), "min_points_per_voxel: %d", min_points_per_voxel_);
    RCLCPP_INFO(this->get_logger(), "max_points_per_voxel_in_large_cluster: %d", max_points_per_voxel_in_large_cluster_);
    RCLCPP_INFO(this->get_logger(), "min_voxel_cluster_size_for_filtering: %d", min_voxel_cluster_size_for_filtering_);
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
    // 开始计时
    auto start_time = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %u points, starting processing...",
                pnt_cloud->width * pnt_cloud->height);
    
    try {
        // 将sensor_msgs::PointCloud2转换为pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pnt_cloud, *cloud);
        
        // 设置点云输入
        input_cloud_ = cloud;
        
        // 运行处理流程
        Run();
        
        // 计算处理时间
        if (enable_calculate_process_time_) {
            auto end_time = this->now();
            auto process_time = (end_time - start_time).seconds() * 1000.0; // 转换为毫秒
            RCLCPP_INFO(this->get_logger(), "Point cloud processing completed in %.2f ms", process_time);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
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
 * @brief 基于2D的欧几里得聚类算法，忽略Z轴，更好地处理地面上的物体
 *
 * @param cloud 输入点云
 * @return std::vector<pcl::PointIndices> 聚类结果
 */
std::vector<pcl::PointIndices> ObstaclesDetectionLidarNode::ClusterPointsIn2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    RCLCPP_INFO(this->get_logger(), "ClusterPointsIn2D: Starting 2D clustering with %zu input points", cloud->points.size());

    std::vector<pcl::PointIndices> cluster_indices;

    if (cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "ClusterPointsIn2D: Input cloud is empty, returning empty clusters");
        return cluster_indices;
    }

    // 1) 使用体素网格滤波，降低点云密度
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    constexpr float Z_AXIS_VOXEL_SIZE = 100000.0f; // 非常大的Z方向体素，忽略Z轴差异
    voxel_grid.setLeafSize(leaf_size_x_, leaf_size_y_, Z_AXIS_VOXEL_SIZE);
    voxel_grid.setMinimumPointsNumberPerVoxel(min_points_per_voxel_);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setSaveLeafLayout(true);
    voxel_grid.filter(*voxel_map_ptr);

    RCLCPP_INFO(this->get_logger(), "ClusterPointsIn2D: Voxel grid filtering completed, resulted in %zu points", voxel_map_ptr->points.size());

    // 2) 创建2D点云（忽略Z轴）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : voxel_map_ptr->points) {
        pcl::PointXYZ point2d;
        point2d.x = point.x;
        point2d.y = point.y;
        point2d.z = 0.0; // 设置Z为0以进行2D聚类
        cloud_2d->push_back(point2d);
    }

    // 3) 基于2D点云进行聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_2d);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); // 聚类距离阈值
    ec.setMinClusterSize(1); // 最小聚类体素数为1，我们将在后面过滤
    ec.setMaxClusterSize(cluster_max_size_); // 最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    std::vector<pcl::PointIndices> voxel_cluster_indices;
    ec.extract(voxel_cluster_indices);

    RCLCPP_INFO(this->get_logger(), "ClusterPointsIn2D: Found %zu voxel clusters", voxel_cluster_indices.size());

    // 4) 记录体素到实际点云的映射
    std::unordered_map<int, int> voxel_to_cluster_map;
    for (size_t cluster_idx = 0; cluster_idx < voxel_cluster_indices.size(); ++cluster_idx) {
        const auto &cluster = voxel_cluster_indices.at(cluster_idx);
        for (const auto &point_idx : cluster.indices) {
            voxel_to_cluster_map[point_idx] = cluster_idx;
        }
    }

    // 5) 标记大型聚类以便特殊处理
    std::vector<bool> is_large_cluster(voxel_cluster_indices.size(), false);
    std::vector<bool> is_extreme_large_cluster(voxel_cluster_indices.size(), false);

    for (size_t cluster_idx = 0; cluster_idx < voxel_cluster_indices.size(); ++cluster_idx) {
        const int cluster_size = static_cast<int>(voxel_cluster_indices[cluster_idx].indices.size());
        is_large_cluster[cluster_idx] = cluster_size > min_voxel_cluster_size_for_filtering_;
        is_extreme_large_cluster[cluster_idx] = cluster_size > cluster_max_size_;
    }

    // 6) 创建聚类点云索引
    // 初始化每个聚类的点云索引
    std::vector<pcl::PointIndices> final_cluster_indices(voxel_cluster_indices.size());
    
    // 跟踪每个体素在每个聚类中的点数
    std::unordered_map<int, std::unordered_map<int, int>> point_counts_per_voxel_per_cluster;
    
    // 随机打乱点云索引以均匀采样
    std::vector<size_t> random_indices(cloud->points.size());
    std::iota(random_indices.begin(), random_indices.end(), 0);
    std::shuffle(random_indices.begin(), random_indices.end(), random_engine_);
    
    for (size_t i = 0; i < random_indices.size(); ++i) {
        const size_t random_index = random_indices[i];
        const auto &point = cloud->points.at(random_index);
        
        // 获取点所在的体素索引
        int voxel_index = voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(point.x, point.y, point.z));
        
        auto voxel_to_cluster_map_it = voxel_to_cluster_map.find(voxel_index);
        if (voxel_to_cluster_map_it != voxel_to_cluster_map.end()) {
            int cluster_idx = voxel_to_cluster_map_it->second;
            
            // 如果是极大型聚类，跳过
            if (is_extreme_large_cluster[cluster_idx]) {
                continue;
            }
            
            // 如果是大型聚类，限制每个体素的点数
            if (is_large_cluster[cluster_idx]) {
                int &voxel_point_count = point_counts_per_voxel_per_cluster[cluster_idx][voxel_index];
                if (voxel_point_count >= max_points_per_voxel_in_large_cluster_) {
                    continue; // 跳过添加此点
                }
                voxel_point_count++;
            }
            
            // 添加点索引到对应的聚类
            final_cluster_indices[cluster_idx].indices.push_back(random_index);
        }
    }
    
    // 7) 过滤小型聚类并整理结果
    std::vector<pcl::PointIndices> result_indices;
    for (size_t i = 0; i < final_cluster_indices.size(); ++i) {
        int cluster_size = static_cast<int>(final_cluster_indices[i].indices.size());
        if (cluster_size >= cluster_min_size_ && !is_extreme_large_cluster[i]) {
            result_indices.push_back(final_cluster_indices[i]);
        }
    }

    RCLCPP_INFO(this->get_logger(), "ClusterPointsIn2D: Final clusters after filtering: %zu", result_indices.size());
    return result_indices;
}

/**
 * @brief 使用OBB（有向边界框）提取障碍物信息
 *
 * @param cloud 输入点云
 * @param cluster_indices 聚类结果索引
 * @return std::vector<ObstacleInfo> 障碍物信息列表
 */
std::vector<ObstaclesDetectionLidarNode::ObstacleInfo> ObstaclesDetectionLidarNode::ExtractObstacleInfoWithOBB(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<pcl::PointIndices>& cluster_indices) {
    
    std::vector<ObstacleInfo> obstacles;
    
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        const auto& indices = cluster_indices[i].indices;
        
        // 提取当前聚类的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : indices) {
            cluster_cloud->points.push_back(cloud->points[idx]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        
        if (cluster_cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty cluster found, skipping...");
            continue;
        }

        // 计算点云的PCA来获取主方向
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_cloud, centroid);
        
        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrix(*cluster_cloud, centroid, covariance_matrix);
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
        Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
        
        // 确保特征向量是正交的右手系统
        eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
        
        // 将特征向量按照特征值大小排序
        Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();
        
        // 需要保证特征向量构成一个右手坐标系
        if (eigen_vectors.determinant() < 0) {
            eigen_vectors.col(2) = -eigen_vectors.col(2);
        }
        
        // 转换点云到主轴坐标系
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
        transform.block<3, 1>(0, 3) = -transform.block<3, 3>(0, 0) * centroid.head<3>();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cluster_cloud, *transformed_cloud, transform);
        
        // 计算OBB
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
        
        // 将OBB转换回原始坐标系
        Eigen::Matrix4f inverse_transform = transform.inverse();
        
        // 计算OBB的8个顶点
        std::vector<Eigen::Vector3f> obb_vertices(8);
        obb_vertices[0] = (inverse_transform * Eigen::Vector4f(min_pt[0], min_pt[1], min_pt[2], 1.0f)).head<3>();
        obb_vertices[1] = (inverse_transform * Eigen::Vector4f(max_pt[0], min_pt[1], min_pt[2], 1.0f)).head<3>();
        obb_vertices[2] = (inverse_transform * Eigen::Vector4f(max_pt[0], max_pt[1], min_pt[2], 1.0f)).head<3>();
        obb_vertices[3] = (inverse_transform * Eigen::Vector4f(min_pt[0], max_pt[1], min_pt[2], 1.0f)).head<3>();
        obb_vertices[4] = (inverse_transform * Eigen::Vector4f(min_pt[0], min_pt[1], max_pt[2], 1.0f)).head<3>();
        obb_vertices[5] = (inverse_transform * Eigen::Vector4f(max_pt[0], min_pt[1], max_pt[2], 1.0f)).head<3>();
        obb_vertices[6] = (inverse_transform * Eigen::Vector4f(max_pt[0], max_pt[1], max_pt[2], 1.0f)).head<3>();
        obb_vertices[7] = (inverse_transform * Eigen::Vector4f(min_pt[0], max_pt[1], max_pt[2], 1.0f)).head<3>();
        
        // 计算OBB的尺寸
        float length = max_pt[0] - min_pt[0];  // 沿最主要方向的长度
        float width = max_pt[1] - min_pt[1];   // 沿次要方向的宽度
        float height = max_pt[2] - min_pt[2];  // 沿最小方向的高度
        
        // 创建障碍物信息
        ObstacleInfo obstacle;
        obstacle.id = i;
        obstacle.position.x = centroid[0];
        obstacle.position.y = centroid[1];
        obstacle.position.z = centroid[2];
        
        // 使用OBB的尺寸作为障碍物的尺寸
        obstacle.dimensions.x = length;
        obstacle.dimensions.y = width;
        obstacle.dimensions.z = height;
        
        // 计算方向（偏航角）- 使用最主要方向的水平投影
        Eigen::Vector3f main_direction = eigen_vectors.col(0);
        float yaw = std::atan2(main_direction[1], main_direction[0]);
        obstacle.orientation.x = 0.0;
        obstacle.orientation.y = 0.0;
        obstacle.orientation.z = std::sin(yaw / 2.0);
        obstacle.orientation.w = std::cos(yaw / 2.0);
        
        // 计算置信度 - 基于点云大小
        obstacle.confidence = std::min(1.0, static_cast<double>(cluster_cloud->points.size()) / 1000.0);
        
        // 设置形状类型 - 暂时使用默认的BOX类型
        obstacle.shape_type = 1;  // BOX
        
        // 保存OBB顶点 - 可用于后续可视化或处理
        for (const auto& vertex : obb_vertices) {
            geometry_msgs::msg::Point point;
            point.x = vertex[0];
            point.y = vertex[1];
            point.z = vertex[2];
            obstacle.obb_vertices.push_back(point);
        }
        
        obstacles.push_back(obstacle);
    }
    
    return obstacles;
}

/**
 * @brief 从聚类中提取障碍物信息（使用AABB）
 *
 * @param cloud 输入点云
 * @param cluster_indices 聚类索引
 * @return std::vector<ObstacleInfo> 障碍物信息列表
 */
std::vector<ObstaclesDetectionLidarNode::ObstacleInfo> ObstaclesDetectionLidarNode::ExtractObstacleInfo(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<pcl::PointIndices>& cluster_indices) {
    
    std::vector<ObstacleInfo> obstacles;
    
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        const auto& indices = cluster_indices[i].indices;
        
        // 提取当前聚类的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : indices) {
            cluster_cloud->points.push_back(cloud->points[idx]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        
        if (cluster_cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty cluster found, skipping...");
            continue;
        }

        // 计算AABB边界框
        pcl::PointXYZ min_point, max_point;
        pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
        
        // 计算中心点
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_cloud, centroid);
        
        // 创建障碍物信息
        ObstacleInfo obstacle;
        obstacle.id = i;
        
        // 设置位置（中心点）
        obstacle.position.x = centroid[0];
        obstacle.position.y = centroid[1];
        obstacle.position.z = centroid[2];
        
        // 设置尺寸（AABB）
        obstacle.dimensions.x = max_point.x - min_point.x;
        obstacle.dimensions.y = max_point.y - min_point.y;
        obstacle.dimensions.z = max_point.z - min_point.z;
        
        // 设置方向（默认无旋转）
        obstacle.orientation.x = 0.0;
        obstacle.orientation.y = 0.0;
        obstacle.orientation.z = 0.0;
        obstacle.orientation.w = 1.0;
        
        // 计算置信度 - 基于点云大小
        obstacle.confidence = std::min(1.0, static_cast<double>(cluster_cloud->points.size()) / 1000.0);
        
        // 设置形状类型 - 默认为BOX
        obstacle.shape_type = 1;  // BOX
        
        // 保存AABB顶点
        std::vector<pcl::PointXYZ> box_points = {
            {min_point.x, min_point.y, min_point.z},
            {max_point.x, min_point.y, min_point.z},
            {max_point.x, max_point.y, min_point.z},
            {min_point.x, max_point.y, min_point.z},
            {min_point.x, min_point.y, max_point.z},
            {max_point.x, min_point.y, max_point.z},
            {max_point.x, max_point.y, max_point.z},
            {min_point.x, max_point.y, max_point.z}
        };
        
        for (const auto& point : box_points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            obstacle.obb_vertices.push_back(p);
        }
        
        obstacles.push_back(obstacle);
    }
    
    return obstacles;
}

/**
 * @brief 将障碍物坐标从base坐标系转换到ENU(map)坐标系
 * 
 * @param obstacles 需要转换的障碍物列表
 * @return std::vector<ObstacleInfo> 转换后的障碍物列表
 */
std::vector<ObstaclesDetectionLidarNode::ObstacleInfo> ObstaclesDetectionLidarNode::Obstacle2ENU(
    const std::vector<ObstacleInfo>& obstacles) {
    
    std::vector<ObstacleInfo> enu_obstacles = obstacles;
    
    // 如果不使用GNSS，直接返回原始障碍物列表
    if (!is_use_gnss_) {
        return enu_obstacles;
    }
    
    // 如果未收到GNSS消息，无法进行转换
    if (!is_gnss_msg_received_) {
        RCLCPP_WARN(this->get_logger(), "GNSS message not received, skipping ENU transformation");
        return enu_obstacles;
    }
    
    try {
        // 获取从base_frame_id_到map_frame_id_的变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            map_frame_id_,
            base_frame_id_,
            tf2::TimePointZero);
            
        for (auto& obstacle : enu_obstacles) {
            // 转换位置
            geometry_msgs::msg::PoseStamped pose_in, pose_out;
            pose_in.pose.position = obstacle.position;
            pose_in.pose.orientation = obstacle.orientation;
            pose_in.header.frame_id = base_frame_id_;
            
            tf2::doTransform(pose_in, pose_out, transform_stamped);
            
            obstacle.position = pose_out.pose.position;
            obstacle.orientation = pose_out.pose.orientation;
            
            // 转换OBB顶点
            for (auto& vertex : obstacle.obb_vertices) {
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.point = vertex;
                point_in.header.frame_id = base_frame_id_;
                
                tf2::doTransform(point_in, point_out, transform_stamped);
                vertex = point_out.point;
            }
        }
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform obstacle to ENU: %s", ex.what());
    }
    
    return enu_obstacles;
}

/**
 * @brief 发布障碍物信息
 * 
 * @param obstacles 障碍物列表
 */
void ObstaclesDetectionLidarNode::PublishObstacles(const std::vector<ObstacleInfo>& obstacles) {
    bot_msg::msg::Obstacles obstacles_msg;
    obstacles_msg.header.stamp = this->now();
    obstacles_msg.header.frame_id = is_use_gnss_ ? map_frame_id_ : base_frame_id_;
    
    for (const auto& obstacle : obstacles) {
        bot_msg::msg::ObstacleInfo msg;
        
        // 填充基本信息
        msg.id = obstacle.id;
        msg.position_x = obstacle.position.x;
        msg.position_y = obstacle.position.y;
        msg.position_z = obstacle.position.z;
        
        // 填充尺寸信息
        msg.length = obstacle.dimensions.x;
        msg.width = obstacle.dimensions.y;
        msg.height = obstacle.dimensions.z;
        
        // 初始化速度信息（静态障碍物）
        msg.velocity_x = 0.0;
        msg.velocity_y = 0.0;
        msg.velocity = 0.0;
        msg.heading = 0.0;
        
        // 设置障碍物类型和状态
        msg.type = 1;   // 默认类型
        msg.status = 1; // 活跃状态
        
        obstacles_msg.obstacles.push_back(msg);
    }
    
    obstacle_pub_->publish(obstacles_msg);
    RCLCPP_INFO(this->get_logger(), "Published %zu obstacles", obstacles.size());
}

/**
 * @brief 发布障碍物可视化marker
 * 
 * @param obstacles 障碍物列表
 */
void ObstaclesDetectionLidarNode::PublishMarkers(const std::vector<ObstacleInfo>& obstacles) {
    // 首先清除旧的marker
    ClearAllObstacleMarkers();
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obstacle = obstacles[i];
        
        // 创建障碍物框Marker
        visualization_msgs::msg::Marker box_marker;
        box_marker.header.frame_id = is_use_gnss_ ? map_frame_id_ : base_frame_id_;
        box_marker.header.stamp = this->now();
        box_marker.ns = "obstacle_boxes";
        box_marker.id = i;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位置和姿态
        box_marker.pose.position = obstacle.position;
        box_marker.pose.orientation = obstacle.orientation;
        
        // 设置尺寸
        box_marker.scale = obstacle.dimensions;
        
        // 设置颜色 (红色，半透明)
        box_marker.color.r = 1.0;
        box_marker.color.g = 0.0;
        box_marker.color.b = 0.0;
        box_marker.color.a = 0.5;
        
        box_marker.lifetime = rclcpp::Duration::from_seconds(0.1);  // 持续时间0.1秒
        
        marker_pub_->publish(box_marker);
        
        // 如果使用OBB，还要创建OBB的线框
        if (use_obb_ && !obstacle.obb_vertices.empty()) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = is_use_gnss_ ? map_frame_id_ : base_frame_id_;
            line_marker.header.stamp = this->now();
            line_marker.ns = "obstacle_lines";
            line_marker.id = i;
            line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 设置线的宽度
            line_marker.scale.x = 0.05;  // 线宽
            
            // 设置颜色 (绿色)
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;
            
            // 连接OBB的顶点形成一个线框
            // 底部四条边
            line_marker.points.push_back(obstacle.obb_vertices[0]);
            line_marker.points.push_back(obstacle.obb_vertices[1]);
            
            line_marker.points.push_back(obstacle.obb_vertices[1]);
            line_marker.points.push_back(obstacle.obb_vertices[2]);
            
            line_marker.points.push_back(obstacle.obb_vertices[2]);
            line_marker.points.push_back(obstacle.obb_vertices[3]);
            
            line_marker.points.push_back(obstacle.obb_vertices[3]);
            line_marker.points.push_back(obstacle.obb_vertices[0]);
            
            // 顶部四条边
            line_marker.points.push_back(obstacle.obb_vertices[4]);
            line_marker.points.push_back(obstacle.obb_vertices[5]);
            
            line_marker.points.push_back(obstacle.obb_vertices[5]);
            line_marker.points.push_back(obstacle.obb_vertices[6]);
            
            line_marker.points.push_back(obstacle.obb_vertices[6]);
            line_marker.points.push_back(obstacle.obb_vertices[7]);
            
            line_marker.points.push_back(obstacle.obb_vertices[7]);
            line_marker.points.push_back(obstacle.obb_vertices[4]);
            
            // 连接顶部和底部的四条边
            line_marker.points.push_back(obstacle.obb_vertices[0]);
            line_marker.points.push_back(obstacle.obb_vertices[4]);
            
            line_marker.points.push_back(obstacle.obb_vertices[1]);
            line_marker.points.push_back(obstacle.obb_vertices[5]);
            
            line_marker.points.push_back(obstacle.obb_vertices[2]);
            line_marker.points.push_back(obstacle.obb_vertices[6]);
            
            line_marker.points.push_back(obstacle.obb_vertices[3]);
            line_marker.points.push_back(obstacle.obb_vertices[7]);
            
            line_marker.lifetime = rclcpp::Duration::from_seconds(0.1);  // 持续时间0.1秒
            
            marker_pub_->publish(line_marker);
        }
    }
    
    // 更新已发布的marker数量
    last_marker_count_ = obstacles.size() * (use_obb_ ? 2 : 1);
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
    std::vector<pcl::PointIndices> cluster_indices = ClusterPointsIn2D(non_ground_cloud);
    RCLCPP_INFO(this->get_logger(), "Step 7 completed: Found %zu clusters", cluster_indices.size());

    // 8. 提取障碍物信息
    RCLCPP_INFO(this->get_logger(), "Step 8: Extracting obstacle information...");
    std::vector<ObstacleInfo> obstacles;
    
    if (use_obb_) {
        // 使用OBB提取障碍物信息
        obstacles = ExtractObstacleInfoWithOBB(non_ground_cloud, cluster_indices);
    } else {
        // 使用原有的AABB方法提取障碍物信息
        obstacles = ExtractObstacleInfo(non_ground_cloud, cluster_indices);
    }
    
    RCLCPP_INFO(this->get_logger(), "Step 8 completed: Extracted %zu obstacles", obstacles.size());

    // 9. 转换障碍物坐标到地图坐标系
    RCLCPP_INFO(this->get_logger(), "Step 9: Transforming obstacles to map frame...");
    std::vector<ObstacleInfo> enu_obstacles = Obstacle2ENU(obstacles);
    RCLCPP_INFO(this->get_logger(), "Step 9 completed: Transformed %zu obstacles", enu_obstacles.size());

    // 10. 发布检测结果
    RCLCPP_INFO(this->get_logger(), "Step 10: Publishing results...");
    PublishObstacles(enu_obstacles);
    PublishMarkers(enu_obstacles);
    RCLCPP_INFO(this->get_logger(), "Step 10 completed: Results published");

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

/**
 * @brief 点云处理的完整流程
 */
void ObstaclesDetectionLidarNode::Run() {
    RCLCPP_INFO(this->get_logger(), "=== 开始执行点云处理流程 ===");
    
    if (!input_cloud_ || input_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "输入点云为空，跳过处理");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "输入点云大小: %zu 点", input_cloud_->points.size());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 1. 移除无效点
    RCLCPP_INFO(this->get_logger(), "步骤 1: 移除无效点...");
    RemoveInvalidPoints(input_cloud_);
    RCLCPP_INFO(this->get_logger(), "步骤 1 完成: 移除无效点后点云大小: %zu",
                input_cloud_->points.size());
                
    // 2. 移除车辆范围内的点
    RCLCPP_INFO(this->get_logger(), "步骤 2: 移除车辆点...");
    RemoveVehiclePoints(input_cloud_);
    RCLCPP_INFO(this->get_logger(), "步骤 2 完成: 移除车辆点后点云大小: %zu",
                input_cloud_->points.size());
                
    // 3. ROI滤波
    RCLCPP_INFO(this->get_logger(), "步骤 3: 应用ROI过滤...");
    FilterROI(input_cloud_);
    RCLCPP_INFO(this->get_logger(), "步骤 3 完成: ROI过滤后点云大小: %zu", 
                input_cloud_->points.size());
                
    // 4. 下采样
    RCLCPP_INFO(this->get_logger(), "步骤 4: 应用下采样...");
    DownsampleCloud(input_cloud_);
    RCLCPP_INFO(this->get_logger(), "步骤 4 完成: 下采样后点云大小: %zu", 
                input_cloud_->points.size());
                
#if DEBUG_PUBLISH_POINT_CLOUD
    RCLCPP_INFO(this->get_logger(), "发布过滤后的点云用于调试...");
    PublishPointCloud(input_cloud_, filtered_cloud_pub_);
    RCLCPP_INFO(this->get_logger(), "过滤后的点云发布成功");
#endif

    // 5. 地面滤除
    RCLCPP_INFO(this->get_logger(), "步骤 5: 应用地面过滤...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    FilterGroundPoints(input_cloud_, ground_cloud, non_ground_cloud);
    RCLCPP_INFO(this->get_logger(), "步骤 5 完成: 地面点: %zu, 非地面点: %zu",
                ground_cloud->points.size(), non_ground_cloud->points.size());
                
#if DEBUG_PUBLISH_POINT_CLOUD
    RCLCPP_INFO(this->get_logger(), "发布地面分割结果用于调试...");
    PublishPointCloud(ground_cloud, ground_seg_cloud_pub_);
    RCLCPP_INFO(this->get_logger(), "地面分割结果发布成功");
#endif

    if (non_ground_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "未找到非地面点，返回空结果");
        return;
    }

    // 6. 聚类
    RCLCPP_INFO(this->get_logger(), "步骤 6: 应用聚类...");
    std::vector<pcl::PointIndices> cluster_indices = ClusterPointsIn2D(non_ground_cloud);
    RCLCPP_INFO(this->get_logger(), "步骤 6 完成: 找到 %zu 个聚类", cluster_indices.size());
    
    // 7. 提取障碍物信息
    RCLCPP_INFO(this->get_logger(), "步骤 7: 提取障碍物信息...");
    std::vector<ObstacleInfo> obstacles;
    
    if (use_obb_) {
        // 使用OBB提取障碍物信息
        obstacles = ExtractObstacleInfoWithOBB(non_ground_cloud, cluster_indices);
    } else {
        // 使用AABB方法提取障碍物信息
        obstacles = ExtractObstacleInfo(non_ground_cloud, cluster_indices);
    }
    
    RCLCPP_INFO(this->get_logger(), "步骤 7 完成: 提取了 %zu 个障碍物", obstacles.size());
    
    // 8. 先将障碍物坐标从激光雷达坐标系转换到base坐标系
    RCLCPP_INFO(this->get_logger(), "步骤 8: 将障碍物转换到base坐标系...");
    std::vector<ObstacleInfo> base_obstacles = Obstacle2Base(obstacles);
    RCLCPP_INFO(this->get_logger(), "步骤 8 完成: 转换了 %zu 个障碍物到base坐标系", base_obstacles.size());
    
    // 9. 再将障碍物坐标从base坐标系转换到ENU(map)坐标系
    RCLCPP_INFO(this->get_logger(), "步骤 9: 将障碍物从base坐标系转换到map坐标系...");
    std::vector<ObstacleInfo> enu_obstacles = Obstacle2ENU(base_obstacles);
    RCLCPP_INFO(this->get_logger(), "步骤 9 完成: 转换了 %zu 个障碍物到map坐标系", enu_obstacles.size());
    
    // 10. 发布检测结果
    RCLCPP_INFO(this->get_logger(), "步骤 10: 发布结果...");
    PublishObstacles(enu_obstacles);
    PublishMarkers(enu_obstacles);
    RCLCPP_INFO(this->get_logger(), "步骤 10 完成: 结果已发布");
    
    // 计算处理时间
    if (enable_calculate_process_time_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(this->get_logger(), "点云处理时间: %ld 毫秒, 找到 %zu 个障碍物", 
                    duration.count(), obstacles.size());
    }
    
    // 可视化
    if (enable_visualization_) {
        RCLCPP_INFO(this->get_logger(), "开始可视化...");
        VisualizePointCloud(non_ground_cloud, "处理后的点云");
        RCLCPP_INFO(this->get_logger(), "可视化完成");
    }
    
    RCLCPP_INFO(this->get_logger(), "=== 点云处理流程成功完成 ===");
}

/**
 * @brief 将障碍物从激光雷达坐标系转换到base坐标系
 * 
 * @param obstacles 需要转换的障碍物列表
 * @return std::vector<ObstacleInfo> 转换后的障碍物列表
 */
std::vector<ObstaclesDetectionLidarNode::ObstacleInfo> ObstaclesDetectionLidarNode::Obstacle2Base(
    const std::vector<ObstacleInfo>& obstacles) {
    
    std::vector<ObstacleInfo> base_obstacles = obstacles;
    
    RCLCPP_INFO(this->get_logger(), "Obstacle2Base: 开始将障碍物转换到base坐标系, 总数: %zu", obstacles.size());
    
    try {
        // 获取从激光雷达坐标系到base坐标系的变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            base_frame_id_,
            front_lidar_frame_id_,
            tf2::TimePointZero);
            
        RCLCPP_INFO(this->get_logger(), "Obstacle2Base: 找到从 %s 到 %s 的变换", 
                   front_lidar_frame_id_.c_str(), base_frame_id_.c_str());
        
        for (auto& obstacle : base_obstacles) {
            // 转换位置
            geometry_msgs::msg::PoseStamped pose_in, pose_out;
            pose_in.pose.position = obstacle.position;
            pose_in.pose.orientation = obstacle.orientation;
            pose_in.header.frame_id = front_lidar_frame_id_;
            
            tf2::doTransform(pose_in, pose_out, transform_stamped);
            
            obstacle.position = pose_out.pose.position;
            obstacle.orientation = pose_out.pose.orientation;
            
            // 转换OBB顶点
            for (auto& vertex : obstacle.obb_vertices) {
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.point = vertex;
                point_in.header.frame_id = front_lidar_frame_id_;
                
                tf2::doTransform(point_in, point_out, transform_stamped);
                vertex = point_out.point;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Obstacle2Base: 坐标转换成功完成");
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Obstacle2Base: 无法转换障碍物到base坐标系: %s", ex.what());
    }
    
    return base_obstacles;
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
