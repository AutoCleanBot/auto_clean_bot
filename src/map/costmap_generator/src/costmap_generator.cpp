#include "costmap_generator/costmap_generator.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace costmap_generator {

/**
 * @brief 代价地图生成器构造函数
 * @param node_options ROS2节点选项
 *
 * 初始化代价地图生成器，声明并读取所有参数，设置订阅者、发布者和定时器
 */
CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions &node_options)
    : Node("costmap_generator", node_options),
      tf_buffer_(this->get_clock(), tf2::durationFromSec(10.0)), // 增加TF缓存时间到10秒
      tf_listener_(tf_buffer_) {
    // 声明所有参数，但使用配置文件中的值
    this->declare_parameter("update_rate", 10.0);
    this->declare_parameter("grid_min_value", 0.0);
    this->declare_parameter("grid_max_value", 1.0);
    this->declare_parameter("grid_resolution", 0.2);
    this->declare_parameter("grid_length_x", 50.0);
    this->declare_parameter("grid_length_y", 50.0);
    this->declare_parameter("grid_position_x", 0.0);
    this->declare_parameter("grid_position_y", 0.0);
    this->declare_parameter("maximum_height_thres", 2.0);
    this->declare_parameter("minimum_height_thres", 0.2);
    this->declare_parameter("costmap_frame", "map");
    this->declare_parameter("input_frame", "base_link");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("input_points_topic", "~/input/points");
    this->declare_parameter("costmap_topic", "~/output/grid_map");
    this->declare_parameter("occupancy_grid_topic", "~/output/occupancy_grid");
    this->declare_parameter("is_pub_pnt_cloud", false);
    // 从配置文件中读取参数
    update_rate_ = this->get_parameter("update_rate").as_double();
    grid_min_value_ = this->get_parameter("grid_min_value").as_double();
    grid_max_value_ = this->get_parameter("grid_max_value").as_double();
    grid_resolution_ = this->get_parameter("grid_resolution").as_double();
    grid_length_x_ = this->get_parameter("grid_length_x").as_double();
    grid_length_y_ = this->get_parameter("grid_length_y").as_double();
    grid_position_x_ = this->get_parameter("grid_position_x").as_double();
    grid_position_y_ = this->get_parameter("grid_position_y").as_double();
    maximum_height_thres_ = this->get_parameter("maximum_height_thres").as_double();
    minimum_height_thres_ = this->get_parameter("minimum_height_thres").as_double();
    costmap_frame_ = this->get_parameter("costmap_frame").as_string();
    input_frame_ = this->get_parameter("input_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();

    // 从配置文件中读取输入话题名
    std::string input_points_topic = this->get_parameter("input_points_topic").as_string();
    std::string costmap_topic = this->get_parameter("costmap_topic").as_string();
    std::string occupancy_grid_topic = this->get_parameter("occupancy_grid_topic").as_string();
    is_pub_pnt_cloud_ = this->get_parameter("is_pub_pnt_cloud").as_bool();
    // Initialize subscribers
    sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_points_topic, rclcpp::SensorDataQoS(),
        std::bind(&CostmapGenerator::onPointCloud, this, std::placeholders::_1));

    // Initialize publishers - 只发布占用栅格地图，不发布GridMap
    // pub_costmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>(costmap_topic, 1);  // 已禁用
    pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_grid_topic, 1);
    pub_pnt_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_points", 1);
    // Initialize timer
    const auto period_ns = rclcpp::Rate(update_rate_).period();
    timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&CostmapGenerator::onTimer, this));

    // Initialize gridmap
    initGridmap();

    RCLCPP_INFO(this->get_logger(), "input_points_topic: %s", input_points_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "input_frame: %s", input_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "costmap_frame: %s", costmap_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "costmap_topic: %s", costmap_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "occupancy_grid_topic: %s", occupancy_grid_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "update_rate: %f", update_rate_);
    RCLCPP_INFO(this->get_logger(), "grid_length_x: %f", grid_length_x_);
    RCLCPP_INFO(this->get_logger(), "grid_length_y: %f", grid_length_y_);
    RCLCPP_INFO(this->get_logger(), "grid_position_x: %f", grid_position_x_);
    RCLCPP_INFO(this->get_logger(), "grid_position_y: %f", grid_position_y_);
    RCLCPP_INFO(this->get_logger(), "grid_resolution: %f", grid_resolution_);
    RCLCPP_INFO(this->get_logger(), "grid_min_value: %f", grid_min_value_);
    RCLCPP_INFO(this->get_logger(), "grid_max_value: %f", grid_max_value_);
    RCLCPP_INFO(this->get_logger(), "maximum_height_thres: %f", maximum_height_thres_);
    RCLCPP_INFO(this->get_logger(), "minimum_height_thres: %f", minimum_height_thres_);
    RCLCPP_INFO(this->get_logger(), "Costmap generator initialized");
}

/**
 * @brief 点云回调函数
 * @param msg 接收到的点云消息
 *
 * 保存接收到的点云数据供后续处理
 */
void CostmapGenerator::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { points_ = msg; }

/**
 * @brief 定时器回调函数
 *
 * 定期执行占用栅格地图生成和发布操作
 */
void CostmapGenerator::onTimer() {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!points_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No point cloud received yet");
        return;
    }

    // 获取坐标变换: 从costmap_frame到input_frame的变换
    auto tf_start = std::chrono::high_resolution_clock::now();
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform =
            tf_buffer_.lookupTransform(costmap_frame_, input_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get transform: %s", ex.what());
        return;
    }
    auto tf_end = std::chrono::high_resolution_clock::now();
    auto tf_duration = std::chrono::duration_cast<std::chrono::microseconds>(tf_end - tf_start);
    RCLCPP_DEBUG(this->get_logger(), "Transform lookup time: %ld μs", tf_duration.count());

    // 保存当前车辆的航向角度
    current_vehicle_yaw_ = tf2::getYaw(transform.transform.rotation);
    RCLCPP_DEBUG(this->get_logger(), "Vehicle yaw: %.2f", current_vehicle_yaw_);

    // 重新初始化固定坐标系中的网格地图
    auto init_start = std::chrono::high_resolution_clock::now();
    initGridmap();
    auto init_end = std::chrono::high_resolution_clock::now();
    auto init_duration = std::chrono::duration_cast<std::chrono::microseconds>(init_end - init_start);
    RCLCPP_DEBUG(this->get_logger(), "Grid map initialization time: %ld μs", init_duration.count());

    // 设置网格中心为车辆当前位置
    auto center_start = std::chrono::high_resolution_clock::now();
    setGridCenter(transform);
    auto center_end = std::chrono::high_resolution_clock::now();
    auto center_duration = std::chrono::duration_cast<std::chrono::microseconds>(center_end - center_start);
    RCLCPP_DEBUG(this->get_logger(), "Grid center setting time: %ld μs", center_duration.count());

    // 生成占用栅格地图
    auto points_start = std::chrono::high_resolution_clock::now();
    const double vehicle_to_map_z = transform.transform.translation.z;
    costmap_[LayerName::points] = generatePointsCostmap(points_, vehicle_to_map_z);
    auto points_end = std::chrono::high_resolution_clock::now();
    auto points_duration = std::chrono::duration_cast<std::chrono::milliseconds>(points_end - points_start);
    RCLCPP_INFO(this->get_logger(), "Points costmap generation time: %ld ms", points_duration.count());

    // 生成组合占用栅格地图
    auto combined_start = std::chrono::high_resolution_clock::now();
    costmap_[LayerName::combined] = generateCombinedCostmap();
    auto combined_end = std::chrono::high_resolution_clock::now();
    auto combined_duration = std::chrono::duration_cast<std::chrono::microseconds>(combined_end - combined_start);
    RCLCPP_DEBUG(this->get_logger(), "Combined costmap generation time: %ld μs", combined_duration.count());

    // 发布占用栅格地图
    auto publish_start = std::chrono::high_resolution_clock::now();
    publishCostmap(costmap_, transform);
    auto publish_end = std::chrono::high_resolution_clock::now();
    auto publish_duration = std::chrono::duration_cast<std::chrono::milliseconds>(publish_end - publish_start);
    RCLCPP_INFO(this->get_logger(), "Occupancy grid publishing time: %ld ms", publish_duration.count());

    // 总耗时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Total processing time: %ld ms", total_duration.count());
}

/**
 * @brief 初始化网格地图
 *
 * 创建并设置网格地图的基本属性，包括图层、坐标系、几何形状等
 */
void CostmapGenerator::initGridmap() {
    costmap_ = GridMap({LayerName::points, LayerName::combined});
    costmap_.setFrameId(costmap_frame_);
    costmap_.setGeometry(Eigen::Vector2d(grid_length_x_, grid_length_y_), grid_resolution_,
                         Eigen::Vector2d(grid_position_x_, grid_position_y_));
    costmap_.setTimestamp(this->now().nanoseconds());

    // 清空所有图层的数据
    costmap_[LayerName::points].setZero();
    costmap_[LayerName::combined].setZero();
}

/**
 * @brief 设置网格中心
 * @param tf 从代价地图坐标系到输入坐标系的变换
 *
 * 根据当前车辆位置更新网格地图中心，实现网格地图跟随车辆移动
 */
void CostmapGenerator::setGridCenter(const geometry_msgs::msg::TransformStamped &tf) {
    // 获取车辆在costmap_frame中的位置
    const double vehicle_x = tf.transform.translation.x;
    const double vehicle_y = tf.transform.translation.y;

    // 直接将网格中心设置为车辆当前位置，确保车辆始终在栅格地图中心
    grid_position_x_ = vehicle_x;
    grid_position_y_ = vehicle_y;

    // 更新网格地图中心位置
    costmap_.move(Eigen::Vector2d(grid_position_x_, grid_position_y_));

    // 输出调试信息
    RCLCPP_DEBUG(this->get_logger(), "Grid center set to vehicle position: x=%.2f, y=%.2f", grid_position_x_,
                 grid_position_y_);
}

/**
 * @brief 发布占用栅格地图
 * @param costmap 要发布的代价地图
 * @param tf 坐标变换信息
 *
 * 将代价地图转换为OccupancyGrid格式并发布（不再发布GridMap）
 */
void CostmapGenerator::publishCostmap(const GridMap &costmap, const geometry_msgs::msg::TransformStamped &tf) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // 更新时间戳
    costmap_.setTimestamp(this->now().nanoseconds());

    // 不再发布GridMap，只发布OccupancyGrid以提高性能
    // grid_map_msgs::msg::GridMap grid_map_msg;
    // costmap.toMessage(grid_map_msg);
    // pub_costmap_->publish(grid_map_msg);

    // Publish OccupancyGrid
    auto occupancy_start = std::chrono::high_resolution_clock::now();
    nav_msgs::msg::OccupancyGrid occupancy_grid;

    try {
        costmap.toOccupancyGrid(LayerName::combined, grid_min_value_, grid_max_value_, occupancy_grid);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting to OccupancyGrid: %s", e.what());
        return;
    }
    auto occupancy_convert_end = std::chrono::high_resolution_clock::now();
    auto occupancy_convert_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(occupancy_convert_end - occupancy_start);
    RCLCPP_DEBUG(this->get_logger(), "OccupancyGrid conversion time: %ld μs", occupancy_convert_duration.count());

    // 计算OccupancyGrid的原点
    // OccupancyGrid的原点在左下角，需要考虑车辆的朝向
    double grid_length_x_half = grid_length_x_ / 2.0;
    double grid_length_y_half = grid_length_y_ / 2.0;

    // 获取车辆航向角
    double yaw = current_vehicle_yaw_;

    // 输出yaw值以确保正确性
    RCLCPP_INFO(this->get_logger(), "Vehicle yaw for OccupancyGrid: %.2f radians (%.2f degrees)", yaw,
                yaw * 180.0 / M_PI);

    // 使用标准的2D旋转矩阵（逆时针为正方向）
    // [ cos(θ)  -sin(θ) ]
    // [ sin(θ)   cos(θ) ]
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    // 计算从中心到左下角的偏移向量
    double offset_x = -grid_length_x_half;
    double offset_y = -grid_length_y_half;

    // 应用旋转到偏移向量 (使用标准2D旋转矩阵)
    double rotated_offset_x = cos_yaw * offset_x - sin_yaw * offset_y;
    double rotated_offset_y = sin_yaw * offset_x + cos_yaw * offset_y;

    // 计算旋转后的原点位置
    geometry_msgs::msg::Pose origin;
    origin.position.x = grid_position_x_ + rotated_offset_x;
    origin.position.y = grid_position_y_ + rotated_offset_y;
    origin.position.z = 0.0;

    // 设置原点的方向
    origin.orientation = tf.transform.rotation;

    // 设置占用栅格地图的原点和方向
    occupancy_grid.info.origin = origin;

    // 确保时间戳是最新的
    occupancy_grid.header.stamp = this->now();
    occupancy_grid.header.frame_id = costmap_frame_;

    RCLCPP_DEBUG(
        this->get_logger(),
        "Publishing OccupancyGrid: origin=(%.2f, %.2f), rotated_offset=(%.2f, %.2f), grid size=%dx%d, resolution=%.2f",
        origin.position.x, origin.position.y, rotated_offset_x, rotated_offset_y, occupancy_grid.info.width,
        occupancy_grid.info.height, occupancy_grid.info.resolution);

    // 发布占用栅格地图
    auto publish_start = std::chrono::high_resolution_clock::now();
    pub_occupancy_grid_->publish(occupancy_grid);
    auto publish_end = std::chrono::high_resolution_clock::now();
    auto publish_duration = std::chrono::duration_cast<std::chrono::microseconds>(publish_end - publish_start);
    RCLCPP_DEBUG(this->get_logger(), "OccupancyGrid publish time: %ld μs", publish_duration.count());

    // 总的发布函数耗时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_DEBUG(this->get_logger(), "Total publishCostmap time: %ld ms", total_duration.count());
}

/**
 * @brief 从点云生成占用栅格地图
 * @param in_points 输入点云
 * @param vehicle_to_map_z 车辆到地图坐标系的Z轴偏移
 * @return 生成的占用栅格地图矩阵
 *
 * 将点云数据转换为占用栅格地图，考虑高度阈值和网格分辨率
 */
Eigen::MatrixXf CostmapGenerator::generatePointsCostmap(const sensor_msgs::msg::PointCloud2::SharedPtr &in_points,
                                                        const double vehicle_to_map_z) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!in_points) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No point cloud data available");
        return costmap_[LayerName::points];
    }

    // 将点云转换到costmap_frame坐标系
    auto transform_lookup_start = std::chrono::high_resolution_clock::now();
    sensor_msgs::msg::PointCloud2 transformed_points;
    geometry_msgs::msg::TransformStamped transform;

    // 添加TF调试信息
    RCLCPP_DEBUG(this->get_logger(), "Looking for transform from %s to %s", in_points->header.frame_id.c_str(),
                 costmap_frame_.c_str());

    // 检查TF是否可用
    bool tf_available = tf_buffer_.canTransform(costmap_frame_, in_points->header.frame_id, tf2::TimePointZero,
                                                tf2::durationFromSec(0.01));
    RCLCPP_DEBUG(this->get_logger(), "TF available: %s", tf_available ? "true" : "false");

    try {
        // 首先尝试使用最新的变换（性能更好）
        transform = tf_buffer_.lookupTransform(costmap_frame_, in_points->header.frame_id, tf2::TimePointZero,
                                               tf2::durationFromSec(0.05)); // 减少等待时间
        RCLCPP_DEBUG(this->get_logger(), "Using latest transform for better performance");
    } catch (tf2::TransformException &ex) {
        RCLCPP_DEBUG(this->get_logger(), "Latest transform failed: %s", ex.what());
        // 如果最新变换失败，尝试使用精确时间戳的变换
        try {
            transform = tf_buffer_.lookupTransform(costmap_frame_, in_points->header.frame_id, in_points->header.stamp,
                                                   tf2::durationFromSec(0.05)); // 减少等待时间
            RCLCPP_DEBUG(this->get_logger(), "Using timestamp-matched transform");
        } catch (tf2::TransformException &ex2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Could not get transform from %s to %s: latest=%s, stamped=%s",
                                 in_points->header.frame_id.c_str(), costmap_frame_.c_str(), ex.what(), ex2.what());
            return costmap_[LayerName::points];
        }
    }
    auto transform_lookup_end = std::chrono::high_resolution_clock::now();
    auto transform_lookup_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(transform_lookup_end - transform_lookup_start);
    RCLCPP_INFO(this->get_logger(), "Point cloud transform lookup time: %ld ms", transform_lookup_duration.count());

    // 输出点云转换信息
    RCLCPP_DEBUG(this->get_logger(),
                 "Point cloud transform: tx=%.2f, ty=%.2f, tz=%.2f, qw=%.2f, qx=%.2f, qy=%.2f, qz=%.2f",
                 transform.transform.translation.x, transform.transform.translation.y,
                 transform.transform.translation.z, transform.transform.rotation.w, transform.transform.rotation.x,
                 transform.transform.rotation.y, transform.transform.rotation.z);

    // 手动转换 transform 到 Eigen::Affine3d
    auto matrix_start = std::chrono::high_resolution_clock::now();
    Eigen::Translation3d translation(transform.transform.translation.x, transform.transform.translation.y,
                                     transform.transform.translation.z);

    Eigen::Quaterniond rotation(transform.transform.rotation.w, transform.transform.rotation.x,
                                transform.transform.rotation.y, transform.transform.rotation.z);

    Eigen::Affine3d transform_eigen = translation * rotation;
    Eigen::Matrix4f transform_matrix = transform_eigen.matrix().cast<float>();
    auto matrix_end = std::chrono::high_resolution_clock::now();
    auto matrix_duration = std::chrono::duration_cast<std::chrono::milliseconds>(matrix_end - matrix_start);
    RCLCPP_INFO(this->get_logger(), "Transform matrix computation time: %ld ms", matrix_duration.count());

    // 变换点云
    auto transform_start = std::chrono::high_resolution_clock::now();
    pcl_ros::transformPointCloud(transform_matrix, *in_points, transformed_points);
    auto transform_end = std::chrono::high_resolution_clock::now();
    auto transform_duration = std::chrono::duration_cast<std::chrono::milliseconds>(transform_end - transform_start);
    RCLCPP_INFO(this->get_logger(), "Point cloud transformation time: %ld ms", transform_duration.count());

    // 确保转换后的点云frame_id设置正确
    transformed_points.header.frame_id = costmap_frame_;
    transformed_points.header.stamp = this->now();

    // 如果需要，发布转换后的点云
    if (is_pub_pnt_cloud_) {
        pub_pnt_cloud_->publish(transformed_points);
    }

    // 转换为PCL点云
    auto pcl_start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
    pcl::fromROSMsg(transformed_points, pcl_pointcloud);
    auto pcl_end = std::chrono::high_resolution_clock::now();
    auto pcl_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pcl_end - pcl_start);
    RCLCPP_INFO(this->get_logger(), "PCL conversion time: %ld ms", pcl_duration.count());

    // 在生成代价地图前清空当前图层
    costmap_[LayerName::points].setZero();

    // 记录调试信息
    RCLCPP_INFO(this->get_logger(), "Processing %zu points for occupancy grid", pcl_pointcloud.size());

    // 计算点云的边界
    auto bounds_start = std::chrono::high_resolution_clock::now();
    if (!pcl_pointcloud.empty()) {
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (const auto &point : pcl_pointcloud) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }

        RCLCPP_DEBUG(this->get_logger(), "Point cloud bounds: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f]", min_x,
                     max_x, min_y, max_y, min_z, max_z);

        // 输出点云相对于网格中心的位置
        RCLCPP_DEBUG(this->get_logger(),
                     "Point cloud relative to grid center: x_center=%.2f (offset=%.2f), y_center=%.2f (offset=%.2f)",
                     (min_x + max_x) / 2.0, (min_x + max_x) / 2.0 - grid_position_x_, (min_y + max_y) / 2.0,
                     (min_y + max_y) / 2.0 - grid_position_y_);
    }
    auto bounds_end = std::chrono::high_resolution_clock::now();
    auto bounds_duration = std::chrono::duration_cast<std::chrono::milliseconds>(bounds_end - bounds_start);
    RCLCPP_INFO(this->get_logger(), "Point cloud bounds calculation time: %ld ms", bounds_duration.count());

    // 从点云生成占用栅格地图，传入车辆航向角用于旋转补偿
    auto costmap_gen_start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXf costmap = points2costmap_.makeCostmapFromPoints(
        maximum_height_thres_, minimum_height_thres_, grid_min_value_, grid_max_value_, costmap_, LayerName::points,
        pcl_pointcloud, current_vehicle_yaw_);
    auto costmap_gen_end = std::chrono::high_resolution_clock::now();
    auto costmap_gen_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(costmap_gen_end - costmap_gen_start);
    RCLCPP_INFO(this->get_logger(), "Occupancy grid generation time: %ld ms", costmap_gen_duration.count());

    // 记录占用栅格地图信息
    auto stats_start = std::chrono::high_resolution_clock::now();
    int non_zero_cells = 0;
    for (int i = 0; i < costmap.rows(); i++) {
        for (int j = 0; j < costmap.cols(); j++) {
            if (costmap(i, j) > 0) {
                non_zero_cells++;
            }
        }
    }
    auto stats_end = std::chrono::high_resolution_clock::now();
    auto stats_duration = std::chrono::duration_cast<std::chrono::milliseconds>(stats_end - stats_start);
    RCLCPP_INFO(this->get_logger(), "Statistics computation time: %ld ms", stats_duration.count());

    RCLCPP_INFO(this->get_logger(), "Generated occupancy grid with %d non-zero cells out of %dx%d", non_zero_cells,
                costmap.rows(), costmap.cols());

    // 总的函数耗时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Total generatePointsCostmap time: %ld ms", total_duration.count());

    return costmap;
}

/**
 * @brief 生成组合占用栅格地图
 * @return 组合后的占用栅格地图矩阵
 *
 * 将多个图层的占用栅格地图组合成一个最终的占用栅格地图
 * 目前仅返回点云图层，未来可扩展为组合多个图层
 */
Eigen::MatrixXf CostmapGenerator::generateCombinedCostmap() {
    // For now, we only have points occupancy grid, so just return it
    return costmap_[LayerName::points];
}

} // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_generator::CostmapGenerator)