#include "costmap_generator/costmap_generator.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    : Node("costmap_generator", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
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

    // Initialize publishers
    pub_costmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>(costmap_topic, 1);
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
 * 定期执行代价地图生成和发布操作
 */
void CostmapGenerator::onTimer() {
    if (!points_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No point cloud received yet");
        return;
    }

    // 获取坐标变换: 从costmap_frame到input_frame的变换
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform =
            tf_buffer_.lookupTransform(costmap_frame_, input_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get transform: %s", ex.what());
        return;
    }

    // 保存当前车辆的航向角度
    current_vehicle_yaw_ = tf2::getYaw(transform.transform.rotation);
    RCLCPP_INFO(this->get_logger(), "Vehicle yaw: %.2f", current_vehicle_yaw_);

    // 重新初始化固定坐标系中的网格地图
    initGridmap();

    // 设置网格中心为车辆当前位置
    setGridCenter(transform);

    // 生成代价地图
    const double vehicle_to_map_z = transform.transform.translation.z;
    costmap_[LayerName::points] = generatePointsCostmap(points_, vehicle_to_map_z);

    // 生成组合代价地图
    costmap_[LayerName::combined] = generateCombinedCostmap();

    // 发布代价地图
    publishCostmap(costmap_, transform);
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
 * @brief 发布代价地图
 * @param costmap 要发布的代价地图
 * @param tf 坐标变换信息
 *
 * 将代价地图转换为ROS消息并发布，包括GridMap和OccupancyGrid两种格式
 */
void CostmapGenerator::publishCostmap(const GridMap &costmap, const geometry_msgs::msg::TransformStamped &tf) {
    // 更新时间戳
    costmap_.setTimestamp(this->now().nanoseconds());

    // Publish GridMap
    grid_map_msgs::msg::GridMap grid_map_msg;
    costmap.toMessage(grid_map_msg);
    pub_costmap_->publish(grid_map_msg);

    // Publish OccupancyGrid
    nav_msgs::msg::OccupancyGrid occupancy_grid;

    try {
        costmap.toOccupancyGrid(LayerName::combined, grid_min_value_, grid_max_value_, occupancy_grid);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting to OccupancyGrid: %s", e.what());
        return;
    }

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

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing OccupancyGrid: origin=(%.2f, %.2f), rotated_offset=(%.2f, %.2f), grid size=%dx%d, resolution=%.2f",
        origin.position.x, origin.position.y, rotated_offset_x, rotated_offset_y, occupancy_grid.info.width,
        occupancy_grid.info.height, occupancy_grid.info.resolution);

    // 发布占用栅格地图
    pub_occupancy_grid_->publish(occupancy_grid);
}

/**
 * @brief 从点云生成代价地图
 * @param in_points 输入点云
 * @param vehicle_to_map_z 车辆到地图坐标系的Z轴偏移
 * @return 生成的代价地图矩阵
 *
 * 将点云数据转换为代价地图，考虑高度阈值和网格分辨率
 */
Eigen::MatrixXf CostmapGenerator::generatePointsCostmap(const sensor_msgs::msg::PointCloud2::SharedPtr &in_points,
                                                        const double vehicle_to_map_z) {

    if (!in_points) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No point cloud data available");
        return costmap_[LayerName::points];
    }

    // 将点云转换到costmap_frame坐标系
    sensor_msgs::msg::PointCloud2 transformed_points;
    geometry_msgs::msg::TransformStamped transform;

    try {
        // 查找从点云坐标系到代价地图坐标系的变换
        transform = tf_buffer_.lookupTransform(costmap_frame_, in_points->header.frame_id, in_points->header.stamp,
                                               tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
        // 如果找不到精确时间戳的变换，尝试使用最新的变换
        try {
            transform = tf_buffer_.lookupTransform(costmap_frame_, in_points->header.frame_id, tf2::TimePointZero,
                                                   tf2::durationFromSec(1.0));
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Using latest transform instead of timestamp-matched transform");
        } catch (tf2::TransformException &ex2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get transform: %s",
                                 ex2.what());
            return costmap_[LayerName::points];
        }
    }

    // 输出点云转换信息
    RCLCPP_INFO(this->get_logger(),
                "Point cloud transform: tx=%.2f, ty=%.2f, tz=%.2f, qw=%.2f, qx=%.2f, qy=%.2f, qz=%.2f",
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
                transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y,
                transform.transform.rotation.z);

    // 手动转换 transform 到 Eigen::Affine3d
    Eigen::Translation3d translation(transform.transform.translation.x, transform.transform.translation.y,
                                     transform.transform.translation.z);

    Eigen::Quaterniond rotation(transform.transform.rotation.w, transform.transform.rotation.x,
                                transform.transform.rotation.y, transform.transform.rotation.z);

    Eigen::Affine3d transform_eigen = translation * rotation;
    Eigen::Matrix4f transform_matrix = transform_eigen.matrix().cast<float>();

    // 变换点云
    pcl_ros::transformPointCloud(transform_matrix, *in_points, transformed_points);

    // 确保转换后的点云frame_id设置正确
    transformed_points.header.frame_id = costmap_frame_;
    transformed_points.header.stamp = this->now();

    // 如果需要，发布转换后的点云
    if (is_pub_pnt_cloud_) {
        pub_pnt_cloud_->publish(transformed_points);
    }

    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
    pcl::fromROSMsg(transformed_points, pcl_pointcloud);

    // 在生成代价地图前清空当前图层
    costmap_[LayerName::points].setZero();

    // 记录调试信息
    RCLCPP_INFO(this->get_logger(), "Processing %zu points for costmap", pcl_pointcloud.size());

    // 计算点云的边界
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

        RCLCPP_INFO(this->get_logger(), "Point cloud bounds: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f]", min_x,
                    max_x, min_y, max_y, min_z, max_z);

        // 输出点云相对于网格中心的位置
        RCLCPP_INFO(this->get_logger(),
                    "Point cloud relative to grid center: x_center=%.2f (offset=%.2f), y_center=%.2f (offset=%.2f)",
                    (min_x + max_x) / 2.0, (min_x + max_x) / 2.0 - grid_position_x_, (min_y + max_y) / 2.0,
                    (min_y + max_y) / 2.0 - grid_position_y_);
    }

    // 从点云生成代价地图，传入车辆航向角用于旋转补偿
    Eigen::MatrixXf costmap = points2costmap_.makeCostmapFromPoints(
        maximum_height_thres_, minimum_height_thres_, grid_min_value_, grid_max_value_, costmap_, LayerName::points,
        pcl_pointcloud, current_vehicle_yaw_);

    // 记录代价地图信息
    int non_zero_cells = 0;
    for (int i = 0; i < costmap.rows(); i++) {
        for (int j = 0; j < costmap.cols(); j++) {
            if (costmap(i, j) > 0) {
                non_zero_cells++;
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Generated costmap with %d non-zero cells out of %dx%d", non_zero_cells,
                costmap.rows(), costmap.cols());

    return costmap;
}

/**
 * @brief 生成组合代价地图
 * @return 组合后的代价地图矩阵
 *
 * 将多个图层的代价地图组合成一个最终的代价地图
 * 目前仅返回点云图层，未来可扩展为组合多个图层
 */
Eigen::MatrixXf CostmapGenerator::generateCombinedCostmap() {
    // For now, we only have points costmap, so just return it
    return costmap_[LayerName::points];
}

} // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_generator::CostmapGenerator)