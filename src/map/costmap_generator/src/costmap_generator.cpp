#include "costmap_generator/costmap_generator.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
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

    // Get transform from costmap_frame to vehicle_frame
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform =
            tf_buffer_.lookupTransform(costmap_frame_, input_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get transform: %s", ex.what());
        return;
    }

    // Set grid center based on current vehicle position
    setGridCenter(transform);

    // Generate costmap from points
    const double vehicle_to_map_z = transform.transform.translation.z;
    costmap_[LayerName::points] = generatePointsCostmap(points_, vehicle_to_map_z);

    // Generate combined costmap
    costmap_[LayerName::combined] = generateCombinedCostmap();

    // Publish costmap
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
}

/**
 * @brief 设置网格中心
 * @param tf 从代价地图坐标系到输入坐标系的变换
 * 
 * 根据当前车辆位置更新网格地图中心，实现网格地图跟随车辆移动
 */
void CostmapGenerator::setGridCenter(const geometry_msgs::msg::TransformStamped &tf) {
    // Get vehicle position in costmap frame
    const double vehicle_x = tf.transform.translation.x;
    const double vehicle_y = tf.transform.translation.y;

    // Calculate offset from current grid center
    const double offset_x = vehicle_x - grid_position_x_;
    const double offset_y = vehicle_y - grid_position_y_;

    // If offset is larger than grid_resolution * 2, update grid center
    if (std::abs(offset_x) > grid_resolution_ * 2.0 || std::abs(offset_y) > grid_resolution_ * 2.0) {
        // Move grid center by a multiple of grid_resolution_
        const double move_x = std::floor(offset_x / grid_resolution_) * grid_resolution_;
        const double move_y = std::floor(offset_y / grid_resolution_) * grid_resolution_;

        grid_position_x_ += move_x;
        grid_position_y_ += move_y;

        costmap_.move(Eigen::Vector2d(grid_position_x_, grid_position_y_));
    }
}

/**
 * @brief 发布代价地图
 * @param costmap 要发布的代价地图
 * @param tf 坐标变换信息
 * 
 * 将代价地图转换为ROS消息并发布，包括GridMap和OccupancyGrid两种格式
 */
void CostmapGenerator::publishCostmap(const GridMap &costmap, const geometry_msgs::msg::TransformStamped &tf) {
    // Publish GridMap
    grid_map_msgs::msg::GridMap grid_map_msg;
    costmap.toMessage(grid_map_msg);
    pub_costmap_->publish(grid_map_msg);

    // Publish OccupancyGrid
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    costmap.toOccupancyGrid(LayerName::combined, grid_min_value_, grid_max_value_, occupancy_grid);
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
    // Transform pointcloud to costmap frame
    sensor_msgs::msg::PointCloud2 transformed_points;
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = tf_buffer_.lookupTransform(costmap_frame_, in_points->header.frame_id, tf2::TimePointZero,
                                               tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get transform: %s", ex.what());
        return costmap_[LayerName::points];
    }

    // 手动转换 transform 到 Eigen::Affine3d
    Eigen::Translation3d translation(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
    
    Eigen::Quaterniond rotation(
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z);
    
    Eigen::Affine3d transform_eigen = translation * rotation;
    Eigen::Matrix4f transform_matrix = transform_eigen.matrix().cast<float>();

    // Transform pointcloud
    pcl_ros::transformPointCloud(transform_matrix, *in_points, transformed_points);

    // 确保转换后的点云frame_id设置正确
    transformed_points.header.frame_id = costmap_frame_;
    transformed_points.header.stamp = this->now();

    // 如果需要，发布转换后的点云
    if (is_pub_pnt_cloud_) {
        pub_pnt_cloud_->publish(transformed_points);
    }

    // Convert to PCL pointcloud
    pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
    pcl::fromROSMsg(transformed_points, pcl_pointcloud);

    // Generate costmap from pointcloud
    return points2costmap_.makeCostmapFromPoints(maximum_height_thres_, minimum_height_thres_, grid_min_value_,
                                                 grid_max_value_, costmap_, LayerName::points, pcl_pointcloud);
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