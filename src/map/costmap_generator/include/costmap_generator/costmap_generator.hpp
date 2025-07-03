#ifndef COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include "costmap_generator/grid_map.hpp"
#include "costmap_generator/points_to_costmap.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

namespace costmap_generator {
/**
 * @class CostmapGenerator
 * @brief 代价地图生成器类
 *
 * 该类负责从点云数据生成代价地图，支持多种参数配置和发布多种格式的地图
 */
class CostmapGenerator : public rclcpp::Node {
  public:
    /**
     * @brief 构造函数
     * @param node_options ROS2节点选项
     */
    explicit CostmapGenerator(const rclcpp::NodeOptions &node_options);

  private:
    // Parameters
    double update_rate_;          ///< 更新频率 (Hz)
    double grid_min_value_;       ///< 网格最小值
    double grid_max_value_;       ///< 网格最大值
    double grid_resolution_;      ///< 网格分辨率 (m)
    double grid_length_x_;        ///< 网格X轴长度 (m)
    double grid_length_y_;        ///< 网格Y轴长度 (m)
    double grid_position_x_;      ///< 网格中心X坐标 (m)
    double grid_position_y_;      ///< 网格中心Y坐标 (m)
    double maximum_height_thres_; ///< 最大高度阈值 (m)
    double minimum_height_thres_; ///< 最小高度阈值 (m)
    double current_vehicle_yaw_;  ///< 保存当前车辆的航向角度
    std::string costmap_frame_;   ///< 代价地图坐标系
    std::string input_frame_;     ///< 输入坐标系
    std::string map_frame_;       ///< 地图坐标系
    bool is_pub_pnt_cloud_;       ///< 是否发布转换后的点云

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_; ///< 点云订阅者
    // rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_costmap_;      ///< 代价地图发布者（已禁用）
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_; ///< 占用栅格发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pnt_cloud_;     ///< 转换后点云发布者

    // Timer
    rclcpp::TimerBase::SharedPtr timer_; ///< 定时器

    // TF
    tf2_ros::Buffer tf_buffer_;              ///< TF缓存
    tf2_ros::TransformListener tf_listener_; ///< TF监听器

    // Data
    sensor_msgs::msg::PointCloud2::SharedPtr points_; ///< 存储接收到的点云数据
    GridMap costmap_;                                 ///< 代价地图

    // Costmap generator
    PointsToCostmap points2costmap_; ///< 点云到代价地图转换器

    // Layer name
    /**
     * @struct LayerName
     * @brief 代价地图图层名称常量
     */
    struct LayerName {
        static constexpr const char *points = "points";     ///< 点云图层名称
        static constexpr const char *combined = "combined"; ///< 组合图层名称
    };

    // Functions
    /**
     * @brief 定时器回调函数
     *
     * 定期执行代价地图生成和发布操作
     */
    void onTimer();

    /**
     * @brief 点云回调函数
     * @param msg 接收到的点云消息
     *
     * 保存接收到的点云数据供后续处理
     */
    void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief 初始化网格地图
     *
     * 创建并设置网格地图的基本属性，包括图层、坐标系、几何形状等
     */
    void initGridmap();

    /**
     * @brief 设置网格中心
     * @param tf 从代价地图坐标系到输入坐标系的变换
     *
     * 根据当前车辆位置更新网格地图中心，实现网格地图跟随车辆移动
     */
    void setGridCenter(const geometry_msgs::msg::TransformStamped &tf);

    /**
     * @brief 发布代价地图
     * @param costmap 要发布的代价地图
     * @param tf 坐标变换信息
     *
     * 将代价地图转换为ROS消息并发布，包括GridMap和OccupancyGrid两种格式
     */
    void publishCostmap(const GridMap &costmap, const geometry_msgs::msg::TransformStamped &tf);

    /**
     * @brief 从点云生成代价地图
     * @param in_points 输入点云
     * @param vehicle_to_map_z 车辆到地图坐标系的Z轴偏移
     * @return 生成的代价地图矩阵
     *
     * 将点云数据转换为代价地图，考虑高度阈值和网格分辨率
     */
    Eigen::MatrixXf generatePointsCostmap(const sensor_msgs::msg::PointCloud2::SharedPtr &in_points,
                                          const double vehicle_to_map_z);

    /**
     * @brief 生成组合代价地图
     * @return 组合后的代价地图矩阵
     *
     * 将多个图层的代价地图组合成一个最终的代价地图
     * 目前仅返回点云图层，未来可扩展为组合多个图层
     */
    Eigen::MatrixXf generateCombinedCostmap();
};
} // namespace costmap_generator

#endif // COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_