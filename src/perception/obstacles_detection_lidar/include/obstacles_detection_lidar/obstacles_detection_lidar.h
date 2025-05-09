
#pragma once

#include "bot_msg/msg/obstacle_info.hpp"
#include "bot_msg/msg/obstacles.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#define DEBUG_PUBLISH_POINT_CLOUD 1

class ObstaclesDetectionLidarNode : public rclcpp::Node {
  public:
    ObstaclesDetectionLidarNode();

  private:
    void InitParameters();
    void InitStaticTransformBroadcaster();
    void PointClould2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr pnt_cloud);
    void GNSSCallback(const geometry_msgs::msg::PoseStamped::SharedPtr gnss_msg);
    void RemoveInvalidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void FillAndPublishObstacleMarker(const bot_msg::msg::Obstacles &obstacle_array_msg, int obstacles_type);
    void Obstacle2ENU(const bot_msg::msg::ObstacleInfo &obstacle);

    
    visualization_msgs::msg::Marker MakeObstacleMarker(const bot_msg::msg::ObstacleInfo &obstacle, int obstacles_type);
    visualization_msgs::msg::Marker MakeObstacleMarker(int x, int y, int z, int width, int length, int height,
                                                       int obstacles_type);
    void VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &title, int stage);
    void VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &title);
    void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                           const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_lidar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;

    rclcpp::Publisher<bot_msg::msg::Obstacles>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // 静态坐标转换
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // 静态坐标转换订阅
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr static_transform_sub_;

#if DEBUG_PUBLISH_POINT_CLOUD
    // 在类定义中创建多个点云发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_seg_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_cloud_pub_;
#endif

    // parameters
    double max_height_;          // 最大高度
    double min_height_;          // 最小高度
    double vehicle_height_;      // 车辆高度
    double vehicle_width_;       // 车辆宽
    double vehicle_length_;      // 车辆长
    double radar_height_;        // 毫米波雷达安装高度
    double cluster_tolerance_;   // 聚类距离阈值
    int cluster_min_size_;       // 聚类最小点数
    int cluster_max_size_;       // 聚类最大点数
    float leaf_size_;            // 体素滤波器的叶子大小
    double roi_width_;           // ROI 宽度
    double plane_point_percent_; // 平面点数占比

    bool enable_visualization_;          // 是否开启可视化
    bool enable_use_roi_;                // 是否使用 ROI 过滤
    bool enable_calculate_process_time_; // 是否计算单步处理时间
    bool enable_downsample_;             // 是否进行下采样
    int segment_ground_type_;            // 地面分割算法类型

    std::string frame_id_;           // 坐标系名称
    bool is_use_front_lidar_;        // 是否使用前雷达
    std::string front_lidar_topic_;  // 前雷达的 topic
    std::string front_lidar_frame_id_; // 前雷达的坐标系名称
    bool is_use_left_lidar_;         // 是否使用左雷达
    std::string left_lidar_topic_;   // 左雷达的 topic
    std::string left_lidar_frame_id_; // 左雷达的坐标系名称
    bool is_use_right_lidar_;        // 是否使用右雷达
    std::string right_lidar_topic_;  // 右雷达的 topic
    std::string right_lidar_frame_id_; // 右雷达的坐标系名称
    // GNSS设备参数
    bool is_use_gnss_;               // 是否使用GNSS设备
    std::string gnss_topic_;         // GNSS设备的 topic
    std::string gnss_frame_id_;      // GNSS设备的坐标系名称
    geometry_msgs::msg::PoseStamped gnss_msg_; // GNSS设备消息

    // 车辆的相对base坐标系
    std::string base_frame_id_;      // 车辆的相对base坐标系

    // 消息标志位
    bool is_gnss_msg_received_;      // GNSS设备消息标志位
};