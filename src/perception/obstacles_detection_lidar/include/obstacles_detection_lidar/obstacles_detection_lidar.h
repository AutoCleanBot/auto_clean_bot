
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
    void RemoveInvalidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void FillAndPublishObstacleMarker(const bot_msg::msg::Obstacles &obstacle_array_msg, int obstacles_type);
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
    // TODO：相机的订阅和处理
    rclcpp::Publisher<bot_msg::msg::Obstacles>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

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

    double lidar_base_x_;     // 雷达到基础坐标系的 x 方向偏移
    double lidar_base_y_;     // 雷达到基础坐标系的 y 方向偏移
    double lidar_base_z_;     // 雷达到基础坐标系的 z 方向偏移
    double lidar_base_yaw_;   // 雷达到基础坐标系的 yaw 旋转值,弧度
    double lidar_base_pitch_; // 雷达到基础坐标系的 pitch 旋转值,弧度
    double lidar_base_roll_;  // 雷达到基础坐标系的 roll 旋转值,弧度
    double radar_base_x_;     // 毫米波到基础坐标系的 x 方向偏移
    double radar_base_y_;     // 毫米波到基础坐标系的 y 方向偏移
    double radar_base_z_;     // 毫米波到基础坐标系的 z 方向偏移
    double radar_base_yaw_;   // 毫米波到基础坐标系的 yaw 旋转值,弧度
    double radar_base_pitch_; // 毫米波到基础坐标系的 pitch 旋转值,弧度
    double radar_base_roll_;  // 毫米波到基础坐标系的 roll 旋转值,弧度

    bool enable_visualization_;          // 是否开启可视化
    bool enable_use_roi_;                // 是否使用 ROI 过滤
    bool enable_calculate_process_time_; // 是否计算单步处理时间
    bool enable_downsample_;             // 是否进行下采样
    int segment_ground_type_;            // 地面分割算法类型

    std::string frame_id_;           // 坐标系名称
    bool is_use_front_lidar_;        // 是否使用前雷达
    std::string front_lidar_topic_;  // 前雷达的 topic
    bool is_use_left_lidar_;         // 是否使用左雷达
    std::string left_lidar_topic_;   // 左雷达的 topic
    bool is_use_right_lidar_;        // 是否使用右雷达
    std::string right_lidar_topic_;  // 右雷达的 topic
    bool is_use_front_camera_;       // 是否使用前相机
    std::string front_camera_topic_; // 前相机的 topic
};