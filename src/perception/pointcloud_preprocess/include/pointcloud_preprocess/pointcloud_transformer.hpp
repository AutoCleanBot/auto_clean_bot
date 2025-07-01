#ifndef POINTCLOUD_PREPROCESS__POINTCLOUD_TRANSFORMER_HPP_
#define POINTCLOUD_PREPROCESS__POINTCLOUD_TRANSFORMER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "pointcloud_preprocess/visibility_control.hpp"

namespace pointcloud_preprocess
{

class PointCloudTransformerNode : public rclcpp::Node
{
public:
  POINTCLOUD_PREPROCESS_PUBLIC
  explicit PointCloudTransformerNode(const rclcpp::NodeOptions & options);

private:
  // 参数
  double update_rate_;
  std::string input_frame_;
  std::string output_frame_;
  bool use_sensor_frame_;
  double timeout_;
  bool use_latest_transforms_;
  
  // 话题参数
  std::string input_topic_;
  std::string output_topic_;
  
  // 车辆过滤参数
  bool filter_vehicle_points_;
  double vehicle_front_length_;  // 车辆前部长度
  double vehicle_back_length_;   // 车辆后部长度
  double vehicle_left_width_;    // 车辆左侧宽度
  double vehicle_right_width_;   // 车辆右侧宽度
  double vehicle_top_height_;    // 车辆上部高度
  double vehicle_bottom_height_; // 车辆下部高度
  double vehicle_x_offset_;
  double vehicle_y_offset_;
  double vehicle_z_offset_;
  double vehicle_length_margin_;
  double vehicle_width_margin_;
  double vehicle_height_margin_;
  
  // 订阅和发布
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud_pub_;
  
  // TF监听
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 回调函数
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timerCallback();
  
  // 处理点云
  void transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud);
  
  // 过滤车辆内部点云
  void filterVehiclePoints(sensor_msgs::msg::PointCloud2 & cloud) const;
};

}  // namespace pointcloud_preprocess

#endif  // POINTCLOUD_PREPROCESS__POINTCLOUD_TRANSFORMER_HPP_ 