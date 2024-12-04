/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NDT_MAPPING_H_
#define NDT_MAPPING_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

enum class MethodType {
    kPclGeneric = 0,
    kPclAnh = 1,
    kPclAnhGpu = 2,
    kPclOpenmp = 3,
};

class NdtMapping : public rclcpp::Node {
  public:
    explicit NdtMapping();

  private:
    // Constants
    static constexpr double kMinScanRange = 1.0;    // [m]
    static constexpr double kMaxScanRange = 120.0;  // [m]
    static constexpr double kMinAddScanShift = 1.0; // [m]

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr save_map_timer_;

    // Transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters
    MethodType method_type_;
    bool use_odom_;
    bool use_imu_;
    bool imu_upside_down_; // IMU是否倒置
    std::string imu_topic_;
    bool incremental_voxel_update_;

    // NDT object
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

    // Point cloud map
    pcl::PointCloud<pcl::PointXYZI> map_;

    // Pose variables
    Pose previous_pose_; // 之前ndt 估计出的位姿
    Pose current_pose_;  // 当前ndt 估计出的位姿
    Pose guess_pose_;
    Pose added_pose_;
    Pose localizer_pose_;
    Pose ndt_pose_;
    Pose current_pose_imu_;
    Pose guess_pose_imu_;

    // Transform matrices
    Eigen::Matrix4f tf_base_to_lidar_; // base to lidar transform
    Eigen::Matrix4f tf_lidar_to_base_; // lidar to base transform

    // IMU and odometry data
    sensor_msgs::msg::Imu latest_imu_data_;
    nav_msgs::msg::Odometry latest_odom_data_;

    // Offset and velocity variables
    double offset_imu_x_ = 0.0;
    double offset_imu_y_ = 0.0;
    double offset_imu_z_ = 0.0;
    double offset_imu_roll_ = 0.0;
    double offset_imu_pitch_ = 0.0;
    double offset_imu_yaw_ = 0.0;

    double current_velocity_imu_x_ = 0.0;
    double current_velocity_imu_y_ = 0.0;
    double current_velocity_imu_z_ = 0.0;

    // ndt parameters
    double trans_eps_;
    double step_size_;
    double ndt_res_;
    int max_iter_;

    // Callback functions
    void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr input);
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr input);

    // Initialization functions
    void InitializePoses();
    void InitParams();
    void SetupNdt();
    void InitializeTransforms();

    // Processing functions
    void ImuCalc(const rclcpp::Time &current_time);
    void OdomCalc(const rclcpp::Time &current_time);
    void UpdateCurrentPose(const Eigen::Matrix4f &transform);
    void PublishCurrentPose();
    void UpdateMap(const pcl::PointCloud<pcl::PointXYZI> &scan);

    // Utility functions
    Eigen::Matrix4f GuessInit();
    Eigen::Matrix4f CreateMatrix(const Pose &p);
    void ProcessImuUpsideDown(sensor_msgs::msg::Imu::SharedPtr input);
    double WrapToPi(double angle);
    double CalcDiffForRadian(double lhs_rad, double rhs_rad);
    void SaveMap();
};

#endif // NDT_MAPPING_H_