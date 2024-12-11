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

// TODO ndt 配准耗时

#include "ndt_mapping/ndt_mapping.h"
#include <pcl/io/pcd_io.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

NdtMapping::NdtMapping() : Node("ndt_mapping") {
    InitParams();
    // Initialize transforms
    InitializeTransforms();

    // Setup publishers
    ndt_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ndt_map", 10);
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

    // Setup subscribers with QoS settings
    auto points_sub_opt = rclcpp::SensorDataQoS();
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic_, points_sub_opt, std::bind(&NdtMapping::PointsCallback, this, std::placeholders::_1));

    auto imu_sub_opt = rclcpp::SensorDataQoS();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_sub_opt, std::bind(&NdtMapping::ImuCallback, this, std::placeholders::_1));

    auto odom_sub_opt = rclcpp::SensorDataQoS();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vehicle/odom", odom_sub_opt, std::bind(&NdtMapping::OdomCallback, this, std::placeholders::_1));

    // initialize timer
    save_map_timer_ = this->create_wall_timer(std::chrono::seconds(60), std::bind(&NdtMapping::SaveMap, this));

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize pose variables
    InitializePoses();

    // Setup NDT
    SetupNdt();

    RCLCPP_INFO(this->get_logger(), "NDT Mapping node initialized");
}

/**
 * @brief Initialize parameters
 */
void NdtMapping::InitParams() {
    // Initialize parameters
    this->declare_parameter("method_type", 0);
    this->declare_parameter("use_odom", false);
    this->declare_parameter("use_imu", false);
    this->declare_parameter("imu_upside_down", false);
    this->declare_parameter("imu_topic", "/imu_raw");
    this->declare_parameter("points_topic", "/points_raw");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("incremental_voxel_update", false);
    this->declare_parameter("trans_eps", 0.01);
    this->declare_parameter("step_size", 0.1);
    this->declare_parameter("ndt_res", 1.0);

    this->declare_parameter("max_iter", 30);

    // Get parameters
    int method_type_tmp;
    this->get_parameter("method_type", method_type_tmp);
    method_type_ = static_cast<MethodType>(method_type_tmp);
    this->get_parameter("use_odom", use_odom_);
    this->get_parameter("use_imu", use_imu_);
    this->get_parameter("imu_upside_down", imu_upside_down_);
    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("points_topic", points_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("incremental_voxel_update", incremental_voxel_update_);
    this->get_parameter("trans_eps", trans_eps_);
    this->get_parameter("step_size", step_size_);
    this->get_parameter("ndt_res", ndt_res_);
    this->get_parameter("max_iter", max_iter_);

    // log parameters
    RCLCPP_INFO(this->get_logger(), "method_type: %d", method_type_);
    RCLCPP_INFO(this->get_logger(), "use_odom: %d", use_odom_);
    RCLCPP_INFO(this->get_logger(), "use_imu: %d", use_imu_);
    RCLCPP_INFO(this->get_logger(), "imu_upside_down: %d", imu_upside_down_);
    RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "points_topic: %s", points_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "incremental_voxel_update: %d", incremental_voxel_update_);
    RCLCPP_INFO(this->get_logger(), "trans_eps: %f", trans_eps_);
    RCLCPP_INFO(this->get_logger(), "step_size: %f", step_size_);
    RCLCPP_INFO(this->get_logger(), "ndt_res: %f", ndt_res_);
    RCLCPP_INFO(this->get_logger(), "max_iter: %d", max_iter_);
}

void NdtMapping::InitializePoses() {
    Pose zero_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    previous_pose_ = zero_pose;
    current_pose_ = zero_pose;
    guess_pose_ = zero_pose;
    added_pose_ = zero_pose;
    ndt_pose_ = zero_pose;
    current_pose_imu_ = zero_pose;
    guess_pose_imu_ = zero_pose;
}

/**
 * @brief Setup NDT parameters
 */
void NdtMapping::SetupNdt() {
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(0.1);
    ndt_.setResolution(2.0);
    ndt_.setMaximumIterations(30);
    // Add additional NDT parameters for better robustness
    ndt_.setOulierRatio(0.55); // Increase outlier ratio threshold
}

// 从本地参数服务器中获取变换参数，初始化变换矩阵
/**
 * @brief 从本地参数中获取
 */
void NdtMapping::InitializeTransforms() {
    // TODO 从Transform节点获取变换参数

    // tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw 是激光雷达坐标系相对于车身底盘坐标系的位姿
    // Get transform parameters
    this->declare_parameter("tf_x", 0.0);
    this->declare_parameter("tf_y", 0.0);
    this->declare_parameter("tf_z", 0.0);
    this->declare_parameter("tf_roll", 0.0);
    this->declare_parameter("tf_pitch", 0.0);
    this->declare_parameter("tf_yaw", 0.0);

    double tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
    this->get_parameter("tf_x", tf_x);
    this->get_parameter("tf_y", tf_y);
    this->get_parameter("tf_z", tf_z);
    this->get_parameter("tf_roll", tf_roll);
    this->get_parameter("tf_pitch", tf_pitch);
    this->get_parameter("tf_yaw", tf_yaw);

    RCLCPP_INFO(this->get_logger(), "tf_x: %f", tf_x);
    RCLCPP_INFO(this->get_logger(), "tf_y: %f", tf_y);
    RCLCPP_INFO(this->get_logger(), "tf_z: %f", tf_z);
    RCLCPP_INFO(this->get_logger(), "tf_roll: %f", tf_roll);
    RCLCPP_INFO(this->get_logger(), "tf_pitch: %f", tf_pitch);
    RCLCPP_INFO(this->get_logger(), "tf_yaw: %f", tf_yaw);

    // Create transform matrices
    Eigen::Translation3f tl_btol(tf_x, tf_y, tf_z);

    Eigen::AngleAxisf rot_x_btol(tf_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(tf_yaw, Eigen::Vector3f::UnitZ());

    // 激光雷达相对于车身底盘坐标系的位姿
    tf_lidar_to_base_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    // 日志输出激光雷达相对于车身底盘坐标系的位姿
    RCLCPP_INFO(this->get_logger(), "tf_lidar_to_base: [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]",
                tf_lidar_to_base_(0, 0), tf_lidar_to_base_(0, 1), tf_lidar_to_base_(0, 2), tf_lidar_to_base_(0, 3),
                tf_lidar_to_base_(1, 0), tf_lidar_to_base_(1, 1), tf_lidar_to_base_(1, 2), tf_lidar_to_base_(1, 3),
                tf_lidar_to_base_(2, 0), tf_lidar_to_base_(2, 1), tf_lidar_to_base_(2, 2), tf_lidar_to_base_(2, 3),
                tf_lidar_to_base_(3, 0), tf_lidar_to_base_(3, 1), tf_lidar_to_base_(3, 2), tf_lidar_to_base_(3, 3));

    // 底盘坐标系到激光雷达的变换矩阵
    if (tf_base_to_lidar_.determinant() != 0) {
        tf_base_to_lidar_ = tf_base_to_lidar_.inverse().eval();
    } else {
        tf_base_to_lidar_ = Eigen::Matrix4f::Identity();
        RCLCPP_ERROR(this->get_logger(), "tf_base_to_lidar_ matrix is singular, cannot invert.");
    }
    // 日志输出激光雷达相对于车身底盘坐标系的位姿
    RCLCPP_INFO(this->get_logger(), "tf_base_to_lidar_: [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]",
                tf_base_to_lidar_(0, 0), tf_base_to_lidar_(0, 1), tf_base_to_lidar_(0, 2), tf_base_to_lidar_(0, 3),
                tf_base_to_lidar_(1, 0), tf_base_to_lidar_(1, 1), tf_base_to_lidar_(1, 2), tf_base_to_lidar_(1, 3),
                tf_base_to_lidar_(2, 0), tf_base_to_lidar_(2, 1), tf_base_to_lidar_(2, 2), tf_base_to_lidar_(2, 3),
                tf_base_to_lidar_(3, 0), tf_base_to_lidar_(3, 1), tf_base_to_lidar_(3, 2), tf_base_to_lidar_(3, 3));
}

void NdtMapping::SaveMap() {
    std::string filename = "ndt_map.pcd";
    pcl::io::savePCDFileASCII(filename, map_);
    RCLCPP_INFO(this->get_logger(), "Saved map to %s", filename.c_str());
}

/**
 * @brief 移除无效点
 *
 * @param cloud 点云数据
 */
void NdtMapping::RemoveInvalidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
                                       [](const pcl::PointXYZ &pnt) {
                                           return std::isnan(pnt.x) || std::isnan(pnt.y) || std::isnan(pnt.z) ||
                                                  !std::isfinite(pnt.x) || !std::isfinite(pnt.y) ||
                                                  !std::isfinite(pnt.z);
                                       }),
                        cloud->points.end());
}

// Add new validation function implementation
bool NdtMapping::ValidateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    if (!cloud || cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty or null point cloud received");
        return false;
    }

    if (!cloud->is_dense) {
        RCLCPP_WARN(this->get_logger(), "Point cloud is not dense, may contain invalid points");
    }

    return true;
}

// Add new preprocessing function implementation
void NdtMapping::PreprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // Remove NaN and infinite points
    RemoveInvalidPoints(cloud);

    // Create a new filtered cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->reserve(cloud->size());

    // Filter based on distance
    for (const auto &point : cloud->points) {
        double distance = ComputePointDistance(point);
        if (distance >= kMinScanRange && distance <= kMaxScanRange) {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    // Replace the input cloud with the filtered one
    cloud = filtered_cloud;
}

double NdtMapping::ComputePointDistance(const pcl::PointXYZ &point) {
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

// Modify the PointsCallback function to use the new validation and preprocessing
// void NdtMapping::PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input) {
//     // RCLCPP_INFO(this->get_logger(), "Received a point cloud");

//     rclcpp::Time current_time = rclcpp::Clock().now();

//     // Convert ROS message to PCL point cloud
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*input, *cloud);

//     // Validate the input cloud
//     if (!ValidateCloud(cloud)) {
//         return;
//     }

//     // RCLCPP_INFO(this->get_logger(), "Before preprocessing: %d points", cloud->points.size());

//     // 将点云数据从激光雷达坐标系转换到车身底盘坐标系
//     pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::transformPointCloud(*cloud, *transformed_cloud, tf_lidar_to_base_);
//     cloud = transformed_cloud;

//     // Preprocess the point cloud
//     PreprocessPointCloud(cloud);

//     // RCLCPP_INFO(this->get_logger(), "After preprocessing: %d points", cloud->points.size());

//     // Check if we have enough points to proceed
//     if (cloud->points.size() < 100) {
//         RCLCPP_WARN(this->get_logger(), "Too few points after filtering (%lu), skipping frame",
//         cloud->points.size()); return;
//     }

//     // Perform NDT matching
//     if (!map_.empty()) {
//         // 对输入地图进行体素过滤
//         pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
//         voxel_grid_filter.setInputCloud(cloud);
//         voxel_grid_filter.setLeafSize(0.5, 0.5, 0.5);
//         voxel_grid_filter.filter(*cloud);
//         // 进行NDT匹配
//         try {
//             ndt_.setInputSource(cloud);
//             pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//             // RCLCPP_INFO(this->get_logger(), "NDT matching started");
//             Eigen::Matrix4f guess_trans = GuessInit();
//             ndt_.align(*output_cloud, guess_trans);
//             // RCLCPP_INFO(this->get_logger(), "NDT has converged: %d", ndt_.hasConverged());
//             double fitness_score = ndt_.getFitnessScore();
//             // RCLCPP_INFO(this->get_logger(), "Fitness score: %f", fitness_score);
//             if (ndt_.hasConverged()) {
//                 // 得到最终的位姿,此时的位置为base_link在map坐标系下的位置
//                 auto ndt_pose = ndt_.getFinalTransformation();
//                 UpdateCurrentPose(ndt_pose);
//                 PublishCurrentPose();
//                 UpdateMap(*cloud);
//             }
//         } catch (const std::exception &e) {
//             RCLCPP_ERROR(this->get_logger(), "Error during NDT alignment: %s", e.what());
//         }
//     } else {
//         // First scan, initialize map
//         map_ = *cloud;
//         ndt_.setInputTarget(map_.makeShared());
//         PublishMap();
//         RCLCPP_INFO(this->get_logger(), "Map initialized with %d points", map_.points.size());
//     }

//     rclcpp::Time time_last_publish = rclcpp::Clock().now();
//     rclcpp::Duration duration = current_time - time_last_publish;
//     double diff_time = duration.seconds();
//     RCLCPP_INFO(this->get_logger(), "diff_time: %f", diff_time);
// }
void NdtMapping::PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    rclcpp::Time start_time = clock.now();
    rclcpp::Time step_time;

    // Step 1: Convert ROS message to PCL point cloud
    step_time = clock.now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    RCLCPP_INFO(this->get_logger(), "Time for conversion to PCL: %f ms", (clock.now() - step_time).seconds() * 1000);

    // Step 2: Validate the input cloud
    step_time = clock.now();
    if (!ValidateCloud(cloud)) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Time for validation: %f ms", (clock.now() - step_time).seconds() * 1000);

    // Step 3: Transform point cloud
    step_time = clock.now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, tf_lidar_to_base_);
    cloud = transformed_cloud;
    RCLCPP_INFO(this->get_logger(), "Time for transformation: %f ms", (clock.now() - step_time).seconds() * 1000);

    // Step 4: Preprocess the point cloud
    step_time = clock.now();
    PreprocessPointCloud(cloud);
    RCLCPP_INFO(this->get_logger(), "Time for preprocessing: %f ms", (clock.now() - step_time).seconds() * 1000);

    // Step 5: Check if we have enough points
    if (cloud->points.size() < 100) {
        RCLCPP_WARN(this->get_logger(), "Too few points after filtering, skipping frame");
        return;
    }

    // Step 6: Perform NDT matching
    step_time = clock.now();
    if (!map_.empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(cloud);
        voxel_grid_filter.setLeafSize(0.5, 0.5, 0.5);
        voxel_grid_filter.filter(*cloud);
        RCLCPP_INFO(this->get_logger(), "Time for voxel grid filtering: %f ms",
                    (clock.now() - step_time).seconds() * 1000);

        try {
            step_time = clock.now();
            ndt_.setInputSource(cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Matrix4f guess_trans = GuessInit();
            ndt_.align(*output_cloud, guess_trans);
            RCLCPP_INFO(this->get_logger(), "Time for NDT alignment: %f ms",
                        (clock.now() - step_time).seconds() * 1000);

            if (ndt_.hasConverged()) {
                auto ndt_pose = ndt_.getFinalTransformation();
                UpdateCurrentPose(ndt_pose);
                PublishCurrentPose();
                UpdateMap(*cloud);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error during NDT alignment: %s", e.what());
        }
    } else {
        map_ = *cloud;
        ndt_.setInputTarget(map_.makeShared());
        PublishMap();
        RCLCPP_INFO(this->get_logger(), "Map initialized with %d points", map_.points.size());
    }

    RCLCPP_INFO(this->get_logger(), "Total time for PointsCallback: %f ms",
                (clock.now() - start_time).seconds() * 1000);
}

/**
 * @brief IMU 回调函数
 */
void NdtMapping::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr input) {
    RCLCPP_INFO(this->get_logger(), "Received an IMU message");
    if (imu_upside_down_) {
        ProcessImuUpsideDown(input);
    }

    rclcpp::Time current_time(input->header.stamp);
    static rclcpp::Time previous_time = current_time;
    rclcpp::Duration duration = current_time - previous_time;
    double diff_time = duration.seconds();
    // Convert quaternion to RPY
    tf2::Quaternion imu_orientation;
    tf2::fromMsg(input->orientation, imu_orientation);
    double imu_roll, imu_pitch, imu_yaw;
    tf2::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

    imu_roll = WrapToPi(imu_roll);
    imu_pitch = WrapToPi(imu_pitch);
    imu_yaw = WrapToPi(imu_yaw);

    static double previous_imu_roll = imu_roll;
    static double previous_imu_pitch = imu_pitch;
    static double previous_imu_yaw = imu_yaw;

    const double diff_imu_roll = CalcDiffForRadian(imu_roll, previous_imu_roll);
    const double diff_imu_pitch = CalcDiffForRadian(imu_pitch, previous_imu_pitch);
    const double diff_imu_yaw = CalcDiffForRadian(imu_yaw, previous_imu_yaw);

    // Update IMU data
    latest_imu_data_.header = input->header;
    latest_imu_data_.linear_acceleration = input->linear_acceleration;
    latest_imu_data_.linear_acceleration.y = 0;
    latest_imu_data_.linear_acceleration.z = 0;

    if (diff_time != 0) {
        latest_imu_data_.angular_velocity.x = diff_imu_roll / diff_time;
        latest_imu_data_.angular_velocity.y = diff_imu_pitch / diff_time;
        latest_imu_data_.angular_velocity.z = diff_imu_yaw / diff_time;
    } else {
        latest_imu_data_.angular_velocity.x = 0;
        latest_imu_data_.angular_velocity.y = 0;
        latest_imu_data_.angular_velocity.z = 0;
    }

    ImuCalc(input->header.stamp);

    previous_time = current_time;
    previous_imu_roll = imu_roll;
    previous_imu_pitch = imu_pitch;
    previous_imu_yaw = imu_yaw;
}

void NdtMapping::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr input) {
    latest_odom_data_ = *input;
    OdomCalc(input->header.stamp);
}

/**
 * @brief 基于IMU数据和时间差，计算当前位姿
 */
void NdtMapping::ImuCalc(const rclcpp::Time &current_time) {
    static rclcpp::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).seconds();

    double diff_imu_roll = latest_imu_data_.angular_velocity.x * diff_time;
    double diff_imu_pitch = latest_imu_data_.angular_velocity.y * diff_time;
    double diff_imu_yaw = latest_imu_data_.angular_velocity.z * diff_time;

    current_pose_imu_.roll += diff_imu_roll;
    current_pose_imu_.pitch += diff_imu_pitch;
    current_pose_imu_.yaw += diff_imu_yaw;

    // Transform accelerations to global frame
    double acc_x1 = latest_imu_data_.linear_acceleration.x;
    double acc_y1 = std::cos(current_pose_imu_.roll) * latest_imu_data_.linear_acceleration.y -
                    std::sin(current_pose_imu_.roll) * latest_imu_data_.linear_acceleration.z;
    double acc_z1 = std::sin(current_pose_imu_.roll) * latest_imu_data_.linear_acceleration.y +
                    std::cos(current_pose_imu_.roll) * latest_imu_data_.linear_acceleration.z;

    double acc_x2 = std::sin(current_pose_imu_.pitch) * acc_z1 + std::cos(current_pose_imu_.pitch) * acc_x1;
    double acc_y2 = acc_y1;
    double acc_z2 = std::cos(current_pose_imu_.pitch) * acc_z1 - std::sin(current_pose_imu_.pitch) * acc_x1;

    double acc_x = std::cos(current_pose_imu_.yaw) * acc_x2 - std::sin(current_pose_imu_.yaw) * acc_y2;
    double acc_y = std::sin(current_pose_imu_.yaw) * acc_x2 + std::cos(current_pose_imu_.yaw) * acc_y2;
    double acc_z = acc_z2;

    // Update offset and velocity based on IMU
    offset_imu_x_ += current_velocity_imu_x_ * diff_time + acc_x * diff_time * diff_time / 2.0;
    offset_imu_y_ += current_velocity_imu_y_ * diff_time + acc_y * diff_time * diff_time / 2.0;
    offset_imu_z_ += current_velocity_imu_z_ * diff_time + acc_z * diff_time * diff_time / 2.0;

    current_velocity_imu_x_ += acc_x * diff_time;
    current_velocity_imu_y_ += acc_y * diff_time;
    current_velocity_imu_z_ += acc_z * diff_time;

    guess_pose_imu_ = previous_pose_;
    guess_pose_imu_.x += offset_imu_x_;
    guess_pose_imu_.y += offset_imu_y_;
    guess_pose_imu_.z += offset_imu_z_;
    guess_pose_imu_.roll += offset_imu_roll_;
    guess_pose_imu_.pitch += offset_imu_pitch_;
    guess_pose_imu_.yaw += offset_imu_yaw_;

    previous_time = current_time;
}

void NdtMapping::OdomCalc(const rclcpp::Time &current_time) {
    static rclcpp::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).seconds();

    double diff_odom_roll = latest_odom_data_.twist.twist.angular.x * diff_time;
    double diff_odom_pitch = latest_odom_data_.twist.twist.angular.y * diff_time;
    double diff_odom_yaw = latest_odom_data_.twist.twist.angular.z * diff_time;

    current_pose_.roll += diff_odom_roll;
    current_pose_.pitch += diff_odom_pitch;
    current_pose_.yaw += diff_odom_yaw;

    double diff_distance = latest_odom_data_.twist.twist.linear.x * diff_time;
    double dx = diff_distance * cos(-current_pose_.pitch) * cos(current_pose_.yaw);
    double dy = diff_distance * cos(-current_pose_.pitch) * sin(current_pose_.yaw);
    double dz = diff_distance * sin(-current_pose_.pitch);

    current_pose_.x += dx;
    current_pose_.y += dy;
    current_pose_.z += dz;

    previous_time = current_time;
}

/**
 * @brief Update current pose based on estimated transform
 *          - 基于NDT算法计算出的变换矩阵，更新当前位姿
 * @param transform 来自于NDT算法的计算出的当前点云相对于地图的位姿
 */
void NdtMapping::UpdateCurrentPose(const Eigen::Matrix4f &transform) {
    auto current_time = rclcpp::Clock().now();
    static rclcpp::Time previous_time = current_time;
    rclcpp::Duration duration = current_time - previous_time;
    double diff_time = duration.seconds();

    // 括号内是行列索引
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3f translation = transform.block<3, 1>(0, 3);

    current_pose_.x = translation(0);
    current_pose_.y = translation(1);
    current_pose_.z = translation(2);
    // 从旋转矩阵中提取欧拉角
    Eigen::Vector3f euler = rotation.eulerAngles(0, 1, 2);
    current_pose_.roll = euler(0);
    current_pose_.pitch = euler(1);
    current_pose_.yaw = euler(2);

    diff_x_ = current_pose_.x - previous_pose_.x;
    diff_y_ = current_pose_.y - previous_pose_.y;
    diff_z_ = current_pose_.z - previous_pose_.z;
    diff_yaw_ = WrapToPi(current_pose_.yaw - previous_pose_.yaw);

    current_velocity_x_ = diff_x_ / diff_time;
    current_velocity_y_ = diff_y_ / diff_time;
    current_velocity_z_ = diff_z_ / diff_time;

    previous_pose_ = current_pose_;
    previous_time = current_time;

    if (use_imu_) {
        offset_imu_x_ = 0.0;
        offset_imu_y_ = 0.0;
        offset_imu_z_ = 0.0;
        offset_imu_roll_ = 0.0;
        offset_imu_pitch_ = 0.0;
        offset_imu_yaw_ = 0.0;
        current_velocity_imu_x_ = 0.0;
        current_velocity_imu_y_ = 0.0;
        current_velocity_imu_z_ = 0.0;
    }
}

/**
 * @brief Publish current pose as PoseStamped message
 */
void NdtMapping::PublishCurrentPose() {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = this->now();

    pose_msg.pose.position.x = current_pose_.x;
    pose_msg.pose.position.y = current_pose_.y;
    pose_msg.pose.position.z = current_pose_.z;

    tf2::Quaternion q;
    q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
    pose_msg.pose.orientation = tf2::toMsg(q);

    current_pose_pub_->publish(pose_msg);

    // Broadcast transform
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = pose_msg.header;
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = current_pose_.x;
    transform_stamped.transform.translation.y = current_pose_.y;
    transform_stamped.transform.translation.z = current_pose_.z;
    transform_stamped.transform.rotation = pose_msg.pose.orientation;

    tf_broadcaster_->sendTransform(transform_stamped);
}

/**
 * @brief Update map with new scan
 *          - Shift check is performed to avoid adding scans too close to each other
 * @param scan new scan
 */
void NdtMapping::UpdateMap(const pcl::PointCloud<pcl::PointXYZ> &scan) {
    double shift =
        std::sqrt(std::pow(current_pose_.x - added_pose_.x, 2.0) + std::pow(current_pose_.y - added_pose_.y, 2.0));

    RCLCPP_INFO(this->get_logger(), "Current shift: %f", shift);
    if (shift >= kMinAddScanShift) {
        pcl::PointCloud<pcl::PointXYZ> transformed_scan;
        // 将scan点云转换到地图坐标系下
        pcl::transformPointCloud(scan, transformed_scan, ndt_.getFinalTransformation());
        map_ += transformed_scan;
        PublishMap();
        added_pose_ = current_pose_;
        ndt_.setInputTarget(map_.makeShared());
    }
}

void NdtMapping::PublishMap() {
    // Publish updated map
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(map_, map_msg);
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = this->now();
    ndt_map_pub_->publish(map_msg);
}

/**
 * @brief Guess initial pose based on IMU and odometry data
 */
Eigen::Matrix4f NdtMapping::GuessInit() {
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

    guess_pose_.x = previous_pose_.x + diff_x_;
    guess_pose_.y = previous_pose_.y + diff_y_;
    guess_pose_.z = previous_pose_.z + diff_z_;
    guess_pose_.roll = previous_pose_.roll;
    guess_pose_.pitch = previous_pose_.pitch;
    guess_pose_.yaw = previous_pose_.yaw + diff_yaw_;

    if (use_imu_) {
        ImuCalc(this->now());
        init_guess = CreateMatrix(guess_pose_imu_);
    } else {
        init_guess = CreateMatrix(guess_pose_);
    }
    // RCLCPP_INFO(this->get_logger(), "Guess transform: %f %f %f %f %f %f", init_guess(0, 0), init_guess(1, 1),
    //             init_guess(2, 2), init_guess(0, 3), init_guess(1, 3), init_guess(2, 3));
    // RCLCPP_INFO(this->get_logger(), "Guess transform: %f %f %f %f %f %f %f", res(0, 0), res(0, 1), res(0, 2), res(0,
    // 3),
    //             res(1, 0), res(1, 1));
    return init_guess;
}

Eigen::Matrix4f NdtMapping::CreateMatrix(const Pose &p) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) || std::isnan(p.roll) || std::isnan(p.pitch) ||
        std::isnan(p.yaw)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pose data: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", p.x, p.y,
                     p.z, p.roll, p.pitch, p.yaw);
        return Eigen::Matrix4f::Identity(); // 返回一个单位矩阵，避免继续传播错误
    }
    // Eigen::Vector3f::UnitX()表示x轴方向的单位向量, [1, 0, 0]
    Eigen::AngleAxisf rot_x(p.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(p.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(p.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f trans(p.x, p.y, p.z);
    return (trans * rot_z * rot_y * rot_x).matrix();
}

/**
 * @brief Wrap angle to [-pi, pi]
 * @param angle input angle
 * @return wrapped angle
 */

double NdtMapping::WrapToPi(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double NdtMapping::CalcDiffForRadian(double lhs_rad, double rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI) {
        diff_rad = diff_rad - 2 * M_PI;
    } else if (diff_rad < -M_PI) {
        diff_rad = diff_rad + 2 * M_PI;
    }
    return diff_rad;
}

void NdtMapping::ProcessImuUpsideDown(sensor_msgs::msg::Imu::SharedPtr input) {
    tf2::Quaternion q;
    tf2::fromMsg(input->orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Invert measurements
    input->angular_velocity.x *= -1;
    input->angular_velocity.y *= -1;
    input->angular_velocity.z *= -1;

    input->linear_acceleration.x *= -1;
    input->linear_acceleration.y *= -1;
    input->linear_acceleration.z *= -1;

    roll *= -1;
    pitch *= -1;
    yaw *= -1;

    q.setRPY(roll, pitch, yaw);
    input->orientation = tf2::toMsg(q);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NdtMapping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}