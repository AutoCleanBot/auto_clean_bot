#include "pointcloud_preprocess/pointcloud_transformer.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <memory>
#include <string>

namespace pointcloud_preprocess {

PointCloudTransformerNode::PointCloudTransformerNode(const rclcpp::NodeOptions &options)
    : Node("pointcloud_transformer", options) {
    // 声明并获取参数
    update_rate_ = declare_parameter("update_rate", 10.0);
    input_frame_ = declare_parameter("input_frame", "");
    output_frame_ = declare_parameter("output_frame", "base_link");
    use_sensor_frame_ = declare_parameter("use_sensor_frame", true);
    timeout_ = declare_parameter("timeout", 0.1);
    use_latest_transforms_ = declare_parameter("use_latest_transforms", false);

    // 声明并获取话题参数
    input_topic_ = declare_parameter("input_topic", "~/input/points");
    output_topic_ = declare_parameter("output_topic", "~/output/points");

    // 读取roi参数
    enable_use_roi_ = declare_parameter("enable_use_roi", false);
    roi_size_ = declare_parameter("roi_width", 20.0);

    // 声明并获取车辆过滤参数
    filter_vehicle_points_ = declare_parameter("filter_vehicle_points", false);
    vehicle_front_length_ = declare_parameter("vehicle_front_length", 2.7);
    vehicle_back_length_ = declare_parameter("vehicle_back_length", 0.55);

    // 从配置文件读取宽度和高度，然后平分为左右和上下
    double vehicle_width = declare_parameter("vehicle_width", 1.425);
    double vehicle_height = declare_parameter("vehicle_height", 2.5);

    // 默认将宽度和高度平均分配
    vehicle_left_width_ = vehicle_width / 2.0;
    vehicle_right_width_ = vehicle_width / 2.0;
    vehicle_top_height_ = vehicle_height / 2.0;
    vehicle_bottom_height_ = vehicle_height / 2.0;

    // 读取左右宽度和上下高度的具体配置（如果有）
    vehicle_left_width_ = declare_parameter("vehicle_left_width", vehicle_left_width_);
    vehicle_right_width_ = declare_parameter("vehicle_right_width", vehicle_right_width_);
    vehicle_top_height_ = declare_parameter("vehicle_top_height", vehicle_top_height_);
    vehicle_bottom_height_ = declare_parameter("vehicle_bottom_height", vehicle_bottom_height_);

    vehicle_x_offset_ = declare_parameter("vehicle_x_offset", 0.0);
    vehicle_y_offset_ = declare_parameter("vehicle_y_offset", 0.0);
    vehicle_z_offset_ = declare_parameter("vehicle_z_offset", 0.0);
    vehicle_length_margin_ = declare_parameter("vehicle_length_margin", 0.1);
    vehicle_width_margin_ = declare_parameter("vehicle_width_margin", 0.1);
    vehicle_height_margin_ = declare_parameter("vehicle_height_margin", 0.1);

    // 声明并获取降采样参数
    enable_downsampling_ = declare_parameter("enable_downsampling", false);
    voxel_leaf_size_ = declare_parameter("voxel_leaf_size", 0.1);

    // 声明并获取性能统计参数
    enable_timing_logs_ = declare_parameter("enable_timing_logs", true);
    timing_log_interval_ = declare_parameter("timing_log_interval", 10);
    enable_detailed_timing_ = declare_parameter("enable_detailed_timing", false);

    // 初始化性能统计变量
    frame_count_ = 0;
    total_processing_time_ = 0.0;
    total_roi_filter_time_ = 0.0;
    total_transform_time_ = 0.0;
    total_vehicle_filter_time_ = 0.0;
    total_downsampling_time_ = 0.0;

    // 设置TF监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建发布者
    output_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    // 创建订阅者
    using std::placeholders::_1;
    input_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), std::bind(&PointCloudTransformerNode::pointCloudCallback, this, _1));

    // 设置定时器（用于参数更新）
    const auto update_period_ns = rclcpp::Rate(update_rate_).period();
    timer_ = rclcpp::create_timer(this, get_clock(), update_period_ns,
                                  std::bind(&PointCloudTransformerNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "PointCloud transformer node initialized");
    RCLCPP_INFO(get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Input frame: %s", input_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output frame: %s", output_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Vehicle filter enabled: %s", filter_vehicle_points_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "ROI enabled: %s", enable_use_roi_ ? "true" : "false");
    if (enable_use_roi_) {
        RCLCPP_INFO(get_logger(), "ROI width: %.2fm", roi_size_);
    }
    if (filter_vehicle_points_) {
        RCLCPP_INFO(get_logger(), "Radar coordinate system: X+ = front, Y+ = left, Z+ = up");
        RCLCPP_INFO(get_logger(), "Vehicle dimensions (X axis): front=%.2fm, back=%.2fm", vehicle_front_length_,
                    vehicle_back_length_);
        RCLCPP_INFO(get_logger(), "Vehicle dimensions (Y axis): left=%.2fm, right=%.2fm", vehicle_left_width_,
                    vehicle_right_width_);
        RCLCPP_INFO(get_logger(), "Vehicle dimensions (Z axis): top=%.2fm, bottom=%.2fm", vehicle_top_height_,
                    vehicle_bottom_height_);
    }
    RCLCPP_INFO(get_logger(), "Downsampling enabled: %s", enable_downsampling_ ? "true" : "false");
    if (enable_downsampling_) {
        RCLCPP_INFO(get_logger(), "Voxel leaf size: %.3fm", voxel_leaf_size_);
    }
    RCLCPP_INFO(get_logger(), "Timing logs enabled: %s", enable_timing_logs_ ? "true" : "false");
    if (enable_timing_logs_) {
        RCLCPP_INFO(get_logger(), "Timing log interval: every %d frames", timing_log_interval_);
        RCLCPP_INFO(get_logger(), "Detailed timing enabled: %s", enable_detailed_timing_ ? "true" : "false");
    }
}

void PointCloudTransformerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 记录开始时间
    double start_time = getCurrentTimeMs();

    // 处理点云
    transformPointCloud(msg);

    // 记录结束时间并计算总处理时间
    double end_time = getCurrentTimeMs();
    double process_time = end_time - start_time;

    // 更新统计信息
    frame_count_++;
    total_processing_time_ += process_time;

    // 根据配置输出耗时日志
    if (enable_timing_logs_ && (frame_count_ % timing_log_interval_ == 0)) {
        if (enable_detailed_timing_) {
            logTimingStatistics();
        } else {
            double avg_time = total_processing_time_ / frame_count_;
            RCLCPP_INFO(this->get_logger(), "Frame %d: Current=%.2fms, Average=%.2fms", frame_count_, process_time,
                        avg_time);
        }
    }

    // 防止计数器溢出，每10000帧重置一次统计
    if (frame_count_ >= 10000) {
        resetTimingStatistics();
    }
}

void PointCloudTransformerNode::timerCallback() {
    // 检查参数更新
    const std::string new_input_frame = this->get_parameter("input_frame").as_string();
    const std::string new_output_frame = this->get_parameter("output_frame").as_string();
    const bool new_use_sensor_frame = this->get_parameter("use_sensor_frame").as_bool();
    const std::string new_input_topic = this->get_parameter("input_topic").as_string();
    const std::string new_output_topic = this->get_parameter("output_topic").as_string();
    const double new_timeout = this->get_parameter("timeout").as_double();
    const bool new_use_latest_transforms = this->get_parameter("use_latest_transforms").as_bool();

    // 检查车辆过滤参数更新
    const bool new_filter_vehicle_points = this->get_parameter("filter_vehicle_points").as_bool();
    const double new_vehicle_front_length = this->get_parameter("vehicle_front_length").as_double();
    const double new_vehicle_back_length = this->get_parameter("vehicle_back_length").as_double();

    // 获取宽度和高度参数
    double new_vehicle_width = this->get_parameter("vehicle_width").as_double();
    double new_vehicle_height = this->get_parameter("vehicle_height").as_double();

    // 计算默认的左右宽度和上下高度
    double new_left_width = new_vehicle_width / 2.0;
    double new_right_width = new_vehicle_width / 2.0;
    double new_top_height = new_vehicle_height / 2.0;
    double new_bottom_height = new_vehicle_height / 2.0;

    // 获取具体的左右宽度和上下高度配置（如果有）
    const double new_vehicle_left_width = this->get_parameter("vehicle_left_width").as_double();
    const double new_vehicle_right_width = this->get_parameter("vehicle_right_width").as_double();
    const double new_vehicle_top_height = this->get_parameter("vehicle_top_height").as_double();
    const double new_vehicle_bottom_height = this->get_parameter("vehicle_bottom_height").as_double();

    const double new_vehicle_x_offset = this->get_parameter("vehicle_x_offset").as_double();
    const double new_vehicle_y_offset = this->get_parameter("vehicle_y_offset").as_double();
    const double new_vehicle_z_offset = this->get_parameter("vehicle_z_offset").as_double();
    const double new_vehicle_length_margin = this->get_parameter("vehicle_length_margin").as_double();
    const double new_vehicle_width_margin = this->get_parameter("vehicle_width_margin").as_double();
    const double new_vehicle_height_margin = this->get_parameter("vehicle_height_margin").as_double();

    // 检查降采样参数更新
    const bool new_enable_downsampling = this->get_parameter("enable_downsampling").as_bool();
    const double new_voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();

    // 检查帧参数更新
    bool frames_changed = false;
    if (new_input_frame != input_frame_) {
        input_frame_ = new_input_frame;
        frames_changed = true;
    }

    if (new_output_frame != output_frame_) {
        output_frame_ = new_output_frame;
        frames_changed = true;
    }

    if (new_use_sensor_frame != use_sensor_frame_) {
        use_sensor_frame_ = new_use_sensor_frame;
        frames_changed = true;
    }

    if (new_timeout != timeout_) {
        timeout_ = new_timeout;
        frames_changed = true;
    }

    if (new_use_latest_transforms != use_latest_transforms_) {
        use_latest_transforms_ = new_use_latest_transforms;
        frames_changed = true;
    }

    if (frames_changed) {
        RCLCPP_INFO(get_logger(), "Transform parameters updated:");
        RCLCPP_INFO(get_logger(), "  Input frame: %s", input_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  Output frame: %s", output_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  Use sensor frame: %s", use_sensor_frame_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  Timeout: %.3f", timeout_);
        RCLCPP_INFO(get_logger(), "  Use latest transforms: %s", use_latest_transforms_ ? "true" : "false");
    }

    // 检查车辆过滤参数更新
    bool vehicle_filter_changed = false;
    if (new_filter_vehicle_points != filter_vehicle_points_) {
        filter_vehicle_points_ = new_filter_vehicle_points;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_front_length != vehicle_front_length_) {
        vehicle_front_length_ = new_vehicle_front_length;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_back_length != vehicle_back_length_) {
        vehicle_back_length_ = new_vehicle_back_length;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_left_width != vehicle_left_width_) {
        vehicle_left_width_ = new_vehicle_left_width;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_right_width != vehicle_right_width_) {
        vehicle_right_width_ = new_vehicle_right_width;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_top_height != vehicle_top_height_) {
        vehicle_top_height_ = new_vehicle_top_height;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_bottom_height != vehicle_bottom_height_) {
        vehicle_bottom_height_ = new_vehicle_bottom_height;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_x_offset != vehicle_x_offset_) {
        vehicle_x_offset_ = new_vehicle_x_offset;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_y_offset != vehicle_y_offset_) {
        vehicle_y_offset_ = new_vehicle_y_offset;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_z_offset != vehicle_z_offset_) {
        vehicle_z_offset_ = new_vehicle_z_offset;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_length_margin != vehicle_length_margin_) {
        vehicle_length_margin_ = new_vehicle_length_margin;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_width_margin != vehicle_width_margin_) {
        vehicle_width_margin_ = new_vehicle_width_margin;
        vehicle_filter_changed = true;
    }

    if (new_vehicle_height_margin != vehicle_height_margin_) {
        vehicle_height_margin_ = new_vehicle_height_margin;
        vehicle_filter_changed = true;
    }

    if (vehicle_filter_changed) {
        RCLCPP_INFO(get_logger(), "Vehicle filter parameters updated:");
        RCLCPP_INFO(get_logger(), "  Filter enabled: %s", filter_vehicle_points_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  Vehicle dimensions (X axis): front=%.2fm, back=%.2fm", vehicle_front_length_,
                    vehicle_back_length_);
        RCLCPP_INFO(get_logger(), "  Vehicle dimensions (Y axis): left=%.2fm, right=%.2fm", vehicle_left_width_,
                    vehicle_right_width_);
        RCLCPP_INFO(get_logger(), "  Vehicle dimensions (Z axis): top=%.2fm, bottom=%.2fm", vehicle_top_height_,
                    vehicle_bottom_height_);
        RCLCPP_INFO(get_logger(), "  Vehicle offset (X,Y,Z): %.2f, %.2f, %.2f", vehicle_x_offset_, vehicle_y_offset_,
                    vehicle_z_offset_);
        RCLCPP_INFO(get_logger(), "  Margin (X,Y,Z): %.2f, %.2f, %.2f", vehicle_length_margin_, vehicle_width_margin_,
                    vehicle_height_margin_);
    }

    // 检查降采样参数更新
    bool downsampling_changed = false;
    if (new_enable_downsampling != enable_downsampling_) {
        enable_downsampling_ = new_enable_downsampling;
        downsampling_changed = true;
    }

    if (new_voxel_leaf_size != voxel_leaf_size_) {
        voxel_leaf_size_ = new_voxel_leaf_size;
        downsampling_changed = true;
    }

    if (downsampling_changed) {
        RCLCPP_INFO(get_logger(), "Downsampling parameters updated:");
        RCLCPP_INFO(get_logger(), "  Downsampling enabled: %s", enable_downsampling_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  Voxel leaf size: %.3fm", voxel_leaf_size_);
    }

    // 检查话题参数更新
    bool topics_changed = false;

    // 检查输入话题是否变化
    if (new_input_topic != input_topic_) {
        input_topic_ = new_input_topic;
        // 重新创建订阅者
        using std::placeholders::_1;
        input_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(), std::bind(&PointCloudTransformerNode::pointCloudCallback, this, _1));
        topics_changed = true;
    }

    // 检查输出话题是否变化
    if (new_output_topic != output_topic_) {
        output_topic_ = new_output_topic;
        // 重新创建发布者
        output_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        topics_changed = true;
    }

    if (topics_changed) {
        RCLCPP_INFO(get_logger(), "Topics updated:");
        RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Output topic: %s", output_topic_.c_str());
    }
}

void PointCloudTransformerNode::filterVehiclePoints(sensor_msgs::msg::PointCloud2 &cloud,
                                                    const std::string &frame_id) const {
    if (!filter_vehicle_points_) {
        return; // 如果过滤功能未启用，直接返回
    }

    double start_time = getCurrentTimeMs();

    // 将ROS2点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud, *pcl_cloud);

    // 创建CropBox滤波器
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(pcl_cloud);

    // 根据坐标系确定轴映射
    Eigen::Vector4f min_point, max_point;
    std::string coord_system_info;

    // 检查是否为base_link坐标系（Y+ = 车头方向，X+ = 车辆右侧）
    if (frame_id == "base_link") {
        coord_system_info = "base_link coordinate system: Y+ = front, X+ = right, Z+ = up";

        // base_link坐标系：Y+ = 车头方向，X+ = 车辆右侧，Z+ = 垂直向上
        // X轴对应车辆左右方向，负值是左侧
        min_point[0] = -vehicle_left_width_ + vehicle_x_offset_ - vehicle_width_margin_;
        // Y轴对应车辆前后方向，负值是后方
        min_point[1] = -vehicle_back_length_ + vehicle_y_offset_ - vehicle_length_margin_;
        // Z轴对应车辆上下方向，负值是下方
        min_point[2] = -vehicle_bottom_height_ + vehicle_z_offset_ - vehicle_height_margin_;
        min_point[3] = 1.0;

        // X轴正方向是车辆右侧
        max_point[0] = vehicle_right_width_ + vehicle_x_offset_ + vehicle_width_margin_;
        // Y轴正方向是车头方向
        max_point[1] = vehicle_front_length_ + vehicle_y_offset_ + vehicle_length_margin_;
        // Z轴正方向是向上
        max_point[2] = vehicle_top_height_ + vehicle_z_offset_ + vehicle_height_margin_;
        max_point[3] = 1.0;
    } else {
        // 默认使用雷达坐标系：X+ = 车头方向，Y+ = 车辆左侧，Z+ = 垂直向上
        coord_system_info = "radar coordinate system: X+ = front, Y+ = left, Z+ = up";

        // X轴对应车辆前后方向，负值是后方
        min_point[0] = -vehicle_back_length_ + vehicle_x_offset_ - vehicle_length_margin_;
        // Y轴对应车辆左右方向，负值是右侧
        min_point[1] = -vehicle_right_width_ + vehicle_y_offset_ - vehicle_width_margin_;
        // Z轴对应车辆上下方向，负值是下方
        min_point[2] = -vehicle_bottom_height_ + vehicle_z_offset_ - vehicle_height_margin_;
        min_point[3] = 1.0;

        // X轴正方向是车头方向
        max_point[0] = vehicle_front_length_ + vehicle_x_offset_ + vehicle_length_margin_;
        // Y轴正方向是车辆左侧
        max_point[1] = vehicle_left_width_ + vehicle_y_offset_ + vehicle_width_margin_;
        // Z轴正方向是向上
        max_point[2] = vehicle_top_height_ + vehicle_z_offset_ + vehicle_height_margin_;
        max_point[3] = 1.0;
    }

    RCLCPP_DEBUG(get_logger(), "Vehicle filter box (%s): min=[%.2f, %.2f, %.2f], max=[%.2f, %.2f, %.2f]",
                 coord_system_info.c_str(), min_point[0], min_point[1], min_point[2], max_point[0], max_point[1],
                 max_point[2]);

    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    // 设置为反向过滤 - 保留框外的点（移除框内的点）
    crop_box.setNegative(true);

    // 应用滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    crop_box.filter(*filtered_cloud);

    // 将过滤后的PCL点云转换回ROS2消息
    pcl::toROSMsg(*filtered_cloud, cloud);

    RCLCPP_DEBUG(get_logger(), "Filtered vehicle points: removed %ld points",
                 pcl_cloud->size() - filtered_cloud->size());

    // 记录车辆过滤耗时
    double end_time = getCurrentTimeMs();
    total_vehicle_filter_time_ += (end_time - start_time);
}

void PointCloudTransformerNode::FilterROI(const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud) {
    double start_time = getCurrentTimeMs();

    // 将ROS2点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // 创建CropBox滤波器
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(pcl_cloud);

    // 设置ROI的最小和最大点
    Eigen::Vector4f min_point, max_point;
    min_point[0] = -roi_size_;
    min_point[1] = -roi_size_;
    min_point[2] = -3.0;
    min_point[3] = 1.0;

    max_point[0] = roi_size_;
    max_point[1] = roi_size_;
    max_point[2] = 20.0;
    max_point[3] = 1.0;

    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    // 应用滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    crop_box.filter(*filtered_cloud);

    // 将过滤后的PCL点云转换回ROS2消息
    pcl::toROSMsg(*filtered_cloud, *input_cloud);

    // 记录ROI过滤耗时
    double end_time = getCurrentTimeMs();
    total_roi_filter_time_ += (end_time - start_time);
}

void PointCloudTransformerNode::transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud) {
    // RCLCPP_INFO(get_logger(), "Transforming point cloud from '%s' to '%s'", input_cloud->header.frame_id.c_str(),
    //             output_frame_.c_str());

    if (enable_use_roi_) {
        FilterROI(input_cloud);
    }

    // 确定源坐标系
    std::string source_frame;
    if (use_sensor_frame_) {
        source_frame = input_cloud->header.frame_id;
    } else if (!input_frame_.empty()) {
        source_frame = input_frame_;
    } else {
        RCLCPP_ERROR(get_logger(), "No source frame specified");
        return;
    }

    // 检查是否需要转换坐标系
    if (source_frame == output_frame_) {
        // 源坐标系和目标坐标系相同，应用车辆点云过滤和降采样后直接发布
        sensor_msgs::msg::PointCloud2 processed_cloud = *input_cloud;
        filterVehiclePoints(processed_cloud, output_frame_);
        downsamplePointCloud(processed_cloud);
        output_cloud_pub_->publish(processed_cloud);
        return;
    }

    // 转换点云到目标坐标系
    try {
        double transform_start_time = getCurrentTimeMs();
        sensor_msgs::msg::PointCloud2 transformed_cloud;

        if (use_latest_transforms_) {
            // 使用最新的变换
            const auto transform = tf_buffer_->lookupTransform(output_frame_, source_frame, tf2::TimePointZero);
            tf2::doTransform(*input_cloud, transformed_cloud, transform);
        } else {
            // 使用点云时间戳对应的变换
            const auto transform_time = input_cloud->header.stamp;
            const auto transform = tf_buffer_->lookupTransform(output_frame_, source_frame, transform_time,
                                                               tf2::durationFromSec(timeout_));
            tf2::doTransform(*input_cloud, transformed_cloud, transform);
        }

        // 记录坐标变换耗时
        double transform_end_time = getCurrentTimeMs();
        total_transform_time_ += (transform_end_time - transform_start_time);

        // 对转换后的点云应用车辆点云过滤和降采样
        filterVehiclePoints(transformed_cloud, output_frame_);
        downsamplePointCloud(transformed_cloud);

        // 发布转换后的点云
        transformed_cloud.header.frame_id = output_frame_;
        output_cloud_pub_->publish(transformed_cloud);

        RCLCPP_INFO(get_logger(), "Transformed point cloud from '%s' to '%s' with %u points", source_frame.c_str(),
                    output_frame_.c_str(), transformed_cloud.width * transformed_cloud.height);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not transform point cloud from '%s' to '%s': %s", source_frame.c_str(),
                    output_frame_.c_str(), ex.what());
    }
}

void PointCloudTransformerNode::downsamplePointCloud(sensor_msgs::msg::PointCloud2 &cloud) const {
    if (!enable_downsampling_) {
        return; // 如果降采样功能未启用，直接返回
    }

    double start_time = getCurrentTimeMs();

    // 将ROS2点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud, *pcl_cloud);

    // 记录原始点云大小
    const size_t original_size = pcl_cloud->size();

    // 创建体素网格滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pcl_cloud);

    // 设置体素网格的叶子大小
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

    // 应用滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_grid.filter(*downsampled_cloud);

    // 将降采样后的PCL点云转换回ROS2消息
    pcl::toROSMsg(*downsampled_cloud, cloud);

    // 记录降采样信息
    const size_t downsampled_size = downsampled_cloud->size();
    const double reduction_ratio =
        (original_size > 0) ? (1.0 - static_cast<double>(downsampled_size) / original_size) : 0.0;

    RCLCPP_DEBUG(get_logger(), "Downsampled point cloud: %ld -> %ld points (%.1f%% reduction)", original_size,
                 downsampled_size, reduction_ratio * 100.0);

    // 记录降采样耗时
    double end_time = getCurrentTimeMs();
    total_downsampling_time_ += (end_time - start_time);
}

double PointCloudTransformerNode::getCurrentTimeMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
}

void PointCloudTransformerNode::logTimingStatistics() const {
    if (frame_count_ == 0)
        return;

    double avg_total = total_processing_time_ / frame_count_;
    double avg_roi = total_roi_filter_time_ / frame_count_;
    double avg_transform = total_transform_time_ / frame_count_;
    double avg_vehicle = total_vehicle_filter_time_ / frame_count_;
    double avg_downsample = total_downsampling_time_ / frame_count_;

    RCLCPP_INFO(get_logger(), "=== Point Cloud Processing Statistics (Frame %d) ===", frame_count_);
    RCLCPP_INFO(get_logger(), "Total Processing:    %.2f ms (avg)", avg_total);
    RCLCPP_INFO(get_logger(), "  - ROI Filter:      %.2f ms (avg)", avg_roi);
    RCLCPP_INFO(get_logger(), "  - Transform:       %.2f ms (avg)", avg_transform);
    RCLCPP_INFO(get_logger(), "  - Vehicle Filter:  %.2f ms (avg)", avg_vehicle);
    RCLCPP_INFO(get_logger(), "  - Downsampling:    %.2f ms (avg)", avg_downsample);

    // 计算各步骤占总时间的百分比
    if (avg_total > 0) {
        RCLCPP_INFO(get_logger(), "Time Distribution:");
        RCLCPP_INFO(get_logger(), "  - ROI Filter:      %.1f%%", (avg_roi / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Transform:       %.1f%%", (avg_transform / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Vehicle Filter:  %.1f%%", (avg_vehicle / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Downsampling:    %.1f%%", (avg_downsample / avg_total) * 100.0);
    }
    RCLCPP_INFO(get_logger(), "================================================");
}

void PointCloudTransformerNode::resetTimingStatistics() const {
    RCLCPP_INFO(get_logger(), "Resetting timing statistics after %d frames", frame_count_);
    frame_count_ = 0;
    total_processing_time_ = 0.0;
    total_roi_filter_time_ = 0.0;
    total_transform_time_ = 0.0;
    total_vehicle_filter_time_ = 0.0;
    total_downsampling_time_ = 0.0;
}

} // namespace pointcloud_preprocess

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocess::PointCloudTransformerNode)