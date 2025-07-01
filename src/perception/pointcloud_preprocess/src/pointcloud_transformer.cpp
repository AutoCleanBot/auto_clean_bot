#include "pointcloud_preprocess/pointcloud_transformer.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
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
    if (filter_vehicle_points_) {
        RCLCPP_INFO(get_logger(), "Vehicle coordinate system: Y+ = front, X+ = right, Z+ = up");
        RCLCPP_INFO(get_logger(), "Vehicle dimensions (Y axis): front=%.2fm, back=%.2fm", 
                    vehicle_front_length_, vehicle_back_length_);
        RCLCPP_INFO(get_logger(), "Vehicle dimensions (X axis): right=%.2fm, left=%.2fm", 
                    vehicle_right_width_, vehicle_left_width_);
        RCLCPP_INFO(get_logger(), "Vehicle dimensions (Z axis): top=%.2fm, bottom=%.2fm", 
                    vehicle_top_height_, vehicle_bottom_height_);
    }
}

void PointCloudTransformerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    transformPointCloud(msg);
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
        RCLCPP_INFO(get_logger(), "  Vehicle dimensions (Y axis): front=%.2fm, back=%.2fm", 
                    vehicle_front_length_, vehicle_back_length_);
        RCLCPP_INFO(get_logger(), "  Vehicle dimensions (X axis): right=%.2fm, left=%.2fm", 
                    vehicle_right_width_, vehicle_left_width_);
        RCLCPP_INFO(get_logger(), "  Vehicle dimensions (Z axis): top=%.2fm, bottom=%.2fm", 
                    vehicle_top_height_, vehicle_bottom_height_);
        RCLCPP_INFO(get_logger(), "  Vehicle offset (X,Y,Z): %.2f, %.2f, %.2f", 
                    vehicle_x_offset_, vehicle_y_offset_, vehicle_z_offset_);
        RCLCPP_INFO(get_logger(), "  Margin (L,W,H): %.2f, %.2f, %.2f", 
                    vehicle_length_margin_, vehicle_width_margin_, vehicle_height_margin_);
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

void PointCloudTransformerNode::filterVehiclePoints(sensor_msgs::msg::PointCloud2 & cloud) const {
    if (!filter_vehicle_points_) {
        return;  // 如果过滤功能未启用，直接返回
    }

    // 将ROS2点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud, *pcl_cloud);

    // 创建CropBox滤波器
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(pcl_cloud);

    // 计算边界，以激光雷达位置为原点
    // 注意：车头方向为Y轴正方向，车辆右侧为X轴正方向
    Eigen::Vector4f min_point;
    // X轴对应车辆左右方向，负值是左侧
    min_point[0] = -vehicle_left_width_ + vehicle_y_offset_ - vehicle_width_margin_;
    // Y轴对应车辆前后方向，负值是后方
    min_point[1] = -vehicle_back_length_ + vehicle_x_offset_ - vehicle_length_margin_;
    // Z轴保持不变，对应车辆上下方向
    min_point[2] = -vehicle_bottom_height_ + vehicle_z_offset_ - vehicle_height_margin_;
    min_point[3] = 1.0;

    Eigen::Vector4f max_point;
    // X轴正方向是车辆右侧
    max_point[0] = vehicle_right_width_ + vehicle_y_offset_ + vehicle_width_margin_;
    // Y轴正方向是车头方向
    max_point[1] = vehicle_front_length_ + vehicle_x_offset_ + vehicle_length_margin_;
    // Z轴正方向是向上
    max_point[2] = vehicle_top_height_ + vehicle_z_offset_ + vehicle_height_margin_;
    max_point[3] = 1.0;

    RCLCPP_DEBUG(get_logger(), "Vehicle filter box: min=[%.2f, %.2f, %.2f], max=[%.2f, %.2f, %.2f]",
                min_point[0], min_point[1], min_point[2], 
                max_point[0], max_point[1], max_point[2]);

    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    // 设置为反向过滤 - 保留框外的点（移除框内的点）
    crop_box.setNegative(true);

    // 应用滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    crop_box.filter(*filtered_cloud);

    // 将过滤后的PCL点云转换回ROS2消息
    pcl::toROSMsg(*filtered_cloud, cloud);

    RCLCPP_INFO(get_logger(), "Filtered vehicle points: removed %ld points", 
                pcl_cloud->size() - filtered_cloud->size());
}

void PointCloudTransformerNode::transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud) {
    RCLCPP_INFO(get_logger(), "Transforming point cloud from '%s' to '%s'", input_cloud->header.frame_id.c_str(),
                output_frame_.c_str());

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
        // 源坐标系和目标坐标系相同，应用车辆点云过滤后直接发布
        sensor_msgs::msg::PointCloud2 filtered_cloud = *input_cloud;
        filterVehiclePoints(filtered_cloud);
        output_cloud_pub_->publish(filtered_cloud);
        return;
    }

    // 转换点云到目标坐标系
    try {
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

        // 应用车辆点云过滤
        filterVehiclePoints(transformed_cloud);

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

} // namespace pointcloud_preprocess

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocess::PointCloudTransformerNode)