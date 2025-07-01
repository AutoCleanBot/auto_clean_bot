#include "ground_filter/ground_filter.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <memory>
#include <string>

namespace ground_filter {

GroundFilterNode::GroundFilterNode(const rclcpp::NodeOptions &options) : Node("ground_filter", options) {
    // 声明并获取参数
    update_rate_ = declare_parameter("update_rate", 10.0);
    min_height_ = declare_parameter("min_height", -0.2);
    max_height_ = declare_parameter("max_height", 0.2);
    base_frame_ = declare_parameter("base_frame", "base_link");
    target_frame_ = declare_parameter("target_frame", "map");
    use_sensor_frame_ = declare_parameter("use_sensor_frame", false);

    // 声明并获取话题参数
    input_topic_ = declare_parameter("input_topic", "~/input/points");
    ground_points_topic_ = declare_parameter("ground_points_topic", "~/output/ground_points");
    no_ground_points_topic_ = declare_parameter("no_ground_points_topic", "~/output/no_ground_points");

    // 创建高度滤波器
    height_filter_ = std::make_unique<HeightFilter>(min_height_, max_height_);

    // 设置TF监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建发布者
    ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_points_topic_, 10);
    no_ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(no_ground_points_topic_, 10);

    // 创建订阅者
    using std::placeholders::_1;
    input_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), std::bind(&GroundFilterNode::pointsCallback, this, _1));

    // 设置定时器（用于参数更新）
    const auto update_period_ns = rclcpp::Rate(update_rate_).period();
    timer_ =
        rclcpp::create_timer(this, get_clock(), update_period_ns, std::bind(&GroundFilterNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Ground filter node initialized");
    RCLCPP_INFO(get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Ground points topic: %s", ground_points_topic_.c_str());
    RCLCPP_INFO(get_logger(), "No ground points topic: %s", no_ground_points_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Target frame: %s", target_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Use sensor frame: %s", use_sensor_frame_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "Min height: %f", min_height_);
    RCLCPP_INFO(get_logger(), "Max height: %f", max_height_);
    RCLCPP_INFO(get_logger(), "Update rate: %f", update_rate_);

    // 添加一条debug级别的日志
    RCLCPP_DEBUG(get_logger(), "Debug level log: ground filter initialization completed");
}

void GroundFilterNode::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { processPointCloud(msg); }

void GroundFilterNode::timerCallback() {
    // 检查参数更新
    const double new_min_height = this->get_parameter("min_height").as_double();
    const double new_max_height = this->get_parameter("max_height").as_double();

    if (new_min_height != min_height_ || new_max_height != max_height_) {
        min_height_ = new_min_height;
        max_height_ = new_max_height;
        height_filter_->setHeightThreshold(min_height_, max_height_);
        RCLCPP_INFO(get_logger(), "Updated height thresholds: min=%.2f, max=%.2f", min_height_, max_height_);
    }

    // 检查话题参数更新
    const std::string new_input_topic = this->get_parameter("input_topic").as_string();
    const std::string new_ground_points_topic = this->get_parameter("ground_points_topic").as_string();
    const std::string new_no_ground_points_topic = this->get_parameter("no_ground_points_topic").as_string();

    bool topics_changed = false;

    // 检查输入话题是否变化
    if (new_input_topic != input_topic_) {
        input_topic_ = new_input_topic;
        // 重新创建订阅者
        using std::placeholders::_1;
        input_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(), std::bind(&GroundFilterNode::pointsCallback, this, _1));
        topics_changed = true;
    }

    // 检查地面点云话题是否变化
    if (new_ground_points_topic != ground_points_topic_) {
        ground_points_topic_ = new_ground_points_topic;
        // 重新创建发布者
        ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_points_topic_, 10);
        topics_changed = true;
    }

    // 检查非地面点云话题是否变化
    if (new_no_ground_points_topic != no_ground_points_topic_) {
        no_ground_points_topic_ = new_no_ground_points_topic;
        // 重新创建发布者
        no_ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(no_ground_points_topic_, 10);
        topics_changed = true;
    }

    if (topics_changed) {
        RCLCPP_INFO(get_logger(), "Topics updated:");
        RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Ground points topic: %s", ground_points_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  No ground points topic: %s", no_ground_points_topic_.c_str());
    }
}

void GroundFilterNode::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg) {

    // 转换点云到目标坐标系
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_msg;

    try {
        // 确定源坐标系
        const std::string &source_frame = use_sensor_frame_ ? cloud_msg->header.frame_id : base_frame_;

        // 检查是否需要转换坐标系
        if (source_frame != target_frame_) {
            transformed_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            const auto transform = tf_buffer_->lookupTransform(target_frame_, source_frame, tf2::TimePointZero);
            tf2::doTransform(*cloud_msg, *transformed_cloud_msg, transform);
        } else {
            transformed_cloud_msg = cloud_msg;
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not transform point cloud: %s", ex.what());
        return;
    }

    // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*transformed_cloud_msg, *pcl_cloud);

    // 分离地面点云和非地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    height_filter_->filter(pcl_cloud, ground_cloud, no_ground_cloud);

    // 将PCL点云转换回ROS消息
    sensor_msgs::msg::PointCloud2 ground_cloud_msg;
    sensor_msgs::msg::PointCloud2 no_ground_cloud_msg;

    pcl::toROSMsg(*ground_cloud, ground_cloud_msg);
    pcl::toROSMsg(*no_ground_cloud, no_ground_cloud_msg);

    // 发布点云
    ground_cloud_msg.header = transformed_cloud_msg->header;
    no_ground_cloud_msg.header = transformed_cloud_msg->header;

    ground_points_pub_->publish(ground_cloud_msg);
    no_ground_points_pub_->publish(no_ground_cloud_msg);

    RCLCPP_DEBUG(get_logger(), "Processed point cloud: %zu total points, %zu ground points, %zu non-ground points",
                 pcl_cloud->size(), ground_cloud->size(), no_ground_cloud->size());
}

} // namespace ground_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ground_filter::GroundFilterNode)