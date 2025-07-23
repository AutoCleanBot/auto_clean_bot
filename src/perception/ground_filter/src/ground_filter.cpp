#include "ground_filter/ground_filter.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <cstring>
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

    // 声明并获取性能统计参数
    enable_timing_logs_ = declare_parameter("enable_timing_logs", true);
    timing_log_interval_ = declare_parameter("timing_log_interval", 10);
    enable_detailed_timing_ = declare_parameter("enable_detailed_timing", false);
    enable_zero_copy_ = declare_parameter("enable_zero_copy", true);

    // 初始化性能统计变量
    frame_count_ = 0;
    total_processing_time_ = 0.0;
    total_transform_time_ = 0.0;
    total_conversion_time_ = 0.0;
    total_filtering_time_ = 0.0;
    total_publish_time_ = 0.0;

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

    // 性能统计配置信息
    RCLCPP_INFO(get_logger(), "Timing logs enabled: %s", enable_timing_logs_ ? "true" : "false");
    if (enable_timing_logs_) {
        RCLCPP_INFO(get_logger(), "Timing log interval: every %d frames", timing_log_interval_);
        RCLCPP_INFO(get_logger(), "Detailed timing enabled: %s", enable_detailed_timing_ ? "true" : "false");
    }
    RCLCPP_INFO(get_logger(), "Zero-copy optimization enabled: %s", enable_zero_copy_ ? "true" : "false");

    // 添加一条debug级别的日志
    RCLCPP_DEBUG(get_logger(), "Debug level log: ground filter initialization completed");
}

void GroundFilterNode::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 记录开始时间
    double start_time = getCurrentTimeMs();

    // 根据配置选择处理方式
    if (enable_zero_copy_) {
        processPointCloudZeroCopy(msg);
    } else {
        processPointCloud(msg);
    }

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
    double transform_start_time = getCurrentTimeMs();
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

    // 记录坐标变换耗时
    double transform_end_time = getCurrentTimeMs();
    total_transform_time_ += (transform_end_time - transform_start_time);

    // 格式转换：ROS消息转换为PCL点云
    double conversion_start_time = getCurrentTimeMs();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*transformed_cloud_msg, *pcl_cloud);
    double conversion_mid_time = getCurrentTimeMs();

    // 地面过滤
    double filtering_start_time = getCurrentTimeMs();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    height_filter_->filter(pcl_cloud, ground_cloud, no_ground_cloud);
    double filtering_end_time = getCurrentTimeMs();

    // 格式转换：PCL点云转换回ROS消息
    sensor_msgs::msg::PointCloud2 ground_cloud_msg;
    sensor_msgs::msg::PointCloud2 no_ground_cloud_msg;
    pcl::toROSMsg(*ground_cloud, ground_cloud_msg);
    pcl::toROSMsg(*no_ground_cloud, no_ground_cloud_msg);
    double conversion_end_time = getCurrentTimeMs();

    // 发布点云
    double publish_start_time = getCurrentTimeMs();
    ground_cloud_msg.header = transformed_cloud_msg->header;
    no_ground_cloud_msg.header = transformed_cloud_msg->header;
    ground_points_pub_->publish(ground_cloud_msg);
    no_ground_points_pub_->publish(no_ground_cloud_msg);
    double publish_end_time = getCurrentTimeMs();

    // 记录各步骤耗时
    total_conversion_time_ +=
        (conversion_mid_time - conversion_start_time) + (conversion_end_time - filtering_end_time);
    total_filtering_time_ += (filtering_end_time - filtering_start_time);
    total_publish_time_ += (publish_end_time - publish_start_time);

    RCLCPP_DEBUG(get_logger(), "Processed point cloud: %zu total points, %zu ground points, %zu non-ground points",
                 pcl_cloud->size(), ground_cloud->size(), no_ground_cloud->size());
}

double GroundFilterNode::getCurrentTimeMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
}

void GroundFilterNode::logTimingStatistics() const {
    if (frame_count_ == 0)
        return;

    double avg_total = total_processing_time_ / frame_count_;
    double avg_transform = total_transform_time_ / frame_count_;
    double avg_conversion = total_conversion_time_ / frame_count_;
    double avg_filtering = total_filtering_time_ / frame_count_;
    double avg_publish = total_publish_time_ / frame_count_;

    RCLCPP_INFO(get_logger(), "=== Ground Filter Processing Statistics (Frame %d) ===", frame_count_);
    RCLCPP_INFO(get_logger(), "Total Processing:    %.2f ms (avg)", avg_total);
    RCLCPP_INFO(get_logger(), "  - Transform:       %.2f ms (avg)", avg_transform);
    RCLCPP_INFO(get_logger(), "  - Conversion:      %.2f ms (avg)", avg_conversion);
    RCLCPP_INFO(get_logger(), "  - Filtering:       %.2f ms (avg)", avg_filtering);
    RCLCPP_INFO(get_logger(), "  - Publishing:      %.2f ms (avg)", avg_publish);

    // 计算各步骤占总时间的百分比
    if (avg_total > 0) {
        RCLCPP_INFO(get_logger(), "Time Distribution:");
        RCLCPP_INFO(get_logger(), "  - Transform:       %.1f%%", (avg_transform / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Conversion:      %.1f%%", (avg_conversion / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Filtering:       %.1f%%", (avg_filtering / avg_total) * 100.0);
        RCLCPP_INFO(get_logger(), "  - Publishing:      %.1f%%", (avg_publish / avg_total) * 100.0);
    }
    RCLCPP_INFO(get_logger(), "Zero-copy mode: %s", enable_zero_copy_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(get_logger(), "================================================");
}

void GroundFilterNode::processPointCloudZeroCopy(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg) {
    // 零拷贝优化：直接在原始消息上操作，避免不必要的内存拷贝

    // 坐标变换（如果需要）
    double transform_start_time = getCurrentTimeMs();
    sensor_msgs::msg::PointCloud2::SharedPtr working_cloud_msg;

    try {
        const std::string &source_frame = use_sensor_frame_ ? cloud_msg->header.frame_id : base_frame_;

        if (source_frame != target_frame_) {
            // 只有在需要坐标变换时才创建新的消息
            working_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            const auto transform = tf_buffer_->lookupTransform(target_frame_, source_frame, tf2::TimePointZero);
            tf2::doTransform(*cloud_msg, *working_cloud_msg, transform);
        } else {
            // 零拷贝：直接使用原始消息
            working_cloud_msg = cloud_msg;
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not transform point cloud: %s", ex.what());
        return;
    }

    double transform_end_time = getCurrentTimeMs();
    total_transform_time_ += (transform_end_time - transform_start_time);

    // 零拷贝地面过滤：直接操作ROS消息，避免PCL转换
    double filtering_start_time = getCurrentTimeMs();

    // 创建输出消息（复用header信息）
    auto ground_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto no_ground_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // 复制header和字段信息
    ground_cloud_msg->header = working_cloud_msg->header;
    ground_cloud_msg->fields = working_cloud_msg->fields;
    ground_cloud_msg->is_bigendian = working_cloud_msg->is_bigendian;
    ground_cloud_msg->point_step = working_cloud_msg->point_step;
    ground_cloud_msg->is_dense = working_cloud_msg->is_dense;

    no_ground_cloud_msg->header = working_cloud_msg->header;
    no_ground_cloud_msg->fields = working_cloud_msg->fields;
    no_ground_cloud_msg->is_bigendian = working_cloud_msg->is_bigendian;
    no_ground_cloud_msg->point_step = working_cloud_msg->point_step;
    no_ground_cloud_msg->is_dense = working_cloud_msg->is_dense;

    // 预分配内存（估算）
    size_t estimated_ground_points = working_cloud_msg->width * working_cloud_msg->height / 4; // 估算25%是地面点
    size_t estimated_no_ground_points = working_cloud_msg->width * working_cloud_msg->height - estimated_ground_points;

    ground_cloud_msg->data.reserve(estimated_ground_points * working_cloud_msg->point_step);
    no_ground_cloud_msg->data.reserve(estimated_no_ground_points * working_cloud_msg->point_step);

    // 直接在字节级别操作，避免PCL转换
    const uint8_t *input_data = working_cloud_msg->data.data();
    const size_t point_step = working_cloud_msg->point_step;
    const size_t total_points = working_cloud_msg->width * working_cloud_msg->height;

    // 找到Z坐标的偏移量（假设是标准的PointXYZ格式）
    size_t z_offset = 8; // X(4字节) + Y(4字节) = 8字节偏移

    size_t ground_count = 0;
    size_t no_ground_count = 0;

    for (size_t i = 0; i < total_points; ++i) {
        const uint8_t *point_data = input_data + i * point_step;

        // 读取Z坐标值
        float z_value;
        std::memcpy(&z_value, point_data + z_offset, sizeof(float));

        // 根据高度阈值分类
        if (z_value <= min_height_) {
            // 地面点：直接复制整个点的数据
            ground_cloud_msg->data.insert(ground_cloud_msg->data.end(), point_data, point_data + point_step);
            ground_count++;
        } else {
            // 非地面点
            no_ground_cloud_msg->data.insert(no_ground_cloud_msg->data.end(), point_data, point_data + point_step);
            no_ground_count++;
        }
    }

    // 设置输出消息的尺寸信息
    ground_cloud_msg->width = ground_count;
    ground_cloud_msg->height = 1;
    ground_cloud_msg->row_step = ground_count * point_step;

    no_ground_cloud_msg->width = no_ground_count;
    no_ground_cloud_msg->height = 1;
    no_ground_cloud_msg->row_step = no_ground_count * point_step;

    double filtering_end_time = getCurrentTimeMs();
    total_filtering_time_ += (filtering_end_time - filtering_start_time);

    // 发布点云
    double publish_start_time = getCurrentTimeMs();
    ground_points_pub_->publish(*ground_cloud_msg);
    no_ground_points_pub_->publish(*no_ground_cloud_msg);
    double publish_end_time = getCurrentTimeMs();

    total_publish_time_ += (publish_end_time - publish_start_time);

    RCLCPP_DEBUG(get_logger(), "Zero-copy processed: %zu total points, %zu ground points, %zu non-ground points",
                 total_points, ground_count, no_ground_count);
}

void GroundFilterNode::resetTimingStatistics() const {
    RCLCPP_INFO(get_logger(), "Resetting timing statistics after %d frames", frame_count_);
    frame_count_ = 0;
    total_processing_time_ = 0.0;
    total_transform_time_ = 0.0;
    total_conversion_time_ = 0.0;
    total_filtering_time_ = 0.0;
    total_publish_time_ = 0.0;
}

} // namespace ground_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ground_filter::GroundFilterNode)