#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "obstacle_marker_test/debug_object.hpp"

class ObstacleMarkerTestNode : public rclcpp::Node {
  public:
    ObstacleMarkerTestNode() : Node("obstacle_marker_test_node"), time_counter_(0.0) {
        // 声明参数
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("publish_rate", 2.0);

        // 获取参数
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate);

        // 设置通道配置
        setupChannels();

        // 创建调试器
        debugger_ = std::make_unique<obstacle_marker_test::TrackerObjectDebugger>(frame_id_, channels_config_);

        // 创建发布器
        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_marker_test/debug_markers", 10);

        // 创建定时器
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
                                         std::bind(&ObstacleMarkerTestNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle marker test node started");
    }

  private:
    void setupChannels() {
        // 设置多个通道用于测试
        obstacle_marker_test::InputChannel lidar_channel;
        lidar_channel.index = 0;
        lidar_channel.input_topic = "/perception/lidar/objects";
        lidar_channel.long_name = "LiDAR Detection";
        lidar_channel.short_name = "LID";
        channels_config_.push_back(lidar_channel);

        obstacle_marker_test::InputChannel camera_channel;
        camera_channel.index = 1;
        camera_channel.input_topic = "/perception/camera/objects";
        camera_channel.long_name = "Camera Detection";
        camera_channel.short_name = "CAM";
        channels_config_.push_back(camera_channel);

        obstacle_marker_test::InputChannel radar_channel;
        radar_channel.index = 2;
        radar_channel.input_topic = "/perception/radar/objects";
        radar_channel.long_name = "Radar Detection";
        radar_channel.short_name = "RAD";
        channels_config_.push_back(radar_channel);
    }

    void timerCallback() {
        // 更新时间计数器
        time_counter_ += 0.5; // 假设每0.5秒调用一次

        // 创建测试数据
        std::vector<std::vector<obstacle_marker_test::ObjectData>> object_data_groups;

        // 创建几个测试对象组
        createTestObjectGroup1(object_data_groups);
        createTestObjectGroup2(object_data_groups);
        createTestObjectGroup3(object_data_groups);

        // 生成marker
        visualization_msgs::msg::MarkerArray marker_array;
        debugger_->draw(object_data_groups, marker_array);

        // 发布marker
        marker_pub_->publish(marker_array);

        RCLCPP_DEBUG(this->get_logger(), "Published %zu markers", marker_array.markers.size());
    }

    void createTestObjectGroup1(std::vector<std::vector<obstacle_marker_test::ObjectData>> &groups) {
        std::vector<obstacle_marker_test::ObjectData> group;

        // 创建一个有关联的对象 - 做圆周运动
        obstacle_marker_test::ObjectData obj_data;
        obj_data.time = this->get_clock()->now();

        // 使用固定的UUID，避免每次都生成新的
        static boost::uuids::uuid fixed_uuid = boost::uuids::random_generator()();
        obj_data.uuid = fixed_uuid;
        obj_data.uuid_str = boost::uuids::to_string(obj_data.uuid);

        // 圆周运动：半径3米，中心在(5, 0)
        double radius = 3.0;
        double center_x = 5.0;
        double center_y = 0.0;
        double angle = time_counter_ * 0.5; // 角速度0.5 rad/s

        obj_data.tracker_point.x = center_x + radius * cos(angle);
        obj_data.tracker_point.y = center_y + radius * sin(angle);
        obj_data.tracker_point.z = 0.0;

        // 检测点稍微偏移一点
        obj_data.detection_point.x = obj_data.tracker_point.x + 0.2;
        obj_data.detection_point.y = obj_data.tracker_point.y + 0.1;
        obj_data.detection_point.z = 0.0;

        obj_data.is_associated = true;
        obj_data.channel_id = 0; // LiDAR channel
        obj_data.existence_vector = {0.8f, 0.0f, 0.0f};
        obj_data.total_existence_probability = 0.8f;

        group.push_back(obj_data);
        groups.push_back(group);
    }

    void createTestObjectGroup2(std::vector<std::vector<obstacle_marker_test::ObjectData>> &groups) {
        std::vector<obstacle_marker_test::ObjectData> group;

        // 创建一个多传感器融合的对象 - 做直线运动
        obstacle_marker_test::ObjectData obj_data1;
        obj_data1.time = this->get_clock()->now();

        // 使用固定的UUID，避免每次都生成新的
        static boost::uuids::uuid fixed_uuid = boost::uuids::random_generator()();
        obj_data1.uuid = fixed_uuid;
        obj_data1.uuid_str = boost::uuids::to_string(obj_data1.uuid);

        // 直线运动：从(-5, -2)到(5, 2)，周期性往返
        double t = fmod(time_counter_, 10.0);                      // 10秒周期
        double progress = (t < 5.0) ? (t / 5.0) : (2.0 - t / 5.0); // 0到1再到0

        double start_x = -5.0, start_y = -2.0;
        double end_x = 5.0, end_y = 2.0;

        obj_data1.tracker_point.x = start_x + progress * (end_x - start_x);
        obj_data1.tracker_point.y = start_y + progress * (end_y - start_y);
        obj_data1.tracker_point.z = 0.0;

        // LiDAR检测点
        obj_data1.detection_point.x = obj_data1.tracker_point.x - 0.1;
        obj_data1.detection_point.y = obj_data1.tracker_point.y - 0.1;
        obj_data1.detection_point.z = 0.0;
        obj_data1.is_associated = true;
        obj_data1.channel_id = 0; // LiDAR
        obj_data1.existence_vector = {0.7f, 0.6f, 0.0f};
        obj_data1.total_existence_probability = 0.85f;

        // 同一个对象的相机检测
        obstacle_marker_test::ObjectData obj_data2 = obj_data1;
        obj_data2.detection_point.x = obj_data1.tracker_point.x + 0.1;
        obj_data2.detection_point.y = obj_data1.tracker_point.y + 0.1;
        obj_data2.channel_id = 1; // Camera

        group.push_back(obj_data1);
        group.push_back(obj_data2);
        groups.push_back(group);
    }

    void createTestObjectGroup3(std::vector<std::vector<obstacle_marker_test::ObjectData>> &groups) {
        std::vector<obstacle_marker_test::ObjectData> group;

        // 创建一个没有关联的对象（灰色显示）- 做振荡运动
        obstacle_marker_test::ObjectData obj_data;
        obj_data.time = this->get_clock()->now();

        // 使用固定的UUID，避免每次都生成新的
        static boost::uuids::uuid fixed_uuid = boost::uuids::random_generator()();
        obj_data.uuid = fixed_uuid;
        obj_data.uuid_str = boost::uuids::to_string(obj_data.uuid);

        // 振荡运动：在Y轴方向上下振荡
        obj_data.tracker_point.x = 0.0;
        obj_data.tracker_point.y = 4.0 + 2.0 * sin(time_counter_ * 0.8); // 振幅2米，频率0.8 rad/s
        obj_data.tracker_point.z = 0.0;

        obj_data.detection_point.x = 0.0; // 不会被使用，因为没有关联
        obj_data.detection_point.y = 0.0;
        obj_data.detection_point.z = 0.0;
        obj_data.is_associated = false; // 没有关联
        obj_data.channel_id = 2;        // Radar channel
        obj_data.existence_vector = {0.0f, 0.0f, 0.4f};
        obj_data.total_existence_probability = 0.4f;

        group.push_back(obj_data);
        groups.push_back(group);
    }

  private:
    std::string frame_id_;
    std::vector<obstacle_marker_test::InputChannel> channels_config_;
    std::unique_ptr<obstacle_marker_test::TrackerObjectDebugger> debugger_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_counter_; // 时间计数器，用于动画
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleMarkerTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
