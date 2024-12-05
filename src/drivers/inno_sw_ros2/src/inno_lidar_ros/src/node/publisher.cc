/*
 *  Copyright (C) 2023 Innovusion Inc.
 *
 *  License: Apache License
 *
 *  $Id$
 */

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "driver_lidar.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#define ROS2_INFO(...) RCLCPP_INFO(rclcpp::get_logger("ivu_pub"), __VA_ARGS__)
#define ROS2_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("ivu_pub"), __VA_ARGS__)
#define ROS2_WARN(...) RCLCPP_WARN(rclcpp::get_logger("ivu_pub"), __VA_ARGS__)
#define ROS2_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("ivu_pub"), __VA_ARGS__)
#define ROS2_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("ivu_pub"), __VA_ARGS__)

namespace innovusion {
class InnovusionComponent : public rclcpp::Node {
  friend class DriverLidar;

 public:
  InnovusionComponent()
      : Node("ivu_pub", rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true)
                            .use_intra_process_comms(true)) {
    using std::string;

    driver_.reset(new innovusion::DriverLidar(inno_log_convert_callback_, rclcpp_shutdown_));

    this->get_parameter_or<string>("output_topic", lidar_topic_, "iv_points");
    this->get_parameter_or<bool>("replay_rosbag", driver_->replay_rosbag_flag_, false);
    this->get_parameter_or<bool>("packets_mode", driver_->packets_mode_, false);
    this->get_parameter_or<uint32_t>("aggregate_num", driver_->aggregate_packets_num_, 10);
    this->get_parameter_or<string>("lidar_log_limit", lidar_log_limit_, "info");
    this->get_parameter_or<string>("lidar_name", driver_->lidar_name_, "falcon");
    this->get_parameter_or<string>("frame_id", driver_->frame_id_, "innovusion");
    this->get_parameter_or<string>("device_ip", driver_->lidar_ip_, "172.168.1.10");
    this->get_parameter_or<string>("pcap_file", driver_->pcap_file_, "");
    this->get_parameter_or<uint32_t>("port", driver_->lidar_port_, 8010);
    this->get_parameter_or<bool>("reflectance_mode", driver_->reflectance_mode_, true);
    this->get_parameter_or<uint32_t>("multiple_return", driver_->multiple_return_, 1);
    this->get_parameter_or<uint32_t>("packet_rate", driver_->packet_rate_, 20);
    this->get_parameter_or<int32_t>("file_rewind", driver_->file_rewind_, 0);
    this->get_parameter_or<int32_t>("udp_port", driver_->udp_port_, 8010);
    this->get_parameter_or<float>("max_range", driver_->max_range_, 2000.0);  // unit: meter
    this->get_parameter_or<float>("min_range", driver_->min_range_, 0.4);     // unit: meter
    this->get_parameter_or<string>("name_value_pairs", driver_->name_value_pairs_, "");
    this->get_parameter_or<int32_t>("continue_live", driver_->continue_live_, 1);
    this->get_parameter_or<uint32_t>("coordinate_mode", driver_->coordinate_mode_, 0);

    ROS2_INFO(
        "\n\tlidar_name: %s, frame_id: %s\n"
        "\tlidar_ip: %s, lidar_port: %d\n"
        "\treflectance: %d, multiple_return: %d\n"
        "\tpcap_file: %s\n"
        "\tpacket_rate: %d\n"
        "\tfile_rewind: %d\n"
        "\tudp_port: %d"
        "\tmax_range: %f, min_range: %f\n"
        "\tname_value_pairs: %s\n"
        "\tcontinue_live: %d\n"
        "\tcoordinate_mode: %d\n"
        "\tlidar_log_limit: %s\n",
        driver_->lidar_name_.c_str(), driver_->frame_id_.c_str(), driver_->lidar_ip_.c_str(), driver_->lidar_port_,
        driver_->reflectance_mode_, driver_->multiple_return_, driver_->pcap_file_.c_str(), driver_->packet_rate_,
        driver_->file_rewind_, driver_->udp_port_, driver_->max_range_, driver_->min_range_,
        driver_->name_value_pairs_.c_str(), driver_->continue_live_, driver_->coordinate_mode_,
        lidar_log_limit_.c_str());

    input_parameter_check_();
    set_lidar_log_level_();

    // point cloud publisher
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    driver_->inno_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_, qos);
    if (driver_->packets_mode_) {
      driver_->inno_pkt_pub_ = this->create_publisher<innovusion::msg::InnovusionScan>("iv_packets", 100);
      driver_->inno_pkt_sub_ = this->create_subscription<innovusion::msg::InnovusionScan>(
          "iv_packets", 100, std::bind(&DriverLidar::process_raw_packet_, driver_.get(), std::placeholders::_1));
    }
    // subscriber data to control roi
    hori_roi_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "hori_roi", 10, std::bind(&InnovusionComponent::process_hori_roi_, this, std::placeholders::_1));
    vertical_roi_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "vertical_roi", 10, std::bind(&InnovusionComponent::process_vertical_roi_, this, std::placeholders::_1));

    driver_->start_lidar();  // do not forget to turn on lidar stream
  }

 private:
  static void rclcpp_shutdown_() {
    rclcpp::shutdown();
  }

  static void inno_log_convert_callback_(void *ctx, enum InnoLogLevel level, const char *header1, const char *header2,
                                         const char *msg) {
    switch (level) {
      case INNO_LOG_LEVEL_FATAL:
      case INNO_LOG_LEVEL_CRITICAL:
        ROS2_FATAL("%s %s", header2, msg);
        break;
      case INNO_LOG_LEVEL_ERROR:
      case INNO_LOG_LEVEL_TEMP:
        ROS2_ERROR("%s %s", header2, msg);
        break;
      case INNO_LOG_LEVEL_WARNING:
      case INNO_LOG_LEVEL_DEBUG:
        ROS2_WARN("%s %s", header2, msg);
        break;
      case INNO_LOG_LEVEL_INFO:
        ROS2_INFO("%s %s", header2, msg);
        break;
      case INNO_LOG_LEVEL_TRACE:
      case INNO_LOG_LEVEL_DETAIL:
      default:
        ROS2_DEBUG("%s %s", header2, msg);
    }
  }

  void process_hori_roi_(const std_msgs::msg::Float64::SharedPtr msg) {
    ROS2_INFO("get hori_roi %f", msg->data);
    double hori_roi = msg->data;
    int ret = driver_->lidar_set_roi(hori_roi, std::numeric_limits<double>::max());
    if (ret != 0) {
      ROS2_ERROR("set hori_roi failed");
    }
  }

  void process_vertical_roi_(const std_msgs::msg::Float64::SharedPtr msg) {
    ROS2_INFO("get vertical_roi %f", msg->data);
    double vert_roi = msg->data;
    int ret = driver_->lidar_set_roi(std::numeric_limits<double>::max(), vert_roi);
    if (ret != 0) {
      ROS2_ERROR("set verti_roi failed");
    }
  }

  void input_parameter_check_() {
    if (driver_->min_range_ >= driver_->max_range_) {
      ROS2_ERROR("The maximum range is less than The minimum range");
      rclcpp::shutdown();
    }

    if (driver_->max_range_ < 2.0) {
      ROS2_ERROR("The maximum range is less than the blind spot");
      rclcpp::shutdown();
    }

    if (driver_->min_range_ > 550.0) {
      ROS2_ERROR("The minimum range is greater than the lidar effective distance");
      rclcpp::shutdown();
    }

    if (!(driver_->packets_mode_)&& driver_->replay_rosbag_flag_) {
      ROS2_WARN("The replay_rosbag is only valid in packets mode, turn on packets mode");
      driver_->packets_mode_ = true;
    }
  }

  void set_lidar_log_level_() {
    if (lidar_log_limit_.compare("info") == 0) {
      driver_->log_level_ = INNO_LOG_LEVEL_INFO;
    } else if (lidar_log_limit_.compare("warn") == 0) {
      driver_->log_level_ = INNO_LOG_LEVEL_WARNING;
    } else if (lidar_log_limit_.compare("error") == 0) {
      driver_->log_level_ = INNO_LOG_LEVEL_ERROR;
    } else {
      driver_->log_level_ = INNO_LOG_LEVEL_INFO;
    }
  }

 private:
  std::shared_ptr<innovusion::DriverLidar> driver_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hori_roi_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vertical_roi_sub_;
  std::string lidar_topic_;
  std::string lidar_log_limit_;
};
}  // namespace innovusion

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<innovusion::InnovusionComponent>());
  rclcpp::shutdown();
  return 0;
}
