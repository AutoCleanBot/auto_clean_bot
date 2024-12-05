/*
 *  Copyright (C) 2023 Innovusion Inc.
 *
 *  License: Apache License
 *
 *  $Id$
 */

#ifndef INNO_DRIVER_LIDAR_H_
#define INNO_DRIVER_LIDAR_H_

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "innovusion/msg/innovusion_scan.hpp"
#include "innovusion/msg/innovusion_packet.hpp"
#include "point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sdk_common/inno_lidar_packet.h"
#include "utils/inno_lidar_log.h"

typedef void (*ShutdownCallBack)();

namespace innovusion {

typedef innovusion_pointcloud::PointXYZIT iVuPoint;
typedef pcl::PointCloud<iVuPoint> iVuPointCloud;
class DriverLidar {
  friend class InnovusionComponent;

 public:
  DriverLidar(InnoLogCallback log_cb, ShutdownCallBack shutdown_cb);
  ~DriverLidar();

  // static callback warpper
  static void lidar_message_callback_s(int handle, void *ctx, uint32_t from_remote, enum InnoMessageLevel level,
                                       enum InnoMessageCode code, const char *error_message);
  static int lidar_data_callback_s(int handle, void *ctx, const InnoDataPacket *pkt);
  static int lidar_status_callback_s(int handle, void *ctx, const InnoStatusPacket *pkt);
  static void lidar_log_callback_s(void *ctx, enum InnoLogLevel level, const char *header1, const char *header2,
                                   const char *msg);
  // lidar configuration
  void start_lidar();
  void stop_lidar();
  int lidar_set_roi(double horz_angle, double vert_angle);

 private:
  // callback group
  int lidar_data_callback_(const InnoDataPacket *pkt);
  void lidar_message_callback_(uint32_t from_remote, enum InnoMessageLevel level, enum InnoMessageCode code,
                               const char *msg);
  int lidar_status_callback_(const InnoStatusPacket *pkt);

  int process_packets_and_publish_frame_(const InnoDataPacket *pkt);
  int collect_packets_and_publish_packets_(const InnoDataPacket *pkt);

  void process_raw_packet_(const innovusion::msg::InnovusionScan::SharedPtr msg);
  void convert_and_parse_(const InnoDataPacket *pkt);
  int lidar_parameter_set_(int handle);
  bool first_call_();
  bool setup_lidar_();
  int lidar_live_process_();
  int pcap_playback_process_();
  int set_config_name_value_(int handle);
  void start_check_datacallback_thread_();
  void data_packet_parse_(const InnoDataPacket *pkt);
  template <typename PointType>
  void point_xyz_data_parse_(bool is_en_data, bool is_use_refl, uint32_t point_num, PointType point_ptr);

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inno_pub_{nullptr};
  rclcpp::Publisher<innovusion::msg::InnovusionScan>::SharedPtr inno_pkt_pub_{nullptr};
  rclcpp::Subscription<innovusion::msg::InnovusionScan>::SharedPtr inno_pkt_sub_{nullptr};
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pc2_msg_;
  std::unique_ptr<innovusion::msg::InnovusionScan> inno_scan_msg_;
  iVuPointCloud::Ptr inno_pc_ptr_;

  bool is_first_call_{false};
  std::string lidar_name_{"inno"};
  std::string lidar_ip_{"172.168.1.10"};
  std::string pcap_file_{""};
  bool packets_mode_{false};
  uint32_t lidar_port_{8010};
  bool reflectance_mode_{true};
  uint32_t multiple_return_{1};
  // replay file
  bool replay_rosbag_flag_{false};
  uint32_t packet_rate_{10000};
  int32_t file_rewind_{0};
  int32_t udp_port_{0};
  int32_t continue_live_{1};  // restart lidar when an exception occurs
  float max_range_{0.0};
  float min_range_{0.0};
  std::string name_value_pairs_;
  // status
  uint32_t table_send_hz_{10};  // X frames send 1 table
  uint32_t frame_count_{0};
  bool anglehv_table_init_{false};  // for robinw_compact
  std::vector<char> anglehv_table_;  // for robinw_compact
  bool is_running_{false};
  uint32_t aggregate_packets_num_{10};
  std::atomic_bool is_receive_data_{false};
  int lidar_handle_{-1};  // lidar handle
  InnoLogCallback log_callback_{NULL};
  ShutdownCallBack shutdown_callback_{NULL};
  int64_t current_frame_id_{-1};
  uint64_t packets_width_{0};
  std::vector<uint8_t> xyz_from_sphere_;
  uint32_t coordinate_mode_{0};
  double current_ts_start_;
  std::string frame_id_{"innovusion"};
  InnoLogLevel log_level_{INNO_LOG_LEVEL_INFO};
};

}  // namespace innovusion
#endif
