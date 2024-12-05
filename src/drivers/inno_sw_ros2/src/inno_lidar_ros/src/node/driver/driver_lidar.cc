/*
 *  Copyright (C) 2023 Innovusion Inc.
 *
 *  License: Apache License
 *
 *  $Id$
 */

#include "driver_lidar.h"

#include <assert.h>

#include <thread>
#include <utility>

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_other_api.h"
#include "sdk_common/inno_lidar_packet_utils.h"

constexpr uint64_t KBUF_SIZE = 1024 * 1024 * 10;
constexpr double us_in_second_c = 1000000.0;
constexpr double ten_us_in_second_c = 100000.0;

namespace innovusion {

static void coordinate_transfer_(iVuPoint *point, int32_t coordinate_mode, float x, float y, float z) {
  switch (coordinate_mode) {
    case 0:
      point->x = x;  // up
      point->y = y;  // right
      point->z = z;  // forward
      break;
    case 1:
      point->x = y;  // right
      point->y = z;  // forward
      point->z = x;  // up
      break;
    case 2:
      point->x = y;  // right
      point->y = x;  // up
      point->z = z;  // forward
      break;
    case 3:
      point->x = z;   // forward
      point->y = -y;  // -right
      point->z = x;   // up
      break;
    case 4:
      point->x = z;  // forward
      point->y = x;  // up
      point->z = y;  // right
      break;
    default:
      // default
      point->x = x;  // up
      point->y = y;  // right
      point->z = z;  // forward
      break;
  }
}

DriverLidar::DriverLidar(InnoLogCallback log_cb,
                         ShutdownCallBack shutdown_cb)
    : log_callback_(log_cb), shutdown_callback_(shutdown_cb) {
  xyz_from_sphere_.resize(KBUF_SIZE);
  inno_pc_ptr_ = iVuPointCloud::Ptr(new iVuPointCloud());
  inno_scan_msg_ = std::make_unique<innovusion::msg::InnovusionScan>();
}

DriverLidar::~DriverLidar() {
  stop_lidar();  // make sure that lidar_handle_ has been closed
  xyz_from_sphere_.clear();
}

int DriverLidar::lidar_data_callback_s(int handle, void *ctx, const InnoDataPacket *pkt) {
  DriverLidar *context = reinterpret_cast<DriverLidar *>(ctx);
  assert(handle == context->lidar_handle_);
  return context->lidar_data_callback_(pkt);
}

void DriverLidar::lidar_message_callback_s(int handle, void *ctx, uint32_t from_remote, enum InnoMessageLevel level,
                                           enum InnoMessageCode code, const char *error_message) {
  DriverLidar *context = reinterpret_cast<DriverLidar *>(ctx);
  assert(handle == context->lidar_handle_);
  context->lidar_message_callback_(from_remote, level, code, error_message);
}

int DriverLidar::lidar_status_callback_s(int handle, void *ctx, const InnoStatusPacket *pkt) {
  DriverLidar *context = reinterpret_cast<DriverLidar *>(ctx);
  assert(handle == context->lidar_handle_);
  return context->lidar_status_callback_(pkt);
}

void DriverLidar::lidar_log_callback_s(void *ctx, enum InnoLogLevel level, const char *header1, const char *header2,
                                       const char *msg) {
  DriverLidar *context = reinterpret_cast<DriverLidar *>(ctx);
  if (level > context->log_level_) {
    return;
  }
  context->log_callback_(ctx, level, header1, header2, msg);
}

void DriverLidar::start_lidar() {
  bool ret = false;
  inno_log_verify(lidar_handle_ <= 0, "lidar_handle_ should <= 0");
  if (!is_first_call_) {
    ret = first_call_();
    if (replay_rosbag_flag_) {
      return;
    }
    inno_log_verify(ret == true, "first_call failed!");
    is_first_call_ = true;
    inno_log_info("## first call for ros2 driver ##");
  } else {
    ret = setup_lidar_();
    inno_log_verify(ret == true, "setup_lidar_ failed!");
  }
  auto r = inno_lidar_start(lidar_handle_);
  inno_log_verify(r == 0, "inno_lidar_start failed!");
  is_running_ = true;
  start_check_datacallback_thread_();
}

void DriverLidar::stop_lidar() {
  if (lidar_handle_ > 0) {
    auto r = inno_lidar_stop(lidar_handle_);
    inno_log_verify(r == 0, "inno_lidar_stop failed!");
    inno_lidar_close(lidar_handle_);
    inno_log_verify(r == 0, "inno_lidar_close failed!");
  }
  current_frame_id_ = -1;
  lidar_handle_ = -1;
  is_running_ = false;
}

int DriverLidar::lidar_set_roi(double horz_angle, double vert_angle) {
  int ret = 0;
  if (lidar_handle_ < 0) {
    inno_log_error("lidar has not been initialized");
    return -1;
  }

  ret = inno_lidar_set_roi(lidar_handle_, horz_angle, vert_angle);
  if (ret != 0) {
    inno_log_error("roi set failed");
    return ret;
  }

  return 0;
}

//////////////////

bool DriverLidar::first_call_() {
  inno_log_info("INNOVUSION LIDAR SDK version=%s build_time=%s", inno_api_version(), inno_api_build_time());
  inno_lidar_setup_sig_handler();
  inno_lidar_set_logs(-1, -1, NULL, 0, 0, lidar_log_callback_s, this, NULL, 0, 0, 1);
  inno_lidar_set_log_level(log_level_);
  inno_log_info("Lidar name is %s", lidar_name_.c_str());
  return setup_lidar_();
}

bool DriverLidar::setup_lidar_() {
  if (pcap_file_.size() > 0) {
    // pcap replay
    if (udp_port_ < 0) {
      inno_log_error("@@ pcap playback mode, udp_port should be set! @@");
      return false;
    }

    if (pcap_playback_process_() != 0) {
      return false;
    }
  } else if (replay_rosbag_flag_) {
    // packet rosbag replay
    inno_log_info("waiting for rosbag to replay...");
    return true;
  } else {
    // live
    if (lidar_live_process_() != 0) {
      return false;
    }
  }

  // set lidar parameters
  if (lidar_parameter_set_(lidar_handle_) != 0) {
    return false;
  }

  return true;
}

int DriverLidar::lidar_parameter_set_(int handle) {
  int ret = 0;
  enum InnoReflectanceMode m = reflectance_mode_ ? INNO_REFLECTANCE_MODE_REFLECTIVITY : INNO_REFLECTANCE_MODE_INTENSITY;
  ret = inno_lidar_set_reflectance_mode(handle, m);
  if (ret != 0) {
    inno_log_warning("set_reflectance ", reflectance_mode_, " return ", ret);
    return ret;
  }

  ret = inno_lidar_set_return_mode(handle, (InnoMultipleReturnMode)multiple_return_);
  if (ret != 0) {
    inno_log_warning("set_return_mode ", multiple_return_, " return ", ret);
    return ret;
  }

  ret = set_config_name_value_(handle);
  if (ret != 0) {
    inno_log_verify(ret == 0, "set config name %d", ret);
    return ret;
  }

  ret = inno_lidar_set_callbacks(handle, lidar_message_callback_s, lidar_data_callback_s, lidar_status_callback_s, NULL,
                                 this);
  if (ret != 0) {
    inno_log_verify(ret == 0, "inno_lidar_set_callbacks failed!, ret: %d", ret);
    return ret;
  }

  return 0;
}

int DriverLidar::lidar_live_process_() {
  enum InnoLidarProtocol protocol_;
  // setup read from live
  uint16_t tmp_udp_port = 0;
  if (udp_port_ >= 0) {
    protocol_ = INNO_LIDAR_PROTOCOL_PCS_UDP;
    tmp_udp_port = udp_port_;
  } else {
    protocol_ = INNO_LIDAR_PROTOCOL_PCS_TCP;
  }

  lidar_handle_ = inno_lidar_open_live(lidar_name_.c_str(), lidar_ip_.c_str(), lidar_port_, protocol_, tmp_udp_port);
  if (lidar_handle_ < 0) {
    inno_log_error("FATAL: Lidar %s invalid handle", lidar_name_.c_str());
    return -1;
  }

  // ros always run externally, so we set timeout longer
  int ret_v = inno_lidar_set_config_name_value(lidar_handle_, "LidarClient_Communication/get_conn_timeout_sec", "5.0");
  if (ret_v != 0) {
    inno_log_error("inno_lidar_set_config_name_value 'get_conn_timeout_sec 5.0' failed %d", ret_v);
  }

  // enable client sdk midorder_fix
  ret_v = inno_lidar_set_config_name_value(lidar_handle_, "LidarClient_StageClientRead/misorder_correct_enable", "1");
  inno_log_verify(ret_v == 0, "inno_lidar_set_config_name_value 'misorder_correct_enable 1' failed %d", ret_v);

  return 0;
}

int DriverLidar::pcap_playback_process_() {
  InputParam param;
  param.pcap_param.source_type = SOURCE_PCAP;
  strncpy(param.pcap_param.filename, pcap_file_.c_str(), pcap_file_.length() + 1);
  strncpy(param.pcap_param.lidar_ip, lidar_ip_.c_str(), lidar_ip_.length() + 1);
  param.pcap_param.data_port = udp_port_;
  param.pcap_param.message_port = udp_port_;
  param.pcap_param.status_port = udp_port_;
  param.pcap_param.play_rate = packet_rate_;
  param.pcap_param.rewind = file_rewind_;
  inno_log_info("## pcap_file is %s, device_ip_ is %s, play_rate is %d, rewind id %d, %d/%d/%d ##", pcap_file_.c_str(),
                lidar_ip_.c_str(), packet_rate_, file_rewind_, udp_port_, udp_port_, udp_port_);
  lidar_handle_ = inno_lidar_open_ctx(lidar_name_.c_str(), &param);
  if (lidar_handle_ < 0) {
    inno_log_error("FATAL: Lidar %s invalid handle", lidar_name_.c_str());
    return -1;
  }
  return 0;
}

int DriverLidar::set_config_name_value_(int handle) {
  char *rest = NULL;
  char *token;
  char *nv = strdup(name_value_pairs_.c_str());
  inno_log_info("Use name_value_pairs %s", name_value_pairs_.c_str());
  if (nv) {
    for (token = strtok_r(nv, ",", &rest); token != NULL; token = strtok_r(NULL, ",", &rest)) {
      char *eq = strchr(token, '=');
      if (eq) {
        *eq = 0;
        if (inno_lidar_set_config_name_value(handle, token, eq + 1) != 0) {
          inno_log_warning("bad name_value pairs %s", nv);
          break;
        }
      } else {
        inno_log_warning("bad name_value pairs %s", nv);
        break;
      }
    }
    free(nv);
  }
  return 0;
}

int DriverLidar::lidar_data_callback_(const InnoDataPacket *pkt) {
  if (current_frame_id_ == -1) {
    current_frame_id_ = pkt->idx;
    inno_log_info("get first frame id %lu", current_frame_id_);
    return 0;
  }

  if ((pkt->type == INNO_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD) && (!anglehv_table_init_)) {
    anglehv_table_.resize(sizeof(InnoAngleHVTable) + sizeof(InnoDataPacket));
    int ret = inno_lidar_get_anglehv_table(lidar_handle_, reinterpret_cast<InnoDataPacket*>(anglehv_table_.data()));
    if (ret == 0) {
      anglehv_table_init_ = true;
      inno_log_info("Get RobinW Compact Table");
    }
  }

  if (packets_mode_) {
    return collect_packets_and_publish_packets_(pkt);
  } else {
    return process_packets_and_publish_frame_(pkt);
  }
}

int DriverLidar::process_packets_and_publish_frame_(const InnoDataPacket *pkt) {
  // check if get the whole frame, and publish
  if (current_frame_id_ != pkt->idx) {
    is_receive_data_ = true;
    pc2_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*inno_pc_ptr_, *pc2_msg_);
    pc2_msg_.get()->header.frame_id = frame_id_;
    int64_t ts_ns = pkt->common.ts_start_us * 1000;
    pc2_msg_->header.stamp.sec = ts_ns / 1000000000;
    pc2_msg_->header.stamp.nanosec = ts_ns % 1000000000;
    pc2_msg_.get()->width = inno_pc_ptr_->width;
    pc2_msg_.get()->height = inno_pc_ptr_->height;
    if (inno_pc_ptr_->width != 0) {
      inno_pub_->publish(std::move(pc2_msg_));
    } else {
      inno_log_info("no data can be published, please check max_range and min_range");
    }
    inno_pc_ptr_->clear();
    current_frame_id_ = pkt->idx;
  }

  convert_and_parse_(pkt);

  return 0;
}

int DriverLidar::collect_packets_and_publish_packets_(const InnoDataPacket *pkt) {
  // check if get the whole frame, and publish
  if (current_frame_id_ != pkt->idx) {
    frame_count_++;
    inno_scan_msg_->header.frame_id = frame_id_;
    int64_t ts_ns = pkt->common.ts_start_us * 1000;
    inno_scan_msg_->header.stamp.sec = ts_ns / 1000000000;
    inno_scan_msg_->header.stamp.nanosec = ts_ns % 1000000000;
    inno_scan_msg_->size = packets_width_;
    inno_scan_msg_->is_last_scan = true;
    packets_width_ = 0;
    inno_pkt_pub_->publish(std::move(inno_scan_msg_));
    current_frame_id_ = pkt->idx;
    inno_scan_msg_ = std::make_unique<innovusion::msg::InnovusionScan>();
  } else if (aggregate_packets_num_ == packets_width_) {
    inno_scan_msg_->is_last_scan = false;
    inno_scan_msg_->size = packets_width_;
    packets_width_ = 0;
    inno_pkt_pub_->publish(std::move(inno_scan_msg_));
    inno_scan_msg_ = std::make_unique<innovusion::msg::InnovusionScan>();
  }
  innovusion::msg::InnovusionPacket msg;
  uint64_t pkt_len = sizeof(InnoDataPacket) + pkt->item_number * pkt->item_size;
  msg.data.resize(pkt_len);
  std::memcpy(msg.data.data(), pkt, pkt_len);
  msg.has_table = false;
  if (anglehv_table_init_ && (frame_count_ == table_send_hz_)) {
    frame_count_ = 0;
    msg.has_table = true;
    msg.table.resize(anglehv_table_.size());
    std::memcpy(msg.table.data(), anglehv_table_.data(), anglehv_table_.size());
  }
  packets_width_++;
  inno_scan_msg_->packets.emplace_back(msg);
  return 0;
}

void DriverLidar::process_raw_packet_(const innovusion::msg::InnovusionScan::SharedPtr msg) {
  for (auto &packet_obj : msg->packets) {
    const InnoDataPacket *pkt = reinterpret_cast<const InnoDataPacket *>(packet_obj.data.data());
    if ((replay_rosbag_flag_) && (!anglehv_table_init_) && (pkt->type == INNO_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD)) {
      if (packet_obj.has_table) {
        anglehv_table_init_ = true;
        anglehv_table_.resize(sizeof(InnoAngleHVTable) + sizeof(InnoDataPacket));
        std::memcpy(anglehv_table_.data(), packet_obj.table.data(), sizeof(InnoAngleHVTable) + sizeof(InnoDataPacket));
        inno_log_info("Get RobinW Compact Table");
      } else {
        return;
      }
    }
    convert_and_parse_(pkt);
  }
  if (msg->is_last_scan) {
    pc2_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*inno_pc_ptr_, *pc2_msg_);
    pc2_msg_.get()->header = msg->header;
    pc2_msg_.get()->width = inno_pc_ptr_->width;
    pc2_msg_.get()->height = inno_pc_ptr_->height;
    if (inno_pc_ptr_->width != 0) {
      inno_pub_->publish(std::move(pc2_msg_));
    } else {
      inno_log_info("no data can be published, please check max_range and min_range");
    }
    inno_pc_ptr_->clear();
    is_receive_data_ = true;
  }
}

void DriverLidar::convert_and_parse_(const InnoDataPacket *pkt) {
  if (CHECK_SPHERE_POINTCLOUD_DATA(pkt->type)) {
    // convert sphere to xyz
    if (anglehv_table_init_) {
      inno_lidar_convert_to_xyz_pointcloud2(
          pkt, reinterpret_cast<InnoDataPacket *>(&xyz_from_sphere_[0]), xyz_from_sphere_.size(), false,
          reinterpret_cast<InnoDataPacket *>(anglehv_table_.data()));
    } else {
      inno_lidar_convert_to_xyz_pointcloud(pkt, reinterpret_cast<InnoDataPacket *>(&xyz_from_sphere_[0]),
                                           xyz_from_sphere_.size(), false);
    }
    data_packet_parse_(reinterpret_cast<InnoDataPacket *>(&xyz_from_sphere_[0]));
  } else if (CHECK_XYZ_POINTCLOUD_DATA(pkt->type)) {
    data_packet_parse_(pkt);
  } else {
    inno_log_error("pkt type %d is not supported", pkt->type);
  }
}

void DriverLidar::data_packet_parse_(const InnoDataPacket *pkt) {
  // calculate the point timestamp
  current_ts_start_ = pkt->common.ts_start_us / us_in_second_c;
  // adapt different data structures form different lidar
  if (CHECK_EN_XYZ_POINTCLOUD_DATA(pkt->type)) {
    const InnoEnXyzPoint *pt =
      reinterpret_cast<const InnoEnXyzPoint *>(reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse_<const InnoEnXyzPoint *>(true, pkt->use_reflectance, pkt->item_number, pt);
  } else {
    const InnoXyzPoint *pt =
      reinterpret_cast<const InnoXyzPoint *>(reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse_<const InnoXyzPoint *>(false, pkt->use_reflectance, pkt->item_number, pt);
  }
}

template <typename PointType>
void DriverLidar::point_xyz_data_parse_(bool is_en_data, bool is_use_refl, uint32_t point_num, PointType point_ptr) {
  for (uint32_t i = 0; i < point_num; ++i, ++point_ptr) {
    iVuPoint point;
    if (point_ptr->radius > max_range_ || point_ptr->radius < min_range_) {
      continue;
    }

    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      if (is_use_refl) {
        point.intensity = point_ptr->reflectance;
      } else {
        point.intensity = point_ptr->intensity;
      }
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point.intensity = point_ptr->refl;
    }
    int32_t roi = point_ptr->in_roi == 3 ? (1 << 2) : 0;
    point.scan_id = point_ptr->scan_id;
    point.scan_idx = point_ptr->scan_idx;
    point.flags = point_ptr->channel | roi | (point_ptr->facet << 3) | (point_ptr->type << 6);
    point.is_2nd_return = point_ptr->is_2nd_return;
    point.elongation = point_ptr->elongation;
    point.timestamp = point_ptr->ts_10us / ten_us_in_second_c + current_ts_start_;
    coordinate_transfer_(&point, coordinate_mode_, point_ptr->x, point_ptr->y, point_ptr->z);
    inno_pc_ptr_->points.push_back(point);
    ++inno_pc_ptr_->width;
    inno_pc_ptr_->height = 1;
  }
}

void DriverLidar::lidar_message_callback_(uint32_t from_remote, enum InnoMessageLevel level, enum InnoMessageCode code,
                                          const char *msg) {
  const char *remote = "";
  if (from_remote) {
    remote = "REMOTE-";
  }
  if (level == INNO_MESSAGE_LEVEL_WARNING) {
    inno_log_warning("%s%s level=%d, code=%d, message=%s", remote, inno_log_header_g[level], level, code, msg);
  } else if (level < INNO_MESSAGE_LEVEL_WARNING) {
    inno_log_error("%s%s level=%d, code=%d, message=%s", remote, inno_log_header_g[level], level, code, msg);
  }

  if (code == INNO_MESSAGE_CODE_READ_FILE_END) {
    inno_log_info("read file end");
    std::thread([=]() {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      stop_lidar();
      shutdown_callback_();
    }).detach();
  } else if ((level <= INNO_MESSAGE_LEVEL_CRITICAL && code != INNO_MESSAGE_CODE_LIB_VERSION_MISMATCH) ||
             (code == INNO_MESSAGE_CODE_CANNOT_READ)) {
    std::thread([&]() {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      stop_lidar();
      if (continue_live_) {
        inno_log_error("live data err, continue_live_ is true, will restart in 1s");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        start_lidar();
      } else {
        shutdown_callback_();
      }
    }).detach();
  }
}

int DriverLidar::lidar_status_callback_(const InnoStatusPacket *pkt) {
  // sanity check
  if (!inno_lidar_check_status_packet(pkt, 0)) {
    inno_log_error("corrupted pkt->idx = %" PRI_SIZEU, pkt->idx);
    return -1;
  }

  static uint64_t cnt = 0;
  if (cnt++ % 100 == 1) {
    constexpr uint64_t buf_size = 2048;
    char buf[buf_size]{0};

    int ret = inno_lidar_printf_status_packet(pkt, buf, buf_size);
    if (ret > 0) {
      inno_log_info("Received status packet #%" PRI_SIZELU ": %s", cnt, buf);
    } else {
      inno_log_warning("Received status packet #%" PRI_SIZELU ": errorno: %d", cnt, ret);
    }
  }
  return 0;
}

void DriverLidar::start_check_datacallback_thread_() {
  std::thread([&]() {
    uint8_t checkout_count = 10;
    while (is_running_) {
      for (auto i = 0; i < checkout_count; i++) {
        if (is_receive_data_.load()) {
          is_receive_data_ = false;
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          if (i == checkout_count - 1) {
            stop_lidar();
            if (continue_live_) {
              inno_log_error(
                  "datacallback function did not receive data for more than 10s, "
                  "continue_live_ is true, will restart in 1s");
              std::this_thread::sleep_for(std::chrono::seconds(1));
              start_lidar();
            } else {
              shutdown_callback_();
            }
            return;
          }
        }
      }
      static uint64_t cnt = 0;
      if (cnt++ % checkout_count == 1) {
        inno_log_info("Continuously receiving datacallback data...");
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }).detach();
}

}  // namespace innovusion
