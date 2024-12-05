/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_STAGE_CLIENT_READ_PCAP_H_
#define SDK_CLIENT_STAGE_CLIENT_READ_PCAP_H_
#if defined(PCAP_INPUT) && PCAP_INPUT == 1
#if !(defined(__MINGW64__) || defined(_WIN32))
#include <arpa/inet.h>
#include <netinet/in.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <netinet/if_ether.h>
#include <pcap/pcap.h>
#endif
#include <string>

#include "sdk_client/stage_client_read.h"

#ifndef IP_DF
#define IP_DF 0x4000  // dont fragment flag
#endif
#ifndef IP_MF
#define IP_MF 0x2000  // more fragments flag
#endif
#ifndef IP_OFFMASK
#define IP_OFFMASK 0x1fff  // mask for fragmenting bits
#endif

namespace innovusion {
class InnoLidarClient;

class PcapInput : public FileInput {
 public:
  explicit PcapInput(InnoLidarClient *lidar, const void *param) : FileInput(lidar, param) {
    lidar_ip_ = input_param_.pcap_param.lidar_ip;
    filename_ = input_param_.pcap_param.filename;
    inno_log_info("filename: %s, play_round: %d", filename_, play_round_);
  }

  virtual ~PcapInput() {
  }
  int read_data() override;

  static const int UDP_PACKET_MAX_SIZE = 65536;
  static const int PCAP_CACHE_LENGTH = 64;

 protected:
  int read_fd_(char *buf, size_t len) override;

 private:
  PcapInput() = delete;
  PcapInput(const PcapInput&) = delete;
  PcapInput operator=(const PcapInput&) = delete;
  bool read_next_packet_();
  int open_file_();

 private:
  struct PcapBuffer {
    uint16_t id = 0;
    uint32_t offset = 0;
    uint32_t len = 0;
    char buf[UDP_PACKET_MAX_SIZE] = {0};
  };
  std::string lidar_ip_;
  pcap_t *pcap_handle_ = NULL;
  PcapBuffer *curr_pkt_ = NULL;
  int32_t curr_pkt_idx_ = -1;
  PcapBuffer cache_[PCAP_CACHE_LENGTH];
  PcapBuffer single_cache_;
};
}  // namespace innovusion

#endif
#endif  // SDK_CLIENT_STAGE_CLIENT_READ_PCAP_H_
