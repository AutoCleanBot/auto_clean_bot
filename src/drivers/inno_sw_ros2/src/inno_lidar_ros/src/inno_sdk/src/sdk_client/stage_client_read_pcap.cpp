/**
 *  Copyright (C) 2023 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */


#include "sdk_client/stage_client_read_pcap.h"
#if defined(PCAP_INPUT) && PCAP_INPUT == 1
#include <pcap/vlan.h>
#include <pcap/dlt.h>
#include <pcap/sll.h>
#include "sdk_client/lidar_client.h"
#include "sdk_common/inno_lidar_packet_utils.h"

namespace innovusion {

////////////////////////////// PcapInput //////////////////////////////
int PcapInput::open_file_() {
  char err_buf[PCAP_ERRBUF_SIZE] = "";
  const char *filename = input_param_.pcap_param.filename;
  pcap_handle_ = pcap_open_offline(filename, err_buf);
  if (!pcap_handle_) {
    inno_log_info("cannot open %s", filename);
    char *pwd = getenv("PWD");

    if (pwd) {
      std::string file = std::string(pwd) + "/" + filename;
      pcap_handle_ = pcap_open_offline(file.c_str(), err_buf);
      if (!pcap_handle_) {
        inno_log_error("pcap_open_offline failed for file %s, error msg: %s", file.c_str(), err_buf);
        return -1;
      } else {
        inno_log_info("open pcap file %s", file.c_str());
      }
    } else {
      inno_log_error("pcap_open_offline failed for file %s, error msg: %s", filename, err_buf);
      return -1;
    }
  }
  return 0;
}

int PcapInput::read_data() {
  int32_t max_file_rewind_ = input_param_.pcap_param.rewind;
  if (max_file_rewind_ >= 0) {
    if (play_round_ >= max_file_rewind_ + 1) {
      return -3;
    }
  }
  int ret = open_file_();
  if (ret >= 0) {
    reach_file_end_ = false;
    int ret = keep_reading_();
    if (pcap_handle_) {
      pcap_close(pcap_handle_);
      pcap_handle_ = NULL;
    }
    if (ret == -2) {
      // end of file is not really an error
      reach_file_end_ = true;
      ret = 0;
    }
    play_round_++;
    set_first_step(true);
    inno_log_info("%s rewind file %s %d/%d", lidar_->get_name(), input_param_.pcap_param.filename,
                                                play_round_, max_file_rewind_);
    return ret;
  } else {
    cannot_open_file_ = true;
    return -1;
  }
}

int PcapInput::read_fd_(char *buf, size_t len) {
  size_t read_len = 0;

  while (read_len < len) {
    if (curr_pkt_ && curr_pkt_idx_ < curr_pkt_->len) {
      size_t read_size = len - read_len;
      if (static_cast<size_t>(curr_pkt_->len - curr_pkt_idx_) < read_size) {
        read_size = static_cast<size_t>(curr_pkt_->len - curr_pkt_idx_);
      }
      memcpy(buf + read_len, curr_pkt_->buf + curr_pkt_idx_, read_size);
      read_len += read_size;
      curr_pkt_idx_ += read_size;
    }

    if (read_len < len) {
      if (curr_pkt_) {
        curr_pkt_->len = 0;
        curr_pkt_->offset = 0;
        curr_pkt_ = nullptr;
      }
      if (!read_next_packet_()) {
        break;
      }
      /*
      Fix the issue in SDK 3.x where get_pcd service cannot parse fixed-length pcap files into pcd files.
      In the fixed-length pcap data file, the end data is obtained by zero padding,
      causing a mismatch between the pcap's payload length and the data structure length defined by seyond.
      So the fixed-length pcap file cannot be parsed.
      */
      if (curr_pkt_ && curr_pkt_->len >= sizeof(InnoCommonHeader)) {
        InnoCommonHeader * header = reinterpret_cast<InnoCommonHeader*>(curr_pkt_->buf);
        curr_pkt_->len = header->size;
      }
    }
  }

  return static_cast<int>(read_len);
}

bool PcapInput::read_next_packet_() {
  struct pcap_pkthdr pcap_header;
  while (1) {
    const u_char *packet = pcap_next(pcap_handle_, &pcap_header);

    if (packet == nullptr) {
      return false;
    }

    const struct iphdr *ipptr = nullptr;
    if (pcap_datalink(pcap_handle_) == DLT_EN10MB) {
      const struct ether_header *ether_net = reinterpret_cast<const struct ether_header *>(packet);
      // https://support.huawei.com/enterprise/zh/doc/EDOC1100088136
      if (ntohs(ether_net->ether_type) == ETHERTYPE_VLAN) {
        const vlan_tag *vlan = reinterpret_cast<const vlan_tag *>(packet + sizeof(struct ether_header));
        if (ntohs(vlan->vlan_tci) == ETHERTYPE_IP) {
          ipptr = reinterpret_cast<const struct iphdr *>(packet + sizeof(struct ether_header) + sizeof(vlan_tag));
        } else {
          // only parse once vlan tag, could be extended to parse multiple vlan tags (such as QinQ)
          continue;
        }
      } else if (ntohs(ether_net->ether_type) == ETHERTYPE_IP) {
        ipptr = reinterpret_cast<const struct iphdr *>(packet + sizeof(struct ether_header));
      } else {
        continue;
      }
    } else if (pcap_datalink(pcap_handle_) == DLT_LINUX_SLL) {
      // https://www.tcpdump.org/linktypes/LINKTYPE_LINUX_SLL.html
      const sll_header *linux_sll = reinterpret_cast<const sll_header *>(packet);
      // only handle arphrd_type and protocol (Ethernet + IP)
      if (ntohs(linux_sll->sll_hatype) == ARPHRD_ETHER && ntohs(linux_sll->sll_protocol) == ETHERTYPE_IP) {
        ipptr = reinterpret_cast<const struct iphdr *>(packet + sizeof(sll_header));
      } else {
        continue;
      }
    } else {
      inno_log_error("pcap datalink %d not supported", pcap_datalink(pcap_handle_));
      return false;
    }

    // ipptr is not null
    if (ipptr->version != 4 || ipptr->protocol != IPPROTO_UDP || inet_ntoa({ipptr->saddr}) != lidar_ip_) {
      continue;
    }

    uint16_t frag_off = ntohs(ipptr->frag_off);
    uint16_t tot_len = ntohs(ipptr->tot_len);
    uint16_t ihl = ipptr->ihl * 4;
    uint16_t offset = (frag_off & IP_OFFMASK) << 3;
    uint16_t id = ntohs(ipptr->id);

    /**
     * there are four cases of fragment:
     * 1. DF = 1, no fragment
     * 2. DF = 0, MF = 0, offset == 0, no fragment
     *    information link: https://www.rfc-editor.org/rfc/rfc791#page-25
     * 3. DF = 0, MF = 1, more fragment
     * 4. DF = 0, MF = 0, offset != 0, last fragment
     */

    if (offset == 0) {
      // check udp packet dest port
      const struct udphdr *udp_header =
          reinterpret_cast<const struct udphdr *>(reinterpret_cast<const char*>(ipptr) + ihl);

      uint16_t dest_port = ntohs(udp_header->dest);

      if (dest_port != input_param_.pcap_param.data_port && dest_port != input_param_.pcap_param.status_port &&
          dest_port != input_param_.pcap_param.message_port) {
        inno_log_info("udp packet port does not match (%u), data is discarded", dest_port);
        continue;
      }
    }

    // no fragment
    if ((frag_off & IP_DF) || (!(frag_off & IP_MF) && offset == 0)) {
      /**
       * packet structure: [ ip header | udp header | udp payload ]
       */

      // ip header length + udp header length
      uint16_t header_len = ihl + sizeof(struct udphdr);
      // udp payload length
      uint16_t payload_len = tot_len - header_len;
      // use udp payload as lidar packet

      memcpy(single_cache_.buf, reinterpret_cast<const char *>(ipptr) + header_len, payload_len);
      single_cache_.len = payload_len;
      curr_pkt_ = &single_cache_;
      curr_pkt_idx_ = 0;
      return true;
    } else {
      PcapBuffer *pkt_buf = &cache_[id & (PCAP_CACHE_LENGTH - 1)];

      // id that does not meet expectations, discard data
      if (pkt_buf->id != id) {
        if (pkt_buf->len != 0) {
          inno_log_warning("id %u != expect_id %u, data is discarded", id, pkt_buf->id);
        }
        pkt_buf->id = id;
        pkt_buf->len = 0;
        pkt_buf->offset = 0;
      }

      /**
       * offset that does not meet expectations, two ways to do:
       * 1. offset != 0, be discarded, it's unlikely to form a complete data set
       * 2. offset = 0, discard old data and reassemble new data
       */
      if (offset != pkt_buf->offset) {
        if (offset != 0) {
          inno_log_warning("offset %u != expect_offset %u, data is discarded", offset, pkt_buf->offset);
          continue;
        }
        inno_log_warning("offset %u != expect_offset %u, reassemble new data", offset, pkt_buf->offset);
        pkt_buf->len = 0;
        pkt_buf->offset = 0;
      }

      /**
       * fragment is divided into two scenarios:
       * 1. offset == 0, packet structure:
       *    [ ip header | udp header | udp payload fragment ]
       * 2. offset != 0, packet structure:
       *    [ ip header | udp payload fragment ]
       */
      uint16_t header_len = ihl;

      if (offset == 0) {
        header_len += sizeof(struct udphdr);
      }

      uint16_t payload_len = tot_len - header_len;

      // buffer overflow prevention
      if (pkt_buf->len + payload_len > UDP_PACKET_MAX_SIZE) {
        inno_log_error("packet buffer does not have enough space, %d + %d > %d", pkt_buf->len, payload_len,
                       UDP_PACKET_MAX_SIZE);
        // current UDP packet is discarded
        pkt_buf->len = 0;
        pkt_buf->offset = 0;
        continue;  // not return, reread
      }

      // update status
      memcpy(pkt_buf->buf + pkt_buf->len, reinterpret_cast<const char *>(ipptr) + header_len, payload_len);
      pkt_buf->len += payload_len;
      pkt_buf->offset += tot_len - ihl;
      if (!(frag_off & IP_MF)) {
        // last fragment
        curr_pkt_ = pkt_buf;
        curr_pkt_idx_ = 0;
        return true;
      }
    }
  }
}
}  // namespace innovusion

#endif
