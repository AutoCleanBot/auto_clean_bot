/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_RING_ID_CONVERTER_H_
#define SDK_CLIENT_RING_ID_CONVERTER_H_

#include <mutex>
#include <string>
#include <unordered_map>

#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/ring_id_converter_interface.h"
#include "utils/utils.h"

namespace innovusion {
class InnoLidarClient;
/**
 * @brief RingIdConverter
 */
class RingIdConverter : public RingIdConverterInterface {
 public:
  static const size_t kGalvoPeridUnitsPerSecond = 15000;
  static const size_t kMaxRPM = 12000;
  static const size_t kPolygonFacet = 5;
  static const size_t kMaxScanLinePerFrame = kMaxRPM * kPolygonFacet / 60 / 10;

  explicit RingIdConverter(InnoLidarClient *lidar);
  ~RingIdConverter();
  /**
   * @brief Update ring id table
   * @return Return 0 for success, others for error
   */
  int update_ring_id_table();

  /**
   * @brief Check if init success
   * @return Return true for success, false for failed
   */
  bool valid() {
    return init_success_;
  }

  /**
   * @brief Get ring id
   * @param mode            InnoLidarMode
   * @param scan_direction  Galvo scan direction
   * @param scan_id         Scan line id
   * @param ch              Channel id
   * @return Return ring id
   */
  inline uint16_t get_ring_id(InnoLidarMode mode, uint32_t scan_direction, uint32_t scan_id, uint32_t ch) override {
    switch (mode) {
      case INNO_LIDAR_MODE_WORK_QUIET:
      case INNO_LIDAR_MODE_WORK_EXHIBITION:
        return ring_id_table_[0][scan_direction][scan_id][ch];
      case INNO_LIDAR_MODE_WORK_NORMAL:
        return ring_id_table_[1][scan_direction][scan_id][ch];
      default:
        return 0;
    }
  }

 private:
  /**
   * @brief Setup NORMAL mode ring id table
   * @return Return 0 for success, others for error
   */
  int setup_ring_id_table_normal_();

  /**
   * @brief Setup QUIET mode ring id table
   * @return Return 0 for success, others for error
   */
  int setup_ring_id_table_quiet_();

  /**
   * @brief Get vertical angle of specific scanline
   * @param scanline        scanline id
   * @param ch              channal id
   * @param first_roi_line  roi first line
   * @return Return angle
   */
  double get_v_angle_of_scanline_(uint32_t scanline, uint32_t ch, uint32_t first_roi_line) const;

  /**
   * @brief Fill ring id table
   * @param first_roi_line_up   First roi line when scan from top to bottom
   * @param first_roi_line_down First roi line when scan from bottom to top
   * @param line_num_per_frame  Line number per frame
   * @param table Ring id table to be filled
   */
  void fill_table_(uint16_t first_roi_line_up, uint16_t first_roi_line_down, uint16_t line_num_per_frame,
                   uint16_t table[INNO_FRAME_DIRECTION_MAX][kMaxScanLinePerFrame][kInnoChannelNumber]);
  /**
   * @brief Fill map from buffer
   * @param buffer  Buffer store motor config
   * @param map_out Address of map to be filled
   * @return Return 0 for success, others for error
   */
  int get_config_map_(const char *buffer, std::unordered_map<std::string, int32_t> *map_out);

 private:
  InnoLidarClient *lidar_;
  uint8_t roi_num_{2};  // get from galvo scan pattern
  uint8_t frame_rate_{0};
  double single_line_scan_time_s_{};  // get from polygon rpm
  uint16_t line_num_per_frame_{};     // get from polygon rpm and frame rate
  uint16_t galvo_periods_[4];

  uint16_t current_first_roi_line_up_{0};
  uint16_t current_first_roi_line_down_{0};

  bool init_success_;

  // 2 for up and down scan direction
  uint16_t ring_id_table_[2][INNO_FRAME_DIRECTION_MAX][kMaxScanLinePerFrame][kInnoChannelNumber];
};
}  // namespace innovusion
#endif  // SDK_CLIENT_RING_ID_CONVERTER_H_
