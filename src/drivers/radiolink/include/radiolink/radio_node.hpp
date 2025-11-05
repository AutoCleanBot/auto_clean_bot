#pragma once

#include <sys/types.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "bot_msg/msg/radio_link.hpp"
#include "rclcpp/rclcpp.hpp"

namespace radio {

constexpr std::size_t kSbusFrameLength = 35;
constexpr std::size_t kSbusChannelCount = 16;
constexpr uint8_t kSbusFrameHeader = 0x0F;

inline float FloatLimit(float value) { return std::fabs(value) < 0.02f ? 0.0f : value; }

union SbusFlags {
    uint8_t raw;
    struct {
        uint8_t reserved : 4;
        uint8_t failsafe : 1;
        uint8_t frame_lost : 1;
        uint8_t ch18 : 1;
        uint8_t ch17 : 1;
    } bits;
};

struct SbusPacket {
    std::array<uint8_t, kSbusFrameLength> data{};
    std::size_t index{0};
    bool collecting{false};
};

struct ChannelMapping {
    int linear_channel{2};                // 线速度通道索引+油门-为刹车 左杆
    int steering_channel{3};              // 转向角通道索引 右杆
    int gear_channel{6};                  // 档位通道索引 E
    int mode_channel{5};                  // 控制模式通道索引 G
    int stop_channel{4};                  // 停止通道索引 F
    double linear_scale{1.0};             // 线速度缩放系数
    double steering_scale{1.0};           // 转向角缩放系数
    double linear_offset{0.0};            // 线速度偏移
    double steering_offset{0.0};          // 转向角偏移
    double gear_forward_threshold{0.2};   // 档位前进阈值
    double gear_reverse_threshold{-0.2};  // 档位倒退阈值
    double mode_switch_threshold{0.4};    // 控制模式阈值
};

class RadioNode : public rclcpp::Node {
public:
    RadioNode();
    ~RadioNode() override;

private:
    void InitParams();
    void InitValues();
    bool DeviceInit();
    void InfoReadLoop();
    void RadioTimerCallback();
    void ParseByte(uint8_t byte);
    void DecodeFrame(const std::array<uint8_t, kSbusFrameLength>& frame);
    void UpdateMessageFromChannels();
    void InitInfoStrSaving();
    void SaveInfoStrToFile(const std::string& info_str) const;
    std::string GenerateTimestampFilename() const;
    void LogChannelsIfNeeded() const;
    bool IsChannelIndexValid(int index) const;

    rclcpp::Publisher<bot_msg::msg::RadioLink>::SharedPtr pub_radio_info_;
    rclcpp::TimerBase::SharedPtr radio_timer_;

    bot_msg::msg::RadioLink radio_msg_;

    std::atomic<bool> running_{false};
    std::thread read_thread_;

    std::string device_name_;
    int baud_rate_{115200};
    int timeout_ms_{20};
    int sockfd_{-1};

    std::string radio_frame_id_;
    std::string radio_topic_name_;
    double radio_publish_rate_{50.0};

    ChannelMapping channel_mapping_;

    bool enable_debug_log_{false};
    int log_interval_{25};
    mutable int parse_count_{0};
    mutable int error_count_{0};
    mutable std::chrono::steady_clock::time_point last_log_time_;

    bool enable_info_str_save_{false};
    std::string info_str_save_dir_;
    mutable int info_str_save_count_{0};
    mutable std::string current_log_filename_;
    mutable std::ofstream info_str_file_;

    SbusPacket packet_;
    std::array<float, kSbusChannelCount> channels_{};
    SbusFlags flags_{};
    std::mutex channels_mutex_;

    // 每个通道的原始值范围配置（用于映射到[-1.0, 1.0]）
    struct ChannelRange {
        uint16_t min_raw{0};        // 通道原始值最小值，映射到-1.0
        uint16_t center_raw{1024};  // 通道原始值中值（中心点），映射到0.0
        uint16_t max_raw{2047};     // 通道原始值最大值，映射到1.0
    };
    std::array<ChannelRange, kSbusChannelCount> channel_ranges_{};

    ssize_t link_count_{};
};

}  // namespace radio
