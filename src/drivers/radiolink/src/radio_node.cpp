#include "radiolink/radio_node.hpp"

#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <thread>

#include "gps_serial.h"

namespace radio {

/**
 * @brief 计算校验码（不包含帧头的33字节异或校验）
 *
 * @param frame 完整的35字节帧
 * @return uint8_t 计算得到的校验码
 */
uint8_t CalculateChecksum(const std::array<uint8_t, kSbusFrameLength>& frame) {
    uint8_t checksum = 0;
    // 从索引1开始（跳过帧头），到索引33（包含flag字节）
    for (std::size_t i = 1; i <= 33; ++i) {
        checksum ^= frame[i];
    }
    return checksum;
}

RadioNode::RadioNode() : Node("radio_node") {
    InitValues();
    InitParams();

    pub_radio_info_ = this->create_publisher<bot_msg::msg::RadioLink>(radio_topic_name_, 10);

    auto publish_period = std::chrono::duration<double>(1.0 / radio_publish_rate_);
    radio_timer_ =
        this->create_wall_timer(publish_period, std::bind(&RadioNode::RadioTimerCallback, this));

    if (DeviceInit()) {
        running_.store(true);
        read_thread_ = std::thread(&RadioNode::InfoReadLoop, this);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Serial device initialization failed: %s",
                     device_name_.c_str());
    }
}

RadioNode::~RadioNode() {
    running_.store(false);

    if (sockfd_ >= 0) {
        Serial::TTYClose(sockfd_);
        sockfd_ = -1;
    }

    if (read_thread_.joinable()) {
        read_thread_.join();
    }

    if (info_str_file_.is_open()) {
        info_str_file_.close();
    }
}

void RadioNode::InitValues() {
    packet_ = {};
    channels_.fill(0.0f);
    flags_.raw = 0U;
    radio_msg_ = bot_msg::msg::RadioLink{};
    radio_msg_.linear_pct = 0.0F;
    radio_msg_.brake_pct = 0.0F;
    radio_msg_.steering_pct = 0.0F;

    radio_msg_.gear = 0;
    radio_msg_.control_mode = 0;
    radio_msg_.stop_mode = 0;

    last_log_time_ = std::chrono::steady_clock::now();
}

void RadioNode::InitParams() {
    this->declare_parameter<std::string>("device_name", "/dev/ttyUART_232_B");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("timeout_ms", 20);

    this->declare_parameter<std::string>("radio_frame_id", "radio_link");
    this->declare_parameter<std::string>("radio_topic_name", "radio/radio_info");
    this->declare_parameter<double>("radio_publish_rate", 50.0);

    this->declare_parameter<int>("linear_channel", channel_mapping_.linear_channel);
    this->declare_parameter<int>("steering_channel", channel_mapping_.steering_channel);
    this->declare_parameter<int>("gear_channel", channel_mapping_.gear_channel);
    this->declare_parameter<int>("mode_channel", channel_mapping_.mode_channel);
    this->declare_parameter<int>("stop_channel", channel_mapping_.stop_channel);
    this->declare_parameter<double>("linear_scale", channel_mapping_.linear_scale);
    this->declare_parameter<double>("linear_offset", channel_mapping_.linear_offset);
    this->declare_parameter<double>("steering_scale", channel_mapping_.steering_scale);
    this->declare_parameter<double>("steering_offset", channel_mapping_.steering_offset);
    this->declare_parameter<double>("gear_forward_threshold",
                                    channel_mapping_.gear_forward_threshold);
    this->declare_parameter<double>("gear_reverse_threshold",
                                    channel_mapping_.gear_reverse_threshold);
    this->declare_parameter<double>("mode_switch_threshold",
                                    channel_mapping_.mode_switch_threshold);

    this->declare_parameter<bool>("enable_debug_log", enable_debug_log_);
    this->declare_parameter<int>("log_interval", log_interval_);
    this->declare_parameter<bool>("enable_info_str_save", false);
    this->declare_parameter<std::string>("info_str_save_dir", std::string("/tmp/radio_logs"));

    // 为每个通道配置原始值范围（用于映射到[-1.0, 1.0]）
    // 包括最小值、中值（中心点）、最大值
    for (int ch = 0; ch < 16; ++ch) {
        std::string param_min = "channel_" + std::to_string(ch) + "_min_raw";
        std::string param_center = "channel_" + std::to_string(ch) + "_center_raw";
        std::string param_max = "channel_" + std::to_string(ch) + "_max_raw";
        this->declare_parameter<int>(param_min, 0);
        this->declare_parameter<int>(param_center, 1024);
        this->declare_parameter<int>(param_max, 2047);
    }

    device_name_ = this->get_parameter("device_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    timeout_ms_ = this->get_parameter("timeout_ms").as_int();

    radio_frame_id_ = this->get_parameter("radio_frame_id").as_string();
    radio_topic_name_ = this->get_parameter("radio_topic_name").as_string();
    radio_publish_rate_ = this->get_parameter("radio_publish_rate").as_double();
    if (radio_publish_rate_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "radio_publish_rate must be positive, resetting to 50.0 Hz");
        radio_publish_rate_ = 50.0;
    }

    channel_mapping_.linear_channel = this->get_parameter("linear_channel").as_int();
    channel_mapping_.steering_channel = this->get_parameter("steering_channel").as_int();
    channel_mapping_.gear_channel = this->get_parameter("gear_channel").as_int();
    channel_mapping_.mode_channel = this->get_parameter("mode_channel").as_int();
    channel_mapping_.stop_channel = this->get_parameter("stop_channel").as_int();
    channel_mapping_.linear_scale = this->get_parameter("linear_scale").as_double();
    channel_mapping_.linear_offset = this->get_parameter("linear_offset").as_double();
    channel_mapping_.steering_scale = this->get_parameter("steering_scale").as_double();
    channel_mapping_.steering_offset = this->get_parameter("steering_offset").as_double();
    channel_mapping_.gear_forward_threshold =
        this->get_parameter("gear_forward_threshold").as_double();
    channel_mapping_.gear_reverse_threshold =
        this->get_parameter("gear_reverse_threshold").as_double();
    channel_mapping_.mode_switch_threshold =
        this->get_parameter("mode_switch_threshold").as_double();

    auto validate_channel = [&](const char* param_name, int& value) {
        if (!IsChannelIndexValid(value)) {
            RCLCPP_WARN(this->get_logger(),
                        "%s is out of range (%d). Valid channels: [0, %zu). Clamping.", param_name,
                        value, kSbusChannelCount);
            value = std::min(std::max(value, 0), static_cast<int>(kSbusChannelCount) - 1);
        }
    };

    validate_channel("linear_channel", channel_mapping_.linear_channel);
    validate_channel("steering_channel", channel_mapping_.steering_channel);
    validate_channel("gear_channel", channel_mapping_.gear_channel);
    validate_channel("mode_channel", channel_mapping_.mode_channel);
    validate_channel("stop_channel", channel_mapping_.stop_channel);

    enable_debug_log_ = this->get_parameter("enable_debug_log").as_bool();
    log_interval_ = this->get_parameter("log_interval").as_int();
    if (log_interval_ <= 0) {
        log_interval_ = 25;
    }

    enable_info_str_save_ = this->get_parameter("enable_info_str_save").as_bool();
    info_str_save_dir_ = this->get_parameter("info_str_save_dir").as_string();

    // 读取每个通道的原始值范围配置（包括最小值、中值、最大值）
    for (int ch = 0; ch < 16; ++ch) {
        std::string param_min = "channel_" + std::to_string(ch) + "_min_raw";
        std::string param_center = "channel_" + std::to_string(ch) + "_center_raw";
        std::string param_max = "channel_" + std::to_string(ch) + "_max_raw";

        int min_raw = this->get_parameter(param_min).as_int();
        int center_raw = this->get_parameter(param_center).as_int();
        int max_raw = this->get_parameter(param_max).as_int();
        // 验证范围
        if (min_raw < 0)
            min_raw = 0;
        if (max_raw > 2047)
            max_raw = 2047;
        if (center_raw < 0)
            center_raw = 0;
        if (center_raw > 2047)
            center_raw = 2047;
        // 验证逻辑：min_raw <= center_raw <= max_raw
        if (min_raw >= center_raw) {
            RCLCPP_WARN(this->get_logger(),
                        "Channel %d: min_raw (%d) >= center_raw (%d), adjusting center_raw", ch,
                        min_raw, center_raw);
            center_raw = min_raw + 1;
            if (center_raw > 2047)
                center_raw = 2047;
        }

        if (center_raw >= max_raw) {
            RCLCPP_WARN(this->get_logger(),
                        "Channel %d: center_raw (%d) >= max_raw (%d), adjusting center_raw", ch,
                        center_raw, max_raw);
            center_raw = max_raw - 1;
            if (center_raw < 0)
                center_raw = 0;
        }

        if (min_raw >= max_raw) {
            RCLCPP_WARN(this->get_logger(),
                        "Channel %d: min_raw (%d) >= max_raw (%d), using defaults (0, 1024, 2047)",
                        ch, min_raw, max_raw);
            min_raw = 0;
            center_raw = 1024;
            max_raw = 2047;
        }

        channel_ranges_[ch].min_raw = static_cast<uint16_t>(min_raw);
        channel_ranges_[ch].center_raw = static_cast<uint16_t>(center_raw);
        channel_ranges_[ch].max_raw = static_cast<uint16_t>(max_raw);

        RCLCPP_INFO(this->get_logger(), "Channel %d: min_raw=%d, center_raw=%d, max_raw=%d", ch,
                    channel_ranges_[ch].min_raw, channel_ranges_[ch].center_raw,
                    channel_ranges_[ch].max_raw);
    }

    RCLCPP_INFO(this->get_logger(), "device: %s", device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "baud_rate: %d", baud_rate_);
    RCLCPP_INFO(this->get_logger(), "topic: %s", radio_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_rate: %.2f", radio_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "linear_channel: %d", channel_mapping_.linear_channel);
    RCLCPP_INFO(this->get_logger(), "steering_channel: %d", channel_mapping_.steering_channel);
    RCLCPP_INFO(this->get_logger(), "gear_channel: %d", channel_mapping_.gear_channel);
    RCLCPP_INFO(this->get_logger(), "mode_channel: %d", channel_mapping_.mode_channel);
    RCLCPP_INFO(this->get_logger(), "stop_channel: %d", channel_mapping_.stop_channel);
}

bool RadioNode::DeviceInit() {
    sockfd_ = Serial::TTYOpen(device_name_.c_str());
    if (sockfd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s", device_name_.c_str());
        return false;
    }

    if (Serial::TTYSetOpt(sockfd_, baud_rate_, 8, 1, 'n') != 0) {
        RCLCPP_ERROR(this->get_logger(), "tty_setopt error on %s", device_name_.c_str());
        Serial::TTYClose(sockfd_);
        sockfd_ = -1;
        return false;
    }

    return true;
}

void RadioNode::InfoReadLoop() {
    std::array<uint8_t, 64> buffer{};
    auto sleep_duration = std::chrono::milliseconds(std::max(timeout_ms_, 1));

    while (running_.load() && rclcpp::ok()) {
        // RCLCPP_INFO(this->get_logger(), "enter the loop");
        if (sockfd_ < 0) {
            RCLCPP_INFO(this->get_logger(), "fail to get message!!");
            break;
        }
        int ret = ::read(sockfd_, buffer.data(), buffer.size());
        if (ret == 0 || ret < 0) {
            ++link_count_;
        } else {
            link_count_ = 0;
        }

        // // 无论 ret 是什么值，我们都打印出来
        // RCLCPP_INFO(this->get_logger(), "read() returned: %d", ret);
        if (enable_debug_log_) {
            std::stringstream ss;
            ss << "Read " << ret << " bytes:";
            for (int i = 0; i < ret; ++i) {
                ss << " " << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                   << static_cast<int>(buffer[i]);
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }

        if (ret > 0) {
            for (int i = 0; i < ret; ++i) {
                ParseByte(buffer[static_cast<std::size_t>(i)]);
            }
        } else if (ret == 0) {
            std::this_thread::sleep_for(sleep_duration);
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(sleep_duration);
                continue;
            }

            ++error_count_;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_);
            if (enable_debug_log_ && (error_count_ % log_interval_ == 0 || elapsed.count() >= 5)) {
                RCLCPP_WARN(this->get_logger(), "Serial read error (%d): %s", error_count_,
                            std::strerror(errno));
                last_log_time_ = now;
            }

            std::this_thread::sleep_for(sleep_duration);
        }
    }
}
/*
void RadioNode::ParseByte(uint8_t byte) {
    if (!packet_.collecting) {
        if (byte == kSbusFrameHeader) {
            packet_.collecting = true;
            packet_.index = 0;
            packet_.data[packet_.index++] = byte;
        }
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "attemp to parse data.");

    if (byte == kSbusFrameHeader && packet_.index == 1) {
        packet_.index = 1;
        packet_.data[0] = byte;
        return;
    }

    if (packet_.index >= kSbusFrameLength) {
        packet_.collecting = false;
        packet_.index = 0;
        return;
    }

    packet_.data[packet_.index++] = byte;

    // [新增日志] 打印正在收集的数据帧进度
    // RCLCPP_INFO(this->get_logger(), "Collecting frame, index: %zu, byte: %02X", packet_.index,
byte);

    if (packet_.index == kSbusFrameLength) {
        // [新增日志] 确认收集完成，即将解码
        // RCLCPP_INFO(this->get_logger(), "Full frame collected. Calling DecodeFrame...");
        packet_.collecting = false;
        packet_.index = 0;

        // std::stringstream ss;
        // for (int i = 0; i < 25; ++i) {
        //     ss << " " << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        //         << static_cast<int>(packet_.data[i]);
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        DecodeFrame(packet_.data);
    }
}
*/

void RadioNode::ParseByte(uint8_t byte) {
    if (!packet_.collecting) {
        if (byte == kSbusFrameHeader) {
            packet_.collecting = true;
            packet_.index = 0;
            packet_.data[packet_.index++] = byte;
        }
        return;
    }

    if (byte == kSbusFrameHeader && packet_.index == 1) {
        packet_.index = 1;
        packet_.data[0] = byte;
        return;
    }

    if (packet_.index >= kSbusFrameLength) {
        packet_.collecting = false;
        packet_.index = 0;
        return;
    }

    packet_.data[packet_.index++] = byte;

    // // 打印当前收集的字节信息
    // RCLCPP_INFO(this->get_logger(), "Received byte: 0x%02X at position: %zu", byte,
    // packet_.index);

    if (packet_.index == kSbusFrameLength) {
        packet_.collecting = false;
        packet_.index = 0;

        // // 打印完整的11位数据帧
        // RCLCPP_INFO(this->get_logger(), "=== Complete 11-bit SBUS Frame Data ===");
        // for (int i = 0; i < kSbusFrameLength; ++i) {
        //     RCLCPP_INFO(this->get_logger(), "Data[%d]: 0x%02X", i, packet_.data[i]);
        // }
        // RCLCPP_INFO(this->get_logger(), "=== End of SBUS Frame ===");

        DecodeFrame(packet_.data);
    }
}

void RadioNode::DecodeFrame(const std::array<uint8_t, kSbusFrameLength>& frame) {
    // 帧头校验
    if (frame[0] != kSbusFrameHeader) {
        if (enable_debug_log_) {
            RCLCPP_WARN(this->get_logger(), "Invalid frame header: 0x%02X, expected 0x%02X",
                        frame[0], kSbusFrameHeader);
        }
        return;
    }

    // 校验码验证（不包含帧头的33字节异或）
    uint8_t calculated_checksum = CalculateChecksum(frame);
    uint8_t received_checksum = frame[34];

    if (calculated_checksum != received_checksum) {
        if (enable_debug_log_) {
            RCLCPP_WARN(this->get_logger(), "Checksum mismatch: calculated=0x%02X, received=0x%02X",
                        calculated_checksum, received_checksum);
        }
        ++error_count_;
        return;
    }

    // // 打印完整的35字节232数据
    // std::stringstream data_ss;
    // data_ss << "232 Data (35 bytes): ";
    // for (std::size_t i = 0; i < frame.size(); ++i) {
    //     data_ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
    //             << static_cast<int>(frame[i]);
    //     if (i + 1 != frame.size()) {
    //         data_ss << " ";
    //     }
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", data_ss.str().c_str());

    std::lock_guard<std::mutex> lock(channels_mutex_);

    /**
     * @brief 解码通道值
     *
     * @param channel_index 通道索引 (0-15)
     * @param raw_value 原始16位整数值
     *
     * 新格式：每个通道用2字节表示，高字节在前，低字节在后
     * 根据每个通道配置的min_raw、center_raw、max_raw，将原始值映射到[-1.0, 1.0]范围
     *
     * 映射规则：
     * - min_raw -> -1.0
     * - center_raw -> 0.0（中值点，可配置，不一定在中间）
     * - max_raw -> 1.0
     *
     * 分段线性映射：
     * - 当 raw_value < center_raw: 映射到 [-1.0, 0.0]
     * - 当 raw_value >= center_raw: 映射到 [0.0, 1.0]
     */
    auto decode_channel = [&](int channel_index, uint16_t raw_value, uint16_t& out_clamped_value,
                              float& out_mapped_value) {
        if (channel_index >= 0 && channel_index < static_cast<int>(kSbusChannelCount)) {
            const auto& range = channel_ranges_[channel_index];

            // 限制原始值在配置的范围内
            raw_value = std::max(std::min(raw_value, range.max_raw), range.min_raw);
            out_clamped_value = raw_value;

            float value = 0.0f;

            if (raw_value < range.center_raw) {
                // 映射到 [-1.0, 0.0] 范围
                // raw_value在[min_raw, center_raw)之间
                float lower_range = static_cast<float>(range.center_raw - range.min_raw);
                if (lower_range > 0.0f) {
                    // 线性映射：min_raw -> -1.0, center_raw -> 0.0
                    // normalized从0变化到1，value从-1.0变化到0.0
                    float normalized = static_cast<float>(raw_value - range.min_raw) / lower_range;
                    value = normalized - 1.0f;  // value = -1.0 + normalized * 1.0
                } else {
                    value = -1.0f;  // 如果lower_range无效，设为-1.0
                }
            } else {
                // 映射到 [0.0, 1.0] 范围
                // raw_value在[center_raw, max_raw]之间
                float upper_range = static_cast<float>(range.max_raw - range.center_raw);
                if (upper_range > 0.0f) {
                    // 线性映射：center_raw -> 0.0, max_raw -> 1.0
                    // normalized从0变化到1，value从0.0变化到1.0
                    float normalized =
                        static_cast<float>(raw_value - range.center_raw) / upper_range;
                    value = normalized * 1.0f;  // value = 0.0 + normalized * 1.0
                } else {
                    value = 1.0f;  // 如果upper_range无效，设为1.0
                }
            }

            out_mapped_value = value;
            channels_[static_cast<std::size_t>(channel_index)] = FloatLimit(value);
        } else {
            out_clamped_value = raw_value;
            out_mapped_value = 0.0f;
        }
    };

    // 解析16个通道，每个通道2字节（高字节在前，低字节在后）
    // 通道数据从frame[1]到frame[32]，共32字节
    std::array<uint16_t, 16> raw_values{};
    std::array<uint16_t, 16> clamped_values{};
    std::array<float, 16> mapped_values{};

    for (int ch = 0; ch < 16; ++ch) {
        // 每个通道占2字节，高字节在前
        std::size_t byte_offset = 1 + ch * 2;
        uint16_t raw_value = (static_cast<uint16_t>(frame[byte_offset]) << 8) |
                             static_cast<uint16_t>(frame[byte_offset + 1]);
        raw_values[ch] = raw_value;

        decode_channel(ch, raw_value, clamped_values[ch], mapped_values[ch]);
    }

    // // 打印所有16个通道的原始十进制值
    // std::ostringstream raw_oss;
    // raw_oss << "所有通道原始十进制值: ";
    // for (int ch = 0; ch < 16; ++ch) {
    //     raw_oss << "Ch" << ch << "=" << raw_values[ch];
    //     if (ch < 15) raw_oss << ", ";
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", raw_oss.str().c_str());

    // // 打印所有16个通道的映射前值（限制后）
    // std::ostringstream clamped_oss;
    // clamped_oss << "所有通道映射前值(限制后): ";
    // for (int ch = 0; ch < 16; ++ch) {
    //     clamped_oss << "Ch" << ch << "=" << clamped_values[ch];
    //     if (ch < 15) clamped_oss << ", ";
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", clamped_oss.str().c_str());

    // // 打印所有16个通道的映射后值（[-1,1]区间）
    // std::ostringstream mapped_oss;
    // mapped_oss << std::fixed << std::setprecision(6);
    // mapped_oss << "所有通道映射后值([-1,1]区间): ";
    // for (int ch = 0; ch < 16; ++ch) {
    //     mapped_oss << "Ch" << ch << "=" << mapped_values[ch];
    //     if (ch < 15) mapped_oss << ", ";
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", mapped_oss.str().c_str());

    // flag字节位于frame[33]
    flags_.raw = frame[33];

    // 检查失效保护标志
    if (flags_.bits.failsafe) {
        if (enable_debug_log_) {
            RCLCPP_WARN(this->get_logger(), "SBUS failsafe activated!");
        }
    }

    if (flags_.bits.frame_lost) {
        if (enable_debug_log_) {
            RCLCPP_WARN(this->get_logger(), "SBUS frame lost!");
        }
    }

    ++parse_count_;
    UpdateMessageFromChannels();
    LogChannelsIfNeeded();

    if (enable_info_str_save_) {
        std::stringstream ss;
        for (std::size_t i = 0; i < frame.size(); ++i) {
            ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
               << static_cast<int>(frame[i]);
            if (i + 1 != frame.size()) {
                ss << ' ';
            }
        }
        SaveInfoStrToFile(ss.str());
    }
}

void RadioNode::UpdateMessageFromChannels() {
    // 处理线速度通道（通道2）：根据映射值设置linear_pct和brake_pct
    if (IsChannelIndexValid(channel_mapping_.linear_channel)) {
        double linear_value = channels_[static_cast<std::size_t>(channel_mapping_.linear_channel)];

        // 在(-0.1, 0.1)区间内不响应，保持为0
        if (linear_value > -0.1 && linear_value < 0.1) {
            radio_msg_.linear_pct = 0.0F;
            radio_msg_.brake_pct = 0.0F;
        } else if (linear_value <= -0.1) {
            // 大于等于0.1时，赋值给linear_pct，brake_pct设为0
            radio_msg_.linear_pct = static_cast<float>(std::fabs(linear_value));
            radio_msg_.brake_pct = 0.0F;
        } else if (linear_value >= 0.1) {
            // 小于等于-0.1时，取绝对值赋值给brake_pct，linear_pct设为0
            radio_msg_.brake_pct = static_cast<float>(linear_value);
            radio_msg_.linear_pct = 0.0F;
        }
    } else {
        radio_msg_.linear_pct = 0.0F;
        radio_msg_.brake_pct = 0.0F;
    }

    // 处理转向通道（通道3）：根据映射值设置steering_pct
    if (IsChannelIndexValid(channel_mapping_.steering_channel)) {
        double steering_value =
            channels_[static_cast<std::size_t>(channel_mapping_.steering_channel)];

        // 在(-0.1, 0.1)区间内不响应，保持为0
        if (steering_value > -0.1 && steering_value < 0.1) {
            radio_msg_.steering_pct = 0.0F;
        } else {
            // 其他时候，直接赋值（保留正负号）给steering_pct
            radio_msg_.steering_pct = static_cast<float>(steering_value);
        }
    } else {
        radio_msg_.steering_pct = 0.0F;
    }

    // 处理模式档位通道（通道6）：根据映射值设置gear
    if (IsChannelIndexValid(channel_mapping_.gear_channel)) {
        double value = channels_[static_cast<std::size_t>(channel_mapping_.gear_channel)];
        if (value > channel_mapping_.gear_forward_threshold) {
            radio_msg_.gear = 1;
        } else if (value < channel_mapping_.gear_reverse_threshold) {
            radio_msg_.gear = -1;
        } else {
            radio_msg_.gear = 0;
        }
    } else {
        radio_msg_.gear = 0;
    }

    // 处理模式控制通道（通道5）：根据映射值设置control_mode
    if (IsChannelIndexValid(channel_mapping_.mode_channel)) {
        double mode_value = channels_[static_cast<std::size_t>(channel_mapping_.mode_channel)];

        // 在(-0.1, 0.1)区间内，control_mode为0
        if (mode_value > -0.1 && mode_value < 0.1) {
            radio_msg_.control_mode = 0;
        } else if (mode_value >= 0.1) {
            // 大于等于0.1时，control_mode为1
            radio_msg_.control_mode = 1;
        } else if (mode_value <= -0.1) {
            // 小于等于-0.1时，control_mode为-1
            radio_msg_.control_mode = -1;
        }
    } else {
        radio_msg_.control_mode = 0;
    }

    // 处理急停控制通道（通道4）：根据映射值设置stop_mode
    if (IsChannelIndexValid(channel_mapping_.stop_channel)) {
        double stop_value = channels_[static_cast<std::size_t>(channel_mapping_.stop_channel)];

        if (stop_value > 0.0) {
            // 大于0时，stop_mode为1
            radio_msg_.stop_mode = 1;
        } else if (stop_value < 0.0) {
            // 小于0时，stop_mode为-1
            radio_msg_.stop_mode = -1;
        } else {
            // 等于0时，stop_mode为0
            radio_msg_.stop_mode = 0;
        }
    } else {
        radio_msg_.stop_mode = 0;
    }
}

void RadioNode::RadioTimerCallback() {
    bot_msg::msg::RadioLink msg_copy{};

    {
        std::lock_guard<std::mutex> lock(channels_mutex_);
        radio_msg_.header.stamp = this->now();
        radio_msg_.header.frame_id = radio_frame_id_;
        if (link_count_ < 50)
            msg_copy = radio_msg_;
    }

    // 打印处理后的所有值，方便检查是否正确反映
    RCLCPP_INFO(this->get_logger(),
                "Radio Control Values - linear_pct: %.3f, brake_pct: %.3f, steering_pct: %.3f, "
                "gear: %d, control_mode: %d, stop_mode: %d",
                msg_copy.linear_pct, msg_copy.brake_pct, msg_copy.steering_pct, msg_copy.gear,
                msg_copy.control_mode, msg_copy.stop_mode);

    pub_radio_info_->publish(msg_copy);
}

void RadioNode::InitInfoStrSaving() {
    try {
        std::filesystem::create_directories(info_str_save_dir_);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                     info_str_save_dir_.c_str(), ex.what());
        enable_info_str_save_ = false;
    }
}

void RadioNode::SaveInfoStrToFile(const std::string& info_str) const {
    if (!enable_info_str_save_) {
        return;
    }

    if (!info_str_file_.is_open()) {
        current_log_filename_ = GenerateTimestampFilename();
        std::string full_path = info_str_save_dir_ + "/" + current_log_filename_;
        info_str_file_.open(full_path, std::ios::app);
        if (!info_str_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", full_path.c_str());
            return;
        }
    }

    info_str_file_ << info_str << std::endl;
    ++info_str_save_count_;
}

std::string RadioNode::GenerateTimestampFilename() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_local;
    localtime_r(&time_t, &tm_local);

    std::ostringstream oss;
    oss << "radio_" << std::put_time(&tm_local, "%Y%m%d_%H%M%S") << ".log";
    return oss.str();
}

void RadioNode::LogChannelsIfNeeded() const {
    if (!enable_debug_log_) {
        return;
    }

    if (parse_count_ % log_interval_ != 0) {
        return;
    }

    std::ostringstream oss;
    oss << "Channels:";
    for (std::size_t i = 0; i < 16 && i < channels_.size(); ++i) {
        oss << " ch" << i << '=' << std::fixed << std::setprecision(3) << channels_[i];
    }
    oss << " gear=" << static_cast<int>(radio_msg_.gear)
        << " mode=" << static_cast<int>(radio_msg_.control_mode);

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

bool RadioNode::IsChannelIndexValid(int index) const {
    return index >= 0 && index < static_cast<int>(kSbusChannelCount);
}

}  // namespace radio

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<radio::RadioNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
