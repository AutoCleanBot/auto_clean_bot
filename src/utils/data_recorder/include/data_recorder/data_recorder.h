#pragma once

#include <bot_msg/msg/chassis_info.hpp>
#include <bot_msg/msg/localization_info.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

namespace data_recorder {
class DataRecorderNode : public rclcpp::Node {
public:
    DataRecorderNode();
    ~DataRecorderNode();

private:
    void TimerCallback();
    void ChassisInfoCallback(const bot_msg::msg::ChassisInfo::SharedPtr msg);
    void LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
    void InitParams();
    void InitCsvFile();
    void WriteCsvRecord();
    std::string GenerateTimestampFilename();

    // ROS2 订阅者
    rclcpp::Subscription<bot_msg::msg::ChassisInfo>::SharedPtr sub_chassis_info_;
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 消息数据
    bot_msg::msg::ChassisInfo::SharedPtr chassis_info_msg_;
    bot_msg::msg::LocalizationInfo::SharedPtr localization_info_msg_;

    // 参数
    std::string chassis_info_topic_;
    std::string localization_info_topic_;
    std::string save_directory_;
    double record_rate_;
    bool enable_record_;

    // 文件处理
    std::ofstream csv_file_;
    std::string current_filename_;
    bool header_written_;

    // 数据计数
    size_t record_count_;
    std::chrono::steady_clock::time_point last_record_time_;
};
}  // namespace data_recorder