#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>


namespace local_record {
class LocalRecordNode : public rclcpp::Node {
public:
    LocalRecordNode();
    ~LocalRecordNode();
private:
    void TimerCallback();
    void LocalizationCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
    void InitParams();

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    // 订阅定位信息
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_;
    // 定位信息
    bot_msg::msg::LocalizationInfo::SharedPtr localization_info_msg_;
    // 保存路径
    std::string save_path_;
    // 保存文件
    std::ofstream save_file_;
    // 保存频率
    double save_rate_;
    // 定位信息的话题名
    std::string topic_name_;


    ssize_t localization_info_count_;
};
}