#include "local_record/local_record.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>

namespace local_record {

LocalRecordNode::LocalRecordNode() : Node("local_record") {
    InitParams();
    // 源文件存在清除原有文件
    std::ifstream file(save_path_);
    if (file.is_open()) {
        file.close();
        std::remove(save_path_.c_str());
    }
    // 打开文件流
    save_file_.open(save_path_,
                    std::ios::out | std::ios::app);
    if (!save_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open or create file: %s",
                     save_path_.c_str());
        // 处理文件未打开的情况，例如记录错误或尝试重新打开文件
        return;
    }
    int time_interval = static_cast<int>(1000 / save_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(time_interval),
        std::bind(&LocalRecordNode::TimerCallback, this));
    this->sub_localization_info_ =
        this->create_subscription<
            bot_msg::msg::LocalizationInfo>(
            topic_name_, 10,
            std::bind(
                &LocalRecordNode::LocalizationCallback,
                this, std::placeholders::_1));
}

LocalRecordNode::~LocalRecordNode() {
    // 程序结束时关闭文件流
    save_file_.close();
    RCLCPP_INFO(this->get_logger(),
                "local_record node stopped");
}

void LocalRecordNode::TimerCallback() {
    static bool log_header_flag = true;
    if (localization_info_msg_ == nullptr ||
        localization_info_count_ >= 20) {
        RCLCPP_INFO(this->get_logger(),
                    "LocalizationInfo not received, "
                    "localization_info_count_:%d",
                    localization_info_count_);
        return;
    }
    static bot_msg::msg::LocalizationInfo pre_local =
        *localization_info_msg_;

    // 确保文件流已打开
    if (!save_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "File not open");
        // 处理文件未打开的情况，例如记录错误或尝试重新打开文件
        return;
    }

    double posi_distance = sqrt(
        pow(localization_info_msg_->north - pre_local.north, 2) +
        pow(localization_info_msg_->east - pre_local.east, 2));
    RCLCPP_INFO(this->get_logger(), "posi_distance: %f",
                posi_distance);
    if (posi_distance < 0.10)  // 位置变化小于 10cm 则不记录
        return;
    // 增加header信息
    if (log_header_flag) {
        save_file_
            << "longtitude,latitude,altitude,north,east,up,"
               "yaw,pitch,roll,vel_speed,vel_north,vel_"
               "east,vel_up,acc_x,acc_y,acc_z,gyro_x,gyro_"
               "y,gyro_z,rtk_status"
            << std::endl;
        log_header_flag = false;
    }
    // 保存 LocalizationInfo 消息中的数据
    save_file_ << localization_info_msg_->longtitude << ","
               << localization_info_msg_->latitude << ","
               << localization_info_msg_->altitude << ","
               << localization_info_msg_->north << ","
               << localization_info_msg_->east << ","
               << localization_info_msg_->up << ","
               << localization_info_msg_->yaw << ","
               << localization_info_msg_->pitch << ","
               << localization_info_msg_->roll << ","
               << localization_info_msg_->vel_speed << ","
               << localization_info_msg_->vel_north << ","
               << localization_info_msg_->vel_east << ","
               << localization_info_msg_->vel_up << ","
               << localization_info_msg_->acc_x << ","
               << localization_info_msg_->acc_y << ","
               << localization_info_msg_->acc_z << ","
               << localization_info_msg_->gyro_x << ","
               << localization_info_msg_->gyro_y << ","
               << localization_info_msg_->gyro_z << ","
               << static_cast<int>(
                      localization_info_msg_
                          ->rtk_status)  // uint8 转换为 int
               << std::endl;

    localization_info_count_++;
    pre_local = *localization_info_msg_;
}

void LocalRecordNode::LocalizationCallback(
    const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    localization_info_msg_ = msg;
    localization_info_count_ = 0;
    RCLCPP_INFO(this->get_logger(),
                "LocalizationInfo received");
}

void LocalRecordNode::InitParams() {
    this->declare_parameter(
        "save_path",
        "~/auto_clean_bot/path/local_record.csv");
    this->declare_parameter("save_rate", 100.0);
    this->declare_parameter("topic_name",
                            "/localization_info");

    save_path_ =
        this->get_parameter("save_path").as_string();
    save_rate_ =
        this->get_parameter("save_rate").as_double();
    topic_name_ =
        this->get_parameter("topic_name").as_string();

    RCLCPP_INFO(this->get_logger(), "save_path: %s",
                save_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "save_rate: %f",
                save_rate_);
    RCLCPP_INFO(this->get_logger(), "topic_name: %s",
                topic_name_.c_str());
}

}  // namespace local_record

// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<local_record::LocalRecordNode>();
    RCLCPP_INFO(node->get_logger(),
                "local_record node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}