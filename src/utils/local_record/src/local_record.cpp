#include "local_record/local_record.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <pwd.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace local_record {

// 添加一个辅助函数来展开波浪号
std::string expandTilde(const std::string &path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    // 获取当前用户的主目录
    const char *home = getenv("HOME");
    if (home == nullptr) {
        struct passwd *pwd = getpwuid(getuid());
        if (pwd) {
            home = pwd->pw_dir;
        }
    }

    if (home == nullptr) {
        return path; // 如果无法获取主目录，返回原始路径
    }

    // 替换波浪号
    if (path.length() == 1) { // 仅有 "~"
        return home;
    }
    if (path[1] == '/') { // "~/xxx"
        return std::string(home) + path.substr(1);
    }
    return path; // "~xxx" 其他情况返回原始路径
}

LocalRecordNode::LocalRecordNode() : Node("local_record") {
    InitParams();

    // 展开路径中的波浪号
    std::string expanded_path = expandTilde(save_path_);

    // 保存最近5次的记录
    const int max_files = 5;
    std::string base_path = expanded_path;
    std::string extension = ".csv";

    // 存储所有已存在的文件信息
    struct FileInfo {
        std::string path;
        time_t mtime; // 使用time_t替代filesystem的时间类型
        int index;
    };
    std::vector<FileInfo> existing_files;

    // 查找现有文件
    int max_index = 0;
    for (int i = 1; i <= max_files; i++) {
        std::string file_path = base_path + "_" + std::to_string(i) + extension;
        struct stat file_stat;
        if (stat(file_path.c_str(), &file_stat) == 0) { // 使用stat替代filesystem
            FileInfo info;
            info.path = file_path;
            info.mtime = file_stat.st_mtime;
            info.index = i;
            existing_files.push_back(info);
            max_index = std::max(max_index, i);
        }
    }

    // 如果已经有5个文件，删除最旧的文件
    if (existing_files.size() >= max_files) {
        // 按修改时间排序
        std::sort(existing_files.begin(), existing_files.end(),
                  [](const FileInfo &a, const FileInfo &b) { return a.mtime < b.mtime; });

        // 删除最旧的文件
        std::remove(existing_files[0].path.c_str());
        max_index = existing_files[0].index;
        existing_files.erase(existing_files.begin());
    }

    // 使用下一个可用的序号创建新文件
    new_file_path_ = base_path + "_" + std::to_string(max_index) + extension;
    RCLCPP_INFO(this->get_logger(), "Creating new file: %s", new_file_path_.c_str());

    // 打开文件流
    save_file_.open(new_file_path_, std::ios::out | std::ios::app);
    if (!save_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open or create file: %s", new_file_path_.c_str());
        return;
    }

    int time_interval = static_cast<int>(1000 / save_rate_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(time_interval),
                                     std::bind(&LocalRecordNode::TimerCallback, this));
    this->sub_localization_info_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
        topic_name_, 10, std::bind(&LocalRecordNode::LocalizationCallback, this, std::placeholders::_1));
}

LocalRecordNode::~LocalRecordNode() {
    // 程序结束时关闭文件流
    save_file_.close();
    RCLCPP_INFO(this->get_logger(), "local_record node stopped, %s closed.", new_file_path_.c_str());
}

void LocalRecordNode::TimerCallback() {
    static bool log_header_flag = true;
    if (localization_info_msg_ == nullptr || localization_info_count_ >= 20) {
        RCLCPP_INFO(this->get_logger(),
                    "LocalizationInfo not received, "
                    "localization_info_count_:%d",
                    localization_info_count_);
        return;
    }
    static bot_msg::msg::LocalizationInfo pre_local = *localization_info_msg_;

    // 确保文件流已打开
    if (!save_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "File not open");
        // 处理文件未打开的情况，例如记录错误或尝试重新打开文件
        return;
    }

    double posi_distance = sqrt(pow(localization_info_msg_->north - pre_local.north, 2) +
                                pow(localization_info_msg_->east - pre_local.east, 2));
    RCLCPP_INFO(this->get_logger(), "posi_distance: %f", posi_distance);
    if (posi_distance < 0.10) // 位置变化小于 10cm 则不记录
        return;
    // 增加header信息
    if (log_header_flag) {
        save_file_ << "longtitude,latitude,altitude,north,east,up,"
                      "yaw,pitch,roll,vel_speed,vel_north,vel_"
                      "east,vel_up,acc_x,acc_y,acc_z,gyro_x,gyro_"
                      "y,gyro_z,rtk_status"
                   << std::endl;
        log_header_flag = false;
    }

    // 保存 LocalizationInfo 消息中的数据
    save_file_ << std::fixed << std::setprecision(8) << localization_info_msg_->longtitude << ","
               << localization_info_msg_->latitude << "," << std::setprecision(3) // 其他数据使用3位小数
               << localization_info_msg_->altitude << "," << localization_info_msg_->north << ","
               << localization_info_msg_->east << "," << localization_info_msg_->up << ","
               << localization_info_msg_->yaw << "," << localization_info_msg_->pitch << ","
               << localization_info_msg_->roll << "," << localization_info_msg_->vel_speed << ","
               << localization_info_msg_->vel_north << "," << localization_info_msg_->vel_east << ","
               << localization_info_msg_->vel_up << "," << localization_info_msg_->acc_x << ","
               << localization_info_msg_->acc_y << "," << localization_info_msg_->acc_z << ","
               << localization_info_msg_->gyro_x << "," << localization_info_msg_->gyro_y << ","
               << localization_info_msg_->gyro_z << "," << static_cast<int>(localization_info_msg_->rtk_status)
               << std::endl;

    localization_info_count_++;
    pre_local = *localization_info_msg_;
}

void LocalRecordNode::LocalizationCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    localization_info_msg_ = msg;
    localization_info_count_ = 0;
    // RCLCPP_INFO(this->get_logger(),
    //             "LocalizationInfo received");
}

void LocalRecordNode::InitParams() {
    this->declare_parameter("save_path", "~/auto_clean_bot/path/local_record"); // 修改默认路径，移除末尾的/
    this->declare_parameter("save_rate", 100.0);
    this->declare_parameter("topic_name", "/localization_info");

    save_path_ = this->get_parameter("save_path").as_string();
    save_rate_ = this->get_parameter("save_rate").as_double();
    topic_name_ = this->get_parameter("topic_name").as_string();

    RCLCPP_INFO(this->get_logger(), "base save_path: %s", save_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "save_rate: %f", save_rate_);
    RCLCPP_INFO(this->get_logger(), "topic_name: %s", topic_name_.c_str());
}
} // namespace local_record

// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_record::LocalRecordNode>();
    RCLCPP_INFO(node->get_logger(), "local_record node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}