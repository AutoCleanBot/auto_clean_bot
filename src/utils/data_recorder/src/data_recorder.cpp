#include "data_recorder/data_recorder.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

namespace data_recorder {

DataRecorderNode::DataRecorderNode() : Node("data_recorder_node") {
    // 初始化变量
    chassis_info_msg_ = nullptr;
    localization_info_msg_ = nullptr;
    header_written_ = false;
    record_count_ = 0;
    last_record_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "DataRecorderNode 初始化中...");

    // 初始化参数
    InitParams();

    // 初始化CSV文件
    InitCsvFile();

    // 创建订阅者
    sub_chassis_info_ = this->create_subscription<bot_msg::msg::ChassisInfo>(
        chassis_info_topic_, 10,
        std::bind(&DataRecorderNode::ChassisInfoCallback, this, std::placeholders::_1));

    sub_localization_info_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
        localization_info_topic_, 10,
        std::bind(&DataRecorderNode::LocalizationInfoCallback, this, std::placeholders::_1));

    // 创建定时器
    int timer_interval = static_cast<int>(1000.0 / record_rate_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval),
                                     std::bind(&DataRecorderNode::TimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "DataRecorderNode 初始化完成");
    RCLCPP_INFO(this->get_logger(), "订阅话题: %s, %s", chassis_info_topic_.c_str(),
                localization_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "记录频率: %.1f Hz", record_rate_);
    RCLCPP_INFO(this->get_logger(), "保存目录: %s", save_directory_.c_str());
}

DataRecorderNode::~DataRecorderNode() {
    if (csv_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "关闭CSV文件，总记录数: %zu", record_count_);
        csv_file_.close();
    }
}

void DataRecorderNode::InitParams() {
    // 声明参数
    this->declare_parameter<std::string>("chassis_info_topic", "/chassis/chassis_info");
    this->declare_parameter<std::string>("localization_info_topic", "/localization/rtk_info");
    this->declare_parameter<std::string>("save_directory", "/home/nvidia/vehicle_data_logs");
    this->declare_parameter<double>("record_rate", 10.0);
    this->declare_parameter<bool>("enable_record", true);

    // 获取参数
    this->get_parameter("chassis_info_topic", chassis_info_topic_);
    this->get_parameter("localization_info_topic", localization_info_topic_);
    this->get_parameter("save_directory", save_directory_);
    this->get_parameter("record_rate", record_rate_);
    this->get_parameter("enable_record", enable_record_);

    // 打印参数
    RCLCPP_INFO(this->get_logger(), "底盘信息话题: %s", chassis_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "定位信息话题: %s", localization_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "保存目录: %s", save_directory_.c_str());
    RCLCPP_INFO(this->get_logger(), "记录频率: %.1f Hz", record_rate_);
    RCLCPP_INFO(this->get_logger(), "启用记录: %s", enable_record_ ? "是" : "否");
}

void DataRecorderNode::InitCsvFile() {
    if (!enable_record_) {
        RCLCPP_INFO(this->get_logger(), "记录功能已禁用");
        return;
    }

    try {
        // 创建保存目录
        std::filesystem::create_directories(save_directory_);
        RCLCPP_INFO(this->get_logger(), "保存目录创建/验证完成: %s", save_directory_.c_str());

        // 生成文件名
        current_filename_ = GenerateTimestampFilename();
        std::string full_path = save_directory_ + "/" + current_filename_;

        // 打开文件
        csv_file_.open(full_path, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开CSV文件: %s", full_path.c_str());
            enable_record_ = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "开始记录数据到: %s", full_path.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "初始化CSV文件失败: %s", e.what());
        enable_record_ = false;
    }
}

std::string DataRecorderNode::GenerateTimestampFilename() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << "vehicle_data_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
    return ss.str();
}

void DataRecorderNode::ChassisInfoCallback(const bot_msg::msg::ChassisInfo::SharedPtr msg) {
    chassis_info_msg_ = msg;
}

void DataRecorderNode::LocalizationInfoCallback(
    const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    localization_info_msg_ = msg;
}

void DataRecorderNode::TimerCallback() {
    if (!enable_record_ || !csv_file_.is_open()) {
        return;
    }

    // 检查是否有数据
    if (chassis_info_msg_ == nullptr || localization_info_msg_ == nullptr) {
        static int no_data_count = 0;
        no_data_count++;
        if (no_data_count % 50 == 0) {  // 每5秒提示一次（假设10Hz频率）
            RCLCPP_WARN(this->get_logger(), "等待数据中... 底盘数据: %s, 定位数据: %s",
                        chassis_info_msg_ ? "有" : "无", localization_info_msg_ ? "有" : "无");
        }
        return;
    }

    WriteCsvRecord();
}

void DataRecorderNode::WriteCsvRecord() {
    // 写入CSV头部（只在第一次写入时）
    if (!header_written_) {
        csv_file_ << "timestamp,";
        csv_file_ << "steer_angle_deg,cur_speed_mps,gear,direction,";
        csv_file_ << "longitude_deg,latitude_deg,altitude_m,";
        csv_file_ << "north_m,east_m,up_m,";
        csv_file_ << "yaw_deg,pitch_deg,roll_deg,";
        csv_file_ << "vel_speed_mps,vel_north_mps,vel_east_mps,vel_up_mps,";
        csv_file_ << "acc_x_mps2,acc_y_mps2,acc_z_mps2,";
        csv_file_ << "gyro_x_dps,gyro_y_dps,gyro_z_dps,";
        csv_file_ << "rtk_status";
        csv_file_ << std::endl;
        header_written_ = true;
        RCLCPP_INFO(this->get_logger(), "CSV头部写入完成");
    }

    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // 写入数据记录
    csv_file_ << std::fixed << std::setprecision(3);

    // 时间戳
    csv_file_ << millis << ",";

    // 底盘信息
    csv_file_ << chassis_info_msg_->steer_angle << ",";
    csv_file_ << chassis_info_msg_->cur_speed << ",";
    csv_file_ << static_cast<int>(chassis_info_msg_->gear) << ",";
    csv_file_ << static_cast<int>(chassis_info_msg_->direction) << ",";

    // 定位信息 - 经纬度坐标（使用高精度）
    csv_file_ << std::setprecision(8);
    csv_file_ << localization_info_msg_->longtitude << ",";
    csv_file_ << localization_info_msg_->latitude << ",";
    csv_file_ << std::setprecision(3);
    csv_file_ << localization_info_msg_->altitude << ",";

    // ENU坐标
    csv_file_ << localization_info_msg_->north << ",";
    csv_file_ << localization_info_msg_->east << ",";
    csv_file_ << localization_info_msg_->up << ",";

    // 姿态信息
    csv_file_ << localization_info_msg_->yaw << ",";
    csv_file_ << localization_info_msg_->pitch << ",";
    csv_file_ << localization_info_msg_->roll << ",";

    // 速度信息
    csv_file_ << localization_info_msg_->vel_speed << ",";
    csv_file_ << localization_info_msg_->vel_north << ",";
    csv_file_ << localization_info_msg_->vel_east << ",";
    csv_file_ << localization_info_msg_->vel_up << ",";

    // 加速度信息
    csv_file_ << localization_info_msg_->acc_x << ",";
    csv_file_ << localization_info_msg_->acc_y << ",";
    csv_file_ << localization_info_msg_->acc_z << ",";

    // 角速度信息
    csv_file_ << localization_info_msg_->gyro_x << ",";
    csv_file_ << localization_info_msg_->gyro_y << ",";
    csv_file_ << localization_info_msg_->gyro_z << ",";

    // RTK状态
    csv_file_ << static_cast<int>(localization_info_msg_->rtk_status);

    csv_file_ << std::endl;
    csv_file_.flush();  // 立即写入磁盘

    record_count_++;

    // 每100条记录输出一次状态
    if (record_count_ % 100 == 0) {
        auto current_time = std::chrono::steady_clock::now();
        auto duration_sec =
            std::chrono::duration_cast<std::chrono::seconds>(current_time - last_record_time_)
                .count();
        if (duration_sec > 0) {
            double actual_rate = 100.0 / duration_sec;
            RCLCPP_INFO(
                this->get_logger(),
                "已记录 %zu 条数据，实际频率: %.1f Hz，方向盘转角: %.1f°，当前速度: %.2f m/s",
                record_count_, actual_rate, chassis_info_msg_->steer_angle,
                chassis_info_msg_->cur_speed);
        }
        last_record_time_ = current_time;
    }
}

}  // namespace data_recorder

// 主函数
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<data_recorder::DataRecorderNode>();
    RCLCPP_INFO(node->get_logger(), "数据记录节点启动");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}