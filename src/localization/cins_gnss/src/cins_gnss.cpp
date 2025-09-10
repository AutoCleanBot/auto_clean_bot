#include "cins_gnss/cins_gnss.h"

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "cins_gnss/cins_serial.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cins_gnss {

CinsGnssNode::CinsGnssNode() : Node("cins_gnss_node") {
    InitValues();
    InitParams();
    bool ret = DeviceInit();
    if (ret) {
        running_ = true;
        read_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&CinsGnssNode::DataReadLoop, this)));
        read_thread_->detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize device");
    }

    // 创建定时器和发布器
    int time_interval = static_cast<int>(1000.0 / local_publish_rate_);
    local_timer_ = this->create_wall_timer(std::chrono::milliseconds(time_interval),
                                           std::bind(&CinsGnssNode::LocalTimerCallback, this));
    time_interval = static_cast<int>(1000.0 / imu_publish_rate_);
    imu_timer_ = this->create_wall_timer(std::chrono::milliseconds(time_interval),
                                         std::bind(&CinsGnssNode::ImuTimerCallback, this));
    time_interval = static_cast<int>(1000.0 / gnss_pose_enu_publish_rate_);
    gnss_timer_ = this->create_wall_timer(std::chrono::milliseconds(time_interval),
                                          std::bind(&CinsGnssNode::GNSSTimerCallback, this));

    pub_localization_info_ =
        this->create_publisher<bot_msg::msg::LocalizationInfo>(local_topic_name_, 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name_, 50);
    pub_gnss_pose_enu_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(gnss_pose_enu_topic_name_, 10);
}

CinsGnssNode::~CinsGnssNode() {
    running_ = false;
    if (data_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Closing data log file. Total records saved: %d",
                    data_save_count_);
        data_file_.close();
    }
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

void CinsGnssNode::LocalTimerCallback() {
    if (running_ && base_point_set_ && data_valid_) {
        pub_localization_info_->publish(cins_msg_);
    }
}

void CinsGnssNode::ImuTimerCallback() {
    if (running_ && data_valid_) {
        pub_imu_->publish(imu_msg_);
    }
}

void CinsGnssNode::GNSSTimerCallback() {
    if (running_ && base_point_set_ && data_valid_) {
        pub_gnss_pose_enu_->publish(gnss_pose_enu_msg_);
    }
}

void CinsGnssNode::WGS84toENU(const PboxData &pbox_data) {
    const double WGS84_A = 6378137.0;
    const double WGS84_E2 = 0.00669437999014;

    double lat_rad = pbox_data.pos_lat * M_PI / 180.0;
    double lon_rad = pbox_data.pos_lon * M_PI / 180.0;
    double base_lat_rad = base_latitude_deg_ * M_PI / 180.0;
    double base_lon_rad = base_longitude_deg_ * M_PI / 180.0;

    double sin_base_lat = sin(base_lat_rad);
    double cos_base_lat = cos(base_lat_rad);
    double sin_base_lon = sin(base_lon_rad);
    double cos_base_lon = cos(base_lon_rad);

    double N0 = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_base_lat * sin_base_lat);
    double x0 = (N0 + base_altitude_m_) * cos_base_lat * cos_base_lon;
    double y0 = (N0 + base_altitude_m_) * cos_base_lat * sin_base_lon;
    double z0 = (N0 * (1 - WGS84_E2) + base_altitude_m_) * sin_base_lat;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);

    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    double x = (N + pbox_data.pos_alt) * cos_lat * cos_lon;
    double y = (N + pbox_data.pos_alt) * cos_lat * sin_lon;
    double z = (N * (1 - WGS84_E2) + pbox_data.pos_alt) * sin_lat;

    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    double x_east = -sin_base_lon * dx + cos_base_lon * dy;
    double y_north =
        -cos_base_lon * sin_base_lat * dx - sin_base_lon * sin_base_lat * dy + cos_base_lat * dz;
    double z_up =
        cos_base_lat * cos_base_lon * dx + cos_base_lat * sin_base_lon * dy + sin_base_lat * dz;

    this->cins_msg_.north = y_north;
    this->cins_msg_.east = x_east;
    this->cins_msg_.up = z_up;
}

void CinsGnssNode::ParsePboxData(const std::vector<uint8_t> &data_buffer) {
    parse_count_++;

    if (data_buffer.size() < sizeof(PboxData)) {
        if (enable_debug_log_) {
            RCLCPP_ERROR(this->get_logger(), "Data buffer too small: %zu bytes",
                         data_buffer.size());
        }
        return;
    }

    if (!VerifyChecksum(data_buffer)) {
        error_count_++;
        if (enable_debug_log_) {
            RCLCPP_ERROR(this->get_logger(), "Checksum verification failed");
        }
        return;
    }

    // 解析二进制数据
    const uint8_t *data = data_buffer.data();
    size_t offset = 0;

    memcpy(&latest_pbox_data_.header, data + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&latest_pbox_data_.length, data + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&latest_pbox_data_.utc_stamp, data + offset, sizeof(uint64_t));
    offset += sizeof(uint64_t);
    memcpy(&latest_pbox_data_.ins_flag, data + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    // IMU数据
    memcpy(&latest_pbox_data_.ang_rate_raw_x, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.ang_rate_raw_y, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.ang_rate_raw_z, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.accel_raw_x, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.accel_raw_y, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.accel_raw_z, data + offset, sizeof(float));
    offset += sizeof(float);

    // 状态信息
    memcpy(&latest_pbox_data_.ins_state, data + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy(&latest_pbox_data_.ins_convergence, data + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy(&latest_pbox_data_.fix_status, data + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy(&latest_pbox_data_.sat_num, data + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy(&latest_pbox_data_.fix_age, data + offset, sizeof(float));
    offset += sizeof(float);

    // 位置信息
    memcpy(&latest_pbox_data_.pos_lat, data + offset, sizeof(double));
    offset += sizeof(double);
    memcpy(&latest_pbox_data_.pos_lon, data + offset, sizeof(double));
    offset += sizeof(double);
    memcpy(&latest_pbox_data_.pos_alt, data + offset, sizeof(float));
    offset += sizeof(float);

    // 位置标准差
    memcpy(&latest_pbox_data_.pos_e_sigma, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.pos_n_sigma, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.pos_u_sigma, data + offset, sizeof(float));
    offset += sizeof(float);

    // 速度信息
    memcpy(&latest_pbox_data_.vel, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.vel_sigma, data + offset, sizeof(float));
    offset += sizeof(float);

    // 姿态信息
    memcpy(&latest_pbox_data_.angle_heading, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.angle_pitch, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.angle_roll, data + offset, sizeof(float));
    offset += sizeof(float);

    // 姿态标准差
    memcpy(&latest_pbox_data_.angle_heading_sigma, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.angle_pitch_sigma, data + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&latest_pbox_data_.angle_roll_sigma, data + offset, sizeof(float));
    offset += sizeof(float);

    // 应用航向偏移
    latest_pbox_data_.angle_heading += heading_offset_;
    if (latest_pbox_data_.angle_heading >= 360) {
        latest_pbox_data_.angle_heading -= 360;
    } else if (latest_pbox_data_.angle_heading < 0) {
        latest_pbox_data_.angle_heading += 360;
    }

    // 调试日志
    bool should_log =
        enable_debug_log_ && (log_interval_ <= 0 || parse_count_ % log_interval_ == 0);
    if (should_log) {
        RCLCPP_INFO(this->get_logger(),
                    "Parsed PBOX data (Parse #%d): UTC:%lu, INS_State:%d, Fix_Status:%d, Sat:%d, "
                    "Lat:%.8f, Lon:%.8f, Alt:%.2f, Heading:%.2f",
                    parse_count_, latest_pbox_data_.utc_stamp, latest_pbox_data_.ins_state,
                    latest_pbox_data_.fix_status, latest_pbox_data_.sat_num,
                    latest_pbox_data_.pos_lat, latest_pbox_data_.pos_lon, latest_pbox_data_.pos_alt,
                    latest_pbox_data_.angle_heading);
    }

    // 使用相同的时间戳
    rclcpp::Time current_timestamp = this->get_clock()->now();

    // 填充定位信息消息
    cins_msg_.header.stamp = current_timestamp;
    cins_msg_.header.frame_id = this->local_frame_id_;
    cins_msg_.longtitude = latest_pbox_data_.pos_lon;
    cins_msg_.latitude = latest_pbox_data_.pos_lat;
    cins_msg_.altitude = latest_pbox_data_.pos_alt;
    cins_msg_.vel_north = 0.0;
    cins_msg_.vel_east = 0.0;
    cins_msg_.vel_up = 0.0;
    cins_msg_.roll = latest_pbox_data_.angle_roll;
    cins_msg_.pitch = latest_pbox_data_.angle_pitch;
    cins_msg_.yaw = latest_pbox_data_.angle_heading;
    cins_msg_.rtk_status = latest_pbox_data_.fix_status;
    cins_msg_.vel_speed = latest_pbox_data_.vel;

    // 将加速度从g转换为m/s²
    cins_msg_.acc_x = latest_pbox_data_.accel_raw_x * 9.81;
    cins_msg_.acc_y = latest_pbox_data_.accel_raw_y * 9.81;
    cins_msg_.acc_z = latest_pbox_data_.accel_raw_z * 9.81;

    // 将角速度从d/s转换为rad/s
    cins_msg_.gyro_x = latest_pbox_data_.ang_rate_raw_x * M_PI / 180.0;
    cins_msg_.gyro_y = latest_pbox_data_.ang_rate_raw_y * M_PI / 180.0;
    cins_msg_.gyro_z = latest_pbox_data_.ang_rate_raw_z * M_PI / 180.0;

    // 填充IMU消息
    imu_msg_.header.stamp = current_timestamp;
    imu_msg_.header.frame_id = imu_frame_id_;
    imu_msg_.linear_acceleration.x = cins_msg_.acc_x;
    imu_msg_.linear_acceleration.y = cins_msg_.acc_y;
    imu_msg_.linear_acceleration.z = cins_msg_.acc_z;
    imu_msg_.angular_velocity.x = cins_msg_.gyro_x;
    imu_msg_.angular_velocity.y = cins_msg_.gyro_y;
    imu_msg_.angular_velocity.z = cins_msg_.gyro_z;

    tf2::Quaternion q;
    q.setRPY(latest_pbox_data_.angle_roll * M_PI / 180.0,
             latest_pbox_data_.angle_pitch * M_PI / 180.0,
             -latest_pbox_data_.angle_heading * M_PI / 180.0);
    imu_msg_.orientation.x = q.x();
    imu_msg_.orientation.y = q.y();
    imu_msg_.orientation.z = q.z();
    imu_msg_.orientation.w = q.w();

    // 转换GPS坐标到ENU坐标（仅在定位有效时）
    if (latest_pbox_data_.fix_status >= 1) {
        WGS84toENU(latest_pbox_data_);

        gnss_pose_enu_msg_.header.stamp = current_timestamp;
        gnss_pose_enu_msg_.header.frame_id = gnss_pose_enu_frame_id_;
        gnss_pose_enu_msg_.pose.position.x = cins_msg_.east;
        gnss_pose_enu_msg_.pose.position.y = cins_msg_.north;
        gnss_pose_enu_msg_.pose.position.z = cins_msg_.up;

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, -latest_pbox_data_.angle_heading * M_PI / 180.0);
        gnss_pose_enu_msg_.pose.orientation.x = orientation.x();
        gnss_pose_enu_msg_.pose.orientation.y = orientation.y();
        gnss_pose_enu_msg_.pose.orientation.z = orientation.z();
        gnss_pose_enu_msg_.pose.orientation.w = orientation.w();
    }

    data_valid_ = true;
}

void CinsGnssNode::DataReadLoop() {
    std::vector<uint8_t> data_buffer;
    const uint16_t PBOX_HEADER = 0xAA55;

    while (running_) {
        uint8_t buf[512] = {0};
        int ret = read(sockfd_, buf, sizeof(buf));

        if (ret < 0) {
            error_count_++;
            auto now = std::chrono::steady_clock::now();
            auto time_since_last_log =
                std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();

            if ((error_count_ % 50 == 0) || (time_since_last_log >= 5)) {
                RCLCPP_WARN(this->get_logger(), "Read error (count: %d): %s", error_count_,
                            strerror(errno));
                last_log_time_ = now;
            }
            continue;
        }

        // 添加读取到的数据到缓冲区
        for (int i = 0; i < ret; i++) {
            data_buffer.push_back(buf[i]);
        }

        // 查找包头0xAA55
        while (data_buffer.size() >= 4) {
            uint16_t header = (data_buffer[1] << 8) | data_buffer[0];
            if (header == PBOX_HEADER) {
                uint16_t length = (data_buffer[3] << 8) | data_buffer[2];

                if (data_buffer.size() >= length) {
                    std::vector<uint8_t> packet(data_buffer.begin(), data_buffer.begin() + length);

                    if (enable_data_save_) {
                        SaveDataToFile(packet);
                    }

                    ParsePboxData(packet);
                    data_buffer.erase(data_buffer.begin(), data_buffer.begin() + length);
                    // 打印接受的包数据
                    if (enable_debug_log_) {
                        std::stringstream ss;
                        for (size_t i = 0; i < packet.size(); i++) {
                            ss << std::hex << std::setfill('0') << std::setw(2) << (int)packet[i]
                               << " ";
                        }
                        RCLCPP_INFO(this->get_logger(), "Received packet: %s", ss.str().c_str());
                    }
                } else {
                    break;
                }

            } else {
                data_buffer.erase(data_buffer.begin());
            }
        }
    }
}

bool CinsGnssNode::VerifyChecksum(const std::vector<uint8_t> &data_buffer) {
    if (data_buffer.size() < 2) {
        return false;
    }

    uint8_t calculated_xor = 0;
    for (size_t i = 0; i < data_buffer.size() - 1; i++) {
        calculated_xor ^= data_buffer[i];
    }

    uint8_t received_xor = data_buffer.back();
    return calculated_xor == received_xor;
}

void CinsGnssNode::InitParams() {
    this->declare_parameter<std::string>("device_name", "/dev/ttyTHS4");
    this->declare_parameter<int>("baud_rate", 460800);
    this->declare_parameter<int>("timeout_ms", 20);
    this->declare_parameter<bool>("enable_debug_log", false);
    this->declare_parameter<int>("log_interval", 10);
    this->declare_parameter<double>("base_latitude", 0.0);
    this->declare_parameter<double>("base_longtitude", 0.0);
    this->declare_parameter<double>("base_altitude", 0.0);
    this->declare_parameter<std::string>("local_frame_id", "cins_frame");
    this->declare_parameter<std::string>("local_topic_name", "cins_fix");
    this->declare_parameter<double>("local_publish_rate", 10.0);
    this->declare_parameter<std::string>("imu_frame_id", "imu_frame");
    this->declare_parameter<std::string>("imu_topic_name", "imu");
    this->declare_parameter<double>("imu_publish_rate", 10.0);
    this->declare_parameter<std::string>("gnss_frame_id", "gnss_pose_enu_frame");
    this->declare_parameter<std::string>("gnss_topic_name", "gnss_pose_enu");
    this->declare_parameter<double>("gnss_publish_rate", 10.0);
    this->declare_parameter<double>("heading_offset", 0.0);
    this->declare_parameter<bool>("enable_data_save", false);
    this->declare_parameter<std::string>("data_save_dir", "/tmp/cins_logs");

    this->get_parameter("device_name", this->device_name_);
    this->get_parameter("baud_rate", this->baud_rate_);
    this->get_parameter("timeout_ms", this->timeout_ms_);
    this->get_parameter("local_frame_id", this->local_frame_id_);
    this->get_parameter("local_topic_name", this->local_topic_name_);
    this->get_parameter("local_publish_rate", this->local_publish_rate_);
    this->get_parameter("imu_frame_id", this->imu_frame_id_);
    this->get_parameter("imu_topic_name", this->imu_topic_name_);
    this->get_parameter("imu_publish_rate", this->imu_publish_rate_);
    this->get_parameter("gnss_frame_id", this->gnss_pose_enu_frame_id_);
    this->get_parameter("gnss_topic_name", this->gnss_pose_enu_topic_name_);
    this->get_parameter("gnss_publish_rate", this->gnss_pose_enu_publish_rate_);
    this->get_parameter("enable_debug_log", this->enable_debug_log_);
    this->get_parameter("log_interval", this->log_interval_);
    this->get_parameter("base_latitude", this->base_latitude_deg_);
    this->get_parameter("base_longtitude", this->base_longitude_deg_);
    this->get_parameter("base_altitude", this->base_altitude_m_);
    this->get_parameter("heading_offset", this->heading_offset_);
    this->get_parameter("enable_data_save", this->enable_data_save_);
    this->get_parameter("data_save_dir", this->data_save_dir_);

    parse_count_ = 0;
    error_count_ = 0;
    last_log_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "CINS GNSS Node initialized with device: %s",
                device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publish topics: %s, %s, %s", local_topic_name_.c_str(),
                imu_topic_name_.c_str(), gnss_pose_enu_topic_name_.c_str());

    if (enable_data_save_) {
        InitDataSaving();
    }
}

bool CinsGnssNode::DeviceInit() {
    sockfd_ = CinsSerial::TTYOpen(device_name_.c_str());
    if (sockfd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open device: %s", device_name_.c_str());
        return false;
    }

    if (CinsSerial::TTYSetOpt(sockfd_, baud_rate_, 8, 1, 'n') != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure device");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Device initialized successfully");
    return true;
}

void CinsGnssNode::InitValues() {
    base_point_set_ = true;
    running_ = false;
    read_thread_ = nullptr;
    data_save_count_ = 0;
    data_valid_ = false;
    sockfd_ = -1;
}

void CinsGnssNode::InitDataSaving() {
    try {
        std::filesystem::create_directories(data_save_dir_);
        RCLCPP_INFO(this->get_logger(), "Data save directory: %s", data_save_dir_.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create data save directory: %s", e.what());
        enable_data_save_ = false;
    }
}

void CinsGnssNode::SaveDataToFile(const std::vector<uint8_t> &data_buffer) {
    if (current_log_filename_.empty() || !data_file_.is_open()) {
        current_log_filename_ = GenerateTimestampFilename();
        std::string full_path = data_save_dir_ + "/" + current_log_filename_;

        if (data_file_.is_open()) {
            data_file_.close();
        }

        data_file_.open(full_path, std::ios::binary | std::ios::app);
        if (!data_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open data log file: %s", full_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Started logging data to: %s", full_path.c_str());
    }

    data_file_.write(reinterpret_cast<const char *>(data_buffer.data()), data_buffer.size());
    data_file_.flush();
    data_save_count_++;
}

std::string CinsGnssNode::GenerateTimestampFilename() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << "cins_data_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".bin";
    return ss.str();
}

}  // namespace cins_gnss

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cins_gnss::CinsGnssNode>();
    RCLCPP_INFO(node->get_logger(), "CINS GNSS node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}