#include "hc_gnss/hc_gnss.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hc_gnss/gnss_serial.h"
#include <chrono>
#include <cstdio>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace hc_gnss {

HCGNSSNode::HCGNSSNode() : Node("hc_gnss_node") {
    // constructor
    InitValues();
    InitParams();
    bool ret = DeviceInit();
    if (ret) {
        running_ = true;
        read_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&HCGNSSNode::InfoReadLoop, this)));
        read_thread_->detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize device");
    }

    // publishers and timers
    int time_interval = static_cast<int>(1000.0 / local_publish_rate_);
    local_timer_ = this->create_wall_timer(std::chrono::milliseconds(time_interval),
                                           std::bind(&HCGNSSNode::LocalTimerCallback, this));
    time_interval = static_cast<int>(1000.0 / imu_publish_rate_);
    imu_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(time_interval), std::bind(&HCGNSSNode::ImuTimerCallback, this));
    time_interval = static_cast<int>(1000.0 / gnss_pose_enu_publish_rate_);
    gnss_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(time_interval), std::bind(&HCGNSSNode::GNSSTimerCallback, this));
    pub_localization_info_ = this->create_publisher<bot_msg::msg::LocalizationInfo>(local_topic_name_, 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name_, 50);
    pub_gnss_pose_enu_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(gnss_pose_enu_topic_name_, 10);
}

HCGNSSNode::~HCGNSSNode() {
    running_ = false;
    if (info_str_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Closing info_str log file. Total records saved: %d", info_str_save_count_);
        info_str_file_.close();
    }
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

void HCGNSSNode::LocalTimerCallback() {
    if (running_ && base_point_set_) {
        pub_localization_info_->publish(gnss_msg_);
    }
}

void HCGNSSNode::ImuTimerCallback() {
    if (running_) {
        pub_imu_->publish(imu_msg_);
    }
}

void HCGNSSNode::GNSSTimerCallback() {
    if (running_ && base_point_set_) {
        pub_gnss_pose_enu_->publish(gnss_pose_enu_msg_);
    }
}

/**
 * @brief parse the GPCHC info string
 * Format: $GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,gyro x,gyro y,gyroz,acc x,accy,accz,Latitude,Longitude,Altitude,Ve,Vn,Vu,V,NS1,NS2,Status,Age,Warming,Cs<CR><LF>
 */
void HCGNSSNode::ParseGPCHCInfo(const std::string &info_str) {
    parse_count_++;

    bool should_log = false;
    if (enable_debug_log_) {
        if (log_interval_ > 0) {
            should_log = (parse_count_ % log_interval_ == 0);
        } else {
            should_log = true;
        }
    }

    Gpchc gpchc;
    int parsed_fields = sscanf(
        info_str.c_str(),
        "$GPCHC,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%lf,%d,%d",
        &gpchc.gps_week, &gpchc.gps_time, &gpchc.heading_deg, &gpchc.pitch_deg, &gpchc.roll_deg,
        &gpchc.gyro_x_deg_s, &gpchc.gyro_y_deg_s, &gpchc.gyro_z_deg_s,
        &gpchc.acc_x_m_s2, &gpchc.acc_y_m_s2, &gpchc.acc_z_m_s2,
        &gpchc.latitude_deg, &gpchc.longitude_deg, &gpchc.altitude_m,
        &gpchc.ve_m_s, &gpchc.vn_m_s, &gpchc.vu_m_s, &gpchc.v_m_s,
        &gpchc.ns1, &gpchc.ns2, &gpchc.status, &gpchc.age, &gpchc.warming, &gpchc.checksum);

    // 支持 23 或 24 字段的数据格式
    if (parsed_fields < 23) {
        if (enable_debug_log_) {
            RCLCPP_ERROR(this->get_logger(), "parsed_fields:%d, Failed to parse GPCHC info: %s", parsed_fields, info_str.c_str());
        }
        return;
    }
    
    // 如果只解析到 23 个字段，设置校验和为 0
    if (parsed_fields == 23) {
        gpchc.checksum = 0;
        if (should_log) {
            RCLCPP_INFO(this->get_logger(), "Received GPCHC packet with 23 fields (checksum set to 0)");
        }
    }

    // 航向偏移处理
    gpchc.heading_deg += heading_offset_;
    if (gpchc.heading_deg >= 360) {
        gpchc.heading_deg -= 360;
    } else if (gpchc.heading_deg < 0) {
        gpchc.heading_deg += 360;
    }

    if (should_log) {
        RCLCPP_INFO(this->get_logger(), "Received GPCHC packet: %s", info_str.c_str());
    }

    // 为所有消息使用相同的时间戳
    rclcpp::Time current_timestamp = this->get_clock()->now();

    // 填充定位信息消息
    gnss_msg_.header.stamp = current_timestamp;
    gnss_msg_.header.frame_id = this->local_frame_id_;
    gnss_msg_.longtitude = gpchc.longitude_deg;
    gnss_msg_.latitude = gpchc.latitude_deg;
    gnss_msg_.altitude = gpchc.altitude_m;
    gnss_msg_.vel_north = gpchc.vn_m_s;
    gnss_msg_.vel_east = gpchc.ve_m_s;
    gnss_msg_.vel_up = gpchc.vu_m_s;
    gnss_msg_.roll = gpchc.roll_deg;
    gnss_msg_.pitch = gpchc.pitch_deg;
    gnss_msg_.yaw = gpchc.heading_deg;
    gnss_msg_.rtk_status = gpchc.status;
    gnss_msg_.vel_speed = gpchc.v_m_s;
    gnss_msg_.acc_x = gpchc.acc_x_m_s2;
    gnss_msg_.acc_y = gpchc.acc_y_m_s2;
    gnss_msg_.acc_z = gpchc.acc_z_m_s2;
    gnss_msg_.gyro_x = gpchc.gyro_x_deg_s;
    gnss_msg_.gyro_y = gpchc.gyro_y_deg_s;
    gnss_msg_.gyro_z = gpchc.gyro_z_deg_s;

    // 填充IMU消息
    imu_msg_.header.stamp = current_timestamp;
    imu_msg_.header.frame_id = imu_frame_id_;
    imu_msg_.linear_acceleration.x = gpchc.acc_x_m_s2;
    imu_msg_.linear_acceleration.y = gpchc.acc_y_m_s2;
    imu_msg_.linear_acceleration.z = gpchc.acc_z_m_s2;
    imu_msg_.angular_velocity.x = gpchc.gyro_x_deg_s * M_PI / 180.0; // 转换为弧度/秒
    imu_msg_.angular_velocity.y = gpchc.gyro_y_deg_s * M_PI / 180.0;
    imu_msg_.angular_velocity.z = gpchc.gyro_z_deg_s * M_PI / 180.0;
    
    tf2::Quaternion q;
    q.setRPY(gpchc.roll_deg * M_PI / 180.0, gpchc.pitch_deg * M_PI / 180.0, -gpchc.heading_deg * M_PI / 180.0);
    imu_msg_.orientation.x = q.x();
    imu_msg_.orientation.y = q.y();
    imu_msg_.orientation.z = q.z();
    imu_msg_.orientation.w = q.w();

    // 转换GPS坐标到ENU坐标
    if (gpchc.status >= 1) {
        WGS84toENU(gpchc);
        
        gnss_pose_enu_msg_.header.stamp = current_timestamp;
        gnss_pose_enu_msg_.header.frame_id = gnss_pose_enu_frame_id_;
        gnss_pose_enu_msg_.pose.position.x = gnss_msg_.east;
        gnss_pose_enu_msg_.pose.position.y = gnss_msg_.north;
        gnss_pose_enu_msg_.pose.position.z = gnss_msg_.up;

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, -gpchc.heading_deg * M_PI / 180.0);
        gnss_pose_enu_msg_.pose.orientation.x = orientation.x();
        gnss_pose_enu_msg_.pose.orientation.y = orientation.y();
        gnss_pose_enu_msg_.pose.orientation.z = orientation.z();
        gnss_pose_enu_msg_.pose.orientation.w = orientation.w();
    }
}

/**
 * @brief convert GPS coordinates to ENU coordinates
 */
void HCGNSSNode::WGS84toENU(const Gpchc &gpchc) {
    const double WGS84_A = 6378137.0;
    const double WGS84_E2 = 0.00669437999014;

    double lat_rad = gpchc.latitude_deg * M_PI / 180.0;
    double lon_rad = gpchc.longitude_deg * M_PI / 180.0;
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
    double x = (N + gpchc.altitude_m) * cos_lat * cos_lon;
    double y = (N + gpchc.altitude_m) * cos_lat * sin_lon;
    double z = (N * (1 - WGS84_E2) + gpchc.altitude_m) * sin_lat;

    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    double x_east = -sin_base_lon * dx + cos_base_lon * dy;
    double y_north = -cos_base_lon * sin_base_lat * dx - sin_base_lon * sin_base_lat * dy + cos_base_lat * dz;
    double z_up = cos_base_lat * cos_base_lon * dx + cos_base_lat * sin_base_lon * dy + sin_base_lat * dz;

    this->gnss_msg_.north = y_north;
    this->gnss_msg_.east = x_east;
    this->gnss_msg_.up = z_up;
}

void HCGNSSNode::InfoReadLoop() {
    std::string data;
    std::string gstart = "$GPCHC";
    std::string gend = "\r\n";
    
    while (running_) {
        char buf[512] = {0};
        int ret = read(sockfd_, buf, sizeof(buf));
        if (ret < 0) {
            error_count_++;
            auto now = std::chrono::steady_clock::now();
            auto time_since_last_log = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();

            if ((error_count_ % 50 == 0) || (time_since_last_log >= 5)) {
                RCLCPP_WARN(this->get_logger(), "Read error (count: %d): %s", error_count_, strerror(errno));
                last_log_time_ = now;
            }
        }
        data += buf;
        
        auto start_pos = data.find(gstart);
        auto end_pos = data.find(gend);
        if (start_pos != std::string::npos && end_pos != std::string::npos) {
            std::string info_str = data.substr(start_pos, end_pos - start_pos + gend.length());
            
            if (enable_info_str_save_) {
                SaveInfoStrToFile(info_str);
            }
            
            ParseGPCHCInfo(info_str);
            data.erase(0, end_pos + gend.length());
        }
    }
}

void HCGNSSNode::InitParams() {
    // declare parameters
    this->declare_parameter<std::string>("device_name", "/dev/ttyTHS4");
    this->declare_parameter<int>("baud_rate", 460800);
    this->declare_parameter<int>("timeout_ms", 20);
    this->declare_parameter<bool>("enable_debug_log", false);
    this->declare_parameter<int>("log_interval", 10);
    this->declare_parameter<double>("base_latitude", 0.0);
    this->declare_parameter<double>("base_longtitude", 0.0);
    this->declare_parameter<double>("base_altitude", 0.0);
    this->declare_parameter<std::string>("local_frame_id", "hc_gnss_frame");
    this->declare_parameter<std::string>("local_topic_name", "hc_gnss_fix");
    this->declare_parameter<double>("local_publish_rate", 10.0);
    this->declare_parameter<std::string>("imu_frame_id", "hc_imu_frame");
    this->declare_parameter<std::string>("imu_topic_name", "hc_imu");
    this->declare_parameter<double>("imu_publish_rate", 10.0);
    this->declare_parameter<std::string>("gnss_frame_id", "hc_gnss_pose_enu_frame");
    this->declare_parameter<std::string>("gnss_topic_name", "hc_gnss_pose_enu");
    this->declare_parameter<double>("gnss_publish_rate", 10.0);
    this->declare_parameter<double>("heading_offset", 0.0);
    this->declare_parameter<bool>("enable_info_str_save", false);
    this->declare_parameter<std::string>("info_str_save_dir", "/tmp/hc_gnss_logs");

    // get parameters
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
    this->get_parameter("enable_info_str_save", this->enable_info_str_save_);
    this->get_parameter("info_str_save_dir", this->info_str_save_dir_);

    parse_count_ = 0;
    error_count_ = 0;
    last_log_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "HC GNSS Device name: %s", this->device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", this->baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Base latitude: %lf", this->base_latitude_deg_);
    RCLCPP_INFO(this->get_logger(), "Base longitude: %lf", this->base_longitude_deg_);
    
    if (enable_info_str_save_) {
        InitInfoStrSaving();
    }
}

bool HCGNSSNode::DeviceInit() {
    sockfd_ = Serial::TTYOpen(device_name_.c_str());
    if (sockfd_ < 0) {
        sockfd_ = -1;
        RCLCPP_ERROR(this->get_logger(), "tty_open failed.");
        return false;
    }
    if (Serial::TTYSetOpt(sockfd_, baud_rate_, 8, 1, 'n') != 0) {
        sockfd_ = -1;
        RCLCPP_ERROR(this->get_logger(), "tty_setopt error.");
        return false;
    }
    return true;
}

void HCGNSSNode::InitValues() {
    base_point_set_ = true;
    running_ = false;
    read_thread_ = nullptr;
    info_str_save_count_ = 0;
}

void HCGNSSNode::InitInfoStrSaving() {
    try {
        std::filesystem::create_directories(info_str_save_dir_);
        RCLCPP_INFO(this->get_logger(), "Info_str save directory created/verified: %s", info_str_save_dir_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create info_str save directory: %s", e.what());
        enable_info_str_save_ = false;
    }
}

void HCGNSSNode::SaveInfoStrToFile(const std::string &info_str) {
    if (current_log_filename_.empty() || !info_str_file_.is_open()) {
        current_log_filename_ = GenerateTimestampFilename();
        std::string full_path = info_str_save_dir_ + "/" + current_log_filename_;
        
        if (info_str_file_.is_open()) {
            info_str_file_.close();
        }
        
        info_str_file_.open(full_path, std::ios::app);
        if (!info_str_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open info_str log file: %s", full_path.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Started logging info_str to: %s", full_path.c_str());
    }
    
    info_str_file_ << info_str << std::endl;
    info_str_file_.flush();
    
    info_str_save_count_++;
}

std::string HCGNSSNode::GenerateTimestampFilename() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << "hc_gnss_info_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".log";
    return ss.str();
}

} // namespace hc_gnss

// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hc_gnss::HCGNSSNode>();
    RCLCPP_INFO(node->get_logger(), "hc_gnss node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}