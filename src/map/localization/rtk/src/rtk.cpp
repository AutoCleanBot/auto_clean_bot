#include "rtk/rtk.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rtk/gps_serial.h"
#include <chrono>
#include <cstdio>
#include <string>

namespace rtk {
RTKNode::RTKNode() : Node("rtk_node") {
    // constructor
    InitValues();
    InitParams();
    bool ret = DeviceInit();
    if (ret) {
        running_ = true;
        read_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&RTKNode::InfoReadLoop, this)));
        read_thread_->detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize device");
    }

    // publishers and subscribers
    int time_interval = static_cast<int>(1000.0 / local_publish_rate_);
    local_timer_ = this->create_wall_timer(std::chrono::milliseconds(time_interval),
                                           std::bind(&RTKNode::LocalTimerCallback, this));
    time_interval = static_cast<int>(1000.0 / imu_publish_rate_);
    imu_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(time_interval), std::bind(&RTKNode::ImuTimerCallback, this));
    time_interval = static_cast<int>(1000.0 / gnss_pose_enu_publish_rate_);
    gnss_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(time_interval), std::bind(&RTKNode::GNSSTimerCallback, this));
    pub_localization_info_ = this->create_publisher<bot_msg::msg::LocalizationInfo>(local_topic_name_, 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name_, 50);
    pub_gnss_pose_enu_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(gnss_pose_enu_topic_name_, 10);
}

void RTKNode::LocalTimerCallback() {
    if (running_ && base_point_set_) {
        pub_localization_info_->publish(rtk_msg_);
    }
}

void RTKNode::ImuTimerCallback() {
    if (running_) {
        pub_imu_->publish(imu_msg_);
    }
}

void RTKNode::GNSSTimerCallback() {
    if (running_ && base_point_set_) {
        pub_gnss_pose_enu_->publish(gnss_pose_enu_msg_);
    }
}

/**
 * @brief convert GPS coordinates to ENU coordinates,
 *          and change the value of rtk_msg_
 *
 * @param giavp
 */
void RTKNode::WGS84toENU(const Giavp &giavp) {
    const double WGS84_A = 6378137.0;         // 地球长半轴 (单位: 米)
    const double WGS84_E2 = 0.00669437999014; // 偏心率的平方

    // 将经纬度从度转换为弧度
    double lat_rad = giavp.latitude_deg * M_PI / 180.0;
    double lon_rad = giavp.longitude_deg * M_PI / 180.0;
    double base_lat_rad = base_latitude_deg_ * M_PI / 180.0;
    double base_lon_rad = base_longitude_deg_ * M_PI / 180.0;

    // 计算基准点的ECEF坐标
    double sin_base_lat = sin(base_lat_rad);
    double cos_base_lat = cos(base_lat_rad);
    double sin_base_lon = sin(base_lon_rad);
    double cos_base_lon = cos(base_lon_rad);

    double N0 = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_base_lat * sin_base_lat);
    double x0 = (N0 + base_altitude_m_) * cos_base_lat * cos_base_lon;
    double y0 = (N0 + base_altitude_m_) * cos_base_lat * sin_base_lon;
    double z0 = (N0 * (1 - WGS84_E2) + base_altitude_m_) * sin_base_lat;

    // 计算目标点的ECEF坐标
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);

    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    double x = (N + giavp.altitude_m) * cos_lat * cos_lon;
    double y = (N + giavp.altitude_m) * cos_lat * sin_lon;
    double z = (N * (1 - WGS84_E2) + giavp.altitude_m) * sin_lat;

    // 计算ECEF坐标差值
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    // 将ECEF转换为ENU坐标
    double x_east = -sin_base_lon * dx + cos_base_lon * dy;
    double y_north = -cos_base_lon * sin_base_lat * dx - sin_base_lon * sin_base_lat * dy + cos_base_lat * dz;
    double z_up = cos_base_lat * cos_base_lon * dx + cos_base_lat * sin_base_lon * dy + sin_base_lat * dz;

    // 填充消息的ENU坐标
    this->rtk_msg_.north = y_north;
    this->rtk_msg_.east = x_east;
    this->rtk_msg_.up = z_up;
}

/**
 * @brief parse the RTK info string
 *
 * @param info_str
 */
void RTKNode::ParseRTKInfo(const std::string &info_str) {
    Giavp giavp;
    int parsed_fields = sscanf(
        info_str.c_str(),
        "$GIAVP,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &giavp.week,
        &giavp.time_sec, &giavp.heading_deg, &giavp.pitch_deg, &giavp.roll_deg, &giavp.latitude_deg,
        &giavp.longitude_deg, &giavp.altitude_m, &giavp.ve_m_s, &giavp.vn_m_s, &giavp.vu_m_s, &giavp.baseline,
        &giavp.nvsv1, &giavp.nvsv2, &giavp.status, &giavp.speed_status, &giavp.vehicle_speed_m_s, &giavp.acc_x_m_s2,
        &giavp.acc_y_m_s2, &giavp.acc_z_m_s2, &giavp.gyro_x_deg_s, &giavp.gyro_y_deg_s, &giavp.gyro_z_deg_s);
    if (parsed_fields != 23 && enable_debug_log_) {
        RCLCPP_ERROR(this->get_logger(), "parsed_fields:%d,Failed to parse RTK info: %s", parsed_fields,
                     info_str.c_str());
    }
    if (enable_debug_log_) {
        RCLCPP_INFO(this->get_logger(), "Received a full packet: %s", info_str.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Received RTK info: "
                    "week:%d, time_sec:%lf, heading_deg:%lf, pitch_deg:%lf, roll_deg:%lf, latitude_deg:%lf, "
                    "longitude_deg:%lf, altitude_m:%lf, ve_m_s:%lf, vn_m_s:%lf, vu_m_s:%lf, baseline:%lf, nvsv1:%d, "
                    "nvsv2:%d, status:%d, speed_status:%lf, vehicle_speed_m_s:%lf, acc_x_m_s2:%lf, acc_y_m_s2:%lf, "
                    "acc_z_m_s2:%lf, gyro_x_deg_s:%lf, gyro_y_deg_s:%lf, gyro_z_deg_s:%lf",
                    giavp.week, giavp.time_sec, giavp.heading_deg, giavp.pitch_deg, giavp.roll_deg, giavp.latitude_deg,
                    giavp.longitude_deg, giavp.altitude_m, giavp.ve_m_s, giavp.vn_m_s, giavp.vu_m_s, giavp.baseline,
                    giavp.nvsv1, giavp.nvsv2, giavp.status, giavp.speed_status, giavp.vehicle_speed_m_s,
                    giavp.acc_x_m_s2, giavp.acc_y_m_s2, giavp.acc_z_m_s2, giavp.gyro_x_deg_s, giavp.gyro_y_deg_s,
                    giavp.gyro_z_deg_s);
    }
    // simply publish the parsed RTK info
    rtk_msg_.header.stamp = this->get_clock()->now();
    rtk_msg_.header.frame_id = this->local_frame_id_;
    rtk_msg_.longtitude = giavp.longitude_deg;
    rtk_msg_.latitude = giavp.latitude_deg;
    rtk_msg_.altitude = giavp.altitude_m;
    rtk_msg_.vel_north = giavp.vn_m_s;
    rtk_msg_.vel_east = giavp.ve_m_s;
    rtk_msg_.vel_up = giavp.vu_m_s;
    rtk_msg_.roll = giavp.roll_deg;
    rtk_msg_.pitch = giavp.pitch_deg;
    rtk_msg_.yaw = giavp.heading_deg;
    rtk_msg_.rtk_status = giavp.status;
    rtk_msg_.vel_speed = sqrt(giavp.vn_m_s * giavp.vn_m_s + giavp.ve_m_s * giavp.ve_m_s);
    rtk_msg_.acc_x = giavp.acc_x_m_s2; // x 轴加速度
    rtk_msg_.acc_y = giavp.acc_y_m_s2;
    rtk_msg_.acc_z = giavp.acc_z_m_s2;
    rtk_msg_.gyro_x = giavp.gyro_x_deg_s;
    rtk_msg_.gyro_y = giavp.gyro_y_deg_s;
    rtk_msg_.gyro_z = giavp.gyro_z_deg_s;

    // fill the IMU msg
    imu_msg_.header.stamp = this->get_clock()->now();
    imu_msg_.header.frame_id = imu_frame_id_;
    imu_msg_.linear_acceleration.x = giavp.acc_x_m_s2;
    imu_msg_.linear_acceleration.y = giavp.acc_y_m_s2;
    imu_msg_.linear_acceleration.z = giavp.acc_z_m_s2;
    imu_msg_.angular_velocity.x = giavp.gyro_x_deg_s;
    imu_msg_.angular_velocity.y = giavp.gyro_y_deg_s;
    imu_msg_.angular_velocity.z = giavp.gyro_z_deg_s;
    tf2::Quaternion q;
    q.setRPY(giavp.roll_deg * M_PI / 180.0, giavp.pitch_deg * M_PI / 180.0, giavp.heading_deg * M_PI / 180.0);
    imu_msg_.orientation.x = q.x();
    imu_msg_.orientation.y = q.y();
    imu_msg_.orientation.z = q.z();
    imu_msg_.orientation.w = q.w();
    // log the IMU message
    if (enable_debug_log_)
        RCLCPP_INFO(this->get_logger(), "IMU, acc: %lf, %lf, %lf, gyro: %lf, %lf, %lf, orientation: %lf, %lf, %lf, %lf",
                    imu_msg_.linear_acceleration.x, imu_msg_.linear_acceleration.y, imu_msg_.linear_acceleration.z,
                    imu_msg_.angular_velocity.x, imu_msg_.angular_velocity.y, imu_msg_.angular_velocity.z, q.x(), q.y(),
                    q.z(), q.w());

    // convert GPS coordinates to ENU coordinates
    if (giavp.status >= 1) {
        WGS84toENU(giavp);
        // fill the gnss msg
        gnss_pose_enu_msg_.header.stamp = this->get_clock()->now();
        gnss_pose_enu_msg_.header.frame_id = gnss_pose_enu_frame_id_;
        gnss_pose_enu_msg_.pose.position.x = rtk_msg_.east;
        gnss_pose_enu_msg_.pose.position.y = rtk_msg_.north;
        gnss_pose_enu_msg_.pose.position.z = rtk_msg_.up;

        tf2::Quaternion orientation;
        orientation.setRPY(giavp.roll_deg * M_PI / 180.0, giavp.pitch_deg * M_PI / 180.0,
                           giavp.heading_deg * M_PI / 180.0);
        gnss_pose_enu_msg_.pose.orientation.x = orientation.x();
        gnss_pose_enu_msg_.pose.orientation.y = orientation.y();
        gnss_pose_enu_msg_.pose.orientation.z = orientation.z();
        gnss_pose_enu_msg_.pose.orientation.w = orientation.w();
        if (enable_debug_log_) {
            // publish the gnss msg
            RCLCPP_INFO(this->get_logger(), "ENU pose: %lf, %lf, %lf, orientation:%lf,%lf,%lf,%lf",
                        gnss_pose_enu_msg_.pose.position.x, gnss_pose_enu_msg_.pose.position.y,
                        gnss_pose_enu_msg_.pose.position.z, gnss_pose_enu_msg_.pose.orientation.x,
                        gnss_pose_enu_msg_.pose.orientation.y, gnss_pose_enu_msg_.pose.orientation.z,
                        gnss_pose_enu_msg_.pose.orientation.w);
        }
    }
    return;
}

void RTKNode::InfoReadLoop() {
    std::string data;
    std::string gstart = "$GIAVP"; // 开头
    std::string gend = "\r\n";
    while (running_) {
        char buf[512] = {0};
        int ret = read(sockfd_, buf, sizeof(buf));
        if (ret < 0) {
            // TODO 串口错误处理
            RCLCPP_WARN(this->get_logger(), "Read error: %s", strerror(errno));
        }
        data += buf;
        // RCLCPP_INFO(this->get_logger(), "Received data: %s", data.c_str());
        auto start_pos = data.find(gstart);
        auto end_pos = data.find(gend);
        if (start_pos != std::string::npos && end_pos != std::string::npos) {
            std::string info_str = data.substr(start_pos, end_pos - start_pos + gend.length());
            ParseRTKInfo(info_str);
            data.erase(0, end_pos + gend.length());
        }
    }
}

/**
 * @brief configure the node with parameters
 *
 */
void RTKNode::InitParams() {
    // declare parameters here
    this->declare_parameter<std::string>("device_name", "/dev/ttyTHS4");
    this->declare_parameter<int>("baud_rate", 460800);
    this->declare_parameter<int>("timeout_ms", 20);
    this->declare_parameter<bool>("enable_debug_log", false);
    this->declare_parameter<double>("base_latitude", 0.0);
    this->declare_parameter<double>("base_longtitude", 0.0);
    this->declare_parameter<double>("base_altitude", 0.0);

    this->declare_parameter<std::string>("local_frame_id", "rtk_frame");
    this->declare_parameter<std::string>("local_topic_name", "rtk_fix");
    this->declare_parameter<double>("local_publish_rate", 10.0);

    this->declare_parameter<std::string>("imu_frame_id", "imu_frame");
    this->declare_parameter<std::string>("imu_topic_name", "imu");
    this->declare_parameter<double>("imu_publish_rate", 10.0);

    this->declare_parameter<std::string>("gnss_frame_id", "gnss_pose_enu_frame");
    this->declare_parameter<std::string>("gnss_topic_name", "gnss_pose_enu");
    this->declare_parameter<double>("gnss_publish_rate", 10.0);

    // set the parameters
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
    this->get_parameter("base_latitude", this->base_latitude_deg_);
    this->get_parameter("base_longtitude", this->base_longitude_deg_);
    this->get_parameter("base_altitude", this->base_altitude_m_);

    // print the parameters
    RCLCPP_INFO(this->get_logger(), "Device name: %s", this->device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", this->baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Timeout (ms): %d", this->timeout_ms_);
    RCLCPP_INFO(this->get_logger(), "Local frame id: %s", this->local_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Local topic name: %s", this->local_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Local publish rate: %lf", this->local_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "IMU frame id: %s", this->imu_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU topic name: %s", this->imu_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU publish rate: %lf", this->imu_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "GNSS pose ENU frame id: %s", this->gnss_pose_enu_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "GNSS pose ENU topic name: %s", this->gnss_pose_enu_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "GNSS pose ENU publish rate: %lf", this->gnss_pose_enu_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "Enable debug log: %d", this->enable_debug_log_);
    RCLCPP_INFO(this->get_logger(), "Base latitude: %lf", this->base_latitude_deg_);
    RCLCPP_INFO(this->get_logger(), "Base longitude: %lf", this->base_longitude_deg_);
    RCLCPP_INFO(this->get_logger(), "Base altitude: %lf", this->base_altitude_m_);
    return;
}

/**
 * @brief initialize the device
 *
 * @return true
 * @return false
 */
bool RTKNode::DeviceInit() {
    // initialize device here

    // 串口设置
    // serial_port_.setPort(device_name_);
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

void RTKNode::InitValues() {
    // initialize values here
    base_point_set_ = false;
    running_ = false;
    read_thread_ = nullptr;
}
} // namespace rtk

// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rtk::RTKNode>();
    RCLCPP_INFO(node->get_logger(), "rtk node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}