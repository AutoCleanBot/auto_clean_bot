#include "rtk/rtk.h"
#include <chrono>
#include <cstdio>
#include <string>
namespace rtk {
RTKNode::RTKNode() : Node("rtk_node") {
    // constructor
    InitParams();
    InitValues();
    bool ret = DeviceInit();
    if (ret) {
        running_ = true;
        read_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&RTKNode::InfoReadLoop, this)));
        read_thread_->detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize device");
    }

    // publishers and subscribers
    int time_interval = static_cast<int>(1000.0 / publish_rate_);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(time_interval), std::bind(&RTKNode::TimerCallback, this));
    pub_localization_info_ = this->create_publisher<bot_msg::msg::LocalizationInfo>(topic_name_, 50);
}

void RTKNode::TimerCallback() {
    if (running_ && base_point_set_) {
        pub_localization_info_->publish(rtk_msg_);
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
    unsigned int checksum; // 用于保存校验和值
    int parsed_fields = sscanf(
        info_str.c_str(),
        "$GIAVP,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &giavp.week,
        &giavp.time_sec, &giavp.heading_deg, &giavp.pitch_deg, &giavp.roll_deg, &giavp.latitude_deg,
        &giavp.longitude_deg, &giavp.altitude_m, &giavp.ve_m_s, &giavp.vn_m_s, &giavp.vu_m_s, &giavp.baseline,
        &giavp.nvsv1, &giavp.nvsv2, &giavp.status, &giavp.speed_status, &giavp.vehicle_speed_m_s, &giavp.acc_x_m_s2,
        &giavp.acc_y_m_s2, &giavp.acc_z_m_s2, &giavp.gyro_x_deg_s, &giavp.gyro_y_deg_s, &giavp.gyro_z_deg_s);
    if (enable_debug_log_) {
        RCLCPP_INFO(this->get_logger(), "Received a full packet: %s", info_str.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Received RTK info: "
                    "$GIAVP,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                    giavp.week, giavp.time_sec, giavp.heading_deg, giavp.pitch_deg, giavp.roll_deg, giavp.latitude_deg,
                    giavp.longitude_deg, giavp.altitude_m, giavp.ve_m_s, giavp.vn_m_s, giavp.vu_m_s, giavp.baseline,
                    giavp.nvsv1, giavp.nvsv2, giavp.status, giavp.speed_status, giavp.vehicle_speed_m_s,
                    giavp.acc_x_m_s2, giavp.acc_y_m_s2, giavp.acc_z_m_s2, giavp.gyro_x_deg_s, giavp.gyro_y_deg_s,
                    giavp.gyro_z_deg_s);
    }
    // simply publish the parsed RTK info
    rtk_msg_.header.stamp = this->get_clock()->now();
    rtk_msg_.header.frame_id = this->frame_id_;
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
    rtk_msg_.acc_x = giavp.acc_x_m_s2;
    rtk_msg_.acc_y = giavp.acc_y_m_s2;
    rtk_msg_.acc_z = giavp.acc_z_m_s2;
    rtk_msg_.gyro_x = giavp.gyro_x_deg_s;
    rtk_msg_.gyro_y = giavp.gyro_y_deg_s;
    rtk_msg_.gyro_z = giavp.gyro_z_deg_s;
    // set the base point
    if (!base_point_set_ && giavp.status >= 1) {
        base_point_set_ = true;
        base_latitude_deg_ = giavp.latitude_deg;
        base_longitude_deg_ = giavp.longitude_deg;
        base_altitude_m_ = giavp.altitude_m;
        RCLCPP_INFO(this->get_logger(), "Base point set: %lf, %lf, %lf", base_latitude_deg_, base_longitude_deg_,
                    base_altitude_m_);
    }
    // convert GPS coordinates to ENU coordinates
    WGS84toENU(giavp);
    return;
}

void RTKNode::InfoReadLoop() {
    std::string data;
    std::string gstart = "$GIAVP"; // 开头
    std::string gend = "\r\n";
    while (running_) {
        if (serial_port_.available() > 0) { // 当缓冲区有数据时
            data += serial_port_.readline();
            // RCLCPP_INFO(this->get_logger(), "Received data: %s", data.c_str());
            auto start_pos = data.find(gstart);
            auto end_pos = data.find(gend);
            if (start_pos != std::string::npos && end_pos != std::string::npos) {
                std::string info_str = data.substr(start_pos, end_pos - start_pos + gend.length());
                ParseRTKInfo(info_str);
                data.erase(0, end_pos + gend.length());
            } else {
                RCLCPP_WARN(this->get_logger(), "Incomplete packet received: %s", data.c_str());
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    this->declare_parameter<std::string>("topic_name", "rtk_fix");
    this->declare_parameter<std::string>("frame_id", "rtk_frame");
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<bool>("enable_debug_log", false);

    // set the parameters
    this->get_parameter("device_name", this->device_name_);
    this->get_parameter("baud_rate", this->baud_rate_);
    this->get_parameter("timeout_ms", this->timeout_ms_);
    this->get_parameter("topic_name", this->topic_name_);
    this->get_parameter("frame_id", this->frame_id_);
    this->get_parameter("publish_rate", this->publish_rate_);
    this->get_parameter("enable_debug_log", this->enable_debug_log_);

    // print the parameters
    RCLCPP_INFO(this->get_logger(), "Device name: %s", this->device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", this->baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Timeout (ms): %d", this->timeout_ms_);
    RCLCPP_INFO(this->get_logger(), "Topic name: %s", this->topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", this->frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publish Rate: %f", this->publish_rate_);
}

/**
 * @brief initialize the device
 *
 * @return true
 * @return false
 */
bool RTKNode::DeviceInit() {
    // initialize device here
    try {
        // 串口设置
        serial_port_.setPort(device_name_);
        serial_port_.setBaudrate(baud_rate_);
        // 设置超时时间
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms_);
        serial_port_.setTimeout(to);
        serial_port_.open();
    } catch (serial::IOException &e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
        return false;
    }
    if (serial_port_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        return false;
    }
    return true;
}

void RTKNode::InitValues() {
    // initialize values here
    base_point_set_ = false;
    running_ = false;
    read_thread_ = nullptr;
    base_latitude_deg_ = 0.0;
    base_longitude_deg_ = 0.0;
    base_altitude_m_ = 0.0;
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