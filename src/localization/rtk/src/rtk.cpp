#include "rtk/rtk.h"
#include <chrono>
#include <cstdio>
#include <string>
namespace rtk {
RTKNode::RTKNode() : Node("rtk_node") {
    // constructor
    InitParams();
    bool ret = DeviceInit();
    if (ret) {
        running_ = true;
        read_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&RTKNode::InfoReadLoop, this)));
        read_thread_->detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize device");
    }
}

/**
 * @brief parse the RTK info string
 *
 * @param info_str
 */
void RTKNode::ParseRTKInfo(const std::string &info_str) {
    char reserved[20];
    Giavp giavp;
    sscanf(info_str.c_str(),
           "$GIAVP,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf*%s",
           &giavp.week, &giavp.time_sec, &giavp.heading_deg, &giavp.pitch_deg, &giavp.roll_deg, &giavp.latitude_deg,
           &giavp.longitude_deg, &giavp.altitude_m, &giavp.ve_m_s, &giavp.vn_m_s, &giavp.vd_m_s, &giavp.nvsv1,
           &giavp.nvsv2, &giavp.status, &giavp.speed_status, &giavp.vehicle_speed_m_s, &giavp.acc_x_m_s2,
           &giavp.acc_y_m_s2, &giavp.acc_z_m_s2, &giavp.gyro_x_deg_s, &giavp.gyro_y_deg_s, &giavp.gyro_z_deg_s,
           reserved);
    // TODO: set localization info
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
                RCLCPP_INFO(this->get_logger(), "Received a full packet: %s", info_str.c_str());
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

    // set the parameters
    this->get_parameter("device_name", this->device_name_);
    this->get_parameter("baud_rate", this->baud_rate_);
    this->get_parameter("timeout_ms", this->timeout_ms_);

    RCLCPP_INFO(this->get_logger(), "Device name: %s", this->device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", this->baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Timeout (ms): %d", this->timeout_ms_);
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