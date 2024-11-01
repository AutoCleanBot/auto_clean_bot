#include "rtk/rtk.h"
#include <string>
namespace rtk {

RTKNode::RTKNode() {
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

void RTKNode::InfoReadLoop() {
    std::string data;
    while (running_) {
        if (serial_port_.available() > 0) {
            data += serial_port_.readline();
            RCLCPP_INFO(this->get_logger(), "Received data: %s", data.c_str());
            std::string gstart = "$GIAVP"; // 开头
            std::string gend = "\r\n";
            int i = 0, start = -1, end = -1;
            double lat, lon, hgt, vel, yaw, pitch;
            // TODO: 解析数据
        } else {
            this_thread::sleep_for(chrono::milliseconds(1));
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