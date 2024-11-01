#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h> // for serial port communication
#include <string>
#include <thread>

namespace rtk {
class RTKNode : public rclcpp::Node {
  public:
    RTKNode();

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback();
    void InitParams();
    void InfoReadLoop();
    bool DeviceInit();

    // ROS2 publishers and subscribers
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;
    // ROS2 parameters
    std::string device_name_;
    int baud_rate_;
    int timeout_ms_;
    // serial port communication
    serial::Serial serial_port_;
    // running flag
    bool running_;
    std::shared_ptr<std::thread> read_thread_;
};
} // namespace rtk