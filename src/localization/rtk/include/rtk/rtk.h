#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h> // for serial port communication
#include <string>
#include <thread>

namespace rtk {

struct Giavp {                // GIAVP data structure
    int week;                 // GPS week number
    double time_sec;          // GPS time (seconds of week)
    double heading_deg;       // heading (degrees)
    double pitch_deg;         // pitch (degrees)
    double roll_deg;          // roll (degrees)
    double latitude_deg;      // latitude (degrees)
    double longitude_deg;     // longitude (degrees)
    double altitude_m;        // altitude (meters)
    double ve_m_s;            // velocity east (meters/second)
    double vn_m_s;            // velocity north (meters/second)
    double vd_m_s;            // velocity down (meters/second)
    int nvsv1;                // GPS satellite number
    int nvsv2;                // GPS satellite number
    int status;               // rtk status
    double speed_status;      // vehicle speed status
    double vehicle_speed_m_s; // vehicle speed (meters/second)
    double acc_x_m_s2;        // acceleration x (meters/second^2)
    double acc_y_m_s2;        // acceleration y (meters/second^2)
    double acc_z_m_s2;        // acceleration z (meters/second^2)
    double gyro_x_deg_s;      // gyro x (degrees/second)
    double gyro_y_deg_s;      // gyro y (degrees/second)
    double gyro_z_deg_s;      // gyro z (degrees/second)
    Giavp()
        : week(0), time_sec(0), heading_deg(0), pitch_deg(0), roll_deg(0), latitude_deg(0), longitude_deg(0),
          altitude_m(0), ve_m_s(0), vn_m_s(0), vd_m_s(0), nvsv1(0), nvsv2(0), status(0), speed_status(0),
          vehicle_speed_m_s(0), acc_x_m_s2(0), acc_y_m_s2(0), acc_z_m_s2(0), gyro_x_deg_s(0), gyro_y_deg_s(0),
          gyro_z_deg_s(0) {}
};

class RTKNode : public rclcpp::Node {
  public:
    RTKNode();

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback();
    void InitParams();
    void InfoReadLoop();
    bool DeviceInit();
    void ParseRTKInfo(const std::string &info_str);

    // ROS2 publishers and subscribers
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;
    bot_msg::msg::LocalizationInfo rtk_msg_;
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