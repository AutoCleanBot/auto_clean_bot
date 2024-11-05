#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h> // for serial port communication
#include <string>
#include <thread>

namespace rtk {

struct Giavp {
    int week;
    double time_sec;
    double heading_deg;
    double pitch_deg;
    double roll_deg;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    double ve_m_s;
    double vn_m_s;
    double vu_m_s;
    double baseline;
    int nvsv1;
    int nvsv2;
    int status;
    double speed_status;
    double vehicle_speed_m_s;
    double acc_x_m_s2;
    double acc_y_m_s2;
    double acc_z_m_s2;
    double gyro_x_deg_s;
    double gyro_y_deg_s;
    double gyro_z_deg_s;
};

class RTKNode : public rclcpp::Node {
  public:
    RTKNode();

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback();
    void InitParams();
    void InitValues();
    void InfoReadLoop();
    bool DeviceInit();
    void ParseRTKInfo(const std::string &info_str);
    void WGS84toENU(const Giavp &giavp);

    // ROS2 publishers and subscribers
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;
    bot_msg::msg::LocalizationInfo rtk_msg_; // localization info message
    // ROS2 parameters
    std::string device_name_;
    int baud_rate_;          // baud rate for serial communication
    int timeout_ms_;         // timeout for serial communication
    std::string frame_id_;   // frame id for localization info
    std::string topic_name_; // topic name for publishing localization info
    double publish_rate_;    // publish rate for localization info
    // serial port communication
    serial::Serial serial_port_; // serial port object

    bool running_;                             // running flag for read loop
    std::shared_ptr<std::thread> read_thread_; // read loop thread
    bool base_point_set_;                      // flag for base point set
    double base_latitude_deg_;                 // base point latitude (degrees)
    double base_longitude_deg_;                // base point longitude (degrees)
    double base_altitude_m_;                   // base point altitude (meters)
    bool enable_debug_log_;                    // enable debug log
};
} // namespace rtk