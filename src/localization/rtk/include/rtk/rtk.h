#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <serial/serial.h> // for serial port communication
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    rclcpp::TimerBase::SharedPtr imu_timer_, local_timer_, gnss_timer_;
    void LocalTimerCallback();
    void ImuTimerCallback();
    void GNSSTimerCallback();
    void InitParams();
    void InitValues();
    void InfoReadLoop();
    bool DeviceInit();
    void ParseRTKInfo(const std::string &info_str);
    void WGS84toENU(const Giavp &giavp);

    // ROS2 publishers and subscribers
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_enu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

    bot_msg::msg::LocalizationInfo rtk_msg_; // localization info message
    sensor_msgs::msg::Imu imu_msg_;
    geometry_msgs::msg::PoseStamped gnss_pose_enu_msg_;
    // ROS2 parameters
    std::string device_name_;
    int baud_rate_;  // baud rate for serial communication
    int timeout_ms_; // timeout for serial communication

    std::string local_frame_id_;   // frame id for localization info
    std::string local_topic_name_; // topic name for publishing localization info
    double local_publish_rate_;    // publish rate for localization info

    std::string imu_frame_id_;   // frame id for imu
    std::string imu_topic_name_; // topic name for publishing imu
    double imu_publish_rate_;    // publish rate for imu

    std::string gnss_pose_enu_frame_id_;   // frame id for gnss pose enu
    std::string gnss_pose_enu_topic_name_; // topic name for publishing gnss pose enu
    double gnss_pose_enu_publish_rate_;    // publish rate for gnss pose enu

    // serial port communication
    int sockfd_;

    bool running_;                             // running flag for read loop
    std::shared_ptr<std::thread> read_thread_; // read loop thread
    bool base_point_set_;                      // flag for base point set
    double base_latitude_deg_;                 // base point latitude (degrees)
    double base_longitude_deg_;                // base point longitude (degrees)
    double base_altitude_m_;                   // base point altitude (meters)

    bool enable_debug_log_; // enable debug log
};
} // namespace rtk