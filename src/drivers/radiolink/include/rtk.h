#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <bot_msg/msg/radio_link.hpp>
#include <chrono>
#include <fstream>
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
namespace radio {
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

class RadioNode : public rclcpp::Node {
  public:
    RadioNode();
    ~RadioNode();

  private:
    rclcpp::TimerBase::SharedPtr imu_timer_, radio_timer_, gnss_timer_;
    void RadioTimerCallback();
    void ImuTimerCallback();
    void GNSSTimerCallback();
    void InitParams();
    void InitValues();
    void InfoReadLoop();
    bool DeviceInit();
    void ParseRTKInfo(const std::string &info_str);
    void WGS84toENU(const Giavp &giavp);
    void InitInfoStrSaving();
    void SaveInfoStrToFile(const std::string &info_str);
    std::string GenerateTimestampFilename();

    // ROS2 publishers and subscribers
    rclcpp::Publisher<bot_msg::msg::RadioLink>::SharedPtr pub_radio_info_;
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_enu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

    bot_msg::msg::RadioLink radio_msg_; // radioization info message
    bot_msg::msg::LocalizationInfo rtk_msg_; // radioization info message
    sensor_msgs::msg::Imu imu_msg_;
    geometry_msgs::msg::PoseStamped gnss_pose_enu_msg_;
    // ROS2 parameters
    std::string device_name_;
    int baud_rate_;  // baud rate for serial communication
    int timeout_ms_; // timeout for serial communication

    std::string radio_frame_id_;   // frame id for radioization info
    std::string radio_topic_name_; // topic name for publishing radioization info
    double radio_publish_rate_;    // publish rate for radioization info

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
    double heading_offset_;                    // the offset of heading (degrees)

    bool enable_debug_log_; // enable debug log

    // 日志频率控制
    int log_interval_;                                            // 日志输出间隔（每N次解析输出一次）
    mutable int parse_count_;                                     // 解析计数器
    mutable int error_count_;                                     // 错误计数器
    mutable std::chrono::steady_clock::time_point last_log_time_; // 上次日志输出时间
    
    // info_str 文件保存相关
    bool enable_info_str_save_;                                   // 是否启用info_str保存功能
    std::string info_str_save_dir_;                              // info_str保存目录
    mutable int info_str_save_count_;                            // info_str保存计数器（用于统计）
    mutable std::string current_log_filename_;                   // 当前日志文件名
    mutable std::ofstream info_str_file_;                        // 文件输出流
};
} // namespace rtk