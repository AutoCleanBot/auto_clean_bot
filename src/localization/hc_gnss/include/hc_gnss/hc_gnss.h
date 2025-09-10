#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>

namespace hc_gnss {

// GPCHC 数据结构：$GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,gyro x,gyro y,gyroz,acc x,accy,accz,Latitude,Longitude,Altitude,Ve,Vn,Vu,V,NS1,NS2,Status,Age,Warming,Cs<CR><LF>
struct Gpchc {
    int gps_week;           // GPS周
    double gps_time;        // GPS时间（秒）
    double heading_deg;     // 航向角（度）
    double pitch_deg;       // 俯仰角（度）
    double roll_deg;        // 横滚角（度）
    double gyro_x_deg_s;    // X轴角速度（度/秒）
    double gyro_y_deg_s;    // Y轴角速度（度/秒）
    double gyro_z_deg_s;    // Z轴角速度（度/秒）
    double acc_x_m_s2;      // X轴加速度（米/秒²）
    double acc_y_m_s2;      // Y轴加速度（米/秒²）
    double acc_z_m_s2;      // Z轴加速度（米/秒²）
    double latitude_deg;    // 纬度（度）
    double longitude_deg;   // 经度（度）
    double altitude_m;      // 高度（米）
    double ve_m_s;          // 东向速度（米/秒）
    double vn_m_s;          // 北向速度（米/秒）
    double vu_m_s;          // 上向速度（米/秒）
    double v_m_s;           // 速度模长（米/秒）
    int ns1;                // 卫星数1
    int ns2;                // 卫星数2
    int status;             // 状态
    double age;             // 差分龄期
    int warming;            // 警告
    int checksum;           // 校验和
};

class HCGNSSNode : public rclcpp::Node {
  public:
    HCGNSSNode();
    ~HCGNSSNode();

  private:
    rclcpp::TimerBase::SharedPtr imu_timer_, local_timer_, gnss_timer_;
    void LocalTimerCallback();
    void ImuTimerCallback();
    void GNSSTimerCallback();
    void InitParams();
    void InitValues();
    void InfoReadLoop();
    bool DeviceInit();
    void ParseGPCHCInfo(const std::string &info_str);
    void WGS84toENU(const Gpchc &gpchc);
    void InitInfoStrSaving();
    void SaveInfoStrToFile(const std::string &info_str);
    std::string GenerateTimestampFilename();

    // ROS2 publishers
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_enu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

    // ROS2 messages
    bot_msg::msg::LocalizationInfo gnss_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    geometry_msgs::msg::PoseStamped gnss_pose_enu_msg_;

    // ROS2 parameters
    std::string device_name_;
    int baud_rate_;
    int timeout_ms_;

    std::string local_frame_id_;
    std::string local_topic_name_;
    double local_publish_rate_;

    std::string imu_frame_id_;
    std::string imu_topic_name_;
    double imu_publish_rate_;

    std::string gnss_pose_enu_frame_id_;
    std::string gnss_pose_enu_topic_name_;
    double gnss_pose_enu_publish_rate_;

    // 串口通信
    int sockfd_;

    // 运行控制
    bool running_;
    std::shared_ptr<std::thread> read_thread_;
    
    // 基准点设置
    bool base_point_set_;
    double base_latitude_deg_;
    double base_longitude_deg_;
    double base_altitude_m_;
    double heading_offset_;

    // 调试日志控制
    bool enable_debug_log_;
    int log_interval_;
    mutable int parse_count_;
    mutable int error_count_;
    mutable std::chrono::steady_clock::time_point last_log_time_;
    
    // info_str 文件保存相关
    bool enable_info_str_save_;
    std::string info_str_save_dir_;
    mutable int info_str_save_count_;
    mutable std::string current_log_filename_;
    mutable std::ofstream info_str_file_;
};

} // namespace hc_gnss