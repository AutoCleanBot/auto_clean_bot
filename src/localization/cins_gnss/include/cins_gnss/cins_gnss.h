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
#include <vector>
#include <cstdint>

namespace cins_gnss {

// 城芯智联PBOX数据结构（基于协议定义）
struct PboxData {
    uint16_t header;           // 固定头0xAA55
    uint16_t length;           // 数据长度
    uint64_t utc_stamp;        // UTC时间戳 (ms)
    uint8_t ins_flag;          // INS数据有效标志
    
    // IMU陀螺仪数据 (d/s)
    float ang_rate_raw_x;
    float ang_rate_raw_y;
    float ang_rate_raw_z;
    
    // IMU加速度计数据 (g)
    float accel_raw_x;
    float accel_raw_y;
    float accel_raw_z;
    
    uint8_t ins_state;         // INS状态
    uint8_t ins_convergence;   // INS收敛状态
    uint8_t fix_status;        // 差分定位状态
    uint8_t sat_num;           // 卫星数
    float fix_age;             // 差分龄期 (s)
    
    // 位置信息
    double pos_lat;            // 纬度 (°)
    double pos_lon;            // 经度 (°)
    float pos_alt;             // 大地高 (m)
    
    // 位置标准差
    float pos_e_sigma;         // 东向位置标准差 (m)
    float pos_n_sigma;         // 北向位置标准差 (m)
    float pos_u_sigma;         // 天向位置标准差 (m)
    
    // 速度信息 (m/s)
    float vel;                 // 速度
    float vel_sigma;           // 速度标准差
    
    // 姿态信息 (°)
    float angle_heading;       // 航向角
    float angle_pitch;         // 俯仰角
    float angle_roll;          // 横滚角
    
    // 姿态标准差 (°)
    float angle_heading_sigma;
    float angle_pitch_sigma;
    float angle_roll_sigma;
    
    uint8_t xor_checksum;      // 异或校验值
};

class CinsGnssNode : public rclcpp::Node {
  public:
    CinsGnssNode();
    ~CinsGnssNode();

  private:
    rclcpp::TimerBase::SharedPtr imu_timer_, local_timer_, gnss_timer_;
    void LocalTimerCallback();
    void ImuTimerCallback();
    void GNSSTimerCallback();
    void InitParams();
    void InitValues();
    void DataReadLoop();
    bool DeviceInit();
    void ParsePboxData(const std::vector<uint8_t> &data_buffer);
    void WGS84toENU(const PboxData &pbox_data);
    void InitDataSaving();
    void SaveDataToFile(const std::vector<uint8_t> &data_buffer);
    std::string GenerateTimestampFilename();
    bool VerifyChecksum(const std::vector<uint8_t> &data_buffer);

    // ROS2 publishers
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_enu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

    // 消息对象
    bot_msg::msg::LocalizationInfo cins_msg_;     // 定位信息消息
    sensor_msgs::msg::Imu imu_msg_;               // IMU消息
    geometry_msgs::msg::PoseStamped gnss_pose_enu_msg_; // GNSS姿态消息

    // ROS2 参数
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

    bool running_;
    std::shared_ptr<std::thread> read_thread_;
    bool base_point_set_;
    double base_latitude_deg_;
    double base_longitude_deg_;
    double base_altitude_m_;
    double heading_offset_;

    bool enable_debug_log_;
    int log_interval_;
    mutable int parse_count_;
    mutable int error_count_;
    mutable std::chrono::steady_clock::time_point last_log_time_;
    
    // 数据保存相关
    bool enable_data_save_;
    std::string data_save_dir_;
    mutable int data_save_count_;
    mutable std::string current_log_filename_;
    mutable std::ofstream data_file_;

    // 最新解析的数据
    PboxData latest_pbox_data_;
    bool data_valid_;
};

} // namespace cins_gnss