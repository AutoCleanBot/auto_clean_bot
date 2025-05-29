#pragma once

#include <bot_msg/msg/adc_trajectory.hpp>
#include <bot_msg/msg/control_cmd.hpp>
#include <bot_msg/msg/localization_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include "control/pid_controller.h"

namespace control {
class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    ~ControlNode();
    void InitParams();
    void TimerCallback();
    void ADCTrajectoryCallback(const bot_msg::msg::ADCTrajectory::SharedPtr msg);
    void LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
    void LateralController();
    void LongitudinalController();

  private:
    // subscribers and publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<bot_msg::msg::ADCTrajectory>::SharedPtr sub_adc_trajectory_;
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_;
    rclcpp::Publisher<bot_msg::msg::ControlCmd>::SharedPtr pub_control_cmd_;

    // variables
    bot_msg::msg::ADCTrajectory::SharedPtr adc_trajectory_msg_;
    bot_msg::msg::LocalizationInfo::SharedPtr localization_info_msg_;
    bot_msg::msg::ControlCmd control_cmd_msg_;

    // parameters
    double publish_rate_;       // in milliseconds
    double preview_time_;       // in seconds, 预瞄时间
    double tolerance_distance_; // in meters, 预瞄距离
    double max_steering_angle_; // in degrees, 最大转向角度
    double wheelbase_;          // 轴距

    double max_linear_velocity_; // in m/s
    double min_linear_velocity_; // in m/s
    double acceleration_limit_;  // in m/s^2
    double deceleration_limit_;  // in m/s^2
    double pursuit_control_rate_; // 纯追踪控制比例
    double stanley_control_rate_; // Stanley控制比例
    double sta_lat_rate_;         // stanley控制中的横向偏差系数, 在低速情况下应该加大该算法的系数
    double turning_radius_ratio_;  // 方向盘转角与前轮转角的比例
    double zero_point_draft_;      // 零点漂移
    // config variables
    std::string adc_traj_topic_name_;
    std::string localization_info_topic_name_;
    std::string control_cmd_topic_name_;
    std::string log_file_path_;

    // 运行中的信息
    size_t closest_idx_; // 当前车辆到轨迹上的最近点

    std::ofstream debug_log_file_;

    // PID控制器参数
    double speed_pid_kp_;
    double speed_pid_ki_;
    double speed_pid_kd_;
    
    // PID控制器
    std::unique_ptr<PIDController> speed_pid_controller_;

    // 上一次控制的时间戳
    rclcpp::Time last_control_time_;
};
} // namespace control