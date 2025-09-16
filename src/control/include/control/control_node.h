#pragma once

#include "control/pid_controller.h"
#include <bot_msg/msg/adc_trajectory.hpp>
#include <bot_msg/msg/chassis_info.hpp>
#include <bot_msg/msg/control_cmd.hpp>
#include <bot_msg/msg/localization_info.hpp>
#include <deque>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace control {
class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    ~ControlNode();
    void InitParams();
    void TimerCallback();
    void ADCTrajectoryCallback(const bot_msg::msg::ADCTrajectory::SharedPtr msg);
    void LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
    void ChassisInfoCallback(const bot_msg::msg::ChassisInfo::SharedPtr msg);
    void LateralController();
    void LongitudinalController();
    double SmoothSpeedCommand(double raw_speed_command);
    double CalculatePathCurvature(size_t index);
    double CalculateAdaptivePreviewDistance(double current_speed, double path_curvature);
    double CalculateAdaptiveHeadingErrorRate(double current_speed);
    double SmoothSteeringAngle(double target_angle, double dt);

  private:
    // subscribers and publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<bot_msg::msg::ADCTrajectory>::SharedPtr sub_adc_trajectory_;
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_;
    rclcpp::Subscription<bot_msg::msg::ChassisInfo>::SharedPtr sub_chassis_info_;
    rclcpp::Publisher<bot_msg::msg::ControlCmd>::SharedPtr pub_control_cmd_;

    // variables
    bot_msg::msg::ADCTrajectory::SharedPtr adc_trajectory_msg_;
    bot_msg::msg::LocalizationInfo::SharedPtr localization_info_msg_;
    bot_msg::msg::ControlCmd control_cmd_msg_;
    bot_msg::msg::ChassisInfo chassis_info_msg_;

    // parameters
    double publish_rate_;       // in milliseconds
    double preview_time_;       // in seconds, 预瞄时间
    double tolerance_distance_; // in meters, 预瞄距离
    double max_steering_angle_; // in degrees, 最大转向角度
    double wheelbase_;          // 轴距

    double max_linear_velocity_;  // in m/s
    double min_linear_velocity_;  // in m/s
    double acceleration_limit_;   // in m/s^2
    double deceleration_limit_;   // in m/s^2
    double pursuit_control_rate_; // 纯追踪控制比例
    double stanley_control_rate_; // Stanley控制比例
    double sta_lat_rate_;         // stanley控制中的横向偏差系数, 在低速情况下应该加大该算法的系数
    double feedforward_rate_;     // 前馈控制比例
    double heading_error_rate_;   // 航向误差比例
    double dec_step_size_;        // 减速步长
    double inc_step_size_;        // 加速步长
    double turning_radius_ratio_; // 方向盘转角与前轮转角的比例
    double zero_point_draft_;     // 零点漂移
    double lat_error_threshold_;  // 横向偏差阈值

    // config variables
    std::string adc_traj_topic_name_;
    std::string localization_info_topic_name_;
    std::string control_cmd_topic_name_;
    std::string chassis_info_topic_name_;
    std::string log_file_path_;

    // 运行中的信息
    size_t closest_idx_; // 当前车辆到轨迹上的最近点

    std::ofstream debug_log_file_;

    // PID控制器参数
    double speed_pid_kp_;
    double speed_pid_ki_;
    double speed_pid_kd_;
    double speed_pid_kf_;

    // PID控制器
    std::unique_ptr<PIDController> speed_pid_controller_;

    // 上一次控制的时间戳
    rclcpp::Time last_control_time_;

    // 速度平滑相关参数
    double previous_speed_command_;            // 上一次的速度命令
    double max_speed_change_rate_;             // 最大速度变化率 (m/s^2)
    int smooth_window_size_;                   // 平滑窗口大小
    std::deque<double> speed_commands_buffer_; // 速度命令缓存
    double max_steering_rate_;                 // 最大转向角速度 (度/秒)
    double previous_steering_angle_;           // 上一次的转向角
    double stanley_min_eff_spd_;               // stanley控制中的最小有效速度
    bool reverse_mode_;
    double path_curvature_;                    // 
  };
} // namespace control