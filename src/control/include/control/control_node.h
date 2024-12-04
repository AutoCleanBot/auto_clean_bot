#pragma once

#include <bot_msg/msg/adc_trajectory.hpp>
#include <bot_msg/msg/control_cmd.hpp>
# include <bot_msg/msg/control_log.hpp>
#include <bot_msg/msg/localization_info.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control {
class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void InitParams();
    void TimerCallback();
    void ADCTrajectoryCallback(const bot_msg::msg::ADCTrajectory::SharedPtr msg);
    void LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
    void LateralController();
    void LongitudinalController();
    void trajectory_callback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
 
  private:
    // subscribers and publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<bot_msg::msg::ADCTrajectory>::SharedPtr sub_adc_trajectory_;
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_;
    rclcpp::Publisher<bot_msg::msg::ControlCmd>::SharedPtr pub_control_cmd_;
    //rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_;

    // variables
    bot_msg::msg::ADCTrajectory::SharedPtr adc_trajectory_msg_;
    bot_msg::msg::LocalizationInfo::SharedPtr localization_info_msg_;
    bot_msg::msg::ControlCmd control_cmd_msg_;

    // parameters
    double publish_rate_;       // in milliseconds
    double preview_time_;       // in seconds
    double tolerance_distance_; // in meters, for preview point
    double max_steering_angle_; // in degrees
    double wheelbase_;          // 轴距

    double max_linear_velocity_; // in m/s
    double min_linear_velocity_; // in m/s
    double acceleration_limit_;  // in m/s^2
    double deceleration_limit_;  // in m/s^2
    // config variables
    std::string adc_traj_topic_name_;
    std::string localization_info_topic_name_;
    std::string control_cmd_topic_name_;
};
} // namespace control