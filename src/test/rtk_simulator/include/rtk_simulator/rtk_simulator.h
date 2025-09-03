#pragma once

#include <bot_msg/msg/localization_info.hpp>
#include <bot_msg/msg/adc_trajectory.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

namespace rtk_simulator {

struct TrajectoryPoint {
    double longtitude;
    double latitude;
    double altitude;
    double north;
    double east;
    double up;
    double yaw;
    double pitch;
    double roll;
    double vel_speed;
    double vel_north;
    double vel_east;
    double vel_up;
    double acc_x;
    double acc_y;
    double acc_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    int rtk_status;
};

class RTKSimulator : public rclcpp::Node {
  public:
    RTKSimulator();
    ~RTKSimulator();

  private:
    void initParams();
    void timerCallback();
    void trajectoryCallback(const bot_msg::msg::ADCTrajectory::SharedPtr msg);
    void handlePlanningTrajectory();
    bool loadTrajectoryFromCSV(const std::string &file_path);
    void addRandomNoise(bot_msg::msg::LocalizationInfo &msg);
    double generateRandomOffset(double base_value, double noise_percentage);

    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_;
    rclcpp::Subscription<bot_msg::msg::ADCTrajectory>::SharedPtr sub_trajectory_;

    // Parameters
    std::string csv_file_path_;
    double publish_frequency_;
    double noise_min_percentage_;
    double noise_max_percentage_;
    bool loop_trajectory_;
    std::string gnss_topic_name_;
    std::string gnss_frame_id_;

    // Trajectory data
    std::vector<TrajectoryPoint> trajectory_points_;
    size_t current_point_index_;
    
    // Planning trajectory data
    bot_msg::msg::ADCTrajectory::SharedPtr current_trajectory_;
    size_t current_trajectory_index_;
    bool use_planning_trajectory_;
    bool waiting_for_nonzero_velocity_;

    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> noise_dist_;
};

} // namespace rtk_simulator
