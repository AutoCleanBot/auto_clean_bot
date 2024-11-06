#include "control/control_node.h"

namespace control {
ControlNode::ControlNode() : Node("control_node") {
    // Initialize subscribers and publishers
    InitParams();
    this->adc_traj_sub_ = this->create_subscription<ADCTrajectory>(
        this->adc_traj_topic_name_, 10, std::bind(&ControlNode::ADCTrajCallback, this, std::placeholders::_1));
    this->localization_info_sub_ = this->create_subscription<LocalizationInfo>(
        this->localization_info_topic_name_, 10,
        std::bind(&ControlNode::LocalizationInfoCallback, this, std::placeholders::_1));
    this->control_cmd_pub_ = this->create_publisher<ControlCommand>(this->control_cmd_topic_name_, 10);

    double control_cycle_time = 1000.0 / this->publish_rate_;
    this->timer_ = this->create_wall_timer(control_cycle_time, std::bind(&ControlNode::TimerCallback, this));
}

void ControlNode::LateralController() {
    double cur_north = localization_info_msg_->north;
    double cur_east = localization_info_msg_->east;
    double cur_up = localization_info_msg_->up;
    double cur_spd = localization_info_msg_->vel_speed;
    double cur_yaw = localization_info_msg_->yaw;

    // 1. 找到当前车辆位置到轨迹上的最近点
    double min_dist = 1000000.0;
    int closest_idx = -1;
    for (int i = 0; i < adc_trajectory_msg_->points.size(); i++) {
        double dist = std::sqrt(std::pow(cur_north - adc_trajectory_msg_->points[i].north, 2) +
                                std::pow(cur_east - adc_trajectory_msg_->points[i].east, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    if (closest_idx == -1) {
        RCLCPP_ERROR(this->get_logger(), "Closest index is -1");
        return;
    }

    // 2. 找到轨迹上的预瞄点
    int preview_idx = closest_idx + 1;
    if (preview_idx >= adc_trajectory_msg_->points.size()) {
        preview_idx = closest_idx;
        RCLCPP_ERROR(this->get_logger(), "Preview index is out of range");
    } else {
        double preview_dist = tolerance_distance_ + preview_time_ * cur_spd;
        double dist = std::sqrt(std::pow(cur_north - adc_trajectory_msg_->points[preview_idx].north, 2) +
                                std::pow(cur_east - adc_trajectory_msg_->points[preview_idx].east, 2));
        while (dist < preview_dist) {
            preview_idx++;
            if (preview_idx >= adc_trajectory_msg_->points.size()) {
                preview_idx = closest_idx;
                RCLCPP_ERROR(this->get_logger(), "Preview index is out of range");
                break;
            }
            dist = std::sqrt(std::pow(cur_north - adc_trajectory_msg_->points[preview_idx].north, 2) +
                             std::pow(cur_east - adc_trajectory_msg_->points[preview_idx].east, 2));
        }
    }

    // 3. 计算横向控制命令
    double target_north = adc_trajectory_msg_->points[preview_idx].north;
    double target_east = adc_trajectory_msg_->points[preview_idx].east;
    double target_yaw = adc_trajectory_msg_->points[preview_idx].pose.orientation.z; // 目标航向角

    // 3.1 计算航向误差和方位角误差
    double heading_error = deg2rad(target_yaw - cur_yaw);                                          // 航向误差
    double angular_error = std::atan2(target_east - cur_east, target_north - cur_north) - cur_yaw; // 方位角误差

    // 限制角度在 [-PI, PI] 范围内，避免跳变
    if (angular_error > M_PI)
        angular_error -= 2 * M_PI;
    if (angular_error < -M_PI)
        angular_error += 2 * M_PI;

    // 3.2 计算横向误差
    int line_start = std::max(0, closest_idx - 20); // 在附近点内找横向误差
    int line_min = line_start;
    for (int i = line_start; i < closest_idx; i++) {
        double dist = std::sqrt(std::pow(adc_trajectory_msg_->points[i].north - cur_north, 2) +
                                std::pow(adc_trajectory_msg_->points[i].east - cur_east, 2));
        if (dist < min_dist) {
            line_min = i;
        }
    }
    double lat_error = std::sqrt(std::pow(adc_trajectory_msg_->points[line_min].north - cur_north, 2) +
                                 std::pow(adc_trajectory_msg_->points[line_min].east - cur_east, 2));

    // 横向误差的方向修正
    double direction_error = std::atan2(target_east - cur_east, target_north - cur_north) -
                             std::atan2(adc_trajectory_msg_->points[line_min].east - cur_east,
                                        adc_trajectory_msg_->points[line_min].north - cur_north);
    if (direction_error > M_PI)
        direction_error -= 2 * M_PI;
    if (direction_error < -M_PI)
        direction_error += 2 * M_PI;
    if (direction_error > 0) {
        lat_error = -lat_error;
    }

    // 3.3 使用混合控制器计算转向角
    double pursuit_control = std::atan2(2 * wheelbase_ * std::sin(angular_error), preview_dist); // 纯追踪控制
    double stanley_control = heading_error + std::atan(0.2 * lat_error / (cur_spd + 1e-5));      // Stanley控制

    // 3.4 计算最终转向角，并限制在合理范围内
    double steer_angle = 0.6 * pursuit_control + 0.4 * stanley_control; // 混合控制
    steer_angle = std::max(-max_steering_angle_,
                           std::min(max_steering_angle_, steer_angle * 180.0 / M_PI)); // 限制在[-30, 30]度之间

    // 输出调试信息
    RCLCPP_INFO(this->get_logger(), "Heading Error: %.2f degrees", heading_error * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Lateral Error: %.2f meters", lat_error);
    RCLCPP_INFO(this->get_logger(), "Steering Angle: %.2f degrees", steer_angle);

    // 4. 赋值给控制命令
    control_cmd_msg_.steer_angle = steer_angle;
}

void ControlNode::LongitudinalController() {}

void ControlNode::ADCTrajectoryCallback(const bot_msg::msg::ADCTrajectory::SharedPtr msg) {
    adc_trajectory_msg_ = msg;
    return;
}

void ControlNode::LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    localization_info_msg_ = msg;
    return;
}

/**
 * @brief Timer callback function
 *
 */
void ControlNode::TimerCallback() {
    if (adc_trajectory_msg_ == nullptr && localization_info_msg_ == nullptr) {
        return;
    }
}

/**
 * @brief Init params
 *
 */
void ControlNode::InitParams() {
    this->declare_parameter<double>("publish_rate", 0.0);
    this->declare_parameter<double>("max_linear_velocity", 0.0);
    this->declare_parameter<double>("max_steering_angle", 0.0);
    this->declare_parameter<double>("preview_time", 0.0);
    this->declare_parameter<double>("tolerance_distance", 0.0);
    this->declare_parameter<std::string>("adc_traj_topic_name", "/planing/adc_traj");
    this->declare_parameter<std::string>("control_cmd_topic_name", "/control/control_cmd");
    this->declare_parameter<std::string>("localization_info_topic_name", "/control/local_info");
    this->declare_parameter<double>("wheelbase", 0.0);
    // Get parameters
    this->control_cycle_time_ = this->get_parameter("publish_rate").get_value<double>();
    this->max_linear_velocity_ = this->get_parameter("max_linear_velocity").get_value<double>();
    this->max_steering_angle_ = this->get_parameter("max_steering_angle").get_value<double>();
    this->preview_time_ = this->get_parameter("preview_time").get_value<double>();
    this->tolerance_distance_ = this->get_parameter("tolerance_distance").get_value<double>();
    this->adc_traj_topic_name_ = this->get_parameter("adc_traj_topic_name").get_value<std::string>();
    this->control_cmd_topic_name_ = this->get_parameter("control_cmd_topic_name").get_value<std::string>();
    this->localization_info_topic_name_ = this->get_parameter("localization_info_topic_name").get_value<std::string>();
    this->wheelbase_ = this->get_parameter("wheelbase").get_value<double>();
    // Print parameters
    RCLCPP_INFO(this->get_logger(), "Publish rate: %f", this->publish_rate_);
    RCLCPP_INFO(this->get_logger(), "Max linear velocity: %f", this->max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Max steering angle: %f", this->max_steering_angle_);
    RCLCPP_INFO(this->get_logger(), "ADC traj topic name: %s", this->adc_traj_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Control cmd topic name: %s", this->control_cmd_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Localization info topic name: %s", this->localization_info_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Preview time: %f", this->preview_time_);
    RCLCPP_INFO(this->get_logger(), "Tolerance distance: %f", this->tolerance_distance_);
    RCLCPP_INFO(this->get_logger(), "Wheelbase: %f", this->wheelbase_);
}
} // namespace control