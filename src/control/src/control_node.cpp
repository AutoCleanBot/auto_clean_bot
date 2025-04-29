#include "control/control_node.h"
#include <cmath>

double inline deg2rad(double deg) { return deg * M_PI / 180.0; }

ssize_t g_debug_cnt = 0;


namespace control {
ControlNode::ControlNode() : Node("control_node") {
    // Initialize subscribers and publishers
    InitParams();
    this->sub_adc_trajectory_ = this->create_subscription<bot_msg::msg::ADCTrajectory>(
        this->adc_traj_topic_name_, 10, std::bind(&ControlNode::ADCTrajectoryCallback, this, std::placeholders::_1));
    this->sub_localization_info_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
        this->localization_info_topic_name_, 10,
        std::bind(&ControlNode::LocalizationInfoCallback, this, std::placeholders::_1));
    this->pub_control_cmd_ = this->create_publisher<bot_msg::msg::ControlCmd>(this->control_cmd_topic_name_, 10);

    int control_cycle_time = static_cast<int>(1000.0 / this->publish_rate_); // 毫秒
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(control_cycle_time),
                                           std::bind(&ControlNode::TimerCallback, this));
}

void ControlNode::LateralController() {
    // TODO 注意航向和方位角误差计算时, 需要将角度转换为弧度
    // 检查输入数据是否有效
    if (!adc_trajectory_msg_ || !localization_info_msg_) {
        RCLCPP_WARN(this->get_logger(), "LateralController: Missing trajectory or localization data");
        return;
    }

    // 检查轨迹点是否为空
    if (adc_trajectory_msg_->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "LateralController: Empty trajectory points");
        return;
    }

    double cur_north = localization_info_msg_->north;
    double cur_east = localization_info_msg_->east;
    // double cur_up = localization_info_msg_->up;
    double cur_spd = localization_info_msg_->vel_speed;
    double cur_yaw = localization_info_msg_->yaw * M_PI / 180.0; // 当前航向角, 弧度

    // 1. 找到当前车辆位置到轨迹上的最近点
    double min_dist = 1000000.0;
    size_t closest_idx_ = 0;
    for (size_t i = 0; i < adc_trajectory_msg_->points.size(); i++) {
        double dist = std::sqrt(std::pow(cur_north - adc_trajectory_msg_->points[i].north, 2) +
                                std::pow(cur_east - adc_trajectory_msg_->points[i].east, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx_ = i;
        }
    }
    if (closest_idx_ >= adc_trajectory_msg_->points.size()) {
        RCLCPP_ERROR(this->get_logger(), "Closest index is invalid");
        return;
    }

    // 2. 找到轨迹上的预瞄点
    size_t preview_idx = closest_idx_ + 1;
    // 预瞄距离 = 预瞄距离容差 + 预瞄时间 * 当前速度
    double preview_dist = tolerance_distance_ + preview_time_ * cur_spd;
    if (preview_idx >= adc_trajectory_msg_->points.size()) {
        preview_idx = closest_idx_;
        RCLCPP_ERROR(this->get_logger(), "Preview index is out of range");
    } else {
        double dist = std::sqrt(std::pow(cur_north - adc_trajectory_msg_->points[preview_idx].north, 2) +
                                std::pow(cur_east - adc_trajectory_msg_->points[preview_idx].east, 2));
        while (dist < preview_dist) {
            preview_idx++;
            if (preview_idx >= adc_trajectory_msg_->points.size()) {
                preview_idx = closest_idx_;
                RCLCPP_ERROR(this->get_logger(), "Preview index is out of range");
                break;
            }
            dist = std::sqrt(std::pow(cur_north - adc_trajectory_msg_->points[preview_idx].north, 2) +
                             std::pow(cur_east - adc_trajectory_msg_->points[preview_idx].east, 2));
        }
    }

    // 3. 计算横向控制命令
    // 获取预瞄点信息
    double target_north = adc_trajectory_msg_->points[preview_idx].north;
    double target_east = adc_trajectory_msg_->points[preview_idx].east;
    double target_yaw = adc_trajectory_msg_->points[preview_idx].yaw * M_PI / 180.0; // 目标航向角, 弧度

    // 3.1 计算航向误差和方位角误差
    double heading_error = target_yaw - cur_yaw;                                                    // 航向误差
    double angular_error = std::atan2(target_east - cur_east, target_north - cur_north) - cur_yaw; // 方位角误差

    // 限制角度在 [-PI, PI] 范围内，避免跳变
    if (angular_error > M_PI)
        angular_error -= 2 * M_PI;
    if (angular_error < -M_PI)
        angular_error += 2 * M_PI;

    // 3.2 计算横向误差
    // 计算路径切线方向
    double path_direction;
    if (closest_idx_ + 1 < adc_trajectory_msg_->points.size()) {
        // 使用前向点计算切线
        path_direction = std::atan2(
            adc_trajectory_msg_->points[closest_idx_ + 1].east - adc_trajectory_msg_->points[closest_idx_].east,
            adc_trajectory_msg_->points[closest_idx_ + 1].north - adc_trajectory_msg_->points[closest_idx_].north);
    } else if (closest_idx_ > 0) {
        // 使用后向点计算切线
        path_direction = std::atan2(
            adc_trajectory_msg_->points[closest_idx_].east - adc_trajectory_msg_->points[closest_idx_ - 1].east,
            adc_trajectory_msg_->points[closest_idx_].north - adc_trajectory_msg_->points[closest_idx_ - 1].north);
    } else {
        // 只有一个点，使用目标航向
        path_direction = adc_trajectory_msg_->points[closest_idx_].yaw * M_PI / 180.0;
    }

    RCLCPP_INFO(this->get_logger(), "Path direction: %.2f degrees", path_direction * 180.0 / M_PI);
    // 计算车辆到最近点的向量
    double dx = cur_east - adc_trajectory_msg_->points[closest_idx_].east;
    double dy = cur_north - adc_trajectory_msg_->points[closest_idx_].north;

    // 计算横向误差（向量在垂直于路径方向上的投影）
    // 使用 (-sin(θ), cos(θ)) 作为法向量进行投影计算
    double lat_error = -dx * std::sin(path_direction) + dy * std::cos(path_direction);


    // 3.3 使用混合控制器计算转向角
    double pursuit_control = std::atan2(2 * wheelbase_ * std::sin(angular_error), preview_dist); // 纯追踪控制
    double stanley_control = heading_error + std::atan(0.2 * lat_error / (cur_spd + 1e-5));      // Stanley控制

    // 3.4 计算最终转向角，并限制在合理范围内
    double front_wheel_rad = 0.6 * pursuit_control + 0.4 * stanley_control; // 混合控制
    // 乘以10.0原因是, 计算出的是前轮转角,控制量是方向盘转角,中间有一个10倍的传动比
    double steer_angle = std::max(-max_steering_angle_,
                           std::min(max_steering_angle_, front_wheel_rad * 180.0 / M_PI * ratio_)); // 限制在[-30, 30]度之间

    if(g_debug_cnt % 10 == 0){
        // 输出调试信息
        RCLCPP_INFO(this->get_logger(), "Heading Error: %.2f degrees", heading_error * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Angular Error: %.2f degrees", angular_error * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Lateral Error: %.2f meters", lat_error);
        RCLCPP_INFO(this->get_logger(), "Front Wheel Deg: %.2f degrees", front_wheel_rad * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Steering Angle: %.2f degrees", steer_angle);
        RCLCPP_INFO(this->get_logger(), "Preview Distance: %.2f meters", preview_dist);
        RCLCPP_INFO(this->get_logger(), "Preview Index: %zu", preview_idx);
        RCLCPP_INFO(this->get_logger(), "Closest Index: %zu", closest_idx_);
        RCLCPP_INFO(this->get_logger(), "Target North: %.2f meters", target_north);
        RCLCPP_INFO(this->get_logger(), "Target East: %.2f meters", target_east);
        RCLCPP_INFO(this->get_logger(), "Target Yaw: %.2f degrees", target_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f degrees", cur_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Current North: %.2f meters", cur_north);
        RCLCPP_INFO(this->get_logger(), "Current East: %.2f meters", cur_east);
        RCLCPP_INFO(this->get_logger(), "Current Speed: %.2f meters/second", cur_spd);
    }

    // 4. 赋值给控制命令
    control_cmd_msg_.steer_angle = steer_angle; 
}

void ControlNode::LongitudinalController() {
    // 检查输入数据是否有效
    if (!adc_trajectory_msg_ || !localization_info_msg_) {
        RCLCPP_WARN(this->get_logger(), "LongitudinalController: Missing trajectory or localization data");
        return;
    }

    // 检查轨迹点是否为空
    if (adc_trajectory_msg_->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "LongitudinalController: Empty trajectory points");
        return;
    }


    double cur_spd = localization_info_msg_->vel_speed;
    // 由于只是速度控制，因此只需要计算速度误差即可
    // 暂时不需要PID控制，直接赋值给控制命令
    // 1. 计算纵向控制命令
    double control_cycle_time = 1.0 / this->publish_rate_; // 控制周期,单位为秒
    double target_speed = adc_trajectory_msg_->points[closest_idx_].vel_speed;
    double speed_error = target_speed - cur_spd;
    double acceleration = std::min(max_linear_velocity_ - cur_spd, acceleration_limit_);
    double deceleration = std::min(cur_spd - min_linear_velocity_, deceleration_limit_);
    double speed_cmd =
        cur_spd + acceleration * control_cycle_time + deceleration * control_cycle_time * control_cycle_time / 2.0;
    speed_cmd = std::max(std::min(speed_cmd, max_linear_velocity_), min_linear_velocity_);

    // 输出调试信息
    RCLCPP_INFO(this->get_logger(), "Speed Error: %.2f meters/second", speed_error);
    RCLCPP_INFO(this->get_logger(), "Acceleration: %.2f meters/second^2", acceleration);
    RCLCPP_INFO(this->get_logger(), "Deceleration: %.2f meters/second^2", deceleration);
    RCLCPP_INFO(this->get_logger(), "Speed Cmd: %.2f meters/second", speed_cmd);

    // 3. 赋值给控制命令
    // control_cmd_msg_.speed = speed_cmd;
    control_cmd_msg_.speed = 1;

}

void ControlNode::ADCTrajectoryCallback(const bot_msg::msg::ADCTrajectory::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", msg->points.size());
    adc_trajectory_msg_ = msg;
    return;
}

void ControlNode::LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received localization, position: (%.2f, %.2f)", msg->east, msg->north);
    localization_info_msg_ = msg;
    return;
}

/**
 * @brief Timer callback function
 *
 */
void ControlNode::TimerCallback() {
    // 检查数据是否准备好
    if (!adc_trajectory_msg_ || !localization_info_msg_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Waiting for trajectory and localization data...");
        return;
    }

    // 1. 计算控制命令
    LateralController();
    LongitudinalController();

    // 2. 发布控制命令
    control_cmd_msg_.header.stamp = this->now();
    control_cmd_msg_.header.frame_id = "base_link";
    // TODO (yangsh) temporary use the gear forward
    control_cmd_msg_.gear = 1;
    this->pub_control_cmd_->publish(control_cmd_msg_);

    ++g_debug_cnt;
    if(g_debug_cnt == 60000){
        g_debug_cnt = 0;
    }
    return;
}

/**
 * @brief Init params
 *
 */
void ControlNode::InitParams() {
    this->declare_parameter<double>("publish_rate", 0.0);
    this->declare_parameter<double>("preview_time", 0.0);
    this->declare_parameter<double>("tolerance_distance", 0.0);
    this->declare_parameter<double>("max_steering_angle", 0.0);
    this->declare_parameter<double>("wheelbase", 0.0);
    this->declare_parameter<double>("max_linear_velocity", 0.0);
    this->declare_parameter<double>("min_linear_velocity", 0.0);
    this->declare_parameter<double>("acceleration_limit", 0.0);
    this->declare_parameter<double>("deceleration_limit", 0.0);
    this->declare_parameter<double>("ratio", 10.0);
    this->declare_parameter<std::string>("adc_traj_topic_name", "/planing/adc_traj");
    this->declare_parameter<std::string>("control_cmd_topic_name", "/control/control_cmd");
    this->declare_parameter<std::string>("localization_info_topic_name", "/control/local_info");
    // Get parameters
    this->publish_rate_ = this->get_parameter("publish_rate").get_value<double>();
    this->preview_time_ = this->get_parameter("preview_time").get_value<double>();
    this->max_steering_angle_ = this->get_parameter("max_steering_angle").get_value<double>();
    this->tolerance_distance_ = this->get_parameter("tolerance_distance").get_value<double>();
    this->wheelbase_ = this->get_parameter("wheelbase").get_value<double>();
    this->max_linear_velocity_ = this->get_parameter("max_linear_velocity").get_value<double>();
    this->min_linear_velocity_ = this->get_parameter("min_linear_velocity").get_value<double>();
    this->acceleration_limit_ = this->get_parameter("acceleration_limit").get_value<double>();
    this->deceleration_limit_ = this->get_parameter("deceleration_limit").get_value<double>();
    this->adc_traj_topic_name_ = this->get_parameter("adc_traj_topic_name").get_value<std::string>();
    this->ratio_ = this->get_parameter("ratio").get_value<double>();
    this->control_cmd_topic_name_ = this->get_parameter("control_cmd_topic_name").get_value<std::string>();
    this->localization_info_topic_name_ = this->get_parameter("localization_info_topic_name").get_value<std::string>();
    // Print parameters
    RCLCPP_INFO(this->get_logger(), "Publish rate: %f", this->publish_rate_);
    RCLCPP_INFO(this->get_logger(), "Max linear velocity: %f", this->max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Min linear velocity: %f", this->min_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Max steering angle: %f", this->max_steering_angle_);
    RCLCPP_INFO(this->get_logger(), "ADC traj topic name: %s", this->adc_traj_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Control cmd topic name: %s", this->control_cmd_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Localization info topic name: %s", this->localization_info_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Preview time: %f", this->preview_time_);
    RCLCPP_INFO(this->get_logger(), "Tolerance distance: %f", this->tolerance_distance_);
    RCLCPP_INFO(this->get_logger(), "Wheelbase: %f", this->wheelbase_);
    RCLCPP_INFO(this->get_logger(), "Acceleration limit: %f", this->acceleration_limit_);
    RCLCPP_INFO(this->get_logger(), "Deceleration limit: %f", this->deceleration_limit_);
    RCLCPP_INFO(this->get_logger(), "Ratio: %f", this->ratio_);
    return;
}
} // namespace control
// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<control::ControlNode>();
    RCLCPP_INFO(node->get_logger(), "control node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}