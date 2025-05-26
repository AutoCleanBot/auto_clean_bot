#include "control/control_node.h"
#include <cmath>
#include <filesystem>

double NormalizeAngle(double angle) {
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}
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

    // 使用参数读取到的日志文件路径打开文件
    // 确保目录存在
    std::filesystem::path log_path(log_file_path_);
    std::filesystem::create_directories(log_path.parent_path()); // 创建父目录（如果不存在）

    debug_log_file_.open(log_file_path_, std::ios::app); // 使用参数指定的路径
    if (!debug_log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open debug log file: %s", log_file_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Debug log file opened: %s", log_file_path_.c_str());
        // 写入CSV文件头（如果文件是新建或空的）
        debug_log_file_ << "heading_error_deg,angular_error_deg,lat_error,pursuit_control_deg,stanley_control_deg,steer_angle_deg,preview_dist,preview_idx,closest_idx,target_east,target_north,target_yaw_deg,closest_east,closest_north,closest_yaw_deg,cur_east,cur_north,cur_yaw_deg,cur_spd" << std::endl;
    }
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
    double cur_yaw = NormalizeAngle(localization_info_msg_->yaw * M_PI / 180.0); // 当前航向角, 弧度

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
    double closest_yaw = NormalizeAngle(adc_trajectory_msg_->points[closest_idx_].yaw * M_PI / 180.0);
    double closest_north = adc_trajectory_msg_->points[closest_idx_].north;
    double closest_east = adc_trajectory_msg_->points[closest_idx_].east;
    // 获取预瞄点信息
    double target_north = adc_trajectory_msg_->points[preview_idx].north;
    double target_east = adc_trajectory_msg_->points[preview_idx].east;
    double target_yaw = NormalizeAngle(adc_trajectory_msg_->points[preview_idx].yaw * M_PI / 180.0); // 目标航向角, 弧度
    // 3.1 计算航向误差和方位角误差
    double heading_error = NormalizeAngle(target_yaw - cur_yaw);                       // 航向误差
    double deg_angular = std::atan2(target_east - cur_east, target_north - cur_north); // 方位角误差
    double angular_error = NormalizeAngle(deg_angular - cur_yaw);

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
            adc_trajectory_msg_->points[closest_idx_].north - adc_trajectory_msg_->points[closest_idx_ - 1].north
        );
    } else {
        // 只有一个点，使用目标航向
        path_direction = adc_trajectory_msg_->points[closest_idx_].yaw * M_PI / 180.0;
    }
    path_direction = NormalizeAngle(path_direction);

    // 计算车辆到最近点的向量
    double dx = cur_east - closest_east;
    double dy = cur_north - closest_north;

    // 计算横向误差（向量在垂直于路径方向上的投影）
     // lat_error 定义为：车辆在路径右侧时为正，左侧时为负。
    double lat_error = dx * std::cos(path_direction) - dy * std::sin(path_direction);

    // 3.3 使用混合控制器计算转向角
    double pursuit_control = std::atan2(2 * wheelbase_ * std::sin(angular_error), preview_dist); // 纯追踪控制
    double stanley_control = heading_error + std::atan(0.2 * lat_error / (cur_spd + 1e-5));      // Stanley控制

    // 3.4 计算最终转向角，并限制在合理范围内
    double front_wheel_rad = pursuit_control_rate_ * pursuit_control 
                            + stanley_control_rate_ * stanley_control; // 混合控制

    double steer_angle = std::max(
        -max_steering_angle_, std::min(max_steering_angle_, front_wheel_rad * 180.0 / M_PI)); // 限制在[-30, 30]度之间

    if (g_debug_cnt % 10 == 0) {
        // 输出调试信息
        RCLCPP_INFO(this->get_logger(),
                    "heading_error,%.2f,angular_error,%.2f,lat_error,%.2f,front_wheel_deg,%.2f,steer_angle,%.2f,"
                    "preview_dist,%.2f,preview_idx,%zu,closest_idx,%zu,target_north,%.2f,target_east,%.2f,target_yaw,%."
                    "2f,cur_yaw,%.2f,cur_north,%.2f,cur_east,%.2f,cur_spd,%.2f,closest_east,%.2f,closest_north,%.2f,closest_yaw,%.2f",
                    heading_error * 180.0 / M_PI, angular_error * 180.0 / M_PI, lat_error,
                    front_wheel_rad * 180.0 / M_PI, steer_angle, preview_dist, preview_idx, closest_idx_, target_north,
                    target_east, target_yaw * 180.0 / M_PI, cur_yaw * 180.0 / M_PI, cur_north, cur_east, cur_spd,
                    closest_east, closest_north, closest_yaw* 180.0 / M_PI );

    }
    if (debug_log_file_.is_open()) {
        debug_log_file_ << heading_error * 180.0 / M_PI 
                        << "," << angular_error * 180.0 / M_PI 
                        << "," << lat_error 
                         << "," << pursuit_control * 180.0 / M_PI 
                         << "," << stanley_control * 180.0 / M_PI 
                         << "," << steer_angle 
                        << "," << preview_dist 
                         << "," << preview_idx 
                        << "," << closest_idx_
                       << "," << target_east 
                         << "," << target_north 
                         << "," << target_yaw * 180.0 / M_PI 
                        << "," << closest_east
                        << "," << closest_north
                        << "," << closest_yaw * 180.0 / M_PI
                        << "," << cur_east 
                        << "," << cur_north 
                        << "," << cur_yaw * 180.0 / M_PI 
                        << "," << cur_spd 
                        << std::endl;
    }

    // 4. 赋值给控制命令
    control_cmd_msg_.steer_angle = steer_angle;
}

void ControlNode::LongitudinalController() {
    // TODO 重点自动停止的相关代码编写
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
    // RCLCPP_INFO(this->get_logger(), "Speed Error: %.2f meters/second", speed_error);
    // RCLCPP_INFO(this->get_logger(), "Acceleration: %.2f meters/second^2", acceleration);
    // RCLCPP_INFO(this->get_logger(), "Deceleration: %.2f meters/second^2", deceleration);
    // RCLCPP_INFO(this->get_logger(), "Speed Cmd: %.2f meters/second", speed_cmd);

    // 3. 赋值给控制命令
    // control_cmd_msg_.speed = target_speed;
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
    if (g_debug_cnt == 60000) {
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
    this->declare_parameter<double>("pursuit_control_rate", 0.0);
    this->declare_parameter<double>("stanley_control_rate", 0.0);
    this->declare_parameter<double>("ratio", 10.0);
    this->declare_parameter<std::string>("adc_traj_topic_name", "/planing/adc_traj");
    this->declare_parameter<std::string>("control_cmd_topic_name", "/control/control_cmd");
    this->declare_parameter<std::string>("localization_info_topic_name", "/control/local_info");
    this->declare_parameter<std::string>("log_file_path", "./control_debug.csv");
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
    this->pursuit_control_rate_ = this->get_parameter("pursuit_control_rate").get_value<double>();
    this->stanley_control_rate_ = this->get_parameter("stanley_control_rate").get_value<double>();
    this->ratio_ = this->get_parameter("ratio").get_value<double>();
    this->adc_traj_topic_name_ = this->get_parameter("adc_traj_topic_name").get_value<std::string>();
    this->control_cmd_topic_name_ = this->get_parameter("control_cmd_topic_name").get_value<std::string>();
    this->localization_info_topic_name_ = this->get_parameter("localization_info_topic_name").get_value<std::string>();
    this->log_file_path_ = this->get_parameter("log_file_path").get_value<std::string>();
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
    RCLCPP_INFO(this->get_logger(), "Pursuit control rate: %f", this->pursuit_control_rate_);
    RCLCPP_INFO(this->get_logger(), "Stanley control rate: %f", this->stanley_control_rate_);
    RCLCPP_INFO(this->get_logger(), "Debug log file path: %s", this->log_file_path_.c_str());
    return;
}

ControlNode::~ControlNode() {
    if (debug_log_file_.is_open()) {
        debug_log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Debug log file closed: %s", log_file_path_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "control node shutting down");
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