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
        debug_log_file_ << "pursuit_control_rate,stanley_control_rate,sta_lat_rate,heading_error_deg,angular_error_deg,"
                           "lat_error,pursuit_control_deg,stanley_control_deg,"
                           "steer_angle_deg,preview_dist,preview_idx,closest_idx,target_east,target_north,target_yaw_"
                           "deg,closest_east,closest_north,closest_yaw_deg,cur_east,cur_north,cur_yaw_deg,cur_spd"
                        << std::endl;
    }

    // 初始化PID控制器参数

    // 创建PID控制器
    speed_pid_controller_ = std::make_unique<PIDController>(speed_pid_kp_, speed_pid_ki_, speed_pid_kd_);
    speed_pid_controller_->setOutputLimits(-deceleration_limit_, acceleration_limit_);

    // 初始化时间戳
    last_control_time_ = this->now();
}

void ControlNode::LateralController() {
    const double kMinStanleyControlSpd = 0.5;
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
    double effective_stanley_spd = std::max(cur_spd, kMinStanleyControlSpd);
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
    double closest_east = adc_trajectory_msg_->points[closest_idx_].east;
    double closest_north = adc_trajectory_msg_->points[closest_idx_].north;
    double closest_yaw =
        NormalizeAngle(adc_trajectory_msg_->points[closest_idx_].yaw * M_PI / 180.0); // 最近点航向角, 弧度
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
            adc_trajectory_msg_->points[closest_idx_].north - adc_trajectory_msg_->points[closest_idx_ - 1].north);
    } else {
        // 只有一个点，使用目标航向
        path_direction = adc_trajectory_msg_->points[closest_idx_].yaw * M_PI / 180.0;
    }
    path_direction = NormalizeAngle(path_direction);

    // 计算车辆到最近点的向量
    double dx = cur_east - adc_trajectory_msg_->points[closest_idx_].east;
    double dy = cur_north - adc_trajectory_msg_->points[closest_idx_].north;

    // 计算横向误差（向量在垂直于路径方向上的投影）
    // 使用 (-sin(θ), cos(θ)) 作为法向量进行投影计算
    // 这样计算的结果是在路径的左侧时,横向误差为负; 在路径的右侧时横向误差为正
    double lat_error = -dx * std::sin(path_direction) + dy * std::cos(path_direction);
    // ! 目前计算结果为左正右负
    // 3.3 使用混合控制器计算转向角
    double pursuit_control = -std::atan2(2 * wheelbase_ * std::sin(angular_error), preview_dist); // 纯追踪控制
    double stanley_control =
        -(heading_error - std::atan(sta_lat_rate_ * lat_error / effective_stanley_spd)); // Stanley控制

    // ! 注意如果出现当前的需要控制情况为右转为正左转为负的情况的话pursuit_control和stanley_control去除负号即可

    // 3.4 计算最终转向角，并限制在合理范围内
    double front_wheel_rad =
        pursuit_control_rate_ * pursuit_control + stanley_control_rate_ * stanley_control; // 混合控制
    // 乘以10.0原因是, 计算出的是前轮转角,控制量是方向盘转角,中间有一个10倍的传动比
    double steer_angle = std::max(
        -max_steering_angle_, std::min(max_steering_angle_, front_wheel_rad * 180.0 / M_PI)); // 限制在[-30, 30]度之间
    // 零点漂移处理
    steer_angle += zero_point_draft_;
    // 自行车模型的转角偏差
    steer_angle *= turning_radius_ratio_;
    if (g_debug_cnt % 10 == 0) {
        // 输出调试信息
        RCLCPP_INFO(this->get_logger(),
                    "heading_error,%.2f,angular_error,%.2f,lat_error,%.2f,steer_angle,%.2f,pursuit_control,%.2f,"
                    "stanley_control,%.2f,preview_dist,%.2f,preview_idx,%zu,closest_idx,%zu,target_north,%.2f,target_"
                    "east,%.2f,target_yaw,%."
                    "2f,cur_yaw,%.2f,cur_north,%.2f,cur_east,%.2f,cur_spd,%.2f,closest_east,%.2f,closest_north,%.2f,"
                    "closest_yaw,%.2f",
                    heading_error * 180.0 / M_PI, angular_error * 180.0 / M_PI, lat_error, steer_angle,
                    pursuit_control * 180.0 / M_PI, stanley_control * 180.0 / M_PI, preview_dist, preview_idx,
                    closest_idx_, target_north, target_east, target_yaw * 180.0 / M_PI, cur_yaw * 180.0 / M_PI,
                    cur_north, cur_east, cur_spd, closest_east, closest_north, closest_yaw * 180.0 / M_PI);
    }
    if (debug_log_file_.is_open()) {
        debug_log_file_ << pursuit_control_rate_ << "," << stanley_control_rate_ << "," << sta_lat_rate_ << ","
                        << heading_error * 180.0 / M_PI << "," << angular_error * 180.0 / M_PI << "," << lat_error
                        << "," << pursuit_control * 180.0 / M_PI << "," << stanley_control * 180.0 / M_PI << ","
                        << steer_angle << "," << preview_dist << "," << preview_idx << "," << closest_idx_ << ","
                        << target_east << "," << target_north << "," << target_yaw * 180.0 / M_PI << "," << closest_east
                        << "," << closest_north << "," << closest_yaw * 180.0 / M_PI << "," << cur_east << ","
                        << cur_north << "," << cur_yaw * 180.0 / M_PI << "," << cur_spd << std::endl;
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

    // 获取当前速度和目标速度
    double current_speed = localization_info_msg_->vel_speed;
    double target_speed = adc_trajectory_msg_->points[closest_idx_].vel_speed;

    // 限制目标速度在合理范围内
    target_speed = std::max(min_linear_velocity_, std::min(max_linear_velocity_, target_speed));

    // 计算时间间隔
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;

    if (dt <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid time interval");
        return;
    }

    // 计算速度误差
    double speed_error = target_speed - current_speed;

    // 使用PID控制器计算加速度命令
    double acceleration = speed_pid_controller_->compute(speed_error, dt);

    // 将加速度转换为目标速度
    double target_speed_command = current_speed + acceleration * dt;
    target_speed_command = std::max(min_linear_velocity_, std::min(max_linear_velocity_, target_speed_command));

    // 更新控制命令
    control_cmd_msg_.speed = target_speed_command;

    // 输出调试信息
    if (g_debug_cnt % 10 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "Speed Control: target=%.2f, current=%.2f, error=%.2f, acceleration=%.2f, command=%.2f",
                    target_speed, current_speed, speed_error, acceleration, target_speed_command);
    }
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
    if (adc_trajectory_msg_->direction == 0) {
        control_cmd_msg_.gear = 1;
    } else if (adc_trajectory_msg_->direction == 1) {
        control_cmd_msg_.gear = 2;
    }else{
        control_cmd_msg_.gear = 1;
        RCLCPP_WARN(this->get_logger(), "Invalid trajectory direction");
    }
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
    this->declare_parameter<double>("sta_lat_rate", 0.1);
    this->declare_parameter<double>("turning_radius_ratio", 1.0);
    this->declare_parameter<double>("zero_point_draft", 0.0);

    this->declare_parameter("speed_pid_kp", 0.5);
    this->declare_parameter("speed_pid_ki", 0.1);
    this->declare_parameter("speed_pid_kd", 0.0);
    this->declare_parameter<std::string>("adc_traj_topic_name", "/planing/adc_traj");
    this->declare_parameter<std::string>("control_cmd_topic_name", "/control/control_cmd");
    this->declare_parameter<std::string>("localization_info_topic_name", "/control/local_info");
    this->declare_parameter<std::string>("log_file_path", "./control_debug.csv");
    // Get parameters
    publish_rate_ = this->get_parameter("publish_rate").get_value<double>();
    preview_time_ = this->get_parameter("preview_time").get_value<double>();
    max_steering_angle_ = this->get_parameter("max_steering_angle").get_value<double>();
    tolerance_distance_ = this->get_parameter("tolerance_distance").get_value<double>();
    wheelbase_ = this->get_parameter("wheelbase").get_value<double>();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").get_value<double>();
    min_linear_velocity_ = this->get_parameter("min_linear_velocity").get_value<double>();
    acceleration_limit_ = this->get_parameter("acceleration_limit").get_value<double>();
    deceleration_limit_ = this->get_parameter("deceleration_limit").get_value<double>();
    pursuit_control_rate_ = this->get_parameter("pursuit_control_rate").get_value<double>();
    stanley_control_rate_ = this->get_parameter("stanley_control_rate").get_value<double>();
    sta_lat_rate_ = this->get_parameter("sta_lat_rate").get_value<double>();
    turning_radius_ratio_ = this->get_parameter("turning_radius_ratio").get_value<double>();
    zero_point_draft_ = this->get_parameter("zero_point_draft").get_value<double>();
    adc_traj_topic_name_ = this->get_parameter("adc_traj_topic_name").get_value<std::string>();
    control_cmd_topic_name_ = this->get_parameter("control_cmd_topic_name").get_value<std::string>();
    localization_info_topic_name_ = this->get_parameter("localization_info_topic_name").get_value<std::string>();
    log_file_path_ = this->get_parameter("log_file_path").get_value<std::string>();
    speed_pid_kp_ = this->get_parameter("speed_pid_kp").as_double();
    speed_pid_ki_ = this->get_parameter("speed_pid_ki").as_double();
    speed_pid_kd_ = this->get_parameter("speed_pid_kd").as_double();
    // Print parameters
    RCLCPP_INFO(this->get_logger(), "Publish rate: %f", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "Max linear velocity: %f", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Min linear velocity: %f", min_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Max steering angle: %f", max_steering_angle_);
    RCLCPP_INFO(this->get_logger(), "ADC traj topic name: %s", adc_traj_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Control cmd topic name: %s", control_cmd_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Localization info topic name: %s", localization_info_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Preview time: %f", preview_time_);
    RCLCPP_INFO(this->get_logger(), "Tolerance distance: %f", tolerance_distance_);
    RCLCPP_INFO(this->get_logger(), "Wheelbase: %f", wheelbase_);
    RCLCPP_INFO(this->get_logger(), "Acceleration limit: %f", acceleration_limit_);
    RCLCPP_INFO(this->get_logger(), "Deceleration limit: %f", deceleration_limit_);
    RCLCPP_INFO(this->get_logger(), "Turning radius ratio: %f", turning_radius_ratio_);
    RCLCPP_INFO(this->get_logger(), "Zero point draft: %f", zero_point_draft_);
    RCLCPP_INFO(this->get_logger(), "Pursuit control rate: %f", pursuit_control_rate_);
    RCLCPP_INFO(this->get_logger(), "Stanley control rate: %f", stanley_control_rate_);
    RCLCPP_INFO(this->get_logger(), "Stanley lat control rate: %f", sta_lat_rate_);
    RCLCPP_INFO(this->get_logger(), "Debug log file path: %s", log_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Speed pid kp: %f", speed_pid_kp_);
    RCLCPP_INFO(this->get_logger(), "Speed pid ki: %f", speed_pid_ki_);
    RCLCPP_INFO(this->get_logger(), "Speed pid kd: %f", speed_pid_kd_);
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