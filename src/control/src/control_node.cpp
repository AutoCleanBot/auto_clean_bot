#include "control/control_node.h"
#include <cmath>
#include <filesystem>

ssize_t g_debug_cnt = 0;
double NormalizeAngle(double angle) {
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}
// 将ROS 2时间点转换为时分秒毫秒格式的字符串
std::string TimeToHumanReadable(const rclcpp::Time &time) {
    // 获取总秒数
    double seconds_since_epoch = time.seconds();

    // 将秒数转换为time_t (整数部分)
    std::time_t time_t_seconds = static_cast<std::time_t>(seconds_since_epoch);

    // 计算毫秒部分
    int milliseconds = static_cast<int>((seconds_since_epoch - time_t_seconds) * 1000);

    // 使用localtime将time_t转换为本地时间
    std::tm *local_time = std::localtime(&time_t_seconds);

    // 格式化时间为字符串
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", local_time); // 时:分:秒

    // 添加毫秒部分并返回完整字符串
    std::ostringstream oss;
    oss << buffer << "." << std::setfill('0') << std::setw(3) << milliseconds;
    return oss.str();
}

namespace control {
ControlNode::ControlNode() : Node("control_node") {
    // Initialize subscribers and publishers
    InitParams();

    // 初始化LQR控制器参数
    this->declare_parameter("use_lqr_controller", false);  // 默认不使用LQR控制器
    this->declare_parameter("cf", 155494.663);  // 前轮侧偏刚度
    this->declare_parameter("cr", 155494.663);  // 后轮侧偏刚度
    this->declare_parameter("mass", 1500.0);    // 车辆质量
    this->declare_parameter("iz", 2500.0);      // 车辆转动惯量
    this->declare_parameter("lqr_max_iterations", 150);  // LQR最大迭代次数
    this->declare_parameter("lqr_eps", 0.01);   // LQR收敛容差

    // 获取LQR参数
    use_lqr_controller_ = this->get_parameter("use_lqr_controller").as_bool();
    cf_ = this->get_parameter("cf").as_double();
    cr_ = this->get_parameter("cr").as_double();
    mass_ = this->get_parameter("mass").as_double();
    iz_ = this->get_parameter("iz").as_double();
    lqr_max_iterations_ = this->get_parameter("lqr_max_iterations").as_int();
    lqr_eps_ = this->get_parameter("lqr_eps").as_double();

    // 初始化LQR控制器
    if (use_lqr_controller_) {
        // 设置LQR控制器的车辆参数
        lqr_controller_.SetVehicleParams(wheelbase_, cf_, cr_, mass_, iz_);
        lqr_controller_.SetSolverParams(lqr_max_iterations_, lqr_eps_);
        
        // 设置状态权重矩阵Q和控制权重矩阵R
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
        Q(0, 0) = 40.0;  // 横向误差权重
        Q(1, 1) = 5.0;   // 横向误差变化率权重
        Q(2, 2) = 30.0;  // 航向误差权重
        Q(3, 3) = 2.0;   // 航向误差变化率权重
        
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
        R(0, 0) = 30.0;  // 控制输入权重
        
        lqr_controller_.Init(Q, R);
        
        RCLCPP_INFO(this->get_logger(), "LQR controller initialized");
    }

    // 初始化误差变化率计算的历史值
    previous_lateral_error_ = 0.0;
    previous_heading_error_ = 0.0;

    this->sub_adc_trajectory_ = this->create_subscription<bot_msg::msg::ADCTrajectory>(
        this->adc_traj_topic_name_, 10, std::bind(&ControlNode::ADCTrajectoryCallback, this, std::placeholders::_1));
    this->sub_localization_info_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
        this->localization_info_topic_name_, 10,
        std::bind(&ControlNode::LocalizationInfoCallback, this, std::placeholders::_1));
    this->pub_control_cmd_ = this->create_publisher<bot_msg::msg::ControlCmd>(this->control_cmd_topic_name_, 10);

    int control_cycle_time = static_cast<int>(1000.0 / this->publish_rate_); // 毫秒
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(control_cycle_time),
                                           std::bind(&ControlNode::TimerCallback, this));
    this->sub_chassis_info_ = this->create_subscription<bot_msg::msg::ChassisInfo>(
        this->chassis_info_topic_name_, 10, std::bind(&ControlNode::ChassisInfoCallback, this, std::placeholders::_1));

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
        debug_log_file_
            << "timestamp,pursuit_control_rate,stanley_control_rate,sta_lat_rate,heading_error_deg,angular_error_deg,"
               "lat_error,pursuit_control_deg,stanley_control_deg,steer_angle_deg,feedback_steer_deg,"
               "preview_dist,preview_idx,closest_idx,target_east,target_north,target_yaw_"
               "deg,closest_east,closest_north,closest_yaw_deg,closest_curvature,cur_east,cur_north,cur_yaw_deg,"
<<<<<<< HEAD
               "controller_type,"  // 增加控制器类型列
=======
               "curvature_feedforward,zero_point_draft"
>>>>>>> fef6ae7375b9adecd73bee1cc1972e3b5e403f0f
               "target_spd,cur_spd,error,acceleration,cmd_spd,integral"
            << std::endl;
    }

    // 创建PID控制器并设置前馈增益
    speed_pid_controller_ = std::make_unique<PIDController>(speed_pid_kp_, speed_pid_ki_, speed_pid_kd_);
    speed_pid_controller_->setFeedForward(speed_pid_kf_);
    speed_pid_controller_->setOutputLimits(-deceleration_limit_, acceleration_limit_);
    speed_pid_controller_->setIntegralLimits(-1.5, 1.5); // 可以根据实际情况调整为-1.0到-2.0之间

    // 初始化时间戳
    last_control_time_ = this->now();

    // 声明速度平滑相关参数
    this->declare_parameter("max_speed_change_rate", 1.0); // 默认最大变化率1m/s^2
    this->declare_parameter("smooth_window_size", 20);     // 默认平滑窗口大小为20

    // 获取参数
    max_speed_change_rate_ = this->get_parameter("max_speed_change_rate").as_double();
    smooth_window_size_ = this->get_parameter("smooth_window_size").as_int();

    // 初始化速度平滑相关变量
    previous_speed_command_ = 0.0;
    speed_commands_buffer_.clear();

    // 声明新的参数
    this->declare_parameter("max_steering_rate", 30.0); // 度/秒
    max_steering_rate_ = this->get_parameter("max_steering_rate").as_double();

    // 初始化状态变量
    previous_steering_angle_ = 0.0;
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
    closest_idx_ = 0;
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
        double accumulated_distance = 0.0;
        // Iterate forward along the trajectory from closest_idx_
        for (size_t i = closest_idx_; i < adc_trajectory_msg_->points.size() - 1; ++i) {
            // Calculate distance between point i and point i+1
            double segment_dist =
                std::hypot(adc_trajectory_msg_->points[i + 1].east - adc_trajectory_msg_->points[i].east,
                           adc_trajectory_msg_->points[i + 1].north - adc_trajectory_msg_->points[i].north);

            if (accumulated_distance + segment_dist >= preview_dist) {
                // Found a segment that contains the preview point.
                // We can either pick point i+1 or interpolate. For simplicity, pick i+1.
                // More advanced: interpolate between points[i] and points[i+1]
                // to get a point exactly at target_preview_distance.
                preview_idx = i + 1;
                break;
            }
            accumulated_distance += segment_dist;
            preview_idx = i + 1; // Keep updating preview_idx to the last point checked
        }
        // If the loop finishes and preview_idx is still not far enough (e.g., end of trajectory reached),
        // preview_idx will be the last point of the trajectory.
    }

<<<<<<< HEAD
    double steer_angle = 0.0;
    
    // 使用LQR控制器
    if (use_lqr_controller_) {
        // 计算当前车辆状态
        VehicleState vehicle_state = ComputeVehicleState();
        
        // 更新LQR系统矩阵
        lqr_controller_.UpdateSystemMatrix(vehicle_state.velocity);
        
        // 求解Riccati方程
        lqr_controller_.SolveRiccatiEquation();
        
        // 计算控制命令
        double lqr_steering_rad = lqr_controller_.ComputeControlCommand(vehicle_state);
        
        // 转换为角度
        steer_angle = lqr_steering_rad * 180.0 / M_PI;
        
        // 计算当前路径曲率，用于前馈控制
        double path_curvature = CalculatePathCurvature(closest_idx_);
        
        // 添加前馈控制项
        double curvature_feedforward = std::atan2(wheelbase_ * path_curvature, 1.0) * 180.0 / M_PI;
        steer_angle += feedforward_rate_ * curvature_feedforward;
        
        if (g_debug_cnt % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "LQR Control: lat_error=%.2f, heading_error=%.2f, lat_error_rate=%.2f, heading_error_rate=%.2f, "
                        "steer_angle=%.2f, curvature_feedforward=%.2f",
                        vehicle_state.lateral_error, vehicle_state.heading_error * 180.0 / M_PI, 
                        vehicle_state.lateral_error_rate, vehicle_state.heading_error_rate * 180.0 / M_PI,
                        steer_angle, curvature_feedforward);
        }
    } 
    
=======
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
    double lat_error = dx * std::cos(path_direction) - dy * std::sin(path_direction);
    // 3.3 使用混合控制器计算转向角
    // ! 目前计算结果为左正右负
    // ! 注意如果出现当前的需要控制情况为右转为正左转为负的情况的话pursuit_control和stanley_control去除负号即可

    // 计算当前路径曲率
    double path_curvature = CalculatePathCurvature(closest_idx_);

    // 计算自适应预瞄距离
    double current_speed = localization_info_msg_->vel_speed;
    // double adaptive_preview_dist = CalculateAdaptivePreviewDistance(current_speed, path_curvature);

    // 根据曲率动态调整控制器权重
    double curvature_based_weight = std::abs(path_curvature);
    const double CURVATURE_THRESHOLD = 0.05; // 曲率阈值

    // 在直线段增加Stanley控制器的权重，在弯道增加Pure Pursuit的权重
    double adaptive_pursuit_rate = pursuit_control_rate_;
    double adaptive_stanley_rate = stanley_control_rate_;
    if(pursuit_control_rate_ != 1.0 && stanley_control_rate_ != 1.0){
        if (curvature_based_weight < CURVATURE_THRESHOLD) {
            adaptive_pursuit_rate *= 0.7;
            adaptive_stanley_rate *= 1.3;
        } else {
            adaptive_pursuit_rate *= 1.3;
            adaptive_stanley_rate *= 0.7;
        }
    }

    // 计算横向误差增益
    double adaptive_lat_rate = sta_lat_rate_;
    // if (std::abs(current_speed) < 0.5) {
    //     // 低速时增大横向误差增益
    //     adaptive_lat_rate *= 2.0;
    // }

    // 根据速度动态调整heading_error_rate_
    heading_error_rate_ = CalculateAdaptiveHeadingErrorRate(current_speed);

    // 使用自适应参数计算控制输出
    double pursuit_control = -std::atan2(2 * wheelbase_ * std::sin(angular_error), preview_dist);
    double stanley_control =
        -(heading_error_rate_ * heading_error - std::atan(adaptive_lat_rate * lat_error / effective_stanley_spd));

    // 应用自适应权重
    double front_wheel_rad = adaptive_pursuit_rate * pursuit_control + adaptive_stanley_rate * stanley_control;

    // 添加前馈控制项
    double curvature_feedforward = std::atan2(wheelbase_ * path_curvature, 1.0);

    front_wheel_rad += feedforward_rate_ * curvature_feedforward;

    // 3.4 计算最终转向角，并限制在合理范围内
    double steer_angle = front_wheel_rad * 180.0 / M_PI;
>>>>>>> fef6ae7375b9adecd73bee1cc1972e3b5e403f0f
    // 零点漂移处理
    steer_angle += zero_point_draft_;
    // 自行车模型的转角偏差
    steer_angle *= turning_radius_ratio_;
    steer_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steer_angle)); // 限制在[-50, 50]度之间

    // 4. 赋值给控制命令
    control_cmd_msg_.steer_angle = steer_angle;

    if (g_debug_cnt % 5 == 0 && debug_log_file_.is_open()) {
        auto time_str = TimeToHumanReadable(this->now());
        auto feedback_steer_angle = chassis_info_msg_.steer_angle;
        
        // 获取当前车辆状态，用于记录日志
        VehicleState vehicle_state = ComputeVehicleState();
        double lat_error = vehicle_state.lateral_error;
        double heading_error = vehicle_state.heading_error * 180.0 / M_PI; // 转换为度
        
        // 为了保持日志格式一致，无论使用哪种控制器，我们都使用相同的列格式
        double pursuit_control_val = 0.0;
        double stanley_control_val = 0.0;

        
        // 获取当前的控制器类型标识
        std::string controller_type = use_lqr_controller_ ? "LQR" : "Hybrid";
        
        debug_log_file_ << time_str << "," << pursuit_control_rate_ << "," << stanley_control_rate_ << ","
                        << sta_lat_rate_ << "," << heading_error << "," << 0.0  // 方位角误差不再使用
                        << "," << lat_error << "," << pursuit_control_val << "," // 保持格式一致
                        << stanley_control_val << "," << steer_angle << "," << feedback_steer_angle << ","
                        << preview_dist << "," << preview_idx << "," << closest_idx_ << "," 
                        << adc_trajectory_msg_->points[preview_idx].east << ","
                        << adc_trajectory_msg_->points[preview_idx].north << "," 
                        << adc_trajectory_msg_->points[preview_idx].yaw << "," 
                        << adc_trajectory_msg_->points[closest_idx_].east << ","
                        << adc_trajectory_msg_->points[closest_idx_].north << "," 
                        << adc_trajectory_msg_->points[closest_idx_].yaw << "," 
                        << CalculatePathCurvature(closest_idx_) << ","
                        << cur_east << "," << cur_north << "," << localization_info_msg_->yaw << "," 
                        << controller_type;  // 使用控制器类型替代前馈控制项
        // 注意：不要在这里结束行，因为纵向控制器会追加更多数据
    }
}

void ControlNode::ChassisInfoCallback(const bot_msg::msg::ChassisInfo::SharedPtr msg) { chassis_info_msg_ = *msg; }

/**
 * @brief 基于阶梯阶跃响应的纵向控制器
 *
 */
void ControlNode::LongitudinalController() {
    static bool first_run = true;
    static double step_target_speed = 0.0;
    const double koffset = 0.2;
    const double SPEED_THRESHOLD = 0.01;
    const double STEP_SIZE = 0.5;
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
    double final_target_speed = adc_trajectory_msg_->points[closest_idx_].vel_speed;

    // 限制最终目标速度在合理范围内
    final_target_speed = std::max(min_linear_velocity_, std::min(max_linear_velocity_, final_target_speed));

    // 初始化阶梯目标速度
    if (first_run) {
        step_target_speed = 0.5;
        first_run = false;
    }

    // 阶梯目标速度更新逻辑
    if (final_target_speed > step_target_speed) {
        // 目标速度高于当前阶梯目标 - 需要加速
        if (current_speed >= step_target_speed - SPEED_THRESHOLD) {
            // 当前速度已达到阶梯目标，增加阶梯
            step_target_speed = std::min(step_target_speed + STEP_SIZE, final_target_speed);
        }
    } else if (final_target_speed < step_target_speed) {
        // 目标速度低于当前阶梯目标 - 需要减速
        if (current_speed <= step_target_speed + SPEED_THRESHOLD) {
            // 当前速度已降至阶梯目标，降低阶梯
            step_target_speed = std::max(step_target_speed - 3 * STEP_SIZE, final_target_speed);
        }
    } else {
        // 最终目标速度等于当前阶梯目标，无需调整
        step_target_speed = final_target_speed;
    }

    // 更新控制命令
    if (final_target_speed > 0.0) {
        control_cmd_msg_.speed = step_target_speed + koffset;
    } else {
        control_cmd_msg_.speed = step_target_speed;
    }
    if (g_debug_cnt % 5 == 0 && debug_log_file_.is_open()) {
        // 纵向控制器只负责追加速度相关数据，并结束行
        debug_log_file_ << "," << final_target_speed << "," << current_speed << ","
                        << final_target_speed - current_speed << "," << 0 << "," << step_target_speed << ","
                        << speed_pid_controller_->getIntegral() << std::endl;
    }
}

// void ControlNode::LongitudinalController() {
//     // 检查输入数据是否有效
//     if (!adc_trajectory_msg_ || !localization_info_msg_) {
//         RCLCPP_WARN(this->get_logger(), "LongitudinalController: Missing trajectory or localization data");
//         return;
//     }

//     // 检查轨迹点是否为空
//     if (adc_trajectory_msg_->points.empty()) {
//         RCLCPP_WARN(this->get_logger(), "LongitudinalController: Empty trajectory points");
//         return;
//     }

//     // 获取当前速度和目标速度
//     double current_speed = localization_info_msg_->vel_speed;
//     double target_speed = adc_trajectory_msg_->points[closest_idx_].vel_speed;

//     // 限制目标速度在合理范围内
//     target_speed = std::max(min_linear_velocity_, std::min(max_linear_velocity_, target_speed));

//     double dt = 0.02; // 0.02s

//     if (dt <= 0.0) {
//         RCLCPP_WARN(this->get_logger(), "Invalid time interval");
//         return;
//     }

//     // 计算速度误差
//     double speed_error = target_speed - current_speed;

//     // 使用带前馈的PID控制器计算加速度命令
//     double acceleration = speed_pid_controller_->computeWithFeedForward(speed_error, target_speed, dt);
//     // double acceleration = speed_pid_controller_->compute(speed_error, dt);

//     // 将加速度转换为目标速度
//     double target_speed_command = current_speed + acceleration * dt;

//     // 定义最小驱动速度阈值（需要根据实际车辆特性调整）
//     const double MIN_DRIVING_SPEED = 0.5; // 假设最小驱动速度为0.2m/s

//     // 如果目标速度大于0但小于最小驱动速度，则将其设置为最小驱动速度
//     if (target_speed > 0.01 && target_speed_command < MIN_DRIVING_SPEED) {
//         target_speed_command = MIN_DRIVING_SPEED;
//     } else if (target_speed < -0.01 && target_speed_command > -MIN_DRIVING_SPEED) {
//         // 处理反向运动的情况
//         target_speed_command = -MIN_DRIVING_SPEED;
//     } else if (std::abs(target_speed) <= 0.01) {
//         // 如果目标速度接近0，则完全停止
//         target_speed_command = 0.0;
//     }

//     // 应用速度限制
//     target_speed_command = std::max(min_linear_velocity_, std::min(max_linear_velocity_, target_speed_command));

//     // 应用速度平滑处理
//     // target_speed_command = SmoothSpeedCommand(target_speed_command);

//     // 定义执行器的速度分辨率
//     // const double SPEED_RESOLUTION = 0.1; // 假设执行器速度分辨率为0.1m/s

//     // // 将速度命令量化到最近的分辨率倍数
//     // target_speed_command = std::round(target_speed_command / SPEED_RESOLUTION) * SPEED_RESOLUTION;

//     // // 如果量化后的速度变化太小，强制使用下一个分辨率级别
//     // if (std::abs(target_speed_command - current_speed) < SPEED_RESOLUTION && std::abs(speed_error) > 0.01) {
//     //     if (speed_error > 0) {
//     //         target_speed_command = current_speed + SPEED_RESOLUTION;
//     //     } else {
//     //         target_speed_command = current_speed - SPEED_RESOLUTION;
//     //     }
//     // }

//     // // 定义积分饱和阈值
//     // const double INTEGRAL_SATURATION_THRESHOLD = 5.0; // 积分项达到5.0时认为饱和

//     // // 如果积分项饱和且速度误差仍然存在，使用阶跃响应
//     // if (std::abs(speed_pid_controller_->getIntegral()) > INTEGRAL_SATURATION_THRESHOLD && std::abs(speed_error) >
//     0.1) {
//     //     // 使用更大的速度步长
//     //     double step_size = MIN_DRIVING_SPEED * 1.5; // 使用比最小驱动速度更大的步长
//     //     if (speed_error > 0) {
//     //         target_speed_command = current_speed + step_size;
//     //     } else {
//     //         target_speed_command = current_speed - step_size;
//     //     }
//     // }

//     // 更新控制命令
//     control_cmd_msg_.speed = target_speed_command;

//     // 输出调试信息
//     if (g_debug_cnt % 10 == 0) {
//         RCLCPP_INFO(
//             this->get_logger(),
//             "Speed Control: target=%.2f, current=%.2f, error=%.2f, acceleration=%.2f, command=%.2f, integral=%.2f",
//             target_speed, current_speed, speed_error, acceleration, target_speed_command,
//             speed_pid_controller_->getIntegral());
//     }
//     if (g_debug_cnt % 10 == 0 && debug_log_file_.is_open()) {
//         debug_log_file_ << "," << target_speed << "," << current_speed << "," << speed_error << "," << acceleration
//                         << "," << target_speed_command << "," << speed_pid_controller_->getIntegral() << std::endl;
//     }
// }

// 根据速度动态调整heading_error_rate_
double ControlNode::CalculateAdaptiveHeadingErrorRate(double current_speed) {
    // 定义速度范围和对应的比率范围
    const double MIN_SPEED = 0.5; // 最低速度阈值，低于此速度使用最大比率
    const double MAX_SPEED = 5.0; // 最高速度阈值，高于此速度使用最小比率
    const double MIN_RATE = 0.4;  // 最小比率
    const double MAX_RATE = 1.0;  // 最大比率

    // 如果速度小于最低阈值，使用最大比率
    if (current_speed <= MIN_SPEED) {
        return MAX_RATE;
    }

    // 如果速度大于最高阈值，使用最小比率
    if (current_speed >= MAX_SPEED) {
        return MIN_RATE;
    }

    // 在速度范围内进行线性插值
    // 插值公式: rate = MAX_RATE - (current_speed - MIN_SPEED) * (MAX_RATE - MIN_RATE) / (MAX_SPEED - MIN_SPEED)
    double rate_range = MAX_RATE - MIN_RATE;
    double speed_range = MAX_SPEED - MIN_SPEED;
    double speed_factor = (current_speed - MIN_SPEED) / speed_range;

    double adaptive_rate = MAX_RATE - speed_factor * rate_range;

    return adaptive_rate;
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
    } else {
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
    this->declare_parameter<double>("feedforward_rate", 0.0);
    this->declare_parameter<double>("heading_error_rate", 0.0);
    this->declare_parameter<double>("sta_lat_rate", 0.1);
    this->declare_parameter<double>("turning_radius_ratio", 1.0);
    this->declare_parameter<double>("zero_point_draft", 0.0);
    this->declare_parameter("speed_pid_kp", 0.5);
    this->declare_parameter("speed_pid_ki", 0.1);
    this->declare_parameter("speed_pid_kd", 0.0);
    this->declare_parameter("speed_pid_kf", 0.5); // 前馈增益默认值
    this->declare_parameter<std::string>("adc_traj_topic_name", "/planing/adc_traj");
    this->declare_parameter<std::string>("control_cmd_topic_name", "/control/control_cmd");
    this->declare_parameter<std::string>("localization_info_topic_name", "/control/local_info");
    this->declare_parameter<std::string>("chassis_info_topic_name", "/control/chassis_info");
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
    feedforward_rate_ = this->get_parameter("feedforward_rate").get_value<double>();
    turning_radius_ratio_ = this->get_parameter("turning_radius_ratio").get_value<double>();
    zero_point_draft_ = this->get_parameter("zero_point_draft").get_value<double>();
    adc_traj_topic_name_ = this->get_parameter("adc_traj_topic_name").get_value<std::string>();
    control_cmd_topic_name_ = this->get_parameter("control_cmd_topic_name").get_value<std::string>();
    localization_info_topic_name_ = this->get_parameter("localization_info_topic_name").get_value<std::string>();
    chassis_info_topic_name_ = this->get_parameter("chassis_info_topic_name").get_value<std::string>();
    log_file_path_ = this->get_parameter("log_file_path").get_value<std::string>();
    speed_pid_kp_ = this->get_parameter("speed_pid_kp").as_double();
    speed_pid_ki_ = this->get_parameter("speed_pid_ki").as_double();
    speed_pid_kd_ = this->get_parameter("speed_pid_kd").as_double();    
    speed_pid_kf_ = this->get_parameter("speed_pid_kf").as_double();

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
    RCLCPP_INFO(this->get_logger(), "Feedforward rate: %f", feedforward_rate_);
    RCLCPP_INFO(this->get_logger(), "Chassis info topic name: %s", chassis_info_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Debug log file path: %s", log_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Speed pid kp: %f", speed_pid_kp_);
    RCLCPP_INFO(this->get_logger(), "Speed pid ki: %f", speed_pid_ki_);
    RCLCPP_INFO(this->get_logger(), "Speed pid kd: %f", speed_pid_kd_);
    RCLCPP_INFO(this->get_logger(), "Speed pid kf: %f", speed_pid_kf_);

    return;
}

double ControlNode::SmoothSpeedCommand(double raw_speed_command) {
    const double dt = 0.02; // 控制周期

    // 限制速度变化率
    double max_speed_change = max_speed_change_rate_ * dt;
    double speed_change = raw_speed_command - previous_speed_command_;

    if (std::abs(speed_change) > max_speed_change) {
        if (speed_change > 0) {
            raw_speed_command = previous_speed_command_ + max_speed_change;
        } else {
            raw_speed_command = previous_speed_command_ - max_speed_change;
        }
    }

    // 使用滑动窗口平均进行平滑
    speed_commands_buffer_.push_back(raw_speed_command);
    if (speed_commands_buffer_.size() > smooth_window_size_) {
        speed_commands_buffer_.pop_front();
    }

    // 计算平滑后的速度
    double smoothed_speed = 0.0;
    for (const auto &speed : speed_commands_buffer_) {
        smoothed_speed += speed;
    }
    smoothed_speed /= speed_commands_buffer_.size();

    // 更新上一次的速度命令
    previous_speed_command_ = smoothed_speed;

    return smoothed_speed;
}

// 修改曲率计算函数，使其返回带符号的曲率
double ControlNode::CalculatePathCurvature(size_t index) {
    // 需要至少三个点来计算曲率和方向
    if (index == 0 || index >= adc_trajectory_msg_->points.size() - 1) {
        return 0.0; // 无法计算或接近轨迹末端，视为直线
    }

    // 获取连续三个点
    const auto &p0 = adc_trajectory_msg_->points[index - 1]; // 前一个点
    const auto &p1 = adc_trajectory_msg_->points[index];     // 当前点 (closest_idx)
    const auto &p2 = adc_trajectory_msg_->points[index + 1]; // 后一个点

    // 将点转换为简单的2D向量，以p1为原点
    double x0 = p0.east - p1.east;
    double y0 = p0.north - p1.north;
    double x2 = p2.east - p1.east;
    double y2 = p2.north - p1.north;

    // 计算向量 P1P0 和 P1P2
    // P1P0: (x0, y0)
    // P1P2: (x2, y2)

    // 使用Menger曲率公式的近似 (适用于离散点)
    // K = 2 * |x1(y2 − y3) + x2(y3 − y1) + x3(y1 − y2)| / (sqrt((x1−x2)^2+(y1−y2)^2) * sqrt((x2−x3)^2+(y2−y3)^2) *
    // sqrt((x3−x1)^2+(y3−y1)^2)) 为了简化并获得符号，我们使用叉积的思想来判断方向 向量 p1->p0 和 p1->p2 叉积的 z 分量:
    // (x0 * y2 - y0 * x2) 如果这个值 > 0，表示从 p1->p0 到 p1->p2 是逆时针（左转弯） 如果这个值 < 0，表示从 p1->p0 到
    // p1->p2 是顺时针（右转弯）

    double cross_product_z = x0 * y2 - x2 * y0;

    // 计算三点构成的三角形面积的两倍（也与叉积相关）
    // area2 = |x0(y1 - y2) + x1(y2 - y0) + x2(y0 - y1)| where p1 is origin (0,0)
    // area2 = |x0(0 - y2) + 0 + x2(y0 - 0)| = |-x0y2 + x2y0| = |x2y0 - x0y2|
    // 上面计算的 cross_product_z 就是 (x2y0 - x0y2) 的相反数，所以符号也是相反的

    double dist_p0_p1 = std::hypot(x0, y0);
    double dist_p1_p2 = std::hypot(x2, y2);
    double dist_p0_p2 = std::hypot(p2.east - p0.east, p2.north - p0.north);

    if (dist_p0_p1 < 1e-6 || dist_p1_p2 < 1e-6 || dist_p0_p2 < 1e-6) {
        return 0.0; // 点重合或非常近，视为直线
    }

    // 使用Menger曲率的公式: K = 4 * Area / (a*b*c)
    // Area 是 p0, p1, p2 构成的三角形面积. 2 * Area = |x0*y2 - x2*y0|
    double area_triangle_times_2 = std::abs(cross_product_z);
    double curvature_magnitude = 2.0 * area_triangle_times_2 / (dist_p0_p1 * dist_p1_p2 * dist_p0_p2);

    // 根据叉积的符号确定曲率的符号
    // 假设左转为正曲率，右转为负曲率
    // 如果 cross_product_z > 0 (P1P0 到 P1P2 是逆时针)，则认为是左转 (正曲率)
    // (注意：这取决于坐标系的定义和期望的转向约定，可能需要调整符号)
    // 通常，如果车辆前进方向为X轴正向，左转弯的航向角增加，对应正的角速度/曲率。
    // 如果 cross_product_z > 0， (p0-p1) 向量转向 (p2-p1) 向量是逆时针。
    // 这通常对应于路径向左弯曲。

    // 我们需要根据车辆的航向来正确定义曲率符号。
    // 一个更通用的方法是判断中间点p1相对于线段p0p2的位置。
    // (y2-y0)*p1.x - (x2-x0)*p1.y + x2*y0 - y2*x0
    // 如果以p1为参考点，可以看 (p2-p1) 相对于 (p1-p0) 的转向
    // 向量 v1 = p1-p0 = (-x0, -y0)
    // 向量 v2 = p2-p1 = (x2, y2)
    // 叉积 v1 x v2 = (-x0)*y2 - (-y0)*x2 = y0*x2 - x0*y2
    // 这就是我们之前计算的 cross_product_z

    double signed_curvature = curvature_magnitude;
<<<<<<< HEAD
    if (cross_product_z > 0) {
        signed_curvature = -curvature_magnitude; // 右转 -> 负曲率
    } else if (cross_product_z < 0) {
        signed_curvature = curvature_magnitude;  // 左转 -> 正曲率
=======
    // 修正后的正确代码
    if (cross_product_z > 0) {
        // 向量 P1P0 -> P1P2 是逆时针，对应左转
        signed_curvature = -curvature_magnitude;  // 右转 -> 负曲率
    } else if (cross_product_z < 0) {
        // 向量 P1P0 -> P1P2 是顺时针，对应右转
        signed_curvature = curvature_magnitude; // 左转 -> 正曲率
>>>>>>> fef6ae7375b9adecd73bee1cc1972e3b5e403f0f
    } else {
        signed_curvature = 0.0; // 直线
    }
    // // 注意：如果 curvature_magnitude 已经为0（直线），符号无所谓

    return signed_curvature;
}

// 计算自适应预瞄距离
double ControlNode::CalculateAdaptivePreviewDistance(double current_speed, double path_curvature) {
    // 基础预瞄距离
    double base_preview = preview_time_ * current_speed;

    // 根据曲率调整预瞄距离
    const double MIN_PREVIEW_DISTANCE = 1.0; // 最小预瞄距离
    const double CURVATURE_FACTOR = 5.0;     // 曲率影响因子

    double preview_dist = base_preview / (1.0 + CURVATURE_FACTOR * std::abs(path_curvature));
    return std::max(MIN_PREVIEW_DISTANCE, preview_dist);
}

// // 添加转向角平滑函数
// double ControlNode::SmoothSteeringAngle(double target_angle, double dt) {
//     double angle_change = target_angle - previous_steering_angle_;
//     double max_change = max_steering_rate_ * dt;

//     if (std::abs(angle_change) > max_change) {
//         if (angle_change > 0) {
//             target_angle = previous_steering_angle_ + max_change;
//         } else {
//             target_angle = previous_steering_angle_ - max_change;
//         }
//     }

//     previous_steering_angle_ = target_angle;
//     return target_angle;
// }

ControlNode::~ControlNode() {
    if (debug_log_file_.is_open()) {
        debug_log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Debug log file closed: %s", log_file_path_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "control node shutting down");
}

// 计算车辆状态，用于LQR控制器
VehicleState ControlNode::ComputeVehicleState() {
    VehicleState state;
    
    // 获取当前车辆位置、航向和速度
    state.x = localization_info_msg_->east;
    state.y = localization_info_msg_->north;
    state.yaw = localization_info_msg_->yaw * M_PI / 180.0; // 转换为弧度
    state.velocity = localization_info_msg_->vel_speed;
    
    // 计算横向误差和航向误差
    if (adc_trajectory_msg_->points.empty() || closest_idx_ >= adc_trajectory_msg_->points.size()) {
        state.lateral_error = 0.0;
        state.heading_error = 0.0;
        state.lateral_error_rate = 0.0;
        state.heading_error_rate = 0.0;
        return state;
    }
    
    // 获取轨迹上的最近点
    const auto &closest_point = adc_trajectory_msg_->points[closest_idx_];
    
    // 计算路径切线方向
    double path_direction;
    if (closest_idx_ + 1 < adc_trajectory_msg_->points.size()) {
        // 使用前向点计算切线
        path_direction = std::atan2(
            adc_trajectory_msg_->points[closest_idx_ + 1].east - closest_point.east,
            adc_trajectory_msg_->points[closest_idx_ + 1].north - closest_point.north);
    } else if (closest_idx_ > 0) {
        // 使用后向点计算切线
        path_direction = std::atan2(
            closest_point.east - adc_trajectory_msg_->points[closest_idx_ - 1].east,
            closest_point.north - adc_trajectory_msg_->points[closest_idx_ - 1].north);
    } else {
        // 只有一个点，使用目标航向
        path_direction = closest_point.yaw * M_PI / 180.0;
    }
    path_direction = NormalizeAngle(path_direction);
    
    // 计算车辆到最近点的向量
    double dx = state.x - closest_point.east;
    double dy = state.y - closest_point.north;
    
    // 计算横向误差（向量在垂直于路径方向上的投影）
    state.lateral_error = std::sin(target_yaw_rad) * dy - std::cos(target_yaw_rad) * dx;
    
    // 计算航向误差
    double target_yaw = closest_point.yaw * M_PI / 180.0; // 转换为弧度
    state.heading_error = NormalizeAngle(target_yaw - state.yaw);
    
    // 计算横向误差变化率和航向误差变化率
    state.lateral_error_rate = ComputeLateralErrorRate(state.lateral_error);
    state.heading_error_rate = ComputeHeadingErrorRate(state.heading_error);
    
    return state;
}

// 计算横向误差变化率
double ControlNode::ComputeLateralErrorRate(double lateral_error) {
    // 使用当前误差和上一次误差计算变化率
    double dt = 0.02; // 控制周期，默认20ms
    double error_rate = (lateral_error - previous_lateral_error_) / dt;
    
    // 更新上一次误差
    previous_lateral_error_ = lateral_error;
    
    return error_rate;
}

// 计算航向误差变化率
double ControlNode::ComputeHeadingErrorRate(double heading_error) {
    // 使用当前误差和上一次误差计算变化率
    double dt = 0.02; // 控制周期，默认20ms
    
    // 需要处理角度跨越±π的情况
    double error_diff = NormalizeAngle(heading_error - previous_heading_error_);
    double error_rate = error_diff / dt;
    
    // 更新上一次误差
    previous_heading_error_ = heading_error;
    
    return error_rate;
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