#include "rtk_simulator/rtk_simulator.h"
#include <pwd.h>
#include <unistd.h>

namespace rtk_simulator {

// 辅助函数：展开波浪号路径
std::string expandTilde(const std::string &path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    const char *home = getenv("HOME");
    if (home == nullptr) {
        struct passwd *pwd = getpwuid(getuid());
        if (pwd) {
            home = pwd->pw_dir;
        }
    }

    if (home == nullptr) {
        return path;
    }

    if (path.length() == 1) {
        return home;
    }
    if (path[1] == '/') {
        return std::string(home) + path.substr(1);
    }
    return path;
}

RTKSimulator::RTKSimulator() : Node("rtk_simulator"), gen_(rd_()) {
    initParams();

    // 创建发布者
    pub_localization_ = this->create_publisher<bot_msg::msg::LocalizationInfo>("localization/rtk_info", 10);
    pub_gnss_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(gnss_topic_name_, 10);

    // 加载轨迹数据
    if (!loadTrajectoryFromCSV(csv_file_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from: %s", csv_file_path_.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points from: %s", trajectory_points_.size(),
                csv_file_path_.c_str());

    // 初始化随机数分布
    noise_dist_ = std::uniform_real_distribution<double>(noise_min_percentage_ / 100.0, noise_max_percentage_ / 100.0);

    // 创建定时器
    int timer_interval_ms = static_cast<int>(1000.0 / publish_frequency_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms),
                                     std::bind(&RTKSimulator::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "RTK Simulator started, publishing at %.1f Hz", publish_frequency_);
    RCLCPP_INFO(this->get_logger(), "Noise range: %.1f%% - %.1f%%", noise_min_percentage_, noise_max_percentage_);
}

RTKSimulator::~RTKSimulator() { RCLCPP_INFO(this->get_logger(), "RTK Simulator stopped"); }

void RTKSimulator::initParams() {
    // 声明参数
    this->declare_parameter("csv_file_path", "~/auto_clean_bot/path/local_record_2.csv");
    this->declare_parameter("publish_frequency", 10.0);
    this->declare_parameter("noise_min_percentage", 1.0);
    this->declare_parameter("noise_max_percentage", 5.0);
    this->declare_parameter("loop_trajectory", true);
    this->declare_parameter("gnss_topic_name", "/gnss/pose");
    this->declare_parameter("gnss_frame_id", "map");

    // 获取参数
    csv_file_path_ = expandTilde(this->get_parameter("csv_file_path").as_string());
    publish_frequency_ = this->get_parameter("publish_frequency").as_double();
    noise_min_percentage_ = this->get_parameter("noise_min_percentage").as_double();
    noise_max_percentage_ = this->get_parameter("noise_max_percentage").as_double();
    loop_trajectory_ = this->get_parameter("loop_trajectory").as_bool();
    gnss_topic_name_ = this->get_parameter("gnss_topic_name").as_string();
    gnss_frame_id_ = this->get_parameter("gnss_frame_id").as_string();

    current_point_index_ = 0;
}

bool RTKSimulator::loadTrajectoryFromCSV(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", file_path.c_str());
        return false;
    }

    std::string line;
    bool header_skipped = false;

    trajectory_points_.clear();

    while (std::getline(file, line)) {
        if (!header_skipped) {
            header_skipped = true;
            continue; // 跳过表头
        }

        std::stringstream ss(line);
        std::string value;
        TrajectoryPoint point;

        try {
            // 按照CSV格式解析数据 (参考routing中的解析方式)
            std::getline(ss, value, ',');
            point.longtitude = std::stod(value);
            std::getline(ss, value, ',');
            point.latitude = std::stod(value);
            std::getline(ss, value, ',');
            point.altitude = std::stod(value);
            std::getline(ss, value, ',');
            point.north = std::stod(value);
            std::getline(ss, value, ',');
            point.east = std::stod(value);
            std::getline(ss, value, ',');
            point.up = std::stod(value);
            std::getline(ss, value, ',');
            point.yaw = std::stod(value);
            std::getline(ss, value, ',');
            point.pitch = std::stod(value);
            std::getline(ss, value, ',');
            point.roll = std::stod(value);
            std::getline(ss, value, ',');
            point.vel_speed = std::stod(value);
            std::getline(ss, value, ',');
            point.vel_north = std::stod(value);
            std::getline(ss, value, ',');
            point.vel_east = std::stod(value);
            std::getline(ss, value, ',');
            point.vel_up = std::stod(value);
            std::getline(ss, value, ',');
            point.acc_x = std::stod(value);
            std::getline(ss, value, ',');
            point.acc_y = std::stod(value);
            std::getline(ss, value, ',');
            point.acc_z = std::stod(value);
            std::getline(ss, value, ',');
            point.gyro_x = std::stod(value);
            std::getline(ss, value, ',');
            point.gyro_y = std::stod(value);
            std::getline(ss, value, ',');
            point.gyro_z = std::stod(value);
            std::getline(ss, value, ',');
            point.rtk_status = std::stoi(value);

            trajectory_points_.push_back(point);
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing line: %s, error: %s", line.c_str(), e.what());
            continue;
        }
    }

    file.close();
    return !trajectory_points_.empty();
}

void RTKSimulator::timerCallback() {
    if (trajectory_points_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No trajectory points available");
        return;
    }

    // 检查是否到达轨迹末尾
    if (current_point_index_ >= trajectory_points_.size()) {
        if (loop_trajectory_) {
            current_point_index_ = 0; // 循环播放
            RCLCPP_INFO(this->get_logger(), "Trajectory loop restarted");
        } else {
            RCLCPP_INFO(this->get_logger(), "Trajectory playback completed");
            return;
        }
    }

    // 获取当前轨迹点
    const TrajectoryPoint &traj_point = trajectory_points_[current_point_index_];

    // 创建LocalizationInfo消息
    bot_msg::msg::LocalizationInfo msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    // 填充基础数据
    msg.longtitude = traj_point.longtitude;
    msg.latitude = traj_point.latitude;
    msg.altitude = traj_point.altitude;
    msg.north = traj_point.north;
    msg.east = traj_point.east;
    msg.up = traj_point.up;
    msg.yaw = static_cast<float>(traj_point.yaw);
    msg.pitch = static_cast<float>(traj_point.pitch);
    msg.roll = static_cast<float>(traj_point.roll);
    msg.vel_speed = static_cast<float>(traj_point.vel_speed);
    msg.vel_north = static_cast<float>(traj_point.vel_north);
    msg.vel_east = static_cast<float>(traj_point.vel_east);
    msg.vel_up = static_cast<float>(traj_point.vel_up);
    msg.acc_x = static_cast<float>(traj_point.acc_x);
    msg.acc_y = static_cast<float>(traj_point.acc_y);
    msg.acc_z = static_cast<float>(traj_point.acc_z);
    msg.gyro_x = static_cast<float>(traj_point.gyro_x);
    msg.gyro_y = static_cast<float>(traj_point.gyro_y);
    msg.gyro_z = static_cast<float>(traj_point.gyro_z);
    msg.rtk_status = static_cast<uint8_t>(traj_point.rtk_status);

    // 添加随机噪声
    addRandomNoise(msg);

    // 创建并发布 GNSS pose 消息
    geometry_msgs::msg::PoseStamped gnss_pose_msg;
    gnss_pose_msg.header.stamp = msg.header.stamp;
    gnss_pose_msg.header.frame_id = gnss_frame_id_;

    // 设置位置信息（与 localization_info 保持一致）
    gnss_pose_msg.pose.position.x = msg.east;
    gnss_pose_msg.pose.position.y = msg.north;
    gnss_pose_msg.pose.position.z = msg.up;

    // 设置姿态信息（从 yaw 角度转换为四元数）
    tf2::Quaternion orientation;
    // 注意：RTK中航向偏转为顺时针，ROS默认为逆时针，所以需要取反
    orientation.setRPY(0, 0, -msg.yaw * M_PI / 180.0);
    gnss_pose_msg.pose.orientation.x = orientation.x();
    gnss_pose_msg.pose.orientation.y = orientation.y();
    gnss_pose_msg.pose.orientation.z = orientation.z();
    gnss_pose_msg.pose.orientation.w = orientation.w();

    // 发布消息
    pub_localization_->publish(msg);
    pub_gnss_pose_->publish(gnss_pose_msg);

    // 移动到下一个点
    current_point_index_++;

    // 每100个点输出一次进度信息
    if (current_point_index_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Published point %zu/%zu", current_point_index_, trajectory_points_.size());
    }
}

void RTKSimulator::addRandomNoise(bot_msg::msg::LocalizationInfo &msg) {
    // 为坐标和航向添加随机偏移
    msg.longtitude = generateRandomOffset(msg.longtitude, noise_dist_(gen_));
    msg.latitude = generateRandomOffset(msg.latitude, noise_dist_(gen_));
    msg.north = generateRandomOffset(msg.north, noise_dist_(gen_));
    msg.east = generateRandomOffset(msg.east, noise_dist_(gen_));
    msg.yaw = static_cast<float>(generateRandomOffset(msg.yaw, noise_dist_(gen_)));

    // 为速度添加较小的噪声
    msg.vel_speed = static_cast<float>(generateRandomOffset(msg.vel_speed, noise_dist_(gen_) * 0.5));
    msg.vel_north = static_cast<float>(generateRandomOffset(msg.vel_north, noise_dist_(gen_) * 0.5));
    msg.vel_east = static_cast<float>(generateRandomOffset(msg.vel_east, noise_dist_(gen_) * 0.5));
}

double RTKSimulator::generateRandomOffset(double base_value, double noise_percentage) {
    // 生成 -noise_percentage 到 +noise_percentage 的随机偏移
    std::uniform_real_distribution<double> offset_dist(-noise_percentage, noise_percentage);
    double offset = offset_dist(gen_);
    return base_value * (1.0 + offset);
}

} // namespace rtk_simulator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rtk_simulator::RTKSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
