#ifndef CONTROL_TEST_NODE_H
#define CONTROL_TEST_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <bot_msg/msg/localization_info.hpp>
#include <bot_msg/msg/adc_trajectory.hpp>
#include <fstream>
#include <vector>
#include <string>

class LocalizationInfoSubscriber : public rclcpp::Node
{
public:
    LocalizationInfoSubscriber()
        : Node("localization_info_subscriber"), file_("localization_data.txt", std::ios::out | std::ios::app)
    {
        subscription_ = this->create_subscription<bot_msg::msg::LocalizationInfo>(
            "localization_info_topic", 10, std::bind(&LocalizationInfoSubscriber::topic_callback, this, std::placeholders::_1));

        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
        } else {
            RCLCPP_INFO(this->get_logger(), "File opened successfully.");
        }
    }

private:
    void topic_callback(const bot_msg::msg::LocalizationInfo::SharedPtr msg) const
    {
        double north = msg->north;
        double east = msg->east;
        double up = msg->up;
        float yaw = msg->yaw;
        float pitch = msg->pitch;
        float roll = msg->roll;

        // Write data to file
        file_ << north << "," << east << "," << up << "," << yaw << "," << pitch << "," << roll << "\n";

        RCLCPP_INFO(this->get_logger(), "Received data: north=%f, east=%f, up=%f, yaw=%f, pitch=%f, roll=%f",
                    north, east, up, yaw, pitch, roll);
    }

    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr subscription_;
    mutable std::ofstream file_;
};

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher()
        : Node("trajectory_publisher"), file_("localization_data.txt", std::ios::in)
    {
        // 从参数服务器读取话题名称
        declare_parameter("trajectory_topic", "default_trajectory_topic");
        std::string topic_name = get_parameter("trajectory_topic").as_string();

        publisher_ = this->create_publisher<bot_msg::msg::ADCTrajectory>(topic_name, 10);

        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for reading.");
        } else {
            RCLCPP_INFO(this->get_logger(), "File opened successfully.");
            read_trajectory_data();
            publish_trajectory();
        }
    }

private:
    void read_trajectory_data()
    {
        std::string line;
        while (std::getline(file_, line)) {
            std::stringstream ss(line);
            double north, east, up;
            float yaw, pitch, roll;
            char delimiter;

            ss >> north >> delimiter >> east >> delimiter >> up >> delimiter >> yaw >> delimiter >> pitch >> delimiter >> roll;

            bot_msg::msg::ADCTrajectory point;
            point.north = north;
            point.east = east;
            point.up = up;
            point.yaw = yaw;
            point.pitch = pitch;
            point.roll = roll;

            trajectory_data_.push_back(point);
        }
    }

    void publish_trajectory()
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&TrajectoryPublisher::timer_callback, this));
    }

    void timer_callback()
    {
        if (index_ < trajectory_data_.size()) {
            publisher_->publish(trajectory_data_[index_]);
            RCLCPP_INFO(this->get_logger(), "Published data: north=%f, east=%f, up=%f, yaw=%f, pitch=%f, roll=%f",
                        trajectory_data_[index_].north, trajectory_data_[index_].east, trajectory_data_[index_].up,
                        trajectory_data_[index_].yaw, trajectory_data_[index_].pitch, trajectory_data_[index_].roll);
            index_++;
        } else {
            RCLCPP_INFO(this->get_logger(), "All data published.");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<bot_msg::msg::ADCTrajectory>::SharedPtr publisher_;
    std::ifstream file_;
    std::vector<bot_msg::msg::ADCTrajectory> trajectory_data_;
    size_t index_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
};
#endif // CONTROL_TEST_NODE_H