#include "local_record_test/local_record_test.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace test_ns{
LocalRecordTest::LocalRecordTest():Node("local_record_test"){
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&LocalRecordTest::TimerCallback, this));
    pub_localization_info_ = this->create_publisher<bot_msg::msg::LocalizationInfo>("/localization/rtk_info", 10);
    
    // 添加PoseStamped消息的发布者
    pub_gnss_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss/pose", 10);
    
    RCLCPP_INFO(this->get_logger(), "发布器初始化完成，将同时发布 /localization/rtk_info 和 /gnss/pose 消息");
}

void LocalRecordTest::TimerCallback() {
    static int count = 0;
    static int count_10 = 0;
    
    // 当前时间戳
    rclcpp::Time current_time = this->now();
    
    // 创建并填充定位信息消息
    bot_msg::msg::LocalizationInfo localization_info_msg;
    // localization_info_msg.north = 10.0;
    // localization_info_msg.east = 0.0;
    localization_info_msg.north =  43.184;
    localization_info_msg.east = 8.613;
    localization_info_msg.up = 0.0;  // 添加高度信息
    localization_info_msg.yaw = 45.0;
    localization_info_msg.roll = 0.0;  // 添加横滚角
    localization_info_msg.pitch = 0.0; // 添加俯仰角
    
    if(count % 20 == 0){
        count_10++;
    }
    localization_info_msg.vel_speed = static_cast<float>(count_10)*0.1;
    if(localization_info_msg.vel_speed >= 3.0){
        localization_info_msg.vel_speed = 3.0;
    }
    
    // 设置header
    localization_info_msg.header.stamp = current_time;
    localization_info_msg.header.frame_id = "map";
    
    // 创建并填充PoseStamped消息
    geometry_msgs::msg::PoseStamped gnss_pose_msg;
    gnss_pose_msg.header.stamp = current_time;
    gnss_pose_msg.header.frame_id = "map";
    
    // 设置位置信息（与localization_info_msg保持一致）
    gnss_pose_msg.pose.position.x = localization_info_msg.east;    // 东向位置
    gnss_pose_msg.pose.position.y = localization_info_msg.north;   // 北向位置
    gnss_pose_msg.pose.position.z = localization_info_msg.up;      // 上方位置
    
    // 从欧拉角计算四元数并设置姿态
    tf2::Quaternion q;
    q.setRPY(
        0,   // 横滚角（弧度）
        0,  // 俯仰角（弧度）
        -localization_info_msg.yaw * M_PI / 180.0     // 偏航角（弧度）
    );
    
    gnss_pose_msg.pose.orientation.x = q.x();
    gnss_pose_msg.pose.orientation.y = q.y();
    gnss_pose_msg.pose.orientation.z = q.z();
    gnss_pose_msg.pose.orientation.w = q.w();
    
    // 发布两种消息
    pub_localization_info_->publish(localization_info_msg);
    pub_gnss_pose_->publish(gnss_pose_msg);
    
    count++; 
}

} // namespace test_ns


// 节点注册
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test_ns::LocalRecordTest>();
    RCLCPP_INFO(node->get_logger(), "local_record_test node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}