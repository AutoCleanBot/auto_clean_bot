#include "local_record_test/local_record_test.h"

namespace test_ns{
LocalRecordTest::LocalRecordTest():Node("local_record_test"){
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&LocalRecordTest::TimerCallback, this));
    pub_localization_info_ = this->create_publisher<bot_msg::msg::LocalizationInfo>("/localization/rtk_info", 10);
}

void LocalRecordTest::TimerCallback() {
    static int count = 0;
    static int count_10 = 0;
    bot_msg::msg::LocalizationInfo localization_info_msg;
    // localization_info_msg.north = 10.0;
    // localization_info_msg.east = 0.0;
    localization_info_msg.east =  4.66374;
    localization_info_msg.north = 40.0285;
    localization_info_msg.yaw = 42.274;
    if(count % 20 == 0){
        count_10++;
    }
    localization_info_msg.vel_speed = 2.08139;
    if(localization_info_msg.vel_speed >= 3.0){
        localization_info_msg.vel_speed = 3.0;
    }
    // localization_info_msg.timestamp = std::chrono::system_clock::now();
    pub_localization_info_->publish(localization_info_msg);
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