#include <bot_msg/msg/localization_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>


namespace test_ns{
class LocalRecordTest : public rclcpp::Node{
    public:
        LocalRecordTest();
        // ~LocalRecordTest();
    private:
        void TimerCallback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<bot_msg::msg::LocalizationInfo>::SharedPtr pub_localization_info_;  
};
}