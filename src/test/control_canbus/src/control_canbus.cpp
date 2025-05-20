#include "bot_msg/msg/control_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>

class ControlCanbus : public rclcpp::Node {
  public:
    ControlCanbus() : Node("control_canbus") {
        RCLCPP_INFO(this->get_logger(), "ControlCanbus node started");
        pub_ = this->create_publisher<bot_msg::msg::ControlCmd>("/control/control_cmd", 10);
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ControlCanbus::TimerCallback, this));
    }

  private:
    void TimerCallback(){
        auto msg = std::make_shared<bot_msg::msg::ControlCmd>();
        if(time_count % 50 == 0){
            steer_angle += 50;
            if(steer_angle > 300){
                steer_angle = 300;
            }
            spd += 0.5;
            if(spd > 4.0){
                spd = 4.0;
            }
        }
        steer_angle = -400;
        msg->steer_angle = steer_angle / 10.0;
        msg->brk = 0;
        msg->thr = 0;
        msg->gear = 1;
        msg->speed = 1.0;
        pub_->publish(*msg);
        ++time_count;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bot_msg::msg::ControlCmd>::SharedPtr pub_;
    double steer_angle = -300;
    double spd = 0;
    int time_count = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlCanbus>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}