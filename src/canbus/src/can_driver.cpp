// subscriber_member_function.cpp  
#include <chrono>  
#include <functional>  
#include <memory>  
#include <string>  
  
#include "rclcpp/rclcpp.hpp"  
#include "std_msgs/msg/string.hpp" 
#include "can_device.hpp"
#include "bot_msg/msg/chassis_info.hpp"
#include "bot_msg/msg/control_cmd.hpp"

using bot_msg::msg::ChassisInfo;
using bot_msg::msg::ControlCmd;
using namespace std::chrono_literals;  

  
class CanNode : public rclcpp::Node  
{  
public:  
    CanNode()  
        : Node("node_can"),count(0) 
    {  
        // 实例化你的自定义类  
        can_device = std::make_unique<CanDevice>(); 

        subscription = this->create_subscription<ControlCmd>(  
            "/chassisInfo", 10,  
            std::bind(&CanNode::controlCmd_callback, this, std::placeholders::_1));  

        publisher = create_publisher<ChassisInfo>("/controlCmd", 10);
        timer = this->create_wall_timer(100ms, std::bind(&CanNode::timer_callback, this));
    }  
  
private:
    void timer_callback()
    {
        // VehicleData vehicleData =  canCtrl->getChassisInfo();
        // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", vehicleData.readytatus); 

        auto chassisInfo = ChassisInfo();
        // message.data = "Hello, world! ";
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count++);
        publisher->publish(chassisInfo);
    }  
    void controlCmd_callback(const ControlCmd::SharedPtr msg) const  
    {  
        can_device->setControlCmdInfo(msg);  
        RCLCPP_INFO(this->get_logger(), "Speed:%d'", msg->speed);  
    }  
    // 类的实例  
    std::unique_ptr<CanDevice> can_device; 
    rclcpp::Publisher<ChassisInfo>::SharedPtr publisher;  
    rclcpp::Subscription<ControlCmd>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer; 
    size_t count;  
};  
  
int main(int argc, char *argv[])  
{  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<CanNode>());  
    rclcpp::shutdown();  
    return 0;  
}