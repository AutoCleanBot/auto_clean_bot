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
        //参数初始化
        init_parameters();

        // 启动CanDevice类的线程
        can_device_thread = new std::thread([this]() {
            can_device = std::make_unique<CanDevice>();
        });
        can_device_thread->detach();

        // 创建订阅者  订阅控制指令
        subscription = this->create_subscription<ControlCmd>(  
            "/controlCmd", 10,  
            std::bind(&CanNode::controlCmd_callback, this, std::placeholders::_1));  

        // 创建发布者  发布底盘信息
        publisher = create_publisher<ChassisInfo>("/chassisInfo", 10);
        //定时发布底盘信息
        timer = this->create_wall_timer(10ms, std::bind(&CanNode::timer_callback, this));
    }

    ~CanNode()  
    {  
        RCLCPP_INFO(this->get_logger(),"can_node destroyed!");  
        if(can_device_thread->joinable()){
            can_device_thread->join();
        }
        delete can_device_thread;
        can_device_thread = nullptr;
    }  
  
private:
    void init_parameters()
    {
        // 读取参数
        declare_parameter("max_steer_angle",max_steer_angle);
        declare_parameter("max_steer_angle_spd", max_steer_angle_spd);
        declare_parameter("min_steer_angle_spd", min_steer_angle_spd);
        declare_parameter("max_acc", max_acc);
        declare_parameter("min_acc", min_acc);
        declare_parameter("max_brake_pressure",max_brake_pressure);
        declare_parameter("driving_mode", driving_mode);

        get_parameter("max_steer_angle",max_steer_angle);
        get_parameter("max_steer_angle_spd", max_steer_angle_spd);
        get_parameter("min_steer_angle_spd", min_steer_angle_spd);
        get_parameter("max_acc", max_acc);
        get_parameter("min_acc", min_acc);
        get_parameter("max_brake_pressure",max_brake_pressure);
        get_parameter("driving_mode", driving_mode);

        // 使用RCLCPP_INFO打印参数  
        RCLCPP_INFO(this->get_logger(), "Max Steer Angle: %f", max_steer_angle);  
        RCLCPP_INFO(this->get_logger(), "Max Steer Angle Speed: %f", max_steer_angle_spd);  
        RCLCPP_INFO(this->get_logger(), "Min Steer Angle Speed: %f", min_steer_angle_spd);  
        RCLCPP_INFO(this->get_logger(), "Max Acceleration: %f", max_acc);  
        RCLCPP_INFO(this->get_logger(), "Min Acceleration: %f", min_acc);  
        RCLCPP_INFO(this->get_logger(), "Max Brake Pressure: %f", max_brake_pressure);  
        RCLCPP_INFO(this->get_logger(), "Driving Mode: %d", driving_mode);
    }
    void timer_callback()
    {
        // VehicleData vehicleData =  canCtrl->getChassisInfo();
        // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", vehicleData.readytatus); 

        auto chassisInfo = ChassisInfo();
        // message.data = "Hello, world! ";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count++);
        //底盘信息
        publisher->publish(chassisInfo);
    }  
    void controlCmd_callback(const ControlCmd::SharedPtr msg) const  
    {  
        //can_device->setControlCmdInfo(msg);  
        RCLCPP_INFO(this->get_logger(), "Speed:%d'", msg->speed);  
    }  
    // 类的实例  
    std::unique_ptr<CanDevice> can_device; 
    rclcpp::Publisher<ChassisInfo>::SharedPtr publisher;  
    rclcpp::Subscription<ControlCmd>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer; 
    size_t count;  
    //vehicle_parameters 
    float max_steer_angle;
    float max_steer_angle_spd;
    float min_steer_angle_spd;
    float max_acc;
    float min_acc;
    float max_brake_pressure;
    uint8_t driving_mode;
    std::thread *can_device_thread;
};  
  
int main(int argc, char *argv[])  
{  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<CanNode>());  
    rclcpp::shutdown();  
    return 0;  
}