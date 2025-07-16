
#pragma once


#include "rclcpp/rclcpp.hpp"
#include  <iomanip>
#include  <stdint.h>
#include  <fcntl.h>
#include  <sys/socket.h>
#include  <sys/ioctl.h>
#include  <linux/can.h>
#include  <linux/can/raw.h>
#include  <net/if.h>
#include  <unistd.h>
#include "std_msgs/msg/int32.hpp"



namespace can1_bus{

#define KEY_CAN_ID  0x0c20122d      //遥控器的ID

class Can1busNode : public rclcpp::Node
{

public:

   Can1busNode();

   ~Can1busNode();


private:


std::string can1_device_name_ ;    
int can_baudrate_;
int can_fd = 0 ;  //can1 接口的文件，描述符
std::thread can1_thread_ ;
int key_num ; //用于发布按键数值



void  InitParams();
bool  InitCansocket(std::string can_device_name ,int baudrate);
void  Can1ThreadFunc();
void  PrintCanDataFrame(const struct can_frame &frame);
void  timer_callback();

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

};

}