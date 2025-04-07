#pragma once

#include <bot_msg/msg/chassis_info.hpp>
#include <bot_msg/msg/control_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>

namespace canbus {

// TODO(Yangsh) 
struct CanRecvMsg {};


class CanbusNode : public rclcpp::Node {
  public:
    CanbusNode();
    ~CanbusNode();

  private:
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    // 订阅者
    rclcpp::Subscription<bot_msg::msg::ControlCmd>::SharedPtr sub_control_cmd_;
    // 发布者
    rclcpp::Publisher<bot_msg::msg::ChassisInfo>::SharedPtr pub_chassis_info_;
    // 变量
    std::string can_device_name_;         // can设备名称
    int can_baudrate_;                    // can波特率
    std::string control_cmd_topic_;  // 控制指令主题名称
    std::string chassis_info_topic_; // 底盘信息主题名称
    int can_fd_ = 0;                      // can fd

    // 回调函数
    void TimerCallback();
    void ControlCmdCallback(const bot_msg::msg::ControlCmd::SharedPtr msg);

    // 功能函数
    void InitParams();
    bool InitCanSocket(std::string can_device_name, int can_baudrate);
    void CanThreadFunc();
};
} // namespace canbus