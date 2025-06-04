#pragma once

#include <atomic>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "bot_msg/msg/remote_controller.hpp"


namespace remote_controller {


class RemoteControllerNode : public rclcpp::Node {
  public:
    RemoteControllerNode();
    ~RemoteControllerNode();

  private:
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 发布者
    rclcpp::Publisher<bot_msg::msg::RemoteController>::SharedPtr pub_remote_controller_;
    // 变量
    std::string can_device_name_;    // can设备名称
    int can_baudrate_;               // can波特率
    std::string remote_controller_topic_;  // 遥控器主题名称
    int can_fd_ = 0;
    std::atomic<bool> running_; // 控制线程运行的标志位
    std::thread can_thread_;


    // 回调函数
    void TimerCallback();

    // 功能函数
    void InitParams();
    bool InitCanSocket(std::string can_device_name, int can_baudrate);
    void CanThreadFunc();

    private:
    uint8_t key_value_; // 按键的键盘值

};
} // namespace remote_controller