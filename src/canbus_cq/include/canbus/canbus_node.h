#pragma once

#include "bot_msg/msg/chassis_info.hpp"
#include "bot_msg/msg/control_cmd.hpp"
#include "bot_msg/msg/remote_controller.hpp"
#include <std_msgs/msg/detail/int32__struct.hpp>
#include <std_msgs/msg/int32.hpp>
#include "canbus/can_protocol.h"
#include <atomic>
#include <cstring>
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace canbus {

/**
 * 来自底盘CAN信息消息集合,目前将几个消息合并为一个消息
 */
struct ChassisInfoLocal {
    uint8_t auto_enable;           // 是否允许进入自动模式状态：1允许，0不允许
    uint8_t whole_mode;            // 整车模式状态：1自动，0手动
    uint8_t emer_stop_mode;        // 急停开关状态：1可以动作，0不动作
    uint8_t service_brake_status;  // 行车制动状态：1制动，0释放
    uint8_t parking_brake_status;  // 驻车制动状态：1释放，0制动
    uint8_t forward_gear_feedback; // 前进档位状态反馈：1有效，0无效
    uint8_t reverse_gear_feedback; // 后退档位状态反馈：1有效，0无效
    uint8_t safety_edge_status;    // 安全触边状态：1触发，0未触发

    double service_brake_percentage_feedback; // 行车制动率百分比反馈：0-255对于100%
    double speed_feedback;                    // 车辆速度 m/s
    double steering_wheel_angle;              // 转向轮转向角度 -5700 ~ 5700 对应 -57° ~ 57°
    double soc;                               // SOC (State of Charge): 范围 0~250, 比例因子 0.4%/bit, 实际量程 0~100%
};

class CanbusNode : public rclcpp::Node {
  public:
    CanbusNode();
    ~CanbusNode();

  private:
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    // 订阅者
    rclcpp::Subscription<bot_msg::msg::ControlCmd>::SharedPtr sub_control_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_remote_controller_;
    // 发布者
    rclcpp::Publisher<bot_msg::msg::ChassisInfo>::SharedPtr pub_chassis_info_;
    // 变量
    std::string can_device_name_;    // can设备名称
    int can_baudrate_;               // can波特率
    std::string control_cmd_topic_;  // 控制指令主题名称
    std::string chassis_info_topic_; // 底盘信息主题名称
    std::string remote_controller_topic_; // 遥控器主题名称
    int can_fd_ = 0;
    std::atomic<bool> running_; // 控制线程运行的标志位
    std::thread can_thread_;
    ChassisInfoLocal chassis_info_local_;

    uint32_t control_cmd_cnt_;
    uint16_t motor_en_cnt_;     
    bool mannula_control_flag_;    // 是否允许can控制

    // 回调函数
    void TimerCallback();
    void ControlCmdCallback(const bot_msg::msg::ControlCmd::SharedPtr msg);
    void RemoteControllerCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // 功能函数
    void InitParams();
    bool InitCanSocket(std::string can_device_name, int can_baudrate);
    void CanThreadFunc();
    void FillCanCtrlCmd(uint8_t data[8], double steer_angle, double brk, uint8_t gear, double spd);
    void SendCtrlMsg(double steer_angle, double brk, uint8_t gear, double spd);
    void FillChassisInfo(bot_msg::msg::ChassisInfo::SharedPtr msg);
    void PrintCanDataFrame(const struct can_frame &frame);
    void RemoteControlCallback(const bot_msg::msg::RemoteController::SharedPtr msg);
};
} // namespace canbus