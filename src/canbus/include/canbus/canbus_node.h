#pragma once

#include "bot_msg/msg/chassis_info.hpp"
#include "bot_msg/msg/control_cmd.hpp"
#include "canbus/can_protocol.h"
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>

namespace canbus {

/**
 * 来自底盘CAN信息消息集合,目前将几个消息合并为一个消息
 */
struct ChassisInfoLocal {
    uint8_t ses_en_sts; // 转向电机使能状态, 0: 禁用, 1: 启用
    uint8_t moter_en_sts; // 驱动电机使能状态, 0: 禁用, 1: 启用
    uint8_t motor_torque; // 驱动电机扭矩, 0~255, 单位0.1Nm
    uint8_t vcu_mode; // VCU模式, 0-初始化；1-人工遥控模式；2-线控模式；3-紧急制动模式；4-预备切换模式
    uint8_t rseb_mode; // 线控制动模式：0-预留，1-位置模式，2-压力模式
    uint8_t bat_num; // 当前使用电池包，1-1号，2-2号
    uint8_t cur_gear; // 当前档位，0-N挡，1-D档，2-R档
    uint8_t bat_warn_sts; // 0-无报警，1-电量低，2-急需充电,3-预留
    uint8_t cur_spd_mode; // 当前速度模式，0-低速模式，1-中速模式，2-高速模式
    uint8_t controller_online_sts; // 控制器在线状态，0-离线，1-在线
    uint8_t ipc_online_sts; // IPC在线状态，0-离线，1-在线
    uint8_t soc; // 电池剩余容量, 0~100, 单位%, 0.4%精度
    uint8_t park_st; // 停车状态, 0-抱死,1-释放
    uint8_t coll_sts; // 碰撞状态, 0-无碰撞, 1-碰撞
    uint8_t bms_sts; // 充电状态，0-未充电，1-正在充电，2-充电完成，3-无效
    uint8_t motor_dir; // 0-静止，1-前进，2-后退,3-预留
    uint8_t auto_mode; // 自动驾驶开关，0-关闭，1-开启
    uint8_t auto_mode_en; // 自动驾驶使能，0-禁用，1-启用
    uint8_t error_level; // 底盘故障等级
    uint8_t error_eps_level; // 线控转向故障等级
    uint8_t error_eb_level; // 线控制动故障等级
    uint8_t error_bms_level; // BMS故障等级
    uint8_t error_motor_level; // 电机故障代码
    uint32_t ode_info; // 里程计信息,单位0.1km
    uint8_t cur_brk_press; // 当前刹车压力, 0~8, 单位0.05MPa
    uint16_t cur_steer_angle; // 当前转向角度, -700~700, 单位0.1度, 偏移量为-3000度
    uint8_t current_speed; // 当前速度, 0~127, 单位0.1m/s， 注意读取时取后7位
    uint8_t current_direction; // 方向, 0-静止，1-前进，2-后退,3-预留
    uint8_t current_gear; // 当前档位, 0-N挡，1-D档，2-R档
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
    // 发布者
    rclcpp::Publisher<bot_msg::msg::ChassisInfo>::SharedPtr pub_chassis_info_;
    // 变量
    std::string can_device_name_;    // can设备名称
    int can_baudrate_;               // can波特率
    std::string control_cmd_topic_;  // 控制指令主题名称
    std::string chassis_info_topic_; // 底盘信息主题名称
    int can_fd_ = 0;
    std::atomic<bool> running_{true};  // 控制线程运行的标志位
    std::thread can_thread_;
    ChassisInfoLocal chassis_info_local_;
    // 回调函数
    void TimerCallback();
    void ControlCmdCallback(const bot_msg::msg::ControlCmd::SharedPtr msg);


    // 功能函数
    void InitParams();
    bool InitCanSocket(std::string can_device_name, int can_baudrate);
    void CanThreadFunc();
    void FillCanCtrlCmd(CanCtrlCmd &ctrl_cmd, double steer_angle, double brk, uint8_t gear, double spd);
    void FillChassisInfo(bot_msg::msg::ChassisInfo::SharedPtr msg);
};
} // namespace canbus