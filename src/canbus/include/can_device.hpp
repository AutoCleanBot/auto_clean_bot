#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#include <linux/can.h>
#include <linux/can/raw.h>        // for struct can_frame
#include <sys/ioctl.h>           // for ioctl
#include <sys/types.h>           // for socket
#include <sys/socket.h>          // for socket
#include <string>                // for std::string
#include <thread>                // for std::thread
#include <mutex>                 // for std::mutex
#include <fcntl.h>               // for fcntl
#include <atomic>                // for std::atomic
#include "canprotocol.hpp"       // for canProtocol

#include "bot_msg/msg/chassis_info.hpp"
#include "bot_msg/msg/control_cmd.hpp"

using bot_msg::msg::ChassisInfo;
using bot_msg::msg::ControlCmd;


class CanDevice {
public:
    CanDevice(const std::string& interface);
    ~CanDevice();                    // destructor

    //init
    void init();
    void start();
    void stop();
    // 控制指令
    void sendControlCmd(const bot_msg::msg::ControlCmd& cmd);
    // 设置底盘信息回调
    const bot_msg::msg::ChassisInfo getChassisInfo();

private:  
    //send struct
    VehicleDriveControl vehicleDriveControl;
    VehicleSignalControl vehicleSignalControl; 
    //recv struct
    VehicleDriveStatus vehicleDriveStatus;   
    VehicleErrorStatus vehicleErrorStatus;  
    VehicleStatus vehicleStatus; 

    std::string interface_name;
    int socket_id;   

    // 线程  
    std::thread thread_recv;
    std::thread thread_send_10ms;   
    std::thread thread_send_100ms;   

    bot_msg::msg::ControlCmd latest_control_cmd;
    bot_msg::msg::ChassisInfo chassis_info;

    // 线程函数
    void canReceive();      // can接收
    void canSend_10ms();    // can发送 周期为10ms
    void canSend_100ms();   // can发送 周期为100ms

    void canFrame2ChassisInfo(const can_frame& frame);
    //刹车踏板计数
    int brake_count; 

    std::mutex control_cmd_mutex;   
    std::mutex chassis_info_mutex; 
};                                        



#endif // CAN_DEVICE_H