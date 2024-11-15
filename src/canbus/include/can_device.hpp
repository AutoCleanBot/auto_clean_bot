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
#include "canprotocol.hpp"       // for canProtocol

#include "bot_msg/msg/chassis_info.hpp"
#include "bot_msg/msg/control_cmd.hpp"

using bot_msg::msg::ChassisInfo;
using bot_msg::msg::ControlCmd;



class CanDevice {
public:
    CanDevice();
    ~CanDevice();                    // destructor

    // 控制指令
    void setControlCmdInfo(const ControlCmd::SharedPtr msg);

    // 
    VehicleDriveStatus getChassisInfo();

public:
    void canReceive();   // can接收
    void canSend_10ms();   // can发送 周期为10ms
    void canSend_100ms();   // can发送 周期为100ms

private:
    //send
    VehicleDriveControl vehicleDriveControl;
    VehicleSignalControl vehicleSignalControl; 
    //recv 
    VehicleDriveStatus vehicleDriveStatus;   
    VehicleErrorStatus vehicleErrorStatus;  
    VehicleStatus vehicleStatus; 

    int socket_id;   
    struct can_frame frame_recv;
    struct can_frame frame_send;   
    std::thread *pThread_recv;
    std::thread *pThread_send_10ms;   
    std::thread *pThread_send_100ms;    

    int count; 

    std::mutex sendMutex; // 添加互斥锁        
};                                        



#endif // CAN_DEVICE_H