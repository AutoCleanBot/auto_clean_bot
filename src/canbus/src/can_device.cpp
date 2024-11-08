#include "can_device.hpp"
#include <sys/ioctl.h>
#include <net/if.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <iomanip>

CanDevice::CanDevice()
{
    memset(&vehicleDriveControl, 0, sizeof(VehicleDriveControl));
    memset(&vehicleSignalControl, 0, sizeof(VehicleSignalControl));
    memset(&vehicleDriveStatus, 0, sizeof(VehicleDriveStatus));
    memset(&vehicleErrorStatus, 0, sizeof(VehicleErrorStatus));
    memset(&vehicleStatus, 0, sizeof(VehicleStatus));

    memset(&frame_recv, 0, sizeof(can_frame));
    memset(&frame_send, 0, sizeof(can_frame));
    frame_send.can_dlc = 0x08;    // 一次发送8个字节数据

    socket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (0 > socket_id)
    {
        perror("socket error"); 
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(socket_id, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(0 > bind(socket_id, (struct sockaddr *)&addr, sizeof(addr)))
    {
        perror("bind error");
    }

    pThread_recv = new std::thread(&CanDevice::canReceive,this);
    pThread_recv->detach();

    pThread_send_10ms = new std::thread(&CanDevice::canSend_10ms,this);
    pThread_send_10ms->detach();

    canSend_100ms();
}

CanDevice::~CanDevice()
{
    close(socket_id);
    if(pThread_recv->joinable()){
        pThread_recv->join();
    }
    delete pThread_recv;
    pThread_recv = nullptr;

    if(pThread_send_10ms->joinable()){
        pThread_send_10ms->join();
    }
    delete pThread_send_10ms;
    pThread_send_10ms = nullptr;

    exit(EXIT_SUCCESS);
}

void CanDevice::setControlCmdInfo(const ControlCmd::SharedPtr msg)
{
    std::cout<<msg->gear;
}

VehicleDriveStatus CanDevice::getChassisInfo()
{
    return VehicleDriveStatus();
}

void CanDevice::canReceive()
{
    while(true){
        /* 接收数据 */
        if (0 > read(socket_id, &frame_recv, sizeof(can_frame)))
        {
            std::cout << "read error" << std::endl;
            return;
        }
        /* 打印数据 */
        for (int i = 0; i < frame_recv.can_dlc; i++)
            std::cout << std::hex << std::setw(2) << std::setfill('0') << frame_recv.data[i];
        std::cout << std::endl;

        /* 校验是否接收到错误帧 */
        if (frame_recv.can_id & CAN_ERR_FLAG)
        {
            std::cout << "frame error" << std::endl;
            return;
        }

        /* 校验帧类型：数据帧还是远程帧 */
        if (frame_recv.can_id & CAN_RTR_FLAG)
        {
            std::cout << "remote request" << std::endl;
            return;
        }

        /* 校验长度：8 */
        if (frame_recv.can_dlc != 0x08)
        {
            std::cout << "length error" << std::endl;
            return;
        }

        switch (frame_recv.can_id)
        {
            case ENUM_FEEDBACK_BRAKE_DRIVE_STEERING:
                memcpy(&vehicleDriveStatus, frame_recv.data, frame_recv.can_dlc);
                break;
            case ENUM_FEEDBACK_FAULT:
                memcpy(&vehicleErrorStatus, frame_recv.data, frame_recv.can_dlc);
                break;
            case ENUM_FEEDBACK_STATUS:
                memcpy(&vehicleStatus, frame_recv.data, frame_recv.can_dlc);
                break;  
            default:
                break;
        }
    }
}

void CanDevice::canSend_10ms()
{
    /* 发送数据 */ 
    while(true){       
        frame_send.can_id = ENUM_CONTROL_BRAKE_DRIVE_STEERING; // 制动、行驶控制、转向控制
        vehicleDriveControl.gear = 0x01;  
        vehicleDriveControl.rotateSpeed = 0x0203;       
        memcpy(&frame_send.data, &vehicleDriveControl, frame_send.can_dlc);
        int ret = write(socket_id, &frame_send, sizeof(frame_send)); // 发送数据
        if (ret == -1) {
            perror("write vehicleDriveControl error");
        }
        // 延时10毫秒  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
}

void CanDevice::canSend_100ms()
{
    /* 发送数据 */ 
    while(true){
        frame_send.can_id = ENUM_CONTROL_VEHICLE_SIGNAL_UPLOAD; // 车辆信号控制、上装作业控制
        memcpy(&frame_send.data, &vehicleSignalControl, frame_send.can_dlc);
        int ret = write(socket_id, &frame_send, sizeof(frame_send)); // 发送数据   
        if (ret == -1) {
            perror("write vehicleSignalControl error");
        }
        // 延时100毫秒  
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
}