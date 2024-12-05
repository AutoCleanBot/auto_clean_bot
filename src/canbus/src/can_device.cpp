#include "can_device.hpp"
#include <sys/ioctl.h>
#include <net/if.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <iomanip>

CanDevice::CanDevice(const std::string& interface)
    : interface_name(interface),
      socket_id(-1),
      brake_count(0)
{
    memset(&vehicleDriveControl, 0, sizeof(VehicleDriveControl));
    memset(&vehicleSignalControl, 0, sizeof(VehicleSignalControl));
    memset(&vehicleDriveStatus, 0, sizeof(VehicleDriveStatus));
    memset(&vehicleErrorStatus, 0, sizeof(VehicleErrorStatus));
    memset(&vehicleStatus, 0, sizeof(VehicleStatus));

    init();
    start();
}

CanDevice::~CanDevice()
{
    //退出智驾
    can_frame frame_send;
    memset(&frame_send, 0, sizeof(can_frame));
    frame_send.can_id = ENUM_CONTROL_BRAKE_DRIVE_STEERING | CAN_EFF_FLAG; // 制动、行驶控制、转向控制
    frame_send.can_dlc = 8;
    vehicleDriveControl.controlMode = 0x00;    
    memcpy(&frame_send.data, &vehicleDriveControl, frame_send.can_dlc);
    write(socket_id, &frame_send, sizeof(frame_send));
    
    close(socket_id);

    stop();

    exit(EXIT_SUCCESS);
}

void CanDevice::canReceive()
{
    struct can_frame frame_recv;
    while(true){
        /* 接收数据 */
        if (0 > read(socket_id, &frame_recv, sizeof(can_frame)))
        {
            std::cout << "read error" << std::endl;
            return;
        }
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
        canFrame2ChassisInfo(frame_recv);
    }
}

void CanDevice::canSend_10ms()
{
    /* 发送数据 */ 
    while(true){    
        can_frame frame_send;
        memset(&frame_send, 0, sizeof(can_frame));
        frame_send.can_id = ENUM_CONTROL_BRAKE_DRIVE_STEERING | CAN_EFF_FLAG; // 制动、行驶控制、转向控制
        frame_send.can_dlc = 8;
        vehicleDriveControl.controlMode = 0x02;
        vehicleDriveControl.setAngle(latest_control_cmd.steer_angle);
        vehicleDriveControl.setSpeed(latest_control_cmd.speed);
        vehicleDriveControl.brake = latest_control_cmd.brk;
        vehicleDriveControl.gear = latest_control_cmd.gear;
        //避免长时间踩刹车电机过热
        if(brake_count >= 2000)
        {
            vehicleDriveControl.controlMode = 0x02;
            vehicleDriveControl.setAngle(latest_control_cmd.steer_angle);
            vehicleDriveControl.setSpeed(0);
            vehicleDriveControl.brake = 0;
            vehicleDriveControl.gear = 0x00;
            vehicleDriveControl.handBrake = 0x01;
        }      
        memcpy(&frame_send.data, &vehicleDriveControl, frame_send.can_dlc);
        int ret = write(socket_id, &frame_send, sizeof(frame_send)); // 发送数据
        if (ret == -1) {
            perror("write vehicleDriveControl error");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }   
}

void CanDevice::canSend_100ms()
{
    /* 发送数据 */ 
    while(true){
        can_frame frame_send;
        memset(&frame_send, 0, sizeof(can_frame));
        frame_send.can_id = ENUM_CONTROL_VEHICLE_SIGNAL_UPLOAD | CAN_EFF_FLAG; // 车辆信号控制、上装作业控制
        frame_send.can_dlc = 8;
        memcpy(&frame_send.data, &vehicleSignalControl, frame_send.can_dlc);
        int ret = write(socket_id, &frame_send, sizeof(frame_send)); // 发送数据   
        if (ret == -1) {
            perror("write vehicleSignalControl error");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    
}

void CanDevice::canFrame2ChassisInfo(const can_frame &frame)
{   
    std::lock_guard<std::mutex> lock(chassis_info_mutex);
    switch (frame.can_id)
    {
        case ENUM_FEEDBACK_BRAKE_DRIVE_STEERING:
            printf("recv brake drive steering\n");
            memcpy(&vehicleDriveStatus, frame.data, frame.can_dlc);
            //赋值给ChassisInfo
            chassis_info.steer_angle = vehicleDriveStatus.getAngle();
            chassis_info.speed = vehicleDriveStatus.getSpeed();
            chassis_info.brake = vehicleDriveStatus.brake;
            chassis_info.hand_brake = vehicleDriveStatus.handBrake;
            chassis_info.gear = vehicleDriveStatus.gear;

            if(vehicleDriveStatus.brake)
            {
                brake_count++;
            }
            else
            {
                brake_count = 0;
            }

            break;
        case ENUM_FEEDBACK_FAULT:
            printf("recv fault\n");
            memcpy(&vehicleErrorStatus, frame.data, frame.can_dlc);
            //赋值给ChassisInfo
            chassis_info.error_code = vehicleErrorStatus.errorCode;
            chassis_info.error_num = vehicleErrorStatus.errorNum;
            chassis_info.vehicle_fault = vehicleErrorStatus.vehicleFault;
            break;
        case ENUM_FEEDBACK_STATUS:
            printf("recv status\n");
            memcpy(&vehicleStatus, frame.data, frame.can_dlc);
            //赋值给ChassisInfo
            chassis_info.unmanned_button = vehicleStatus.unmannedButton;
            chassis_info.horn = vehicleStatus.horn;
            chassis_info.high_beam_light = vehicleStatus.highBeamLight;
            chassis_info.low_beam_light = vehicleStatus.lowBeamLight;
            chassis_info.position_light = vehicleStatus.positionLight;
            chassis_info.brake_light = vehicleStatus.brakeLight;
            chassis_info.right_turn_light = vehicleStatus.rightTurnLight;
            chassis_info.left_turn_light = vehicleStatus.leftTurnLight;
            chassis_info.ready_signal = vehicleStatus.readySignal;
            chassis_info.emergency_stop_signal = vehicleStatus.emergencyStopSignal;
            chassis_info.low_water_level = vehicleStatus.lowWaterLevel;
            chassis_info.battery_level = vehicleStatus.batteryLevel;
            chassis_info.driving_mode = vehicleStatus.drivingMode;
            break;  
        default:
            printf("recv error\n");
            break;
    }
}

void CanDevice::init()
{
    socket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (0 > socket_id)
    {
        perror("create CAN socket error"); 
        return;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_name.c_str());
    if (ioctl(socket_id, SIOCGIFINDEX, &ifr) < 0) 
    {
        perror("get interface index error"); 
        return;
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN; 
    addr.can_ifindex = ifr.ifr_ifindex;
    if(0 > bind(socket_id, (struct sockaddr *)&addr, sizeof(addr)))
    {
        perror("bind CAN socket error");
    }
}

void CanDevice::start()
{
    thread_recv = std::thread(&CanDevice::canReceive,this);
    thread_recv.detach(); 

    thread_send_10ms = std::thread(&CanDevice::canSend_10ms,this);
    thread_send_10ms.detach();

    thread_send_100ms = std::thread(&CanDevice::canSend_100ms,this);
    thread_send_100ms.detach();
}

void CanDevice::stop()
{   
    if(thread_recv.joinable()){
        thread_recv.join();
    }
    if(thread_send_10ms.joinable()){
        thread_send_10ms.join();
    }
    if(thread_send_100ms.joinable()){
        thread_send_100ms.join();
    }
}

void CanDevice::sendControlCmd(const bot_msg::msg::ControlCmd &cmd)
{
    std::lock_guard<std::mutex> lock(control_cmd_mutex);
    latest_control_cmd = cmd;
}

const bot_msg::msg::ChassisInfo CanDevice::getChassisInfo()
{
    std::lock_guard<std::mutex> lock(chassis_info_mutex);
    chassis_info.header.stamp = rclcpp::Clock().now();
    chassis_info.header.frame_id = "can_device";
    return chassis_info;
}
