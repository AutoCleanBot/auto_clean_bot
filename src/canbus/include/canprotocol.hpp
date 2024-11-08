#ifndef CANPROTOCOL_H
#define CANPROTOCOL_H

#include <stdint.h>  

//can接收帧ID定义 can0
enum AutoDriveCanID {
    // 制动、行驶控制、转向控制
    ENUM_CONTROL_BRAKE_DRIVE_STEERING = 0x1801EF01,
    // 车辆信号控制、上装作业控制
    ENUM_CONTROL_VEHICLE_SIGNAL_UPLOAD = 0x1801EF02,
    // 制动、行驶、转向反馈
    ENUM_FEEDBACK_BRAKE_DRIVE_STEERING = 0x1801EE01,  
    // 故障反馈
    ENUM_FEEDBACK_FAULT = 0x1801EE02,
    // 状态反馈
    ENUM_FEEDBACK_STATUS = 0x1801EE03,
} ;

// 制动、行驶控制、转向控制
//ENUM_CONTROL_BRAKE_DRIVE_STEERING = 0x1801EF01
struct VehicleDriveControl{  
    // Byte 0: 控制模式 请求状态  
    uint8_t controlMode; // 0x00: 人工驾驶, 0x01: 遥控驾驶(暂时不用), 0x02: 自动驾驶    
    // Byte 1: 挡位 需求挡位  
    uint8_t gear; // 0x00: P/N挡位, 0x01: D挡位, 0x02: R挡位     
    // Byte 2 - Byte 3: 转速 车速计算：车速 = (转速 * 0.377 * 0.296) / 18.2  
    uint16_t rotateSpeed;    
    // Byte 4 - Byte 5: 需求角度  精度：0.1°/bit，偏移量：-6500
    uint16_t angle;    
    // Byte 6: 手刹控制 需求控制  
    uint8_t handBrake; // 0x00: 不控制, 0x01: 拉紧手刹, 0x02: 松开手刹     
    // Byte 7: 制动控制 需求压力  
    // 范围：0-100%，最大值：8MPa  
    uint8_t brake;  

    // 速度获取 
    float getSpeed(void) {  
        return (rotateSpeed * 0.377 * 0.296) / 18.2 ;  
    }  

    // 转速设置 
    void setRotateSpeed(float speed) { 
        rotateSpeed = 0;   
        rotateSpeed =  speed * 18.2 / (0.377 * 0.296);  
    } 
  
    // 角度获取 
    float getAngle(void) {    
        return (angle - 6500) * 0.1;  
    }   
    
    // 角度设置
    void setAngle(float angle) {    
        angle = 0;
        angle = 10 * angle + 6500;
    }
} ; 

// 车辆信号控制、上装作业控制
// ENUM_CONTROL_VEHICLE_SIGNAL_UPLOAD = 0x1801EF02
struct VehicleSignalControl{
    // Byte0
    uint8_t diaphragmPump : 1;  // 隔膜泵 0：关闭 1：开启
    uint8_t reserved0 : 7;       // 保留
    // Byte1
    uint8_t oneKeyWork : 1;  // 一键作业 0：关闭 1：开启
    uint8_t reserved1: 7;  // 预留
    // Byte2
    uint8_t horn : 1;  // 喇叭 0：关闭 1：开启
    uint8_t highBeamLight : 1;  // 远光灯 0：关闭 1：开启
    uint8_t lowBeamLight : 1;  // 近光灯 0：关闭 1：开启
    uint8_t positionLight : 1;  // 位置灯 0：关闭 1：开启
    uint8_t reverseLight : 1;  // 倒车灯 0：关闭 1：开启
    uint8_t brakeLight : 1;  // 制动灯 0：关闭 1：开启
    uint8_t rightTurnLight : 1;  // 右转灯 0：关闭 1：开启
    uint8_t leftTurnLight : 1;  // 左转灯 0：关闭 1：开启
    // Byte3 - Byte7
    uint8_t reserved2[5];  // 预留
} ;

// 制动、行驶、转向反馈
// ENUM_FEEDBACK_BRAKE_DRIVE_STEERING = 0x1801EE01
struct VehicleDriveStatus{
    // Byte0 - Byte1
    uint16_t steeringAngle;  // 转向反馈角度 精度：0.1°/bit,偏移量：-6500
    // Byte2 - Byte3
    uint16_t rotationSpeed;  // 转速反馈 精度：1 RPM/bit 偏移量：10000 车速=（转速*0.377*0.296）/18.2
    // Byte 4: 挡位  
    uint8_t gear;            // 0x00: P/N挡位, 0x01: D挡位, 0x02: R挡位
    // Byte5
    uint8_t reserved1[2];     // 预留
    // Byte6
    uint8_t handBrake;  // 手刹反馈（0x00松开，0x01拉紧）
    // Byte7
    uint8_t brake;  // 制动反馈当前压力（0 - 100%）

    // 角度获取 
    float getAngle(void) {    
        return (steeringAngle - 6500) * 0.1;  
    }

    // 车速获取 
    float getSpeed(void) {  
        return (rotationSpeed * 0.377 * 0.296) / 18.2 ;  
    }
};

// 故障反馈
// ENUM_FEEDBACK_FAULT = 0x1801EE02,
struct VehicleErrorStatus{
    // Byte0 - Byte3
    uint8_t reserved2[4];       // 预留
    // Byte4 - Byte5
    uint16_t errorCode;          // 故障码表（16进制）故障码格式：0xABCD 模式：A（故障代号）B（故障等级）C（故障码高4位）D（故障码低4位）
    // Byte6
    uint8_t errorNum;           // 故障数
    // Byte7
    uint8_t vehicleFault;      // 整车故障 0:正常，1：故障
};

// 状态反馈
//ENUM_FEEDBACK_STATUS = 0x1801EE03,
struct VehicleStatus{
    // Byte0
    uint8_t unmannedButton : 1;    // 无人驾驶按钮 0x00:无效，0x01:有效
    uint8_t reserved0 : 7;        // 预留
    // Byte1
    uint8_t horn : 1;             // 喇叭 0x00:关闭，0x01:开启
    uint8_t highBeamLight : 1;    // 远光灯 0x00:关闭，0x01:开启   
    uint8_t lowBeamLight : 1;     // 近光灯 0x00:关闭，0x01:开启
    uint8_t positionLight : 1;    // 位置灯 0x00:关闭，0x01:开启
    uint8_t reverseLight : 1;     // 倒车灯 0x00:关闭，0x01:开启
    uint8_t brakeLight : 1;       // 制动灯 0x00:关闭，0x01:开启
    uint8_t rightTurnLight : 1;   // 右转灯 0x00:关闭，0x01:开启
    uint8_t leftTurnLight : 1;    // 左转灯 0x00:关闭，0x01:开启     
    // Byte2
    uint8_t readySignal : 1;      // Ready信号 0x00:无效，0x01:有效
    uint8_t emergencyStopSignal : 1; // 急停信号 0x00:无效，0x01:有效
    uint8_t lowWaterLevel : 1;    // 水位低 0x00:无效，0x01:有效     
    uint8_t reserved2 : 5;        // 预留
    // Byte3 - Byte5
    uint8_t reserved3[3];        // 预留
    // Byte6
    uint8_t batteryLevel;     // 电池电量  0-100%
    // Byte7
    uint8_t drivingMode;      // 驾驶模式（0x00:人工驾驶，0x01:遥控驾驶，0x02:自动驾驶）
} ;


#if 0
enum CanRecvID{
    // can0
    // 制动、行驶、转向反馈
    ENUM_FEEDBACK_BRAKE_DRIVE_STEERING = 0x1801EE01,  
    // 故障反馈
    ENUM_FEEDBACK_FAULT = 0x1801EE02,
    // 状态反馈
    ENUM_FEEDBACK_STATUS = 0x1801EE03,
    // can1
    //BMS数据包
    ENUM_BMS_DATA = 0x18FF0148,
    //BMS单体电压数据包
    Enum_BMS_CELL_VOLTAGE = 0x18FF0248,
    //BMS电池温度
    Enum_BMS_BATTERT_TEMPERATURE = 0x18FF0348,
    //BMS报警一级报警，维护报警
    ENUM_BMS_ALARM_1 = 0x18FF0448,
    //BMS报警二级报警，三级报警
    ENUM_BMS_ALARM_2 = 0x18FF0548,
    //端压,绝缘
    ENUM_VOLTAGE_INSULATION = 0x18FF0648,
    //硬件，软件
    ENUM_HARDWARE_SOFTWARE = 0x18FF0748,
    //行驶电机状态1
    ENUM_DRIVE_MOTOR_STATUS_1 = 0x18FF41F0,
    //行驶电机状态2
    ENUM_DRIVE_MOTOR_STATUS_2 = 0x18FF51F0,
    //制动电机状态1
    ENUM_BRAKE_MOTOR_STATUS_1 = 0x154,
    //制动电机状态2
    ENUM_BRAKE_MOTOR_STATUS_2 = 0x142,
    //制动电机状态3
    ENUM_BRAKE_MOTOR_STATUS_3 = 0x143,
    //转向状态
    ENUM_STEERING_STATUS = 0x18FF0110,
    // can2
    //风机反馈
    ENUM_FAN_STATUS = 0x18FF46F1,
}

//can发送帧ID定义
enum CanSendID {
    //can0
    // 制动、行驶控制、转向控制
    ENUM_CONTROL_BRAKE_DRIVE_STEERING = 0x1801EF01,
    // 车辆信号控制、上装作业控制
    ENUM_CONTROL_VEHICLE_SIGNAL_UPLOAD = 0x1801EF02,
    //can1
    //行驶电机控制
    ENUM_DRIVE_MOTOR_CONTROL = 0x18FF31F0,
    //转向控制
    ENUM_STEERING_CONTROL = 0x18FF0346,
    //仪表模块1
    ENUM_INSTRUMENT_MODULE_1 = 0x18FF0546,
    //仪表模块2
    ENUM_INSTRUMENT_MODULE_2 = 0x18FF0646,
    //仪表模块3
    ENUM_INSTRUMENT_MODULE_3 = 0x18F40117,
    //can2
    //风机控制
    ENUM_FAN_CONTROL = 0x18FFF146,
    //语音模块1
    ENUM_VOICE_MODULE_1 = 0x18FF1701,
} ;



// 制动、行驶控制、转向控制
//ENUM_CONTROL_BRAKE_DRIVE_STEERING = 0x1801EF01
struct VehicleDriveControl{  
    // Byte 0: 控制模式 请求状态  
    uint8_t control_mode; // 0x00: 人工驾驶, 0x01: 遥控驾驶(暂时不用), 0x02: 自动驾驶    
    // Byte 1: 挡位 需求挡位  
    uint8_t gear; // 0x00: P/N挡位, 0x01: D挡位, 0x02: R挡位     
    // Byte 2 - Byte 3: 转速 车速计算：车速 = (转速 * 0.377 * 0.296) / 18.2  
    uint16_t rotate_speed;    
    // Byte 4 - Byte 5: 需求角度  精度：0.1°/bit，偏移量：-6500
    uint16_t angle;    
    // Byte 6: 手刹控制 需求控制  
    uint8_t handbrake; // 0x00: 不控制, 0x01: 拉紧手刹, 0x02: 松开手刹     
    // Byte 7: 制动控制 需求压力  
    // 范围：0-100%，最大值：8MPa  
    uint8_t brake;  

    // 速度获取 
    float get_speed(void) {  
        return (rotate_speed * 0.377 * 0.296) / 18.2 ;  
    }  

    // 转速设置 
    void set_rotate_speed(float speed) { 
        rotate_speed = 0;   
        rotate_speed =  speed * 18.2 / (0.377 * 0.296);  
    } 
  
    // 角度获取 
    float get_angle(void) {    
        return (angle - 6500) * 0.1;  
    }   
    
    // 角度设置
    void set_angle(float angle) {    
        angle = 0;
        angle = 10 *  angle + 6500;
    }
} ; 

// 车辆信号控制、上装作业控制
// ENUM_CONTROL_VEHICLE_SIGNAL_UPLOAD = 0x1801EF02
struct VehicleSignalControl{
    // Byte0
    uint8_t diaphragmPump : 1;  // 隔膜泵 0：关闭 1：开启
    uint8_t sweepDisk : 1;      // 扫盘 0：关闭 1：旋转
    uint8_t trashBin : 2;       // 垃圾桶 0：停止 1：下降 2：上升
    uint8_t suckerControl : 2;  // 吸盘控制 0：停止 1：下降 2：上升
    uint8_t fanSpeed : 2;  // 风机开启状态 0：关闭1：低档2：中档3：高档    
    // Byte1
    uint8_t oneKeyWork : 1;  // 一键作业 0：关闭 1：开启
    uint8_t drySweepWork : 1;  // 干扫作业 0：关闭 1：开启
    uint8_t reserved: 6;  // 预留
    // Byte2
    uint8_t horn : 1;  // 喇叭 0：关闭 1：开启
    uint8_t highBeamLight : 1;  // 远光灯 0：关闭 1：开启
    uint8_t lowBeamLight : 1;  // 近光灯 0：关闭 1：开启
    uint8_t positionLight : 1;  // 位置灯 0：关闭 1：开启
    uint8_t reverseLight : 1;  // 倒车灯 0：关闭 1：开启
    uint8_t brakeLight : 1;  // 制动灯 0：关闭 1：开启
    uint8_t rightTurnLight : 1;  // 右转灯 0：关闭 1：开启
    uint8_t leftTurnLight : 1;  // 左转灯 0：关闭 1：开启
    // Byte3 - Byte6
    uint8_t reserved2[4];  // 预留
    // Byte7
    uint8_t throttleOpening;  // 油门开度（0 - 100%）
} ;

// 制动、行驶、转向反馈
// ENUM_FEEDBACK_BRAKE_DRIVE_STEERING = 0x1801EE01
struct VehicleDriveStatus{
    // Byte0 - Byte1
    uint16_t steeringAngle;  // 转向反馈角度 精度：0.1°/bit,偏移量：-6500
    // Byte2 - Byte3
    uint16_t rotationSpeed;  // 转速反馈 精度：1 RPM/bit 偏移量：10000 车速=（转速*0.377*0.296）/18.2
    // Byte4 - Byte5
    uint8_t reserved1[2];     // 预留
    // Byte6
    uint8_t handbrakeFeedback;  // 手刹反馈（0x00松开，0x01拉紧）
    // Byte7
    uint8_t brakeFeedbackPressure;  // 制动反馈当前压力（0 - 100%）

    // 角度获取 
    float get_angle(void) {    
        return (steeringAngle - 6500) * 0.1;  
    }

    // 车速获取 
    float get_speed(void) {  
        return (rotationSpeed * 0.377 * 0.296) / 18.2 ;  
    }
};

// 故障反馈
// ENUM_FEEDBACK_FAULT = 0x1801EE02,
struct VehicleErrorStatus{
    // Byte0
    uint8_t steeringComFault : 1; // 转向通讯故障 0:正常，1：故障
    uint8_t brakingComFault : 1;  // 制动通讯故障 0:正常，1：故障
    uint8_t drivingComFault : 1;  // 行驶通讯故障 0:正常，1：故障
    uint8_t bmsComFault : 1;      // BMS通讯故障 0:正常，1：故障
    uint8_t fanComFault : 1;      // 风机通讯故障 0:正常，1：故障
    uint8_t voiceComFault : 1;    // 语音通讯故障 0:正常，1：故障  
    uint8_t reserved1 : 2;        // 预留
    // Byte1 - Byte2
    uint8_t reserved2[2];       // 预留
    // Byte3
    uint8_t steeringMotorFault; // 转向电机故障 0:正常，1：故障
    // Byte4
    uint8_t brakingMotorFault; // 制动电机故障 0:正常，1：故障
    // Byte5
    uint8_t drivingMotorFault; // 行驶电机故障 0:正常，1：故障
    // Byte6
    uint8_t bmsFaultIndication; // BMS故障指示 0:正常，1：故障
    // Byte7
    uint8_t vehicleFault;      // 整车故障 0:正常，1：故障
};

// 状态反馈
//ENUM_FEEDBACK_STATUS = 0x1801EE03,
struct VehicleStatus{
    // Byte0
    uint8_t unmannedButton : 1;    // 无人驾驶按钮
    uint8_t reserved0 : 7;        // 预留
    // Byte1
    uint8_t horn : 1;             // 喇叭
    uint8_t highBeamLight : 1;    // 远光灯
    uint8_t lowBeamLight : 1;     // 近光灯
    uint8_t positionLight : 1;    // 位置灯
    uint8_t reverseLight : 1;     // 倒车灯
    uint8_t brakeLight : 1;       // 制动灯
    uint8_t rightTurnLight : 1;   // 右转灯
    uint8_t leftTurnLight : 1;    // 左转灯      
    // Byte2
    uint8_t readySignal : 1;      // Ready信号
    uint8_t emergencyStopSignal : 1; // 急停信号
    uint8_t lowWaterLevel : 1;    // 水位低
    uint8_t handbrakeSignal : 1;  // 手刹信号
    uint8_t suckerInPlace : 1;    // 吸盘到位 
    uint8_t reserved2 : 3;        // 预留
    // Byte3 - Byte5
    uint8_t reserved3[3];        // 预留
    // Byte6
    uint8_t batteryLevel;     // 电池电量
    // Byte7
    uint8_t drivingMode : 2;      // 驾驶模式（0x00:人工驾驶，0x01:遥控驾驶，0x02:自动驾驶）
    uint8_t reserved7 : 6;        // 预留
} ;

//整车系统 can1
// BMS数据包结构体
struct BMSData{
    // Byte 0: Ubus_H (电池系统总电压值高字节, 0.1V/bit，偏移0)
    uint8_t Ubus_H;
    // Byte 1: Ubus_L (电池系统总电压值低字节, 0.1V/bit，偏移0)
    uint8_t Ubus_L;
    // Byte 2: Ibattery_H (电池充放电电流高字节)
    uint8_t Ibattery_H;
    // Byte 3: Ibattery_L (电池充放电电流低字节, 0.1A/bit，偏移-3200A，放电为正，充电为负)
    int8_t  Ibattery_L; 
    // Byte 4: SOC (电池模块的SOC, 0.4%/bit，偏移0)
    uint8_t SOC;
    // Byte 5: SOH (电池模块的SOH, 0.4%/bit，偏移0)
    uint8_t SOH;
    // Byte 6: 继电器状态
    uint8_t DCDC_relay : 1;       // DCDC继电器
    uint8_t heating_relay : 1;    // 加热继电器
    uint8_t slow_charge_relay : 1;// 慢充继电器
    uint8_t fast_charge_relay : 1;// 快充继电器
    uint8_t main_positive_relay : 1;// 主正继电器
    uint8_t main_negative_relay : 1;// 主负继电器
    uint8_t reserved : 2;    // 保留位

    // Byte 7: LIFE 0-255
    uint8_t LIFE;

    // 电池电压值  0.1V/bit，偏移0
    float getUbus() {
        int16_t Ubus = ((uint16_t)Ubus_H << 8) | Ubus_L;
        return (Ubus * 0.1);
    }

    // 电池电流值 0.1A/bit，偏移-3200A，放电为正，充电为负
    float getIbattery() {
        int16_t ibattery = ((int16_t)Ibattery_H << 8) | ((int8_t)Ibattery_L);
        return ((ibattery - 3200) * 0.1);
    }

    // 电池模块的SOC, 0.4%/bit，偏移0
    float getSOC() {
        return (SOC* 0.4);
    }   

    float getSOH() {
        return (SOH* 0.4);
    } 

};
  
// BMS单体电压数据包 
struct BMSCellVoltage{  
    // Byte 0: 最高单体电压从控序号 (1~255)  
    uint8_t max_cell_voltage_controller_index;   
    // Byte 1: 最高单体电压位置序号 (1~255, 每个从控单独计算位置)  
    uint8_t max_cell_voltage_position_index;   
    // Byte 2: 最高单体电压高字节 (1mv/bit，偏移0)  
    uint8_t max_cell_voltage_high;    
    // Byte 3: 最高单体电压低字节 (1mv/bit，偏移0)  
    uint8_t max_cell_voltage_low;   
    // Byte 4: 最低单体电压从控序号 (1~255)  
    uint8_t min_cell_voltage_controller_index;    
    // Byte 5: 最低单体电压位置序号 (1~255, 每个从控单独计算位置)  
    uint8_t min_cell_voltage_position_index;   
    // Byte 6: 最低单体电压高字节 (1mv/bit，偏移0)  
    uint8_t min_cell_voltage_high;    
    // Byte 7: 最低单体电压低字节 (1mv/bit，偏移0)  
    uint8_t min_cell_voltage_low;  
  
    // 最高单体电压值  
    uint16_t getMaxCellVoltage() const {  
        return ((uint16_t)max_cell_voltage_high << 8) | max_cell_voltage_low;
    }  
  
    // 最低单体电压值  
    uint16_t getMinCellVoltage() const {  
        return ((uint16_t)min_cell_voltage_high << 8) | min_cell_voltage_low;
    }  
  
} ; 

// BMS电池温度 
struct BMSBatteryTemperature{  
    // Byte 0: 最高电池温度从控序号 (1~255)  
    uint8_t max_temp_controller_index;   
    // Byte 1: 最高电池温度位置序号 (1~255, 每个从控单独计算位置)  
    uint8_t max_temp_position_index;    
    // Byte 2: 最高电池温度 (1℃/bit，偏移-40℃)  
    uint8_t max_battery_temp;  
    // Byte 3: 最低电池温度从控序号 (1~255)  
    uint8_t min_temp_controller_index;  
    // Byte 4: 最低电池温度位置序号 (1~255, 每个从控单独计算位置)  
    uint8_t min_temp_position_index;  
    // Byte 5: 最低电池温度 (1℃/bit，偏移-40℃)  
    uint8_t min_battery_temp;  
    // Byte 6 - 7: 预留 (0xFF)  
    uint8_t reserved[2];  
  
    // 获取最高电池温度值（摄氏度）  
    int8_t getMaxBatteryTemp() const {  
        return (int8_t)(max_battery_temp - 40);  
    }  
  
    // 获取最低电池温度值（摄氏度）  
    int8_t getMinBatteryTemp() const {  
        return (int8_t)(min_battery_temp - 40);  
    }  
  
} ; 
  
// BMS报警1级报警，维护报警
struct BMS_Alarm_1{  
    // Byte0  
    uint8_t VoltageSamplingError : 1;              // 电压采样异常
    uint8_t CurrentLoopDamage : 1;                 // 电流环损坏
    uint8_t TotalVoltageSamplingError : 1;         // 总压采样异常  
    uint8_t CollisionDetection : 1;                // 碰撞  
    uint8_t PrechargeInhibitFlag : 1;              // 预充抑制 
    uint8_t HighVoltageLockAbnormal : 1;           // 高压互锁异常  
    uint8_t ExtremelyLowVoltageChargingInhibit : 1; // 极低电压禁止充电      
    uint8_t reserved : 1;                           // 保留位 
    // Byte1  
    uint8_t SlowChargeRelayOpen : 1;               // 慢充继电器断路  
    uint8_t FastChargeRelayStick : 1;              // 快充继电器粘连  
    uint8_t FastChargeRelayOpen : 1;               // 快充继电器断路
    uint8_t CCSignalAbnormal : 1;                  // CC信号异常  
    uint8_t CC2SignalAbnormal : 1;                 // CC2信号异常  
    uint8_t HeatingMembraneDamageFlag : 1;         // 加热膜损坏  
    uint8_t PrechargeFailure : 1;                  // 预充失败  
    uint8_t HeatingRelayOpen : 1;                  // 加热继电器断路   
    // Byte2  
    uint8_t HighVoltageUnlocked : 1;               // 高压未互锁  
    uint8_t MainPositiveRelayOpen : 1;             // 主正继电器断路  
    uint8_t PowerSupplyVoltageAbnormal : 1;        // 供电电压异常  
    uint8_t HighVoltageAcquisitionFault : 1;       // 高压采集故障
    uint8_t TempSensorWireDamage : 1;              // 温敏线损坏 
    uint8_t MainNegativeRelayStick : 1;            // 主负继电器粘连
    uint8_t MainNegativeRelayOpen : 1;             // 主负继电器断路 
    uint8_t SlowChargeRelayStick : 1;              // 慢充继电器粘连
    // Byte3  
    uint8_t InternalNetworkCommunicationFailure : 1; // 内网通讯失败 
    uint8_t ChargerCommunicationFailure : 1;      // 充电机通讯失败
    uint8_t VCUCommunicationFailure : 1;          // VCU通讯失败（VCU是车辆控制单元）
    uint8_t AFEChipFault : 1;                     // AFE芯片故障（AFE通常指模拟前端）
    uint8_t BatteryFault : 1;                     // 电池故障 
    uint8_t MainPositiveRelayStick : 1;           // 主正继电器粘连 
    uint8_t HeatingRelayStick : 1;                // 加热继电器粘连 
    uint8_t InsulationDetectionFailure : 1;       // 绝缘检测失败  
    // Byte4  
    uint8_t reserved1;  
    // Byte5  
    uint8_t FeedbackCurrentLarge_1 : 1;              // 回馈电流大
    uint8_t ChargeTemperatureLow_1: 1;              // 充电温度低
    uint8_t reserved2 : 6;                         // 保留位 
    // Byte6  
    uint8_t ChargeCurrentLarge_1 : 1;                // 充电电流大
    uint8_t DischargeCurrentLarge_1 : 1;             // 放电电流大
    uint8_t BatteryTemperatureDifferenceLarge_1 : 1; // 电池温差大
    uint8_t InsulationLow_1 : 1;                     // 绝缘低
    uint8_t TerminalVoltageHigh_1 : 1;               // 端接电压高
    uint8_t TerminalVoltageLow_1 : 1;                // 端接电压低
    uint8_t ExternalTemperatureHigh_1 : 1;           // 外接温度高  
    uint8_t ChargeTemperatureHigh_1 : 1;             // 充电温度高 
    // Byte7  
    uint8_t DischargeTemperatureHigh_1 : 1;          // 放电温度高  
    uint8_t DischargeTemperatureLow_1 : 1;           // 放电温度低
    uint8_t CellVoltageHigh_1 : 1;                   // 单体电压高
    uint8_t CellVoltageLow_1 : 1;                    // 单体电压低
    uint8_t CellVoltageDifferenceLarge_1 : 1;        // 单体压差大
    uint8_t SOCLow_1 : 1;                            // SOC低
    uint8_t SOCHigh_1 : 1;                           // SOC高
    uint8_t reserved3 : 1;                         // 保留位 
};   
  
// 二级报警,三级报警
struct BMS_Alarm_2{  
    // Byte 0 备份 
    uint8_t reserved;   
    // Byte 1  
    uint8_t charging_temp_low : 1;     // 充电温度低
    uint8_t feedback_current_high : 1; // 回馈电流大    
    uint8_t reserved1 : 6;             // 保留   
    // Byte 2  
    uint8_t charging_current_high : 1;  // 充电电流大
    uint8_t discharge_current_high : 1; // 放电电流大      
    uint8_t battery_temp_diff_high : 1; // 电池温差大
    uint8_t insulation_low : 1;         // 绝缘低 
    uint8_t terminal_voltage_high : 1;  // 端接电压高  
    uint8_t terminal_voltage_low : 1;   // 端接电压低
    uint8_t external_temp_high : 1;     // 外接温度高
    uint8_t charging_temp_high : 1;     // 充电温度高 
    // Byte 3  
    uint8_t discharge_temp_high : 1;    // 放电温度高
    uint8_t discharge_temp_low : 1;     // 放电温度低
    uint8_t cell_voltage_high : 1;      // 单体电压高
    uint8_t cell_voltage_low : 1;       // 单体电压低
    uint8_t cell_voltage_diff_high : 1; // 单体压差大
    uint8_t soc_low : 1;                // SOC低
    uint8_t soc_high : 1;               // SOC高
    uint8_t reserved2 : 1;              // 备份  
    // Byte 4 备份  
    uint8_t reserved3;   
    // Byte 5  
    uint8_t charging_temp_low_2 : 1;    // 充电温度低
    uint8_t reback_current_low_2 : 1;    // 回馈电流大
    uint8_t reserved4 : 6;              // 备份  
    // Byte 6  
    uint8_t charging_current_high_2 : 1; // 充电电流大
    uint8_t discharge_current_high_2 : 1;// 放电电流大 
    uint8_t battery_temp_diff_high_2 : 1;// 电池温差大  
    uint8_t insulation_low_2 : 1;        // 绝缘低
    uint8_t terminal_voltage_high_2 : 1; // 端接电压高 
    uint8_t terminal_voltage_low_2 : 1;  // 端接电压低
    uint8_t external_temp_high_2 : 1;    // 外接温度高
    uint8_t charging_temp_high_2 : 1;    // 充电温度高
    // Byte 7  
    uint8_t discharge_temp_high_2 : 1;   // 放电温度高
    uint8_t discharge_temp_low_2 : 1;    // 放电温度低
    uint8_t cell_voltage_high_2 : 1;     // 单体电压高
    uint8_t cell_voltage_low_2 : 1;      // 单体电压低
    uint8_t cell_voltage_diff_high_2 : 1;// 单体压差大
    uint8_t soc_low_2 : 1;               // SOC低
    uint8_t soc_high_2 : 1;              // SOC高
    uint8_t reserved5 : 1;               // 备份 (Bit 7)  
} ;  

//
struct VoltageInsulation{  
    uint8_t PositiveInsulationResistance_H; // 正极绝缘高字节
    uint8_t PositiveInsulationResistance_L; // 正极绝缘低字节，1KΩ/bit，偏移0 
    uint8_t NegativeInsulationResistance_H; // 负极绝缘高字节
    uint8_t NegativeInsulationResistance_L; // 负极绝缘低字节，1KΩ/bit，偏移0 
    uint8_t PositiveTerminalVoltage_H;      // 正极端压高字节  
    uint8_t PositiveTerminalVoltage_L;      // 正极端压低字节，0.1V/bit，偏移0  
    uint8_t NegativeTerminalVoltage_H;      // 负极端压高字节
    uint8_t NegativeTerminalVoltage_L;      // 负极端压低字节，0.1V/bit，偏移0  
         
    // 正极绝缘  
    float getPositiveInsulation() {  
        return ((uint16_t)PositiveInsulationResistance_H << 8) | PositiveInsulationResistance_L;  
    }  

    // 负极绝缘
    float getNegativeInsulation() {  
        return ((uint16_t)NegativeInsulationResistance_H << 8) | NegativeInsulationResistance_L;  
    } 
    
    // 正极端压
    float getPositiveTerminalVoltage() {  
        return (((uint16_t)PositiveTerminalVoltage_H << 8) | PositiveTerminalVoltage_L) * 0.1;  
    }

    // 负极端压
    float getNegativeTerminalVoltage() {  
        return (((uint16_t)NegativeTerminalVoltage_H << 8) | NegativeTerminalVoltage_L) * 0.1;  
    }
} ;  

//硬件软件信息
struct HardwareSoftware{  
    uint8_t CellCount;         // 单体电池数量，1~255 
    uint8_t TemperatureSensors; // 温度传感器数量，1~255 
    uint8_t RatedCapacity_H;     // 额定电量高字节，单位为0.01AH
    uint8_t RatedCapacity_L;     // 额定电量低字节，单位为0.01AH  
    uint8_t SoftwareVersion_H;   // 软件版本高字节
    uint8_t SoftwareVersion_L;   // 软件版本低字节
    uint8_t HardwareVersion_H;   // 硬件版本高字节
    uint8_t HardwareVersion_L;   // 硬件版本低字节
     
    // 额定电量 
    float getRatedCapacity() {  
        return (((uint16_t)RatedCapacity_H << 8) | RatedCapacity_L) * 0.01;  
    }
     
    // 软件版本
    uint16_t getSoftwareVersion() {  
        return ((uint16_t)SoftwareVersion_H << 8) | SoftwareVersion_L;  
    } 

    // 硬件版本
    uint16_t getHardwareVersion() {  
        return ((uint16_t)HardwareVersion_H << 8) | HardwareVersion_L;  
    }
     
} ; 

//行驶电机控制
struct DriveMotorControl{  
    uint8_t EnableSignal;            // VCU使能MCU控制命令，0x00:Disable, 0x01:Enable  
    int16_t Torque;            // 电机目标扭矩请求，单位为0.1 Nm，有偏移量-500，低字节在前，高字节在后
    uint16_t RotateSpeed;            // 电机目标转速请求，单位为RPM，低字节在前，高字节在后 
    uint8_t Gear;                    // 当前挡位状态，0x00:N, 0x01:D, 0x02:R  
    uint8_t ControlMode;             // 控制模式，0x00:不控制, 0x01:转矩模式, 0x02:转速模式  
    uint8_t LIFE;                    // 电机LIFE值，0-255  

    void setTorque(float torque){
        Torque = 0;
        Torque = torque * 10 + 500;
    }

    void setRotateSpeed(float rotateSpeed){
        RotateSpeed = 0;
        RotateSpeed = rotateSpeed * 10 + 500;
    }
} ; 

//行驶电机状态
struct DriveMotorStatus_1{     
    uint16_t RotateSpeed;            // 电机当前转速，单位为RPM，-10000偏移量
    uint16_t Torque;                 // 电机当前扭矩，单位为0.1 Nm，有偏移量-500
    uint8_t ControlMode;             // 电机当前模式，0x00:不控制, 0x01:转矩模式, 0x02:转速模式  
    uint8_t BusCurrent;             // 电机母线电流，单位为A，
    uint8_t BusVoltage;             // 电机母线电压，单位为V，
    uint8_t LIFE;                    // 电机LIFE值，0-255  

    int16_t getRotateSpeed(){
        return (RotateSpeed - 10000);
    }

    float getTorque(){
        return (RotateSpeed - 500) * 0.1;
    }
} ;

struct DriveMotorStatus_2{  
    uint8_t MotorControllerTemperature; // 电机控制器温度，单位为℃，有偏移量-40，1℃/bit
    uint8_t MotorTemperature;         // 电机温度，单位为℃，有偏移量-40，1℃/bit 
    uint8_t Reserved;                // 保留字节
    uint8_t FaultIndication;         // 电机故障指示，0x00：正常, 0x01：故障
    uint16_t FaultCode;              // 故障码，低字节在前，高字节在后 
    uint8_t FaultLevel;              // 电机故障等级，0x00:正常, 0x01:Level 1, 0x02:Level 2, 0x03:Level 3  
    uint8_t LIFE;                    // 电机LIFE值，0-255   

    int8_t getMotorControllerTemperature(){
        return (MotorControllerTemperature - 40);
    }  

    int8_t getMotorTemperature(){
        return (MotorTemperature - 40);
    }
};  

struct BrakeMotorStatus_1{  
    uint8_t VCU_DBS_Valid;              // Byte0 请求有效标志位，0：无效；1:有效 
    uint8_t VCU_DBS_Work_Mode;          // Byte1 排气模式，0：线控模式，1：排气模式  
    uint8_t VCU_DBS_HP_Pressure;        // Byte2 主动制动目标压力，单位MPa，精度0.1，范围0-8 
    uint8_t ABS_Active;                 // Byte3 ABS工作标志位，0：不工作 ；1：工作   
    uint8_t Reserved[4];               // Byte4 - Byte7 保留  

    float getVCU_DBS_HP_Pressure(){
        return VCU_DBS_HP_Pressure * 0.1;
    }  
};

struct BrakeMotorStatus_2{  
    //Byte0
    union
    {
        uint8_t SystemFaultStatus:2; //系统故障状态
        uint8_t Reserved0:4;        // 
        uint8_t ParkingWarning:2;   //驻车警告0：正常工作； 1：驻车时间超过1分钟，发出警告，建议EPB驻车请求;
        uint8_t placeholders;
    }Warining_t;
    
    //Byte1
    uint8_t DBSWorkMode;        //工作模式 0：线控液压控制 1：排气模式 2：踏板模式
    //Byte2
    uint8_t DBSAck;             //制动压力请求应答 0x0:Not ACK 0x1: ACK  
    //Byte3
    uint8_t MasterCylinderPressure; // 主缸压力值，精度0.1  
    // Byte4
    int8_t PedalPosition;       // 踏板开度，精度0.5，偏移量-20 
    // Byte5 
    int8_t CurrentValue;        // 电流值
    // Byte6
    union
    {
        uint8_t Counter:6;          // Byte6 Bit0 计数
        uint8_t EmergencyStop:1;    // Byte6 Bit6 急停按钮闭合标志位
        uint8_t PedalInserted:1;    // Byte6 Bit7 表明踏板是否插入
        uint8_t placeholders;
    }ByteWithBitFields_t;
    
    // Byte7
    uint8_t CheckSum;           // Byte7 校验位，前7位异或校验   

    float getMasterCylinderPressure(){
        return MasterCylinderPressure * 0.1;
    }  

    float getPedalPosition(){
        return (PedalPosition - 20) * 0.5;
    }

    

    uint8_t  getCheckSum(){
        uint8_t checksum = 0;  
        checksum ^= Warining_t.placeholders;  
        checksum ^= DBSWorkMode; 
        checksum ^= DBSAck; 
        checksum ^= MasterCylinderPressure;  
        checksum ^= PedalPosition;  
        checksum ^= CurrentValue;
        checksum ^= ByteWithBitFields_t.placeholders; 
        return checksum;   
    }
};  


struct BrakeMotorStatus_3{  
    // Byte 0  
    uint8_t OverloadFault:1;       // 第0bit：过载故障  
    uint8_t OverTempOrOverVoltage:1; // 第1bit：过温或过压故障  
    uint8_t MOSShortCircuit:1;     // 第2bit：MOS短路故障  
    uint8_t UnderVoltageFault:1;   // 第3bit：电源欠压故障  
    uint8_t OverVoltageFault:1;    // 第4bit：电源过压故障  
    uint8_t PressureInsufficient:1; // 第5bit：压力不足  
    uint8_t PhaseLossFault:1;      // 第6bit：缺相故障  
    uint8_t CommunicationFault:1;  // 第7bit：通讯故障   
    // Byte 1  
    uint8_t CurrentSamplingFault:1; // 第8bit：电流采样故障  
    uint8_t DriveFault:1;          // 第9bit：驱动故障  
    uint8_t ResolverFault:1;       // 第10bit：磁编故障  
    uint8_t PressureSensorFault:1; // 第11bit：压力传感器故障  
    uint8_t PedalPositionSensorFault:1; // 第12bit：踏板位置传感器故障  
    uint8_t MechanicalFault:1;     // 第13bit：机械故障  
    uint8_t Reserved1:1;         // 第14bit：保留位   
    // Byte 2 保留
    uint8_t Reserved2;    
    // Byte 3  
    uint8_t EmergencyStopButtonPressed:1; // Bit0：急停按键闭合警告  
    uint8_t ParkingTooLong:1;             // Bit1：驻车时间过长，超1Min  
    uint8_t CAN1Interrupt:1;              // Bit2：CAN1中断警告  
    uint8_t CAN2Interrupt:1;              // Bit3：CAN2中断警告  
    uint8_t IgnitionLineDisconnected:1;    // Bit4：点火线断开警告  
    // Bit5-Bit7 
    uint8_t Reserved:3;    
    // Byte 4  
    uint8_t ActualPress; // 实际压力请求  乘以0.1   
    // Byte 6  
    uint8_t Angle; // 电机角度，精度为0.1   
    // Byte 7  
    uint8_t Checksum; // 校验位，Checksum = byte0 XOR byte1 XOR byte2 XOR ... XOR byte6  

    float geActualPress(){
        return ActualPress * 0.1;
    }  

    float getAngle(){
        return Angle * 0.1;
    }
} ;
  
struct SteeringControl{  
    // Byte 0: 控制方式  
    uint8_t controlMode:1;        // Bit0: 1表示线控模式, 0表示手动模式  
    uint8_t reserved1:3;           // Bit1-Bit3: 保留
    uint8_t setToNeutral:1;       // Bit4: 1表示设置为中位, 0表示无效控制  
    uint8_t reserved2:3;          // Bit5-Bit7: 保留 
    // Byte1 - Byte2: 需求角度，精度为1°/bit，偏移量为-650  
    uint16_t angle; 
    // Byte3 - Byte4: 需求角速度，精度为1°/s，偏移量为0 
    uint16_t angularVelocity;   
    // Byte5 - Byte6: 预留    
    uint8_t reserved3[2];    
    // Byte 7: LIFE，范围0-255  
    uint8_t LIFE;    
};   

struct SterringStatus{  
    // Byte 0: FaultCode  
    uint8_t thirdLevelFault : 1;    // 三级故障：1-故障，0-无故障（对应Bit0）
    uint8_t secondLevelFault : 1;   // 二级故障：1-故障，0-无故障（对应Bit1）
    uint8_t firstLevelFault : 1;    // 一级故障：1-故障，0-无故障（对应Bit2）  
    uint8_t communicationFault : 1; // 通讯故障：1-故障，0-无故障（对应Bit3） 
    /*0x5：扭矩传感器故障  0xA:角度中位未标定
    0x1：输入过压 0x2：输入欠压  0x3：控制器普通过温 0x4：控制器严重过温 
    0x5：扭矩传感器故障  0x6：角度传感器故障 0x7：控制器内部故障 0x8:EEPROM故障
    0x9:通讯故障 0xA:角度中位未标定 0xB:电机堵转 0xC:电机过流
    0xD电机及驱动故障*/     
    uint8_t specificFault : 4;       // 通讯故障：0x0：无故障       
    // Byte 1: Temperature  
    uint8_t temperature; // 温度，单位：℃，偏移量：-40    
    // Byte 2: Current  
    uint8_t current; // 电流，单位：A，偏移量：-125  
    // Byte3 - Byte4: AngularVelocity  
    uint16_t angularVelocity; // 角速度，单位：°/s，偏移量：0  
    // Byte5 - Byte6: Angle  
    uint16_t angle; // 角度值，单位：°/bit，偏移量-650  
    // Byte 7: Mode (包含ECU状态)  
    uint8_t mode : 1; // 模式：000-手动模式，001-线控模式（其他组合保留）  
    uint8_t ecuFaultDetected : 1; // ECU故障检测：0-正常，1-检测到故障  
    uint8_t neutralCalibrated : 1; // 中位标定：0-已标定，1-未标定  
    uint8_t reserved : 5; // 保留位（未使用）  
  
    // 温度  
    int8_t getTemperature() {  
        return temperature - 40;  
    }  

    // 电流 
    int8_t getCurrent() {  
        return current - 125;  
    }

    // 角度 
    int16_t getAngle() {  
        return angle - 650;  
    }
}; 
  
struct InstrumentModule1Control_1{  
    // Byte0     
    uint8_t fan_speed_level; // 风机转速等级，1：低-1档，2：中-2档，3：高-3档  
    // Byte1 
    uint8_t fan_status;  // 风机状态，0：停机，1：启动  
    // Byte2 - Byte3 
    uint16_t fan_speed;  // 风机转速   
    //Byte4 
    uint8_t soc_below_20_stop_operation:1; // SOC低于20%导致的停止作业 
    uint8_t fault_caused_stop_operation:1; // 故障导致的停止作业 
    uint8_t soc_below_15_power_reduction:1; // SOC低于15%导致的降功率  
    uint8_t operation_or_reverse_power_reduction:1; // 作业或倒挡导致的降功率 
    uint8_t fault_caused_power_reduction:1; // 故障导致的降功率 
    uint8_t soc_below_10_stop_driving:1; // SOC低于10%导致的停止行驶 
    uint8_t fault_caused_stop_driving:1; // 故障导致的停止行驶
    uint8_t soc_below_8_hv_cutoff:1;  // SOC低于8%导致的下高压  
    //Byte5
    uint8_t fault_caused_hv_cutoff:1; // 故障导致的下高压，可能需要更详细的划分 
    uint8_t reserved1:7;           // 保留位  
    //Byte6       
    uint8_t vehicle_fault;       // 整车故障，0无故障，1有故障   
    // Byte7
    uint8_t life; // LIFE信号，0～255 循环计数  
};  
  
struct InstrumentModule1Control_2{  
    // Byte0 
    uint8_t onSignal             : 1; // ON信号
    uint8_t startSignal          : 1; // START信号，注意这里可能与下面的ON信号有重叠，需要确认具体用法 
    uint8_t readySignal          : 1; // READY信号  
    uint8_t mainPower            : 1; // 总电源
    uint8_t emergencyStop        : 1; // 急停
    uint8_t remoteControlMode    : 1; // 遥控模式  
    uint8_t DCDCStartStop        : 1; // DCDC启停
    uint8_t drySweepSignal       : 1; // 干扫信号  
    // Byte1  
    uint8_t fanMotor             : 1; // 风机电机
    uint8_t suctionNozzleNotInPlace : 1; // 吸嘴未到位
    uint8_t lowWaterLevel        : 1; // 水位低  
    uint8_t chargingCoverNotClosed : 1; // 充电防护盖未闭合  
    uint8_t highVoltageNotLocked : 1; // 高压未互锁 
    uint8_t bucketNotInPlace     : 1; // 桶未到位  
    uint8_t workStart            : 1; // 作业启动 
    uint8_t leftRightSweeper     : 1; // 左右扫盘  
    // Byte2  
    uint8_t liftBucketSignal     : 1; // 提桶信号
    uint8_t liftDropBucket       : 1; // 提落桶 
    uint8_t bucketDropSignal     : 1; // 落桶信号
    uint8_t suctionNozzleUp      : 1; // 吸嘴升  
    uint8_t suctionNozzleDown    : 1; // 吸嘴降  
    uint8_t diaphragmPump        : 1; // 隔膜泵 
    uint8_t diaphragmPumpSolenoidValve : 1; // 隔膜泵电磁阀   
    uint8_t handBrakeSignal      : 1; // 手刹信号  
    // Byte3  
    uint8_t D_gear               : 1; // D档 
    uint8_t R_gear               : 1; // R档
    uint8_t P_gear               : 1; // P档  
    uint8_t reserved1            : 1; // 备份 
    uint8_t ignitionLock         : 1; // 电门锁  
    uint8_t fanHighSpeed         : 1; // 风机高速  
    uint8_t hazardSignal         : 1; // 双闪信号
    uint8_t brakeExhaust         : 1; // 制动排气     
    // Byte4 
    uint8_t horn                 : 1; // 喇叭  
    uint8_t leftTurnLight        : 1; // 左转灯 
    uint8_t rightTurnLight       : 1; // 右转灯 
    uint8_t brakeLight           : 1; // 制动灯
    uint8_t positionLight        : 1; // 位置灯
    uint8_t reverseLight         : 1; // 倒车灯
    uint8_t autonomousDriving    : 1; // 自动驾驶信号      
    uint8_t reserved2            : 1; // 备份   
    // Byte5 - Byte6  
    uint8_t faultCode;                //故障码  
    // Byte7 
    uint8_t life;                     // LIFE
} ;  

struct InstrumentModule1Control_3{  
    // Byte0 
    uint8_t fanMotorSpeedLevel;    //设定风机电机转速档位（1-6）  
    // Byte1  
    uint8_t driveMotorSpeedLevel;   //设定行驶电机转速档位（1-6）   
    // Byte2  
    uint8_t pumpMotorSpeedLevel;   //设定水泵电机转速档位（1-6）  
    // Byte3 
    uint8_t radarBrakeEnable;      //雷达主动刹车启停 0：开启  1：关闭   
    // Byte4 - Byte6  
    uint8_t reserved[3];   
    // Byte7: EEROM烧写标志  
    uint8_t eeromWrite;            // EEROM烧写 1：风机参数写入 2：行驶参数写入 3：水泵参数写入 4：雷达主动刹车启停 
};  


//CAN2（上装系统）
//语音模块1控制
struct VoiceModule1Control_1{  
    // Byte0  
    uint8_t reverseWarning : 1;         // Bit0: 倒车，请注意！  
    uint8_t safetyWarning : 1;          // Bit1: 车辆作业中，请注意安全！  
    uint8_t boxNotInPlace : 1;          // Bit2: 请注意！箱体未到位！  
    uint8_t rearDoorNotClosed : 1;      // Bit3: 请注意！后门未到位！  
    uint8_t autonomousDriving : 1;      // Bit4: 自动驾驶作业中，注意避让！  
    uint8_t obstacleAhead : 1;          // Bit5: 前方有障碍，注意避让！   
    uint8_t reserved1 : 2;  
    // Byte1  
    uint8_t lowBatteryShutdown : 1;     // Bit0: 电量低于百分之二十，停止作业！  
    uint8_t powerReduce15 : 1;          // Bit1: 电量低于百分之十五，开始降功率！  
    uint8_t noDrivingBelow10 : 1;       // Bit2: 电量低于百分之十，禁止行驶！  
    uint8_t lowBattery8ShutdownHV : 1;  // Bit3: 电量低于百分之八，整车下高压  
    uint8_t systemStopped : 1;          // Bit4: 故障，作业系统已停止！  
    uint8_t powerReduce : 1;            // Bit5: 故障，开始降功率！  
    uint8_t drivingSystemStopped : 1;   // Bit6: 故障，行驶系统已停止!  
    uint8_t faultShutdownHV : 1;        // Bit7: 故障，整车已下高压！   
    // Byte2  
    uint8_t driverlessSystemFault : 1;  // Bit0: 无人驾驶系统故障，请停车检查！  
    uint8_t reserved2 : 7;    
    // Byte3  
    uint8_t fanSpeed;                   // 风机转速，0-255范围   
    // Byte4  
    uint8_t reserved3 : 7;
    uint8_t socBelow8Percent : 1;       // Bit7: SOC低于8%导致的下高压  
    // Byte5  
    uint8_t digitalVoice;               // 数字语音，0到255数字（控制器减1处理）   
    // Byte6  
    uint8_t volumeLevel;                // 音量，0-16级   
    // Byte7  
    uint8_t life;                       // LIFE信号，0～255 循环计数
}; 

// 风机控制  
struct FanControl{  
    uint8_t operation_command; // 运行指令，0：停止，1：运行  
    uint16_t rpm;  // 转速
    uint8_t reserved[4]; // 保留
    uint8_t life;             //     
}; 

// 风机反馈  
struct FanStatus{  
    uint8_t motor_enable; // 电机使能 0x00:停止     0x01:运行  
    uint16_t motor_rpm;  // 电机转速
    uint16_t speed;  // 转速反馈
    uint8_t reserved[3]; // 保留  
};
#endif   

#endif// CANPROTOCOL_HPP