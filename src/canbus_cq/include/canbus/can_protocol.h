#pragma once
#include <stdint.h>

enum CanProtoId {
    CONTROL_CMD = 0x201,      // 控制器下发的控制命令
    PERIPH_CMD = 0x301,       // 外设控制指令
    CONTROL_INFO = 0x181,     // 整车控制信息反馈
    CONTROL_PHY_INFO = 0x281, // 整车控制物理信息反馈
    VEHICLE_STATUS_FEEDBACK = 0x381,
    BMS_STATUS_FEEDBACK = 0x19E
};

// Enum for 脱挂钩状态反馈 (Unhook/Hook Status Feedback)
enum HookStatusFeedback  {
    INVALID_STATUS = 0, // 无效状态 (故障停止、首次上电未操作等非正常状态)
    HOOKING = 1,        // 挂钩中
    HOOKED = 2,         // 挂钩完成
    UNHOOKING = 4,      // 脱钩中
    UNHOOKED = 8        // 脱钩完成
};


#pragma pack(push, 1)
/**
 * 控制器下发的控制命令,下发
 */
struct CanCtrlCmd{
    // Byte 0 (Enablement Flags)
    uint8_t auto_mode_enable : 1;       // Bit0: 自动模式使能: 1有效, 0无效
    uint8_t travel_enable : 1;          // Bit1: 行走使能: 1有效, 0无效
    uint8_t service_brake_enable : 1;   // Bit2: 行车制动使能: 1有效, 0无效
    uint8_t steering_enable : 1;        // Bit3: 转向使能: 1有效, 0无效
    uint8_t charge_enable : 1;          // Bit4: 充电使能: 1有效, 0无效
    uint8_t forward_enable : 1;         // Bit5: 前进使能: 1有效, 0无效
    uint8_t reverse_enable : 1;         // Bit6: 后退使能: 1有效, 0无效
    uint8_t parking_brake_enable : 1;   // Bit7: 驻车制动使能: 1有效, 0无效

    // Byte 1
    uint8_t service_brake_percentage;   // 行车制动百分比: 0-255对应0-100%

    // Byte 2, 3 (Low byte, High byte)
    // On little-endian systems, Byte2 will be the LSB, Byte3 the MSB.
    int16_t steering_target_angle;      // 转向控制量: 目标角度 -5700~5700对应-57°~57°

    // Byte 4, 5 (Low byte, High byte)
    // On little-endian systems, Byte4 will be the LSB, Byte5 the MSB.
    uint16_t travel_motor_target_speed; // 行走电机目标转速: 0~5000对应0~5000 rpm

    // Byte 6
    uint8_t travel_motor_acceleration_rate; // 行走电机加速率: 1~255对应0.1s~25.5s

    // Byte 7
    uint8_t travel_motor_deceleration_rate; // 行走电机减速率: 1~255对应0.1s~25.5s
};

/**
 * @brief 外设信息控制结构体,下发
 * 
 */
struct PeriphCmd {
    // Byte 0
    uint8_t reserved_byte0; // Byte0: 预留

    // Byte 1 (Light and Horn Status)
    uint8_t left_turn_light_status : 1;  // Bit0: 左向灯: 1亮闪, 0灭 (Left Turn Signal: 1 flashing/lit, 0 off)
    uint8_t right_turn_light_status : 1; // Bit1: 右向灯: 1亮闪, 0灭 (Right Turn Signal: 1 flashing/lit, 0 off)
    uint8_t headlight_status : 1;        // Bit2: 前大灯: 1亮, 0灭 (Headlight: 1 lit, 0 off)
    uint8_t marker_light_status : 1;     // Bit3: 示宽灯: 1亮, 0灭 (Marker Light: 1 lit, 0 off)
    uint8_t warning_light_status : 1;    // Bit4: 警示灯: 1亮, 0灭 (Warning Light: 1 lit, 0 off)
    uint8_t horn_status : 1;             // Bit5: 喇叭: 1鸣笛, 0无效 (Horn: 1 sounding, 0 inactive/off)
    uint8_t rotary_beacon_status : 1;    // Bit6: 旋转蜂鸣: 1有效, 0无效 (Rotary Beacon: 1 active, 0 inactive) - Assuming this means a rotary beacon/light
    uint8_t wiper_status : 1;            // Bit7: 雨刮器: 1有效, 0无效 (Wiper: 1 active, 0 inactive)

    // Byte 2 (Other Component Status/Control)
    uint8_t air_conditioning_status : 1; // Bit0: 空调: 1有效, 0无效 (Air Conditioning: 1 active, 0 inactive)
    uint8_t unhook_control : 1;          // Bit1: 脱钩: 1有效, 0无效 (Unhook Control: 1 active, 0 inactive) - Could be command or status
    uint8_t hook_control : 1;            // Bit2: 挂钩: 1有效, 0无效 (Hook Control: 1 active, 0 inactive) - Could be command or status
    uint8_t roof_light_status : 1;       // Bit3: 顶灯: 1亮, 0灭 (Roof Light: 1 lit, 0 off)
    uint8_t rear_work_light_status : 1;  // Bit4: 后工作灯: 1亮, 0灭 (Rear Work Light: 1 lit, 0 off)
    uint8_t reserved_b2_bit5 : 1;        // Bit5: 预留 (Byte2 Bit5)
    uint8_t reserved_b2_bit6 : 1;        // Bit6: 预留 (Byte2 Bit6)
    uint8_t reserved_b2_bit7 : 1;        // Bit7: 预留 (Byte2 Bit7)

    // Byte 3
    uint8_t reserved_byte3; // Byte3: 预留

    // Byte 4, 5 (Low byte, High byte)
    // On little-endian systems, Byte4 will be the LSB, Byte5 the MSB.
    uint16_t reserved_bytes4_5; // Byte4, 5: 预留

    // Byte 6, 7 (Low byte, High byte)
    // On little-endian systems, Byte6 will be the LSB, Byte7 the MSB.
    uint16_t smart_drive_system_fault_feedback; // 智驾系统故障反馈
};

/**
 * @brief 控制信息反馈, 包含线控底盘的状态信息,来自线控底盘, 发送周期20ms
 *
 */
struct ControlInfo {
    // Byte 0
    uint8_t auto_enable : 1;                // Bit0: 是否允许进入自动模式状态：1允许，0不允许
    uint8_t whole_mode : 1;                 // Bit1: 整车模式状态：1自动，0手动
    uint8_t emer_stop_mode : 1;             // Bit2: 急停开关状态：1可以动作，0不动作
    uint8_t service_brake_status : 1;       // Bit3: 行车制动状态：1制动，0释放
    uint8_t parking_brake_status : 1;       // Bit4: 驻车制动状态：1释放，0制动
    uint8_t forward_gear_feedback : 1;      // Bit5: 前进档位状态反馈：1有效，0无效
    uint8_t reverse_gear_feedback : 1;      // Bit6: 后退档位状态反馈：1有效，0无效
    uint8_t safety_edge_status : 1;         // Bit7: 安全触边状态：1触发，0未触发

    // Byte 1
    uint8_t vcu_smart_drive_can_status : 1; // Bit0: VCU连接智驾CAN连接状态：1连接，0未连接
    uint8_t vcu_travel_ecu_can_status : 1;  // Bit1: VCU连接行走电控CAN连接状态：1连接，0未连接
    uint8_t vcu_steering_ecu_can_status : 1;            // Bit2: VCU连接转向电控CAN连接状态：1连接，0未连接
    uint8_t vcu_steering_abs_encoder_can_status : 1;    // Bit3: VCU连接转向向绝对值编码器CAN连接状态：1连接，0未连接
    uint8_t vcu_bms_can_status : 1;                     // Bit4: VCU连接BMS CAN连接状态：1连接，0未连接
    uint8_t vcu_e_pusher_can_status : 1;                // Bit5: VCU连接电动推杆 CAN连接状态：1连接，0未连接
    uint8_t vcu_oil_pump_ecu_can_status : 1;            // Bit6: VCU连接油泵电控CAN连接状态：1连接，0未连接
    uint8_t reserved_b1_bit7 : 1;                       // Bit7: 预留 (Byte1 Bit7)

    // Byte 2
    uint8_t manual_steer_signal_trigger : 1;    // Bit0: 人工介入方向盘信号触发
    uint8_t manual_brake_signal_trigger : 1;    // Bit1: 人工介入制动信号触发
    uint8_t manual_throttle_signal_trigger : 1; // Bit2: 人工介入油门信号触发
    uint8_t jog_forward_switch_status : 1;      // Bit3: 点动前进开关状态：1有效，0无效
    uint8_t jog_reverse_switch_status : 1;      // Bit4: 点动后退开关状态：1有效，0无效
    uint8_t reserved_b2_bit5 : 1;               // Bit5: 预留 (Byte2 Bit5)
    uint8_t reserved_b2_bit6 : 1;               // Bit6: 预留 (Byte2 Bit6)
    uint8_t reserved_b2_bit7 : 1;               // Bit7: 预留 (Byte2 Bit7)

    // Byte 3
    uint8_t service_brake_percentage_feedback;  // 行车制动率百分比反馈：0-255对于100%

    // Byte 4, 5 (Low byte, High byte)
    // On little-endian systems, this will naturally map Byte4 to LSB and Byte5 to MSB.
    int16_t travel_motor_speed_feedback;        // 行走电机转速反馈 -5000 ~ 5000 对应-5000 ~ 5000 rpm

    // Byte 6, 7 (Low byte, High byte)
    // On little-endian systems, this will naturally map Byte6 to LSB and Byte7 to MSB.
    int16_t steering_wheel_angle;               // 转向轮转向角度 -5700 ~ 5700 对应 -57° ~ 57°
};

struct ControlPhyInfo {
    // Byte 0, 1 (Low byte, High byte)
    // On little-endian systems, Byte0 will be the LSB, Byte1 the MSB.
    int16_t travel_motor_current_feedback;      // 行走电机电流反馈: -10000 ~ 10000 对应 -1000 ~ 1000 A

    // Byte 2, 3 (Low byte, High byte)
    // On little-endian systems, Byte2 will be the LSB, Byte3 the MSB.
    int16_t travel_motor_temperature_feedback;  // 行走电机温度反馈: -1000 ~ 3000 对应 -100 ~ 300°C

    // Byte 4, 5 (Low byte, High byte)
    // On little-endian systems, Byte4 will be the LSB, Byte5 the MSB.
    int16_t travel_controller_temperature_feedback; // 行走控制器温度反馈: -1000 ~ 3000 对应 -100 ~ 300°C

    // Byte 6, 7 (Low byte, High byte)
    // On little-endian systems, Byte6 will be the LSB, Byte7 the MSB.
    uint16_t travel_ecu_fault_code_feedback;    // 行走电控故障代码反馈

    // Total size: 2 + 2 + 2 + 2 = 8 bytes, matching the typical CAN data length.
};

struct VechicleStatusFeedback {
    // Byte 0, 1 (Low byte, High byte)
    // On little-endian systems, Byte0 will be the LSB, Byte1 the MSB.
    uint16_t steering_ecu_fault_code_feedback; // 转向电控故障代码反馈

    // Byte 2
    uint8_t hook_status_feedback; // 脱挂钩状态反馈

    // Byte 3 (Individual fault bits)
    uint8_t steering_param_setting_fault : 1;       // Bit0: 转向参数设置故障
    uint8_t parking_brake_system_fault : 1;         // Bit1: 驻车制动系统故障
    uint8_t service_brake_system_fault : 1;         // Bit2: 行车制动系统故障
    uint8_t smart_drive_cmd_threshold_exceeded : 1; // Bit3: 智驾系统指令阈值超限
    uint8_t direction_sequence_fault : 1;           // Bit4: 方向顺序故障
    uint8_t reserved_b3_bit5 : 1;                   // Bit5: 预留 (Byte3 Bit5)
    uint8_t reserved_b3_bit6 : 1;                   // Bit6: 预留 (Byte3 Bit6)
    uint8_t reserved_b3_bit7 : 1;                   // Bit7: 预留 (Byte3 Bit7)

    // Byte 4, 5 (Low byte, High byte)
    // On little-endian systems, Byte4 will be the LSB, Byte5 the MSB.
    uint16_t vcu_software_version;                  // VCU软件版本号

    // Byte 6 (Individual light/indicator bits)
    uint8_t headlight_status : 1;           // Bit0: 前大灯状态反馈: 1亮, 0灭 (Headlight Status: 1 lit, 0 off)
    uint8_t marker_light_status : 1;        // Bit1: 示宽灯状态反馈: 1亮, 0灭 (Marker Light Status: 1 lit, 0 off)
    uint8_t brake_light_status : 1;         // Bit2: 刹车灯状态反馈: 1亮, 0灭 (Brake Light Status: 1 lit, 0 off)
    uint8_t left_turn_light_status : 1;     // Bit3: 左转向灯状态反馈: 1亮, 0灭 (Left Turn Signal Status: 1 lit, 0 off)
    uint8_t right_turn_light_status : 1;    // Bit4: 右转向灯状态反馈: 1亮, 0灭 (Right Turn Signal Status: 1 lit, 0 off)
    uint8_t horn_status : 1;                // Bit5: 喇叭状态反馈: 1响, 0停 (Horn Status: 1 sounding, 0 stopped)
    uint8_t wiper_status : 1;               // Bit6: 雨刮状态反馈: 1刷, 0停 (Wiper Status: 1 wiping, 0 stopped)
    uint8_t reserved_b6_bit7 : 1;           // Bit7: 预留 (Byte6 Bit7)

    // Byte 7
    uint8_t oil_pump_ecu_fault_code_feedback; // 油泵电控故障代码反馈

    // Total size: 2 (Bytes 0-1) + 1 (Byte 2) + 1 (Byte 3) + 2 (Bytes 4-5) + 1 (Byte 6) + 1 (Byte 7) = 8 bytes.
};

/**
 * @brief 电池状态信息反馈结构体
 * 
 */
struct BmsStatusFeedback {
    // Byte 0, 1 (Low byte, High byte)
    // On little-endian systems, Byte0 will be the LSB, Byte1 the MSB.
    uint16_t total_battery_voltage_feedback; // 电池总电压反馈: 范围 0~10000, 比例因子 0.1V/bit, 实际量程 0~1000 V

    // Byte 2, 3 (Low byte, High byte)
    // On little-endian systems, Byte2 will be the LSB, Byte3 the MSB.
    uint16_t total_battery_current_feedback; // 电池总电流反馈: 范围 0~65535, 偏移量 -32000, 比例因子 0.1A/bit, 实际量程 -3200A~3353.5 A

    // Byte 4
    uint8_t soc; // SOC (State of Charge): 范围 0~250, 比例因子 0.4%/bit, 实际量程 0~100%

    // Byte 5
    uint8_t battery_capacity; // 电池容量: 范围 0~250, 比例因子 5Ah/bit, 实际量程 0~1250Ah

    // Byte 6 (Battery Fault/Protection bits) - "电池故障信息" (Battery Fault Info) is the general category for these bits
    uint8_t total_voltage_over_voltage : 1; // Bit0: 总压过高: 0正常, 1总压过高
    uint8_t single_cell_over_discharge : 1; // Bit1: 单体过放: 0正常, 1单体过放
    uint8_t communication_interrupted : 1;  // Bit2: 通讯中断: 0正常, 1中断
    uint8_t single_cell_under_voltage : 1;  // Bit3: 单体欠压: 0正常, 1单体欠压
    uint8_t over_current : 1;               // Bit4: 过电流: 0正常, 1过电流
    uint8_t over_temperature_protection : 1; // Bit5: 过温保护: 0正常, 1过温保护
    uint8_t under_temperature_protection : 1; // Bit6: 温度保护: 0正常, 1温度保护 (This seems to be under-temperature based on context, but image says 过温保护 again. Assuming a typo in image and it means under-temp protection based on pairing with over-temp). Let's keep the original description in comment and a more specific name.
    uint8_t charge_connection_status : 1;   // Bit7: 充电连接: 0正常, 1充电连接

    // Byte 7 (Control/Status bits)
    uint8_t forced_full_charge : 1;         // Bit0: 强制满充: 0正常, 1车辆需求充电 (Forced Full Charge: 0 Normal, 1 Vehicle requires charging) - *See long description next to this bit in the image*
    uint8_t power_cut_protection : 1;       // Bit1: 断功率保护: 0正常, 1电池停止放电 (Power Cut Protection: 0 Normal, 1 Battery stops discharging)
    uint8_t reserved_b7_bit2 : 1;           // Bit2: 预留 (Byte7 Bit2)
    uint8_t reserved_b7_bit3 : 1;           // Bit3: 预留 (Byte7 Bit3)
    uint8_t reserved_b7_bit4 : 1;           // Bit4: 预留 (Byte7 Bit4)
    uint8_t reserved_b7_bit5 : 1;           // Bit5: 预留 (Byte7 Bit5)
    uint8_t reserved_b7_bit6 : 1;           // Bit6: 预留 (Byte7 Bit6)
    uint8_t reserved_b7_bit7 : 1;           // Bit7: 预留 (Byte7 Bit7)

    // Total size: 2 (Bytes 0-1) + 2 (Bytes 2-3) + 1 (Byte 4) + 1 (Byte 5) + 1 (Byte 6) + 1 (Byte 7) = 8 bytes.
};

#pragma pack(pop)
