#pragma once
#include <stdint.h>

enum CanProtoId {
    CONTROL_CMD = 0xA1, // 控制器下发的控制命令
    VCU_INFO_1 = 0xB1,
    VCU_INFO_2 = 0xB2,
    VCU_INFO_DIAG = 0xB3,
    VCU_INFO_SPD = 0x250,
    SEB_INFO = 0x721,
    SES_INFO = 0x201
};
#pragma pack(push, 1)
/**
 * 控制器下发的控制命令
 */
struct CanCtrlCmd {
    // byte0
    uint8_t motor_enable : 1; // 驱动电机使能, 0: 禁用, 1: 启用
    uint8_t target_gear : 2;  // 目标档位, 0: N挡, 1: D档, 2: R档
    uint8_t target_mode : 1;  // 目标模式, 0: 人工, 1: 线控模式
    uint8_t life_signal : 4;  // 生命信号, 0~15变化
    // byte1
    uint8_t target_speed : 7;   // 目标速度, 0~127, 单位0.1m/s
    uint8_t target_spd_val : 1; // 目标速度值有效位, 0: 无效, 1: 有效
    // byte2
    uint8_t target_brk_press : 8; // 目标刹车压力, 0~8, 单位0.05MPa
    // byte3~4
    uint16_t target_steer_angle : 15; // 目标转向角度, -700~700, 单位0.1度, 偏移量为-700度
    uint16_t steer_moter_enable : 1;  // z轴转向电机使能, 0: 禁用, 1: 启用
    // byte5
    uint8_t target_torque : 8; // 目标扭矩, 0~255, 单位0.1Nm
    // byte6
    uint8_t torque_rpm_mode : 1;  // 扭矩转速模式, 0: 扭矩模式, 1: 速度模式
    uint8_t target_turnlight : 2; // 目标转向灯, 0: 关闭, 1: 左转, 2: 右转, 3: 双闪
    uint8_t target_spd_mode : 2;  // 目标速度模式, 0:低速模式, 1: 中速模式, 2: 高速模式
    uint8_t oec_clear_flag : 1;   // OEC清除标志, 0: 无效, 1: 清除
};

/**
 * VCU信息1, 包含线控底盘的状态信息,来自线控底盘, 发送周期20ms
 */
struct CanVCUInfo1 {
    // byte0
    uint8_t ses_en_sts; // 转向电机使能状态, 0: 禁用, 1: 启用,当前bit第八位
    // byte1
    uint8_t moter_en_sts; // 驱动电机使能状态, 0: 禁用, 1: 启用, 当前bit第八位
    // byte2
    uint8_t tmp; // 保留
    // byte3
    uint8_t motor_torque; // 驱动电机扭矩, 0~255, 单位0.1Nm
    // byte4
    uint8_t vcu_mode : 4;        // VCU模式, 0-初始化；1-人工遥控模式；2-线控模式；3-紧急制动模式；4-预备切换模式
    uint8_t vcu_life_signal : 4; // VCU生命信号, 0~15变化
    // byte5
    uint8_t tmp_1; // 保留
    // byte6
    uint8_t rseb_mode : 2;    // 线控制动模式：0-预留，1-位置模式，2-压力模式
    uint8_t bat_num : 2;      // 当前使用电池包，1-1号，2-2号
    uint8_t cur_gear : 2;     // 当前档位，0-N挡，1-D档，2-R档
    uint8_t bat_warn_sts : 2; // 0-无报警，1-电量低，2-急需充电,3-预留
    // byte7
    uint8_t cur_spd_mode : 2;          // 当前速度模式，0-低速模式，1-中速模式，2-高速模式
    uint8_t controller_online_sts : 1; // 控制器在线状态，0-离线，1-在线
    uint8_t ipc_online_sts : 1;        // IPC在线状态，0-离线，1-在线
};

/**
 * VCU信息1, 包含线控底盘的状态信息,来自线控底盘, 发送周期100ms
 */
struct CanVCUInfo2 {
    uint8_t soc;              // 电池剩余容量, 0~100, 单位%, 0.4%精度
    uint8_t park_st;          // 停车状态, 0-抱死,1-释放
    uint8_t coll_sts;         // 碰撞状态, 0-无碰撞, 1-碰撞
    uint8_t tmp;              // 保留
    uint8_t bms_sts;          // 充电状态，0-未充电，1-正在充电，2-充电完成，3-无效
    uint8_t tmp1;             // 保留
    uint8_t motor_dir;        // 0-静止，1-前进，2-后退,3-预留
    uint8_t auto_mode : 1;    // 自动驾驶开关，0-关闭，1-开启
    uint8_t auto_mode_en : 1; // 自动驾驶使能，0-禁用，1-启用
};

/**
 * VCU信息诊断信息, 包含线控底盘的诊断信息,来自线控底盘, 发送周期100ms
 */
struct CanVCUInfoDiag {
    uint8_t error_level : 2;     // 底盘故障等级
    uint8_t error_eps_level : 2; // 线控转向故障等级
    uint8_t error_eb_level : 2;  // 线控制动故障等级
    uint8_t error_bms_level : 2; // BMS故障等级
    uint8_t error_motor_level;   // 电机故障代码
    uint32_t ode_info;           // 里程计信息,单位0.1km
};

/**
 *  线控底盘的速度信息, 来自线控底盘, 发送周期20ms
 */
struct CanVCUInfoSpd {
    uint8_t current_speed;         // 当前速度, 0~127, 单位0.1m/s， 注意读取时取后7位
    uint8_t current_direction : 2; // 方向, 0-静止，1-前进，2-后退,3-预留
    uint8_t current_gear : 2;      // 当前档位, 0-N挡，1-D档，2-R档
};

/**
 * 线控底盘的制动信息,来自线控底盘, 发送周期10ms
 */
struct SEBInfo {
    uint8_t tmp;           // 保留
    uint8_t tmp_1;         // 保留
    uint8_t tmp_2;         // 保留
    uint8_t cur_brk_press; // 当前刹车压力, 0~8, 单位0.05MPa
};
/**
 * 线控底盘的转向信息,来自线控底盘, 发送周期10ms
 */
struct SESInfo {
    uint8_t tmp;              // 保留
    uint8_t tmp_1;            // 保留
    uint16_t cur_steer_angle; // 当前转向角度, -700~700, 单位0.1度, 偏移量为-3000度
};

#pragma pack(pop)
