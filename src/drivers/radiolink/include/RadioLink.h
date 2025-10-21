#pragma once

#include <cstdint>   // 标准整型类型定义，例如 uint8_t
#include <cmath>     // fabs
#include <iostream>  // 可选：用于调试打印

// 浮点限幅函数宏
#define FLOAT_LIMIT(x)   ( (std::fabs(x) < 0.02f) ? 0.0f : (x) )

// SBUS 状态标志位结构
union sbus_flags_t
{
    uint8_t R; // 原始8位数据
    struct
    {
        uint8_t reserved   : 4;  // 保留
        uint8_t failsafe   : 1;  // 失效保护标志
        uint8_t frame_lost : 1;  // 丢帧标志
        uint8_t ch18       : 1;  // 通道18
        uint8_t ch17       : 1;  // 通道17
    } B;
};

// SBUS 数据结构
struct SBUS_MSG_t
{
    sbus_flags_t flags;
    uint8_t data[25];
    float CH[16];

    uint8_t flag = 0;  // 初始化
    bool ready = false;
};

// 全局变量声明
extern SBUS_MSG_t sbus_msg;

// 函数声明
void Radio_enqueue(uint8_t data);
void Radio_CtrlRun(void);
void Radio_AnalyzeLoop(void);
void Radio_printf(void);
