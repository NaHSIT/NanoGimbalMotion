// 防止该头文件在一次编译中被重复包含。
#pragma once

// 引入 Arduino 的引脚、串口、数据类型及基础函数定义。
#include <Arduino.h>

// ==================== 串口配置 ====================

// Nano 与 ESP8266 通信使用的串口波特率；两端必须保持一致。
#define BAUD_RATE 57600

// ==================== TMC2208 公共控制引脚 ====================

// 三个 TMC2208 共用的使能引脚；低电平使能，高电平关闭。
#define PIN_ENABLE 12

// TMC2208 独立模式下的细分选择引脚 MS1。
#define PIN_MS1 11

// TMC2208 独立模式下的细分选择引脚 MS2。
#define PIN_MS2 10

// ==================== 滑轨轴引脚 ====================

// 滑轨轴的 STEP 脉冲输出引脚。
#define PIN_STEP_SLIDER 4

// 滑轨轴的 DIR 方向输出引脚。
#define PIN_DIRECTION_SLIDER 3

// ==================== 俯仰轴引脚 ====================

// 俯仰轴的 STEP 脉冲输出引脚。
#define PIN_STEP_TILT 6

// 俯仰轴的 DIR 方向输出引脚。
#define PIN_DIRECTION_TILT 5

// ==================== 水平轴引脚 ====================

// 水平轴的 STEP 脉冲输出引脚。
#define PIN_STEP_PAN 8

// 水平轴的 DIR 方向输出引脚。
#define PIN_DIRECTION_PAN 7

// ==================== TMC2208 细分常量 ====================

// 1/2 细分：一个电机整步被拆分为 2 个 STEP 脉冲。
#define HALF_STEP 2

// 1/4 细分：一个电机整步被拆分为 4 个 STEP 脉冲。
#define QUARTER_STEP 4

// 1/8 细分：一个电机整步被拆分为 8 个 STEP 脉冲。
#define EIGHTH_STEP 8

// 1/16 细分：一个电机整步被拆分为 16 个 STEP 脉冲。
#define SIXTEENTH_STEP 16

// ==================== 机械换算参数 ====================

// 滑轨同步轮齿数；配合 2 mm 齿距计算每毫米所需脉冲数。
#define SLIDER_PULLEY_TEETH 36.0f

// 水平轴按当前云台实测为 1:1 传动。
#define PAN_GEAR_RATIO 1.0f

// 俯仰轴校准系数；结合 ESP8266 端 30/48 的预补偿后使用。
#define TILT_GEAR_RATIO 3.2f

// ==================== 模块公开函数 ====================

// 初始化 Nano 云台与滑轨控制模块。
void initPanTilt();

// 执行一次串口处理和电机脉冲更新。
void mainLoop();
