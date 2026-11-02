#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "zf_common_headfile.h"

// ------------------ 硬件层接口 ------------------
// 只负责外设的初始化、原始数据采样换算与执行器输出；
// 滤波、PID、状态机全部由 run 层完成，本层不参与任何控制运算。

// ------------------ 反馈数据：由 2ms 速度任务写入，控制层只读 ------------------
typedef struct
{
    int16 raw_left_speed;   // 左轮原始线速度，单位 cm/s，正负表示前进/后退
    int16 raw_right_speed;  // 右轮原始线速度，单位 cm/s，正负表示前进/后退
} hardware_feedback_t;

// ------------------ IMU 原始反馈：仅硬件状态与原始采样，不参与控制计算 ------------------
typedef struct
{
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
    uint8 available;            // 原始通信及唤醒状态可用，不等同于标准芯片身份通过
    uint8 device_id;
    uint8 identity_verified;
    uint8 power_management;
} hardware_imu_feedback_t;

void hardware_imu_init(void);                              // 控制定时器启动前调用一次，不因异常 ID 无限重试
void hardware_sample_imu(hardware_imu_feedback_t *feedback); // 主循环采样，不在控制中断中访问 SPI

void hardware_init(void);                                    // 一次性初始化电机、编码器、舵机，须在控制中断启动前调用
void hardware_sample_2ms(hardware_feedback_t *feedback);      // 一次采样并返回左右轮原始速度，固定 2ms 调用
void hardware_reset_feedback(void);                          // 清空编码器脉冲与速度反馈，启动/重跑前调用
void hardware_set_motors(int16 left_duty, int16 right_duty); // 输出左右电机占空比（0~10000），正负号决定转向
void hardware_set_servo(int16 angle);                        // 设置舵机角度（0 为中位），内部限幅后换算成占空比
void hardware_stop(void);                                    // 电机停转、舵机回中，用于停车、故障和状态切换

#endif
