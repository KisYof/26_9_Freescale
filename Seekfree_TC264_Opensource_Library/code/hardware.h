#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "zf_common_headfile.h"

// ------------------ 硬件层接口 ------------------
// 只负责外设的初始化、原始数据采样换算与执行器输出；
// 滤波、PID、状态机全部由 run 层完成，本层不参与任何控制运算。

// ------------------ 反馈数据：由 5ms 中断写入，控制层只读 ------------------
typedef struct
{
    int16 raw_left_speed;   // 左轮原始线速度，单位 cm/s，正负表示前进/后退
    int16 raw_right_speed;  // 右轮原始线速度，单位 cm/s，正负表示前进/后退
} hardware_feedback_t;

void hardware_init(void);                                    // 一次性初始化电机、编码器、舵机，须在控制中断启动前调用
void hardware_update_5ms(void);                              // 由 5ms 控制中断调用：读取脉冲、换算 cm/s、清空计数器
void hardware_get_feedback(hardware_feedback_t *feedback);   // 取出最近一次采样的左右轮原始速度
void hardware_reset_feedback(void);                          // 清空编码器脉冲与速度反馈，启动/重跑前调用
void hardware_set_motors(int16 left_duty, int16 right_duty); // 输出左右电机占空比（0~10000），正负号决定转向
void hardware_set_servo(int16 angle);                        // 设置舵机角度（0 为中位），内部限幅后换算成占空比
void hardware_stop(void);                                    // 电机停转、舵机回中，用于停车、故障和状态切换

#endif
