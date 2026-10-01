#ifndef _RUN_H_
#define _RUN_H_

#include "zf_common_headfile.h"

// ------------------ 调试功能开关：1=开启，0=关闭 ------------------
#define RUN_WIFI_IMAGE_ENABLE     (1) // WiFi 图像、边线和控制中线传输
#define RUN_WIFI_PID_DEBUG_ENABLE (1) // WiFi PID 参数调节与示波器传输
#define RUN_UART_PID_DEBUG_ENABLE (0) // 串口 PID 文本调参与数据输出

void run_init(void);
void run_while(void);
void run_motor_left_speed(int16 speed);
void run_motor_right_speed(int16 speed);
void run_servo_motor_angle(int16 angle);
void run_direct(int16 speed, int16 gap);
double run_get_gap(void);
void run_speed_decision(void);

#endif
