#ifndef _RUN_H_
#define _RUN_H_

#include "zf_common_headfile.h"

void run_init(void);
void run_while(void);
void run_uart2_rx_isr(void);
void run_motor_left_speed(int16 speed);
void run_motor_right_speed(int16 speed);
void run_servo_motor_angle(int16 angle);
void run_direct(int16 speed, int16 gap);
double run_get_gap(void);
void run_speed_decision(void);

#endif
