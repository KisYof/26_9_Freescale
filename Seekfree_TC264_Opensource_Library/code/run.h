#ifndef _RUN_H_
#define _RUN_H_

#include "zf_common_headfile.h"

void run_init(void);
void run_motor_left_speed(int8 speed);
void run_motor_right_speed(int8 speed);
void run_servo_motor_angle(int8 angle);

#endif
