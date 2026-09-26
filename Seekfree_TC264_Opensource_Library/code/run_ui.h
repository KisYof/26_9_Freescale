#ifndef _RUN_UI_H_
#define _RUN_UI_H_

#include "zf_common_headfile.h"
#include "pid.h"

void run_ui_init(void);
void run_ui_tick_10ms(void);
uint8 run_ui_while(
    uint8 running,
    const PID_Controller_t *left_pid,
    const PID_Controller_t *right_pid,
    const PID_Controller_t *gap_pid);

#endif
