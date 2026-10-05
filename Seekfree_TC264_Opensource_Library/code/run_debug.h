#ifndef _RUN_DEBUG_H_
#define _RUN_DEBUG_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "run.h"

// ------------------ 调试层接口 ------------------
// 按键、IPS200 显示、WiFi 图像与示波器、串口 PID 调参。
// 通过 control_ref 指针读写 run 层参数，不参与滤波和控制运算。

#define RUN_KEY_START_STOP (1U)  // KEY1：启动/停止，位标志，可与其他按键按位或

// ------------------ 控制层引用：init 时保存指针，调参直接生效 ------------------
typedef struct
{
    volatile int16 *speed_setting;  // 目标速度，改这里即可改变设定值
    PID_Controller_t *left_pid;     // 左轮速度环
    PID_Controller_t *right_pid;    // 右轮速度环
    PID_Controller_t *gap_pid;      // 转向（中线偏差）环
} run_debug_control_ref_t;

void run_debug_init(const run_debug_control_ref_t *control); // 初始化按键、屏幕与 WiFi/串口，须在控制中断启动前调用
void run_debug_while(void);                                  // 主循环轮询：刷屏、收调参命令、发示波器与图像
void run_debug_tick_10ms(void);                              // 10ms 中断：按键消抖，并置位低频任务标志
void run_debug_discard_keys(void);                           // 丢弃未处理的按键事件，状态切换时调用
uint8 run_debug_take_keys(void);                             // 原子读取并清空按键事件，返回 RUN_KEY_* 位标志

#endif
