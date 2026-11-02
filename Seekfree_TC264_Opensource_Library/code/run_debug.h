#ifndef _RUN_DEBUG_H_
#define _RUN_DEBUG_H_

#include "run.h"

// ------------------ 通信调试接口 ------------------
// 只处理 WiFi 图传/示波器和串口调参，不负责按键、页面或屏幕刷新。
void run_debug_imu_report(void);  // 默认调试串口输出一次 IMU 快照，main 不直接调用
void run_debug_init(void);        // 根据宏初始化通信，基础控制须已就绪
void run_debug_while(void);       // 主循环收发数据，通过 run 的统一接口提交参数
void run_debug_tick_10ms(void);   // 中断内只设置低频发送标志

#endif
