#ifndef _RUN_UI_H_
#define _RUN_UI_H_

#include "run.h"

// ------------------ 本地 UI 接口 ------------------
// CPU0 管理逐飞按键事件、菜单和 IPS200；与 run_debug 互不调用。
void run_ui_init(void);       // 初始化 IPS 屏幕和逐飞按键，由 run_init 调用
void run_ui_tick_1ms(void);   // 中断内只扫描库按键、捕获事件，不绘屏
void run_ui_while(void);      // 主循环处理菜单、参数草稿和限频绘屏

#endif
