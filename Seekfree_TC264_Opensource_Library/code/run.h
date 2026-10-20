#ifndef _RUN_H_
#define _RUN_H_

#include "zf_common_headfile.h"

// ------------------ 运行层接口 ------------------
// 1ms 基础调度、2ms 双轮速度 PID、10ms 方向 PID、堵转和视觉超时保护。
// 状态只允许在控制中断里改写，主循环通过 run_get_status() 只读。

// 1: 模糊 PD；0: 原位置式 PD (1.4 / 0 / 0.5)，重编译后切换。
#ifndef RUN_STEER_FUZZY_ENABLE
#define RUN_STEER_FUZZY_ENABLE (1)
#endif

// 0: 原 16 通道；1: 将方向组及部分规划通道切换为模糊 PD 观测量。
#ifndef RUN_STEER_FUZZY_DEBUG_ENABLE
#define RUN_STEER_FUZZY_DEBUG_ENABLE (1)
#endif

// 1: 新速度规划；0: 原版速度映射/直道许可。安全修复始终保留。
#ifndef RUN_SPEED_PLANNER_ENABLE
#define RUN_SPEED_PLANNER_ENABLE (1)
#endif

// 0: 保留原示波器通道；1: 速度规划观测，优先于模糊 PD 调试模式。
// 模式 1: 6=risk, 7=curve limit, 13=straight confirmed，其余通道不变。
#ifndef RUN_SPEED_PLANNER_DEBUG_ENABLE
#define RUN_SPEED_PLANNER_DEBUG_ENABLE (0)
#endif

// ------------------ 调试功能开关：1=开启，0=关闭 ------------------
#define RUN_WIFI_IMAGE_ENABLE     (1)   // WiFi 图像、边线、控制中线和前瞻标记传输
#define RUN_WIFI_PID_DEBUG_ENABLE (1)   // WiFi PID 参数调节与示波器传输
#define RUN_UART_PID_DEBUG_ENABLE (0)   // 串口 PID 文本调参与数据输出

// 运行状态，只由控制中断修改，主循环只读取。
typedef enum
{
    RUN_STATE_IDLE = 0,       // 空闲：等待 KEY1
    RUN_STATE_RUNNING,        // 运动：执行闭环和保护
    RUN_STATE_FAULT           // 故障：锁定停车，重新上电后清除
} run_state_enum;

// 故障位标志：按位或，左右轮堵转与视觉超时可同时记录
typedef enum
{
    RUN_FAULT_NONE = 0,           // 无故障
    RUN_FAULT_STALL_LEFT = 1,     // 左轮堵转（位 0）
    RUN_FAULT_STALL_RIGHT = 2,    // 右轮堵转（位 1）
    RUN_FAULT_VISION_TIMEOUT = 4, // 视觉数据超时（位 2）
    RUN_FAULT_VISION_STOP = 8     // 视觉持续丢线停车（位 3），重新上电清除
} run_fault_enum;

// 运行状态快照，故障时保留现场数据供事后分析
typedef struct
{
    run_state_enum state;      // 当前运行状态
    uint8 fault;               // 位标志：左右轮可同时堵转
    int16 left_target;         // 以下六项保留故障瞬间的数据
    int16 right_target;
    int16 left_actual;
    int16 right_actual;
    int16 left_pwm;
    int16 right_pwm;
} run_status_struct;

void run_get_status(run_status_struct *status); // 关中断拷贝状态，供主循环和调试层只读
void run_init(void);                            // 初始化硬件、PID、滤波并启动 1ms/10ms 中断
void run_while(void);                           // 主循环入口，只转发给调试层轮询
double run_get_gap(void);                       // 取最近一帧的转向 PID 输出（左右轮差速），非运行状态返回 0

#endif
