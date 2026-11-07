#ifndef _RUN_H_
#define _RUN_H_

#include "zf_common_headfile.h"

// ------------------ 运行层接口 ------------------
// 1ms 基础调度、2ms 双轮速度 PID、10ms 方向 PID、堵转和视觉超时保护。
// 运行期间的状态只由控制中断改写，UI/通信通过快照和参数接口访问。

// 0: 基础示波器通道；1: 将方向组及部分规划通道切换为模糊 PD 观测量。
#ifndef RUN_STEER_FUZZY_DEBUG_ENABLE
#define RUN_STEER_FUZZY_DEBUG_ENABLE (1)
#endif

// 1: 弯道调试优先；保留双轮和方向 E/EC/u，9=有符号斜率x1000。
// 仅切换遥测通道，不改变速度规划、PID 或车辆控制；0 恢复下方原观测模式。
#ifndef RUN_CURVE_DEBUG_ENABLE
#define RUN_CURVE_DEBUG_ENABLE (1)
#endif

// 0: 保留原示波器通道；1: 速度规划观测，优先于模糊 PD 调试模式。
// 模式 1: 6=risk, 7=curve limit，其余通道沿用基础配置。
#ifndef RUN_SPEED_PLANNER_DEBUG_ENABLE
#define RUN_SPEED_PLANNER_DEBUG_ENABLE (0)
#endif

// ------------------ 调试功能开关：1=开启，0=关闭 ------------------
#ifndef RUN_WIFI_IMAGE_ENABLE
#define RUN_WIFI_IMAGE_ENABLE     (1)   // WiFi 图像、边线、控制中线和前瞻标记传输
#endif
#ifndef RUN_WIFI_PID_DEBUG_ENABLE
#define RUN_WIFI_PID_DEBUG_ENABLE (1)   // WiFi PID 参数调节与示波器传输
#endif
#ifndef RUN_UART_PID_DEBUG_ENABLE
#define RUN_UART_PID_DEBUG_ENABLE (0)   // 串口 PID 文本调参与数据输出
#endif
#ifndef CHEN
#define CHEN (1)
#endif

// IMU 只作原始数据观测；采样在主循环完成，不进入当前方向/速度闭环。
#ifndef RUN_IMU_ENABLE
#define RUN_IMU_ENABLE (0) // 0: 关闭（默认）；1: 初始化并在主循环读取陀螺仪
#endif
#ifndef RUN_IMU_UART_DEBUG_ENABLE
#define RUN_IMU_UART_DEBUG_ENABLE (1) // 200ms 输出一次；启用串口 PID 时自动停用，避免混流
#endif

typedef struct
{
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
    uint8 available;
    uint8 device_id;
    uint8 identity_verified;
    uint8 power_management;
    uint32 sample_count;
    uint16 sample_age_ms;       // 主循环受图传/联网延迟时可观测，饱和到 65535ms
} run_imu_monitor_t;

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

// 参数业务边界统一由 run 定义，UI 和通信不再各自维护范围和组号。
#define RUN_PARAM_SPEED_MAX (1000)
#define RUN_PARAM_GAIN_MAX (100.0)

typedef enum
{
    RUN_PID_LEFT = 0,
    RUN_PID_RIGHT,
    RUN_PID_STEER,
    RUN_PID_GROUP_COUNT
} run_pid_group_enum;

// 控制层拥有参数；UI/通信只能取副本和提交整组更新，不能取得 PID 写指针。
typedef struct
{
    double kp;
    double ki;
    double kd;
} run_gains_t;

typedef struct
{
    int16 speed;             // 目标速度设定值，单位 cm/s
    run_gains_t pid[RUN_PID_GROUP_COUNT];       // 0=左轮，1=右轮，2=方向基准参数
} run_params_t;

// 主循环读取的一致副本；所有字段均由 run 层复制，不暴露控制对象指针。
typedef struct
{
    run_status_struct status;
    uint8 ready;             // 摄像头和双核初始化已完成
    uint8 start_pending;     // 正在等待视觉复位和新帧
    uint8 mid_valid;
    uint8 line_lost;
    uint32 frame_sequence;
    uint16 vision_age_ms;     // 距最新一致视觉帧的时间，单位 ms，超出范围时饱和
    int32 mid_error;
    int16 planned_speed;
    int16 ramped_speed;
    int16 track_slope;
    int16 curve_risk;
    int16 curve_speed_limit;
    uint8 track_slope_valid;
    uint8 straight_long;
    uint8 straight_confirmed;
    uint8 cross;
    uint8 ring_active;
    uint8 ring_stage;
    uint8 offline_row;
    int ring_exit_y;
    double pid_values[9];     // 左轮/右轮/方向，每组三项：目标、实际、输出
    double fuzzy_error;
    double fuzzy_delta;
    double fuzzy_kp;
    double fuzzy_kd;
    double fuzzy_output;
    uint8 fuzzy_saturated;
} run_monitor_t;

// ------------------ main 只调用以下两个运行入口 ------------------
void run_init(void);     // 摄像头初始化前调用：执行器、控制对象、按键和 IPS 屏幕
void run_while(void);    // 双核同步后调用；首次启动调度/通信，此后轮询 UI 和 debug

// ------------------ UI/通信使用的控制服务接口 ------------------
void run_request_toggle(void);       // 只投递启停请求，状态由控制中断改写
void run_get_monitor(run_monitor_t *monitor);
void run_get_imu(run_imu_monitor_t *monitor); // 一致副本；不暴露硬件/设备层内部对象
void run_get_params(run_params_t *params);
uint8 run_params_equal(const run_params_t *left, const run_params_t *right);
uint8 run_apply_params(const run_params_t *params); // 仅 IDLE 且无启动请求时接受

#endif
