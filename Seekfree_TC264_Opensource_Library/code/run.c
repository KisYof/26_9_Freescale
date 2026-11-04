#include "run.h"
#include "run_ui.h"
#include "run_debug.h"
#include "pid.h"
#include "filter.h"
#include "camera.h"
#include "hardware.h"
#include <math.h>

#define MOTOR_STALL_PWM_THRESHOLD (1500) // 判定堵转所需的最小 PWM 幅值

// 1ms 基础节拍下分出 2ms 速度任务和 10ms 方向任务；以下参数保持原来的实际时长。
#define RUN_SPEED_PERIOD_TICKS (2U)
#define RUN_DIRECTION_PERIOD_TICKS (10U)
#define RUN_IMU_PERIOD_TICKS (5U)      // 仅置采样请求；实际 SPI 留在主循环，允许合并积压请求
#define RUN_VISION_TIMEOUT_TICKS (20U) // 方向任务计数：视觉 200ms 未更新即判视觉超时
#define RUN_STALL_GRACE_TICKS (150U)   // 速度任务计数：启动后 300ms 内不做堵转判断
#define RUN_STALL_CONFIRM_TICKS (150U) // 速度任务计数：堵转条件连续成立 300ms 才确认
#define RUN_STALL_TARGET_MIN (20)      // 目标速度大于 20cm/s 才谈堵转
#define RUN_STALL_SPEED_MAX (5)        // 实际速度绝对值小于 5cm/s 视为停转
#define RUN_SPEED_ACCEL_STEP (3)       // 10ms 加速步长，保守初值，需实车标定
#define RUN_SPEED_DECEL_STEP (10)       // 10ms 减速步长，必须大于加速步长，需实车标定
#define SPEED_MID (220)                // 调试层默认目标速度
#define STEER_OUTPUT_MAP_SCALE (0.25)  // 转向输出换算成舵机角的缩放系数

// 原版回退参数（新版普通赛道使用下方标定表）：Track_Slope_x1000 是弯道趋势而非严格几何曲率，以下参数均需实车标定。
#define RUN_STRAIGHT_SLOPE_THRESHOLD (120) // 新版 <=120；回退模式 <120 且 straight_long 有效
#define RUN_CURVE_SLOPE_LOW (150)          // 轻弯起点 S1
#define RUN_CURVE_SLOPE_MID (500)          // 中弯分段点 S2
#define RUN_CURVE_SLOPE_HIGH (1000)        // 急弯分段点 S3
#define RUN_SPEED_CURVE_MILD (220)         // 未确认长直道/中等弯道的速度上限
#define RUN_SPEED_CURVE_MIN (170)          // 急弯最低速度上限
#define RUN_SPEED_UNKNOWN_LIMIT (150)      // slope 无效时的保守速度上限
#define RUN_SPEED_CROSS_LIMIT (180)        // 十字速度上限
#define RUN_SPEED_RING_LIMIT (180)         // 环岛速度上限
#define RUN_SPEED_LINE_LOST_LIMIT (120)    // 丢线时的安全速度上限

#if RUN_SPEED_PLANNER_ENABLE
#define RUN_STRAIGHT_CONFIRM_FRAMES (8U)   // 至少八个有效新帧，不按调度次数累计
#define RUN_STRAIGHT_REVOKE_SLOPE (120)
#define RUN_STRAIGHT_MID_CONFIRM (10)
#define RUN_STRAIGHT_MID_REVOKE (10)
#define RUN_SPEED_UNCONFIRMED_LIMIT (185)
#else
#define RUN_STRAIGHT_CONFIRM_FRAMES (4U)   // 原版回退参数
#endif

// 原始速度与滤波后速度：只在 2ms 速度任务里更新
volatile int16 raw_left = 0;
volatile int16 raw_right = 0;
volatile int16 encoder_left = 0;
volatile int16 encoder_right = 0;
int16 speed = 0;                                 // 斜坡爬升后真正执行的目标速度
static volatile int16 speed_setting = SPEED_MID; // 目标速度设定值，UI/WiFi/串口在空闲时通过参数接口修改
static run_status_struct run_status = {RUN_STATE_IDLE, RUN_FAULT_NONE, 0, 0, 0, 0, 0, 0};

// 运行层自己的视觉副本，只读取 camera 原有变量，不改动视觉模块。
typedef struct
{
    uint32 frame_sequence; // 帧序号，用于判断是否收到新帧
    int32 mid_error;       // 控制中线偏差，转向 PID 的输入
    uint8 mid_valid;       // 当前控制中线是否有效
    uint8 line_lost;       // 视觉丢线标志
    uint8 stop;            // 正常斑马线停车，回 IDLE
    uint8 vision_stop;     // 持续丢线保护，锁入 FAULT
    int16 track_slope;     // 远近控制中线得到的赛道倾斜趋势（放大 1000 倍）
    uint8 track_slope_valid;
    uint8 straight_long;   // 原视觉标志，新版保留作辅助观测
    uint8 cross;
    uint8 zebra;           // 斑马线阶段不能取得普通直道高速许可
    uint8 ring_active;     // 0=无环岛，非 0=环岛方向/活动状态
    uint8 ring_stage;
    uint8 ring_size;
} run_vision_struct;

static run_vision_struct run_vision;
static uint16 run_state_ticks = 0;       // 进入当前状态后的 2ms 速度任务计数
static uint16 run_vision_ticks = 0;      // 距上一帧新视觉数据的 10ms 方向任务计数
static uint16 run_left_stall_ticks = 0;  // 左轮堵转条件连续成立的 2ms 计数
static uint16 run_right_stall_ticks = 0; // 右轮堵转条件连续成立的 2ms 计数
static uint8 run_speed_tick_count = 0;   // 1ms 基础节拍到 2ms 速度任务的分频计数
static uint8 run_direction_tick_count = 0; // 1ms 基础节拍到 10ms 方向任务的分频计数
static int32 run_held_mid_error = 0;     // 丢线时保持最后一次有效方向误差
static double run_direction_gap = 0;    // 最近一次新视觉帧计算得到的方向输出
static volatile int16 run_planned_speed = 0;       // 多重限速求最小值后的目标基础速度
static volatile int16 run_curve_speed_limit = 0;   // 弯道趋势/直道许可产生的速度上限
static volatile int16 run_element_speed_limit = 0; // 十字/环岛产生的速度上限
static volatile int16 run_safety_speed_limit = 0;  // 丢线产生的速度上限
static uint8 run_straight_confirm_count = 0;
static volatile uint8 run_straight_confirmed = 0;
static volatile int16 run_curve_risk = 0; // 速度规划专用绝对倾斜风险，不进入方向环
#if RUN_SPEED_PLANNER_ENABLE
static uint32 run_curve_risk_x4 = 0;     // Q2 保留小数，下降权重 3/4
#endif
static uint32 run_speed_planner_frame_sequence = 0;
static run_imu_monitor_t run_imu_monitor;
static volatile uint16 run_imu_age_ms = 65535U;
#if RUN_IMU_ENABLE
static uint8 run_imu_tick_count = 0;
static volatile uint8 run_imu_sample_pending = 0;
#endif
static volatile uint8 run_ready = 0;
static volatile uint8 run_toggle_request = 0;
static uint8 run_start_pending = 0;     // KEY1 请求后等待 CPU1 完成原有复位接口
static uint16 run_start_wait_ticks = 0; // 独立复位等待计时，不被持续来帧清零
static uint32 run_start_frame_sequence = 0;

// 三路 PID：左右轮速度环为增量式，转向环为位置式
static PID_Controller_t pid_left;
static PID_Controller_t pid_right;
static PID_Controller_t pid_gap; // 模糊模式下保留基准参数和原调试通道，不执行普通 PID
#if RUN_STEER_FUZZY_ENABLE
static FuzzyPID_Controller_t fuzzy_gap;
#endif

// 编码器滤波：滑动平均窗口 + 一阶低通；steer_input_filter 用于中线偏差
#define ENCODER_FILTER_WINDOW (20)
static filter_ring_t encoder_left_ring;
static filter_ring_t encoder_right_ring;
static float encoder_left_buffer[ENCODER_FILTER_WINDOW];
static float encoder_right_buffer[ENCODER_FILTER_WINDOW];
static filter_rc_state_t encoder_left_filter;
static filter_rc_state_t encoder_right_filter;
static filter_rc_state_t steer_input_filter;

// ------------------ 内部函数前向声明 ------------------
static void run_request_start(void);               // 启动：清滤波、编码器与视觉计时
static void run_enter_state(run_state_enum state); // 切状态：清计数、停输出、复位 PID 与滤波
static void run_control_tick_1ms(void);            // 1ms 基础调度：方向到期时先执行，再执行速度任务
static void run_direction_tick_10ms(void);         // 10ms 读取最新视觉结果并更新方向 PID
static void run_speed_tick_2ms(void);              // 2ms 采样编码器、速度 PID、堵转保护和电机输出
static void run_update_direction(void);            // 用当前视觉快照更新一次方向 PID
static int16 run_calc_curve_speed_limit(const run_vision_struct *vision, int16 user_limit);
static int16 run_calc_element_speed_limit(const run_vision_struct *vision);
static int16 run_calc_safety_speed_limit(const run_vision_struct *vision);
static int16 run_speed_planner(const run_vision_struct *vision);
static void run_update_speed_ramp(int16 target_speed);

#if RUN_STEER_FUZZY_ENABLE
// 初始启发式标定，E/EC 保持现有误差单位和 10ms 离散差分，不除以秒。
static void run_init_fuzzy_direction(void)
{
    static const double error_points[FUZZY_PID_LEVEL_COUNT] =
        {-60.0, -35.0, -17.0, 0.0, 17.0, 35.0, 60.0};
    static const double delta_points[FUZZY_PID_LEVEL_COUNT] =
        {-16.0, -9.0, -4.0, 0.0, 4.0, 9.0, 16.0};
    static const double kp_offsets[FUZZY_PID_LEVEL_COUNT] =
        {-0.35, -0.25, -0.13, 0.0, 0.13, 0.25, 0.35};
    static const double kd_offsets[FUZZY_PID_LEVEL_COUNT] =
        {-0.15, -0.08, -0.03, 0.0, 0.03, 0.08, 0.15};
    static const unsigned char kp_rules[7][7] = {
        {6, 5, 5, 5, 4, 4, 3},   // NB 行右侧 +1
        {5, 5, 4, 4, 3, 3, 2},
        {4, 4, 3, 2, 2, 2, 1},
        {1, 1, 2, 2, 2, 1, 1},   // ZO 行不动
        {1, 2, 2, 2, 3, 4, 4},
        {2, 3, 3, 4, 4, 5, 5},
        {4, 4, 4, 5, 5, 5, 6}    // PB 行左侧 +1~+2
    };

    static const unsigned char kd_rules[7][7] = {
        {4, 3, 2, 3, 4, 5, 5},   // NB 行左侧 +1~+2
        {2, 2, 3, 4, 5, 5, 6},
        {2, 3, 3, 4, 5, 5, 6},
        {2, 2, 2, 1, 2, 2, 2},
        {6, 5, 5, 4, 3, 3, 2},
        {6, 5, 5, 4, 3, 2, 2},
        {5, 5, 4, 3, 2, 3, 4}    // PB 行右侧 +1~+2
    };
    FuzzyPID_Config_t config;
    unsigned char row;
    unsigned char column;

    FuzzyPID_Config_InitReference(&config);
    for (row = 0; row < FUZZY_PID_LEVEL_COUNT; row++)
    {
        config.error_points[row] = error_points[row];
        config.delta_error_points[row] = delta_points[row];
        config.kp_offsets[row] = kp_offsets[row];
        config.ki_offsets[row] = 0.0;
        config.kd_offsets[row] = kd_offsets[row];
        for (column = 0; column < FUZZY_PID_LEVEL_COUNT; column++)
        {
            config.kp_rules[row][column] = kp_rules[row][column];
            config.ki_rules[row][column] = 3U;
            config.kd_rules[row][column] = kd_rules[row][column];
        }
    }
    config.kp_min = 0.65;
    config.kp_max = 1.65;
    config.ki_min = config.ki_max = 0.0;
    config.kd_min = 0.02;
    config.kd_max = 0.60;
    config.delta_error_limit = 16.0;
    config.integral_min = config.integral_max = 0.0;
    config.output_min = -90.0;
    config.output_max = 90.0;
    FuzzyPID_Init(&fuzzy_gap, &config, 1.25, 0.0, 0.36);
}
#endif

// 启动和上电共用的反馈复位，不参与普通停车或故障现场记录。
static void run_reset_speed_feedback(void)
{
    filter_ring_init(&encoder_left_ring, encoder_left_buffer, ENCODER_FILTER_WINDOW);
    filter_ring_init(&encoder_right_ring, encoder_right_buffer, ENCODER_FILTER_WINDOW);
    filter_rc_state_reset(&encoder_left_filter);
    filter_rc_state_reset(&encoder_right_filter);
    hardware_reset_feedback();
    raw_left = raw_right = 0;
    encoder_left = encoder_right = 0;
}

#if RUN_IMU_ENABLE
// 主循环访问 SPI；采完后只在短临界区发布快照，通信格式化不占用临界区。
static void run_sample_imu(void)
{
    hardware_imu_feedback_t feedback;
    uint32 interrupt_state;

    hardware_sample_imu(&feedback);
    interrupt_state = interrupt_global_disable();
    run_imu_monitor.available = feedback.available;
    run_imu_monitor.device_id = feedback.device_id;
    run_imu_monitor.identity_verified = feedback.identity_verified;
    run_imu_monitor.power_management = feedback.power_management;
    if (feedback.available)
    {
        run_imu_monitor.acc_x = feedback.acc_x;
        run_imu_monitor.acc_y = feedback.acc_y;
        run_imu_monitor.acc_z = feedback.acc_z;
        run_imu_monitor.gyro_x = feedback.gyro_x;
        run_imu_monitor.gyro_y = feedback.gyro_y;
        run_imu_monitor.gyro_z = feedback.gyro_z;
        run_imu_monitor.sample_count++;
        run_imu_age_ms = 0;
    }
    interrupt_global_enable(interrupt_state);
}

static void run_imu_while(void)
{
    uint8 sample_pending;
    uint32 interrupt_state = interrupt_global_disable();

    sample_pending = run_imu_sample_pending;
    run_imu_sample_pending = 0;
    interrupt_global_enable(interrupt_state);
    if (sample_pending)
    {
        run_sample_imu();
    }
}
#endif

// 与现有 monitor 接口一致：先复制副本，再在临界区外使用。
void run_get_imu(run_imu_monitor_t *monitor)
{
    uint32 interrupt_state = interrupt_global_disable();

    *monitor = run_imu_monitor;
    monitor->sample_age_ms = run_imu_age_ms;
    interrupt_global_enable(interrupt_state);
}

// 第一阶段：main 在摄像头初始化前调用，先保证执行器安全并准备屏幕。
// 此处不启动定时器、不联网，摄像头重试期间不会响应启动请求。
void run_init(void)
{
    hardware_init();
    memset(&run_imu_monitor, 0, sizeof(run_imu_monitor));
    run_imu_age_ms = 65535U;
    PID_Controller_Init(&pid_left, PID_ALGORITHM_INC, 13.3, 0.05, 0.0, 4000, -4000, 0, 0);
    PID_Controller_Init(&pid_right, PID_ALGORITHM_INC, 13.3, 0.035, 0.0, 4000, -4000, 0, 0);
#if RUN_STEER_FUZZY_ENABLE
    PID_Controller_Init(&pid_gap, PID_ALGORITHM_POS, 0.80, 0.0, 0.22, 100, -100, 0, 0);
    run_init_fuzzy_direction();
#else
    PID_Controller_Init(&pid_gap, PID_ALGORITHM_POS, 1.4, 0.0, 0.5, 90, -90, 0, 0);
#endif
    run_reset_speed_feedback();
    run_ready = 0;
    run_toggle_request = 0;
    run_enter_state(RUN_STATE_IDLE);
    run_ui_init();
}

// 第二阶段：main 必须在摄像头初始化和双核同步后才进入 run_while。
// 先开放控制调度并绘制首屏，再初始化通信，网络等待不阻塞控制中断。
static void run_start_runtime(void)
{
    run_toggle_request = 0;
    run_ready = 1;
    // 屏幕/摄像头和双核同步完成后再建立 SPI0，避免后续启动阶段覆盖采样状态。
    memset(&run_imu_monitor, 0, sizeof(run_imu_monitor));
    run_imu_age_ms = 65535U;
#if RUN_IMU_ENABLE
    run_imu_tick_count = 0;
    run_imu_sample_pending = 0;
    hardware_imu_init();
    run_sample_imu();
#endif
    run_debug_imu_report();
    pit_ms_init(CCU60_CH0, 1);
    pit_ms_init(CCU60_CH1, 10);
    run_ui_while();
    run_debug_init();
}

void run_while(void)
{
    if (!run_ready)
    {
        run_start_runtime();
    }
#if RUN_IMU_ENABLE
    run_imu_while();
#endif
    run_ui_while();
    run_debug_while();
}

// 只投递事件；运行状态和执行器输出仍由控制中断决定。
void run_request_toggle(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    if (run_ready && RUN_STATE_FAULT != run_status.state)
    {
        run_toggle_request = 1;
    }
    interrupt_global_enable(interrupt_state);
}

// 短暂关中断复制参数，避免主循环读到控制中断正在使用的半个 double。
void run_get_params(run_params_t *params)
{
    uint32 interrupt_state = interrupt_global_disable();
    params->speed = speed_setting;
    params->pid[RUN_PID_LEFT].kp = pid_left.controller.inc.Kp;
    params->pid[RUN_PID_LEFT].ki = pid_left.controller.inc.Ki;
    params->pid[RUN_PID_LEFT].kd = pid_left.controller.inc.Kd;
    params->pid[RUN_PID_RIGHT].kp = pid_right.controller.inc.Kp;
    params->pid[RUN_PID_RIGHT].ki = pid_right.controller.inc.Ki;
    params->pid[RUN_PID_RIGHT].kd = pid_right.controller.inc.Kd;
    params->pid[RUN_PID_STEER].kp = pid_gap.controller.pos.Kp;
    params->pid[RUN_PID_STEER].ki = pid_gap.controller.pos.Ki;
    params->pid[RUN_PID_STEER].kd = pid_gap.controller.pos.Kd;
    interrupt_global_enable(interrupt_state);
}

// UI 的草稿冲突检查与串口的变更检测共用此规则，不比较结构体填充字节。
uint8 run_params_equal(const run_params_t *left, const run_params_t *right)
{
    uint8 group;
    if (left == NULL || right == NULL || left->speed != right->speed)
    {
        return 0;
    }
    for (group = 0; group < RUN_PID_GROUP_COUNT; group++)
    {
        if (left->pid[group].kp != right->pid[group].kp
            || left->pid[group].ki != right->pid[group].ki
            || left->pid[group].kd != right->pid[group].kd)
        {
            return 0;
        }
    }
    return 1;
}

// 先在临界区外校验数值，再原子检查状态并提交整组参数。
// UI/通信不能直接改 PID 指针，运行或等待启动时一律拒绝更新。
uint8 run_apply_params(const run_params_t *params)
{
    uint8 i;
    uint32 interrupt_state;
    if (!params || params->speed < 0 || params->speed > RUN_PARAM_SPEED_MAX)
    {
        return 0;
    }
    for (i = 0; i < RUN_PID_GROUP_COUNT; ++i)
    {
        if (!isfinite(params->pid[i].kp) || !isfinite(params->pid[i].ki) || !isfinite(params->pid[i].kd)
            || params->pid[i].kp < 0 || params->pid[i].kp > RUN_PARAM_GAIN_MAX
            || params->pid[i].ki < 0 || params->pid[i].ki > RUN_PARAM_GAIN_MAX
            || params->pid[i].kd < 0 || params->pid[i].kd > RUN_PARAM_GAIN_MAX)
        {
            return 0;
        }
    }
#if RUN_STEER_FUZZY_ENABLE
    if (params->pid[RUN_PID_STEER].ki != 0.0)
    {
        return 0;
    }
#endif
    interrupt_state = interrupt_global_disable();
    if (RUN_STATE_IDLE != run_status.state || run_start_pending || run_toggle_request)
    {
        interrupt_global_enable(interrupt_state);
        return 0;
    }
    speed_setting = params->speed;
    pid_left.controller.inc.Kp = params->pid[RUN_PID_LEFT].kp;
    pid_left.controller.inc.Ki = params->pid[RUN_PID_LEFT].ki;
    pid_left.controller.inc.Kd = params->pid[RUN_PID_LEFT].kd;
    pid_right.controller.inc.Kp = params->pid[RUN_PID_RIGHT].kp;
    pid_right.controller.inc.Ki = params->pid[RUN_PID_RIGHT].ki;
    pid_right.controller.inc.Kd = params->pid[RUN_PID_RIGHT].kd;
    pid_gap.controller.pos.Kp = params->pid[RUN_PID_STEER].kp;
    pid_gap.controller.pos.Ki = params->pid[RUN_PID_STEER].ki;
    pid_gap.controller.pos.Kd = params->pid[RUN_PID_STEER].kd;
    interrupt_global_enable(interrupt_state);
    return 1;
}

// 一次临界区内复制观测量；格式化、绘屏和发送必须在恢复中断后执行。
void run_get_monitor(run_monitor_t *monitor)
{
    uint32 interrupt_state = interrupt_global_disable();
    monitor->status = run_status;
    monitor->ready = run_ready;
    monitor->start_pending = run_start_pending;
    monitor->frame_sequence = run_vision.frame_sequence;
    monitor->vision_age_ms = run_vision_ticks > 6553U ? 65530U : run_vision_ticks * 10U;
    monitor->mid_valid = run_vision.mid_valid;
    monitor->mid_error = run_vision.mid_error;
    monitor->line_lost = run_vision.line_lost;
    monitor->planned_speed = run_planned_speed;
    monitor->ramped_speed = speed;
    monitor->track_slope = run_vision.track_slope;
    monitor->track_slope_valid = run_vision.track_slope_valid;
    monitor->straight_long = run_vision.straight_long;
    monitor->straight_confirmed = run_straight_confirmed;
    monitor->cross = run_vision.cross;
    monitor->ring_active = run_vision.ring_active;
    monitor->curve_risk = run_curve_risk;
    monitor->curve_speed_limit = run_curve_speed_limit;
    monitor->pid_values[0] = pid_left.controller.inc.target;
    monitor->pid_values[1] = pid_left.controller.inc.actual;
    monitor->pid_values[2] = pid_left.controller.inc.output;
    monitor->pid_values[3] = pid_right.controller.inc.target;
    monitor->pid_values[4] = pid_right.controller.inc.actual;
    monitor->pid_values[5] = pid_right.controller.inc.output;
    monitor->pid_values[6] = pid_gap.controller.pos.target;
    monitor->pid_values[7] = pid_gap.controller.pos.actual;
    monitor->pid_values[8] = pid_gap.controller.pos.output;
    monitor->fuzzy_error = monitor->fuzzy_delta = monitor->fuzzy_kp = monitor->fuzzy_kd = monitor->fuzzy_output = 0;
    monitor->fuzzy_saturated = 0;
#if RUN_STEER_FUZZY_ENABLE
    monitor->fuzzy_error = fuzzy_gap.error;
    monitor->fuzzy_delta = fuzzy_gap.delta_error;
    monitor->fuzzy_kp = fuzzy_gap.active_kp;
    monitor->fuzzy_kd = fuzzy_gap.active_kd;
    monitor->fuzzy_output = fuzzy_gap.output;
    monitor->fuzzy_saturated = fuzzy_gap.output >= fuzzy_gap.config.output_max
                      || fuzzy_gap.output <= fuzzy_gap.config.output_min;
#endif
    interrupt_global_enable(interrupt_state);
}

// 切换运行状态：清各类计数、停输出、复位三路 PID 和转向滤波；
// 进入故障态时顺带清空启停请求，保持故障锁定。
static void run_enter_state(run_state_enum state)
{
    run_status.state = state;
    run_start_pending = 0;
    run_state_ticks = 0;
    run_left_stall_ticks = 0;
    run_right_stall_ticks = 0;
    speed = 0;
    hardware_stop();
    PID_Controller_Reset(&pid_left);
    PID_Controller_Reset(&pid_right);
    PID_Controller_Reset(&pid_gap);
#if RUN_STEER_FUZZY_ENABLE
    FuzzyPID_Reset(&fuzzy_gap);
#endif
    filter_rc_state_reset(&steer_input_filter);
    run_held_mid_error = 0;
    run_direction_gap = 0;
    run_planned_speed = 0;
    run_curve_speed_limit = 0;
    run_element_speed_limit = 0;
    run_safety_speed_limit = 0;
    run_straight_confirm_count = 0;
    run_straight_confirmed = 0;
    run_curve_risk = 0;
#if RUN_SPEED_PLANNER_ENABLE
    run_curve_risk_x4 = 0;
#endif
    run_speed_planner_frame_sequence = 0;
    if (RUN_STATE_FAULT == state)
        run_toggle_request = 0;
}

// 记录故障位与现场数据（目标/实际/PWM），随后锁进故障状态。
static void run_enter_fault(uint8 fault)
{
    run_status.fault = fault;
    run_status.left_target = (int16)pid_left.controller.inc.target;
    run_status.right_target = (int16)pid_right.controller.inc.target;
    run_status.left_actual = encoder_left;
    run_status.right_actual = encoder_right;
    run_status.left_pwm = (int16)pid_left.controller.inc.output;
    run_status.right_pwm = (int16)pid_right.controller.inc.output;
    run_enter_state(RUN_STATE_FAULT);
}

// 启动运行：先切到 RUNNING，再清空滤波窗口、编码器与硬件反馈，
// 避免把停车期间积累的旧数据当成起步速度。
static void run_request_start(void)
{
    run_enter_state(RUN_STATE_RUNNING);
    run_reset_speed_feedback();
    run_vision_ticks = 0;
}

// 每个 10ms 方向周期调用一次：有效中线更新保持值，丢线时沿用上次有效误差，
// 再按原有 alpha=1 的设置经过方向输入低通并计算方向 PID。
static void run_update_direction(void)
{
    float filtered_mid_error;

    if (RUN_STATE_RUNNING != run_status.state)
        return;

    // 只有控制中线有效且当前未丢线时才更新；丢线期间保持最后一次有效误差。
    if (run_vision.mid_valid && !run_vision.line_lost)
        run_held_mid_error = run_vision.mid_error;

    filtered_mid_error = filter_rc_low_pass(&steer_input_filter, (float)run_held_mid_error, 0.6);
#if RUN_STEER_FUZZY_ENABLE
    // 调参仍写原 pid_gap 的 Kp/Kd；下一次 10ms 周期使用新基准值，Ki 永久为零。
    pid_gap.controller.pos.Ki = 0.0;
    FuzzyPID_SetBaseGains(&fuzzy_gap, pid_gap.controller.pos.Kp, 0.0,
                         pid_gap.controller.pos.Kd);
    // E = 0 - filtered_mid_error；重复视觉误差时 EC 自动归零。
    run_direction_gap = FuzzyPID_Calc(&fuzzy_gap, 0, (double)filtered_mid_error);
    // 同步原 target / actual / output 调试通道，保持上位机和串口数据顺序。
    pid_gap.controller.pos.target = fuzzy_gap.target;
    pid_gap.controller.pos.actual = fuzzy_gap.actual;
    pid_gap.controller.pos.output = fuzzy_gap.output;
    pid_gap.controller.pos.Error1 = pid_gap.controller.pos.Error0;
    pid_gap.controller.pos.Error0 = fuzzy_gap.error;
    pid_gap.controller.pos.ErrorInt = 0.0;
#else
    run_direction_gap = PID_Controller_Calc(&pid_gap, 0, (double)filtered_mid_error);
#endif
}

// 返回最近一个 10ms 方向周期产生的方向输出，供 2ms 速度任务复用。
static double run_get_gap(void)
{
    if (RUN_STATE_RUNNING != run_status.state)
        return 0;
    return run_direction_gap;
}

// 单轮堵转判定：目标速度足够大、实际速度接近 0、PWM 却已推到阈值以上，
// 三个条件连续保持 RUN_STALL_CONFIRM_TICKS 个周期才返回 1，任一条件断开计数清零。
static uint8 run_stall_check(int16 target, int16 actual, int16 pwm, uint16 *ticks)
{
    if ((target >= RUN_STALL_TARGET_MIN || target <= -RUN_STALL_TARGET_MIN) && actual > -RUN_STALL_SPEED_MAX && actual < RUN_STALL_SPEED_MAX && (pwm >= MOTOR_STALL_PWM_THRESHOLD || pwm <= -MOTOR_STALL_PWM_THRESHOLD))
    {
        if (*ticks < RUN_STALL_CONFIRM_TICKS)
            (*ticks)++;
    }
    else
    {
        *ticks = 0;
    }
    return (*ticks >= RUN_STALL_CONFIRM_TICKS);
}

// 一个控制周期的执行链：目标速度 ± 差速 -> 双轮 PID -> 堵转检查 -> 视觉停车 -> 输出 PWM。
// 状态不是 RUNNING 时直接停输出返回；一旦判故障或收到停车标志就不再下发 PWM。
static void run_update_wheel_control(int16 target_speed, int16 gap, uint8 stop_requested)
{
    int16 left_target = target_speed + gap;
    int16 right_target = target_speed - gap;
    int16 left_pwm;
    int16 right_pwm;
    uint8 fault = RUN_FAULT_NONE;

    if (RUN_STATE_RUNNING != run_status.state)
    {
        hardware_stop();
        return;
    }

    left_pwm = (int16)PID_Controller_Calc(&pid_left, left_target, encoder_left);
    right_pwm = (int16)PID_Controller_Calc(&pid_right, right_target, encoder_right);
    // 起步宽限期后才判堵转，避免刚启动、速度尚未起来时误判
    if (run_state_ticks > RUN_STALL_GRACE_TICKS)
    {
        if (run_stall_check(left_target, encoder_left, left_pwm, &run_left_stall_ticks))
            fault |= RUN_FAULT_STALL_LEFT;
        if (run_stall_check(right_target, encoder_right, right_pwm, &run_right_stall_ticks))
            fault |= RUN_FAULT_STALL_RIGHT;
    }
    if (fault)
    {
        run_enter_fault(fault);
        return;
    }
    if (stop_requested)
    {
        // 第二次斑马线处理完成：回空闲态，run_enter_state 里已停输出
        run_enter_state(RUN_STATE_IDLE);
        return;
    }
    hardware_set_motors(left_pwm, right_pwm);
}

// 成对读取视觉变量：序号为 0 或奇数（视觉层正在写）直接失败；
// 读前后各用 __dsync 隔开并比对序号，期间换帧也失败，保证快照来自同一帧。
static uint8 run_read_vision(run_vision_struct *snapshot)
{
    run_vision_struct copy;
    uint32 sequence = VisionFrameSequence;

    if (sequence == 0 || (sequence & 1U))
        return 0;
    __dsync();
    copy.frame_sequence = sequence;
    copy.mid_error = ControlMidError;
    copy.mid_valid = ControlMidValid;
    copy.line_lost = LineLostFlag;
    copy.stop = ImageFlag.Stop;
    copy.vision_stop = VisionStopFlag;
    copy.track_slope = Track_Slope_x1000;
    copy.track_slope_valid = Track_SlopeValid;
    copy.straight_long = ImageFlag.straight_long;
    copy.cross = ImageFlag.Cross_Flag;
    copy.zebra = ImageFlag.Zebra_Flag;
    copy.ring_active = ImageFlag.image_element_rings;
    copy.ring_stage = ImageFlag.image_element_rings_flag;
    copy.ring_size = ImageFlag.ring_big_small;
    __dsync();
    if (sequence != VisionFrameSequence)
        return 0;
    *snapshot = copy;
    return 1;
}

// 在两个标定点之间做整数线性插值，保证相邻 slope 分段的速度限制连续。
static int16 run_speed_limit_lerp(int16 value, int16 input_low, int16 input_high,
                                  int16 output_low, int16 output_high)
{
    int32 numerator;

    if (input_high <= input_low)
        return output_high;
    numerator = (int32)(value - input_low) * (int32)(output_high - output_low);
    return (int16)(output_low + numerator / (input_high - input_low));
}

// 普通赛道：风险滤波后连续单调映射，未确认直道限制在 185。
// straight_long 保留作观测参考；普通直道由倾斜和中线偏差连续八帧确认。
static int16 run_calc_curve_speed_limit(const run_vision_struct *vision, int16 user_limit)
{
#if RUN_SPEED_PLANNER_ENABLE
    // 连续弯道试验标定：风险增加只能降低限速；表尾以上保持最后一档。
    static const int16 slope_points[] = {0, 120, 150, 300, 500, 800, 1100};
    static const int16 speed_points[] = {185, 185, 180, 175, 170, 160, 150};
    int16 limit = speed_points[sizeof(speed_points) / sizeof(speed_points[0]) - 1U];
    int16 slope = run_curve_risk;
    uint8 point;

    // 失效不能恢复速度：结合上次最终规划上限，丢线额外由安全限速压低。
    if (!vision->track_slope_valid || !vision->mid_valid || vision->line_lost)
    {
        limit = RUN_SPEED_UNKNOWN_LIMIT;
        if (run_planned_speed > 0 && run_planned_speed < limit)
            limit = run_planned_speed;
        return (user_limit < limit) ? user_limit : limit;
    }
    // 有效特殊元素不套用普通中线斜率；由独立元素限速约束，不能申请高速。
    if (vision->cross || vision->ring_active || vision->zebra)
        return (user_limit < RUN_SPEED_UNCONFIRMED_LIMIT) ? user_limit : RUN_SPEED_UNCONFIRMED_LIMIT;

    for (point = 1; point < (uint8)(sizeof(slope_points) / sizeof(slope_points[0])); point++)
    {
        if (slope <= slope_points[point])
        {
            limit = run_speed_limit_lerp(slope, slope_points[point - 1], slope_points[point],
                                         speed_points[point - 1], speed_points[point]);
            break;
        }
    }
    // 许可只在低风险区放行用户上限；其他区域继续按连续映射限速。
    if (run_straight_confirmed && slope <= RUN_STRAIGHT_SLOPE_THRESHOLD)
        limit = user_limit;
    if (!run_straight_confirmed && limit > RUN_SPEED_UNCONFIRMED_LIMIT)
        limit = RUN_SPEED_UNCONFIRMED_LIMIT;
    return (user_limit < limit) ? user_limit : limit;
#else
    int16 mild_limit = (user_limit < RUN_SPEED_CURVE_MILD) ? user_limit : RUN_SPEED_CURVE_MILD;
    int16 minimum_limit = (user_limit < RUN_SPEED_CURVE_MIN) ? user_limit : RUN_SPEED_CURVE_MIN;
    int32 slope = vision->track_slope;

    if (!vision->track_slope_valid)
    {
        return (user_limit < RUN_SPEED_UNKNOWN_LIMIT) ? user_limit : RUN_SPEED_UNKNOWN_LIMIT;
    }

    if (slope < 0)
        slope = -slope;
    if (slope > 32767)
        slope = 32767;
    if (slope <= RUN_CURVE_SLOPE_LOW)
    {
        return run_straight_confirmed ? user_limit : mild_limit;
    }
    if (slope < RUN_CURVE_SLOPE_MID)
    {
        return run_speed_limit_lerp((int16)slope,
                                    RUN_CURVE_SLOPE_LOW, RUN_CURVE_SLOPE_MID,
                                    user_limit, mild_limit);
    }
    if (slope < RUN_CURVE_SLOPE_HIGH)
    {
        return run_speed_limit_lerp((int16)slope,
                                    RUN_CURVE_SLOPE_MID, RUN_CURVE_SLOPE_HIGH,
                                    mild_limit, minimum_limit);
    }
    return minimum_limit;
#endif
}

// 环岛和十字的补线会影响 slope，因此使用独立上限，并与其他限制取最小值。
static int16 run_calc_element_speed_limit(const run_vision_struct *vision)
{
    int16 limit = RUN_PARAM_SPEED_MAX;

    if (vision->cross && RUN_SPEED_CROSS_LIMIT < limit)
        limit = RUN_SPEED_CROSS_LIMIT;
    if (vision->ring_active && RUN_SPEED_RING_LIMIT < limit)
        limit = RUN_SPEED_RING_LIMIT;
    return limit;
}

// 丢线期间方向环仍保持最后一次有效误差，因此必须同时降低基础速度。
static int16 run_calc_safety_speed_limit(const run_vision_struct *vision)
{
    return vision->line_lost ? RUN_SPEED_LINE_LOST_LIMIT : RUN_PARAM_SPEED_MAX;
}

// 综合用户上限、弯道、特殊元素和安全限制；只产生基础目标速度，不操作执行器或 PID。
static int16 run_speed_planner(const run_vision_struct *vision)
{
    int16 planned = speed_setting;
    int32 slope = vision->track_slope;

    if (planned > RUN_PARAM_SPEED_MAX)
        planned = RUN_PARAM_SPEED_MAX;
    if (planned < -RUN_PARAM_SPEED_MAX)
        planned = -RUN_PARAM_SPEED_MAX;

    // 反向/停车命令保留原语义；当前前向赛道规划仅处理正速度。
    if (planned <= 0)
    {
#if RUN_SPEED_PLANNER_ENABLE
        run_straight_confirm_count = 0;
        run_straight_confirmed = 0;
        run_speed_planner_frame_sequence = vision->frame_sequence;
#endif
        run_curve_speed_limit = planned;
        run_element_speed_limit = planned;
        run_safety_speed_limit = planned;
        return planned;
    }

#if RUN_SPEED_PLANNER_ENABLE
    // 重复同一视觉帧不更新 risk，也不累计直道帧；任何危险标志立即撤销许可。
    if (slope < 0)
        slope = -slope;
    if (slope > 32767)
        slope = 32767;
    if (!vision->track_slope_valid || !vision->mid_valid || vision->line_lost
        || vision->cross || vision->ring_active || vision->zebra
        || vision->stop || vision->vision_stop || run_status.fault
        || RUN_STATE_RUNNING != run_status.state
        || run_vision_ticks >= RUN_VISION_TIMEOUT_TICKS
        || slope > RUN_STRAIGHT_REVOKE_SLOPE
        || vision->mid_error > RUN_STRAIGHT_MID_REVOKE
        || vision->mid_error < -RUN_STRAIGHT_MID_REVOKE)
    {
        run_straight_confirm_count = 0;
        run_straight_confirmed = 0;
    }
    if (vision->frame_sequence != run_speed_planner_frame_sequence)
    {
        run_speed_planner_frame_sequence = vision->frame_sequence;
        if (vision->track_slope_valid && vision->mid_valid && !vision->line_lost
            && !vision->cross && !vision->ring_active && !vision->zebra
            && !vision->stop && !vision->vision_stop && !run_status.fault
            && RUN_STATE_RUNNING == run_status.state
            && run_vision_ticks < RUN_VISION_TIMEOUT_TICKS)
        {
            uint32 sample_x4 = (uint32)slope * 4U;
            if (sample_x4 >= run_curve_risk_x4)
                run_curve_risk_x4 = sample_x4;
            else
                run_curve_risk_x4 = (3U * run_curve_risk_x4 + sample_x4 + 3U) / 4U;
            // 向上量化避免低估风险；误差不足一个 slope 单位。
            run_curve_risk = (int16)((run_curve_risk_x4 + 3U) / 4U);
            if (slope <= RUN_STRAIGHT_SLOPE_THRESHOLD
                && vision->mid_error <= RUN_STRAIGHT_MID_CONFIRM
                && vision->mid_error >= -RUN_STRAIGHT_MID_CONFIRM)
            {
                if (run_straight_confirm_count < RUN_STRAIGHT_CONFIRM_FRAMES)
                    run_straight_confirm_count++;
                if (run_straight_confirm_count >= RUN_STRAIGHT_CONFIRM_FRAMES)
                    run_straight_confirmed = 1;
            }
            else
                run_straight_confirm_count = 0;
        }
        // 失效/元素期间不把 risk 清零，不用该帧的低 slope 提前恢复高速。
    }
#else
    // 直道确认按“新视觉帧”计数，而不是按 10ms 调度次数重复累计同一帧。
    if (vision->frame_sequence != run_speed_planner_frame_sequence)
    {
        run_speed_planner_frame_sequence = vision->frame_sequence;
        if (slope < 0)
            slope = -slope;

        if (vision->track_slope_valid
            && slope < RUN_STRAIGHT_SLOPE_THRESHOLD
            && vision->straight_long)
        {
            if (run_straight_confirm_count < RUN_STRAIGHT_CONFIRM_FRAMES)
                run_straight_confirm_count++;
            if (run_straight_confirm_count >= RUN_STRAIGHT_CONFIRM_FRAMES)
                run_straight_confirmed = 1;
        }
        else
        {
            run_straight_confirm_count = 0;
            // slope 阈值留出 S1-StraightThreshold 的回差，避免临界值轻微抖动反复切换。
            if (!vision->track_slope_valid
                || !vision->straight_long
                || slope >= RUN_CURVE_SLOPE_LOW)
            {
                run_straight_confirmed = 0;
            }
        }
    }

#endif

    run_curve_speed_limit = run_calc_curve_speed_limit(vision, planned);
    run_element_speed_limit = run_calc_element_speed_limit(vision);
    run_safety_speed_limit = run_calc_safety_speed_limit(vision);

    if (run_curve_speed_limit < planned)
        planned = run_curve_speed_limit;
    if (run_element_speed_limit < planned)
        planned = run_element_speed_limit;
    if (run_safety_speed_limit < planned)
        planned = run_safety_speed_limit;
    return planned;
}

// 10ms 基础速度斜坡：快速减速、缓慢加速，并保证不会越过目标值。
static void run_update_speed_ramp(int16 target_speed)
{
    int16 delta;

    if (speed < target_speed)
    {
        delta = target_speed - speed;
        speed += (delta > RUN_SPEED_ACCEL_STEP) ? RUN_SPEED_ACCEL_STEP : delta;
    }
    else if (speed > target_speed)
    {
        delta = speed - target_speed;
        speed -= (delta > RUN_SPEED_DECEL_STEP) ? RUN_SPEED_DECEL_STEP : delta;
    }
}

// 10ms 方向任务：读取最新的一致视觉快照；视觉约 20ms 更新一次，因此同一帧
// 通常连续参与两次方向 PID，但方向控制与执行器输出保持固定 10ms 周期。
static void run_direction_tick_10ms(void)
{
    uint8 keys;
    run_vision_struct snapshot;
    uint32 interrupt_state = interrupt_global_disable();
    keys = run_toggle_request;
    run_toggle_request = 0;
    interrupt_global_enable(interrupt_state);

    // 视觉超时以 10ms 方向任务计数；只有真正的新帧才清零。
    if (run_vision_ticks < 65535U)
        run_vision_ticks++;
    if (run_read_vision(&snapshot))
    {
        if (snapshot.frame_sequence != run_vision.frame_sequence)
        {
            run_vision_ticks = 0;
            run_vision = snapshot;
        }
    }

    // 状态机：IDLE 等启动键，RUNNING 处理方向与保护，FAULT 只保持停车。
    switch (run_status.state)
    {
    case RUN_STATE_IDLE:
        // 正常停车后 KEY1 重启：由 CPU1 复位元素/斑马线计数，等待新的完整帧。
        // 不直接改视觉全局变量；FAULT 不经过此流程，仍须重新上电。
        if (keys)
        {
            if (run_start_pending)
                run_start_pending = 0;
            else
            {
                run_start_frame_sequence = run_vision.frame_sequence;
                Camera_RequestRuntimeReset();
                run_start_pending = 1;
                run_start_wait_ticks = 0;
                run_vision_ticks = 0;
            }
        }
        if (run_start_pending)
        {
            if (run_start_wait_ticks < RUN_VISION_TIMEOUT_TICKS)
                run_start_wait_ticks++;
            if (!Camera_RuntimeResetPending()
                && run_vision.frame_sequence != run_start_frame_sequence
                && run_vision.frame_sequence == VisionFrameSequence)
            {
                if (run_vision.vision_stop)
                    run_enter_fault(RUN_FAULT_VISION_STOP);
                else if (!run_vision.stop)
                    run_request_start();
            }
            if (run_start_pending && run_start_wait_ticks >= RUN_VISION_TIMEOUT_TICKS)
                run_enter_fault(RUN_FAULT_VISION_TIMEOUT);
        }
        break;

    case RUN_STATE_RUNNING:
        // 独立安全修复：持续丢线停止优先于普通停车，不允许按键清除故障锁。
        if (run_vision.vision_stop)
        {
            run_enter_fault(RUN_FAULT_VISION_STOP);
            break;
        }
        // 运行中再按一次启动键立即停车
        if (keys)
        {
            run_enter_state(RUN_STATE_IDLE);
            break;
        }
        // 视觉超过 200ms 没有新帧，判视觉超时故障
        if (run_vision_ticks >= RUN_VISION_TIMEOUT_TICKS)
        {
            run_enter_fault(RUN_FAULT_VISION_TIMEOUT);
            break;
        }
        // 视觉停车标志在方向任务中立即生效，不等待下一次 2ms 速度任务。
        if (run_vision.stop)
        {
            run_enter_state(RUN_STATE_IDLE);
            break;
        }
        // 同一视觉快照同时供速度规划与方向控制使用；规划只改变基础目标速度。
        run_planned_speed = run_speed_planner(&run_vision);
        run_update_speed_ramp(run_planned_speed);
        // 固定 10ms 更新方向 PID 和舵机；速度任务读取同一个最新 gap。
        run_update_direction();
        hardware_set_servo((int16)(run_direction_gap * STEER_OUTPUT_MAP_SCALE));
        break;

    case RUN_STATE_FAULT:
        // 故障态不动作，靠函数末尾的统一停输出保持锁定
        break;

    default:
        // 状态异常直接回空闲，防止跑飞后继续驱动电机
        run_enter_state(RUN_STATE_IDLE);
        break;
    }
    // 非运行状态每个方向任务兜底停一次输出
    if (RUN_STATE_RUNNING != run_status.state)
        hardware_stop();
}

// 左右轮复用同一滤波流程，但各自保存窗口和低通状态。
// 保留原实现的顺序：滑动平均后先转为 int16，再进入低通，避免重构改变量化结果。
static int16 run_filter_wheel_speed(int16 raw_speed, filter_ring_t *ring,
                                    filter_rc_state_t *filter)
{
    int16 averaged_speed;
    filter_ring_write(ring, raw_speed);
    averaged_speed = (int16)filter_moving_average(ring);
    return (int16)filter_rc_low_pass(filter, averaged_speed, 0.4);
}

// 2ms 速度任务：采样/滤波编码器并执行双轮速度 PID，读取最近一次方向任务的 gap。
static void run_speed_tick_2ms(void)
{
    hardware_feedback_t feedback;

    hardware_sample_2ms(&feedback);

    // raw_* 保留未滤波原始速度，encoder_* 为滤波后速度，速度 PID 只使用 encoder_*。
    raw_left = feedback.raw_left_speed;
    raw_right = feedback.raw_right_speed;
    encoder_left = run_filter_wheel_speed(raw_left, &encoder_left_ring, &encoder_left_filter);
    encoder_right = run_filter_wheel_speed(raw_right, &encoder_right_ring, &encoder_right_filter);

    if (RUN_STATE_RUNNING == run_status.state)
    {
        if (run_state_ticks < 65535U)
            run_state_ticks++;
        if (run_vision.vision_stop)
            run_enter_fault(RUN_FAULT_VISION_STOP);
        else
            run_update_wheel_control(speed, (int16)run_get_gap(), run_vision.stop);
    }
    else
    {
        hardware_stop();
    }
}

// 1ms 基础调度：10ms 与 2ms 同时到期时，严格先方向、后速度。
static void run_control_tick_1ms(void)
{
#if RUN_IMU_ENABLE
    if (run_imu_age_ms < 65535U)
    {
        run_imu_age_ms++;
    }
    run_imu_tick_count++;
    if (run_imu_tick_count >= RUN_IMU_PERIOD_TICKS)
    {
        run_imu_tick_count = 0;
        run_imu_sample_pending = 1;
    }
#endif
    run_direction_tick_count++;
    run_speed_tick_count++;

    if (run_direction_tick_count >= RUN_DIRECTION_PERIOD_TICKS)
    {
        run_direction_tick_count = 0;
        run_direction_tick_10ms();
    }
    if (run_speed_tick_count >= RUN_SPEED_PERIOD_TICKS)
    {
        run_speed_tick_count = 0;
        run_speed_tick_2ms();
    }
}

// ------------------ CPU0 定时器中断入口 ------------------
// run 统一编排模块调用；按键先于控制执行，方向与速度同时到期时仍先方向后速度。
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    run_ui_tick_1ms();
    run_control_tick_1ms();
}

// 只设置发送标志，绘屏、通信收发和参数解析均留在 run_while 调用链中。
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    run_debug_tick_10ms();
}
