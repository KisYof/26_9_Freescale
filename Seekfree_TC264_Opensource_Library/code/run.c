#include "run.h"
#include "pid.h"
#include "filter.h"
#include "camera.h"
#include "hardware.h"
#include "run_debug.h"

#define MOTOR_STALL_PWM_THRESHOLD (1500) // 判定堵转所需的最小 PWM 幅值

// 所有计时以 5ms 为单位；以下为初始参数，需要实车标定。
#define RUN_VISION_TIMEOUT_TICKS (40U) // 视觉 200ms 未更新即判视觉超时
#define RUN_STALL_GRACE_TICKS (60U)    // 启动后 300ms 内不做堵转判断
#define RUN_STALL_CONFIRM_TICKS (60U)  // 堵转条件连续成立 300ms 才确认
#define RUN_STALL_TARGET_MIN (20)      // 目标速度大于 20cm/s 才谈堵转
#define RUN_STALL_SPEED_MAX (5)        // 实际速度绝对值小于 5cm/s 视为停转
#define RUN_SPEED_LIMIT (1000)         // 目标速度限幅
#define RUN_SPEED_STEP (2)             // 每 5ms 最多变化 2cm/s，实现斜坡加减速
#define SPEED_MID (170)                // 调试层默认目标速度
#define STEER_OUTPUT_MAP_SCALE (0.5)   // 转向输出换算成舵机角的缩放系数

// 原始速度与滤波后速度：只在 5ms 控制中断里更新
volatile int16 raw_left = 0;
volatile int16 raw_right = 0;
volatile int16 encoder_left = 0;
volatile int16 encoder_right = 0;
int16 speed = 0;                                 // 斜坡爬升后真正执行的目标速度
static volatile int16 speed_setting = SPEED_MID; // 目标速度设定值，可被 WiFi/串口在线修改
static run_status_struct run_status = {RUN_STATE_IDLE, RUN_FAULT_NONE, 0, 0, 0, 0, 0, 0};

// 运行层自己的视觉副本，只读取 camera 原有变量，不改动视觉模块。
typedef struct
{
    uint32 frame_sequence; // 帧序号，用于判断是否收到新帧
    int32 mid_error;       // 控制中线偏差，转向 PID 的输入
    uint8 zebra;           // 斑马线标志，置位时停车
} run_vision_struct;

static run_vision_struct run_vision;
static uint16 run_state_ticks = 0;       // 进入当前状态后的 5ms 计数
static uint16 run_vision_ticks = 0;      // 距上一帧新视觉数据的 5ms 计数
static uint16 run_left_stall_ticks = 0;  // 左轮堵转条件连续成立的 5ms 计数
static uint16 run_right_stall_ticks = 0; // 右轮堵转条件连续成立的 5ms 计数

// 三路 PID：左右轮速度环为增量式，转向环为位置式
static PID_Controller_t pid_left;
static PID_Controller_t pid_right;
static PID_Controller_t pid_gap;

// 编码器滤波：滑动平均窗口 + 一阶低通；steer_input_filter 用于中线偏差
#define ENCODER_FILTER_WINDOW (32)
static filter_ring_t encoder_left_ring;
static filter_ring_t encoder_right_ring;
static float encoder_left_buffer[ENCODER_FILTER_WINDOW];
static float encoder_right_buffer[ENCODER_FILTER_WINDOW];
static filter_rc_state_t encoder_left_filter;
static filter_rc_state_t encoder_right_filter;
static filter_rc_state_t steer_input_filter;

// ------------------ 内部函数前向声明 ------------------
static void run_stop_outputs(void);                // 停输出：电机停转、舵机回中
static void run_request_start(void);               // 启动：清滤波、编码器与视觉计时
static void run_enter_state(run_state_enum state); // 切状态：清计数、停输出、复位 PID 与滤波
static void run_control_tick_5ms(void);            // 5ms 控制周期主体：采样、状态机、输出

// 上电调用一次：初始化硬件、三路 PID、编码器滤波，注册调试引用，
// 进入空闲态后再启动 5ms 控制中断与 10ms 调试中断。
void run_init(void)
{
    run_debug_control_ref_t debug_control;

    hardware_init();

    // 左右轮速度环：增量式，输出限 ±2000；转向环：位置式，输出限 ±80
    PID_Controller_Init(&pid_left, PID_ALGORITHM_INC, 0.01, 0.05, 0.0, 2000, -2000, 0, 0);
    PID_Controller_Init(&pid_right, PID_ALGORITHM_INC, 0.01, 0.05, 0.0, 2000, -2000, 0, 0);
    PID_Controller_Init(&pid_gap, PID_ALGORITHM_POS, 0.9, 0.0, 0.2, 80, -80, 0, 0);

    // 滑动平均窗口与三个一阶滤波器全部清零
    filter_ring_init(&encoder_left_ring, encoder_left_buffer, ENCODER_FILTER_WINDOW);
    filter_ring_init(&encoder_right_ring, encoder_right_buffer, ENCODER_FILTER_WINDOW);
    filter_rc_state_reset(&encoder_left_filter);
    filter_rc_state_reset(&encoder_right_filter);
    filter_rc_state_reset(&steer_input_filter);

    // 把可调参数的指针交给调试层，WiFi/串口改参即刻生效
    debug_control.speed_setting = &speed_setting;
    debug_control.left_pid = &pid_left;
    debug_control.right_pid = &pid_right;
    debug_control.gap_pid = &pid_gap;
    run_debug_init(&debug_control);

    // run_debug_init() 不绘屏；IPS200 初始化完成后，主循环才执行绘屏。
    run_enter_state(RUN_STATE_IDLE);

    // WiFi 初始化等待已结束，控制对象也全部就绪后，再启动周期中断。
    pit_ms_init(CCU60_CH0, 5);
    pit_ms_init(CCU60_CH1, 10);
}

// 主循环入口：控制运算全部在 5ms 中断里，这里只跑调试层的低频任务。
void run_while(void)
{
    run_debug_while();
}

// 统一停输出：电机停转、舵机回中，任何非运行状态都调用它兜底。
static void run_stop_outputs(void)
{
    hardware_stop();
}

// 关中断拷贝一份状态快照，保证读到的字段来自同一时刻。
void run_get_status(run_status_struct *status)
{
    uint32 interrupt_state = interrupt_global_disable();
    *status = run_status;
    interrupt_global_enable(interrupt_state);
}

// 切换运行状态：清各类计数、停输出、复位三路 PID 和转向滤波；
// 进入故障态时顺带丢弃按键事件，防止停车键被带进新状态。
static void run_enter_state(run_state_enum state)
{
    run_status.state = state;
    run_state_ticks = 0;
    run_left_stall_ticks = 0;
    run_right_stall_ticks = 0;
    speed = 0;
    run_stop_outputs();
    PID_Controller_Reset(&pid_left);
    PID_Controller_Reset(&pid_right);
    PID_Controller_Reset(&pid_gap);
    filter_rc_state_reset(&steer_input_filter);
    if (RUN_STATE_FAULT == state)
        run_debug_discard_keys();
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
    filter_ring_init(&encoder_left_ring, encoder_left_buffer, ENCODER_FILTER_WINDOW);
    filter_ring_init(&encoder_right_ring, encoder_right_buffer, ENCODER_FILTER_WINDOW);
    filter_rc_state_reset(&encoder_left_filter);
    filter_rc_state_reset(&encoder_right_filter);
    hardware_reset_feedback();
    raw_left = raw_right = 0;
    encoder_left = encoder_right = 0;
    run_vision_ticks = 0;
}

// 转向环输出：中线偏差低通滤波后送转向 PID，得到左右轮差速（供差速和舵机共用）；
// 非运行状态直接返回 0，并复位中线偏差滤波。
double run_get_gap(void)
{
    static double held_gap = 0;
    float filtered_mid_error;

    if (RUN_STATE_RUNNING != run_status.state)
    {
        held_gap = 0;
        filter_rc_state_reset(&steer_input_filter);
        return 0;
    }

    filtered_mid_error = filter_rc_low_pass(&steer_input_filter, (float)run_vision.mid_error, 1);
    held_gap = PID_Controller_Calc(&pid_gap, 0, (double)filtered_mid_error);
    return held_gap;
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

// 一个控制周期的执行链：目标速度 ± 差速 -> 双轮 PID -> 堵转检查 -> 斑马线停车 -> 输出 PWM。
// 状态不是 RUNNING 时直接停输出返回；一旦判故障或斑马线就不再下发 PWM。
static void run_direct(int16 target_speed, int16 gap, uint8 zebra_stop)
{
    int16 left_target = target_speed + gap;
    int16 right_target = target_speed - gap;
    int16 left_pwm;
    int16 right_pwm;
    uint8 fault = RUN_FAULT_NONE;

    if (RUN_STATE_RUNNING != run_status.state)
    {
        run_stop_outputs();
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
    if (zebra_stop)
    {
        // 斑马线：回空闲态，run_enter_state 里已停输出
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
    copy.zebra = ImageFlag.Zebra_Flag;
    __dsync();
    if (sequence != VisionFrameSequence)
        return 0;
    *snapshot = copy;
    return 1;
}

// 5ms 控制中断主体：采样并滤波编码器、更新视觉快照、按状态机推进
// （取按键 -> 目标速度斜坡 -> 转向环 -> 双轮环与保护 -> 下发 PWM）。
static void run_control_tick_5ms(void)
{
    hardware_feedback_t feedback;
    uint8 keys = run_debug_take_keys();
    run_vision_struct snapshot;
    double gap;
    int16 setting;

    // 硬件层只采样和换算原始速度；滤波仍由控制层完成。
    hardware_update_5ms();
    hardware_get_feedback(&feedback);
    // raw_* 保留未滤波原始值，encoder_* 为滤波后值，PID 只使用 encoder_*
    raw_left = feedback.raw_left_speed;
    encoder_left = raw_left;
    filter_ring_write(&encoder_left_ring, encoder_left);
    encoder_left = filter_moving_average(&encoder_left_ring);
    encoder_left = filter_rc_low_pass(&encoder_left_filter, encoder_left, 0.2);
    raw_right = feedback.raw_right_speed;
    encoder_right = raw_right;
    filter_ring_write(&encoder_right_ring, encoder_right);
    encoder_right = filter_moving_average(&encoder_right_ring);
    encoder_right = filter_rc_low_pass(&encoder_right_filter, encoder_right, 0.2);

    // 计数封顶在 65535，避免长时间运行后溢出回绕
    if (run_state_ticks < 65535U)
        run_state_ticks++;
    if (run_vision_ticks < 65535U)
        run_vision_ticks++;
    // 只有拿到新帧才刷新视觉超时计时，旧帧不算数
    if (run_read_vision(&snapshot))
    {
        if (snapshot.frame_sequence != run_vision.frame_sequence)
            run_vision_ticks = 0;
        run_vision = snapshot;
    }

    // 状态机：IDLE 等启动键，RUNNING 跑闭环与保护，FAULT 只保持停车
    switch (run_status.state)
    {
    case RUN_STATE_IDLE:
        // 空闲态按启动键进入运行
        if (keys & RUN_KEY_START_STOP)
            run_request_start();
        break;

    case RUN_STATE_RUNNING:
        // 运行中再按一次启动键立即停车
        if (keys & RUN_KEY_START_STOP)
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
        // 目标速度先限幅，再按每周期步长斜坡逼近，避免阶跃
        setting = speed_setting;
        if (setting > RUN_SPEED_LIMIT)
            setting = RUN_SPEED_LIMIT;
        if (setting < -RUN_SPEED_LIMIT)
            setting = -RUN_SPEED_LIMIT;
        if (speed < setting)
            speed += (setting - speed > RUN_SPEED_STEP) ? RUN_SPEED_STEP : setting - speed;
        else if (speed > setting)
            speed -= (speed - setting > RUN_SPEED_STEP) ? RUN_SPEED_STEP : speed - setting;
        // 转向环只算一次：差速给双轮，缩放后给舵机
        gap = run_get_gap();
        run_direct(speed, (int16)gap, run_vision.zebra);
        // run_direct 可能已切状态（斑马线/故障），仍运行才更新舵机
        if (RUN_STATE_RUNNING == run_status.state)
            hardware_set_servo((int16)(gap * STEER_OUTPUT_MAP_SCALE));
        break;

    case RUN_STATE_FAULT:
        // 故障态不动作，靠函数末尾的统一停输出保持锁定
        break;

    default:
        // 状态异常直接回空闲，防止跑飞后继续驱动电机
        run_enter_state(RUN_STATE_IDLE);
        break;
    }
    // 非运行状态每周期兜底停一次输出
    if (RUN_STATE_RUNNING != run_status.state)
        run_stop_outputs();
}

// CCU60 CH0 中断：5ms 控制周期，先开高优先级嵌套再清标志
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    run_control_tick_5ms();
}

// CCU60 CH1 中断：10ms 调试周期，只做按键扫描与刷新、发送节拍
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    run_debug_tick_10ms();
}
