#include "run.h"
#include "pid.h"
#include "filter.h"
#include "camera.h"
#include "hardware.h"
#include "run_debug.h"

#define MOTOR_STALL_PWM_THRESHOLD (1500) // 判定堵转所需的最小 PWM 幅值

// 1ms 基础节拍下分出 2ms 速度任务和 10ms 方向任务；以下参数保持原来的实际时长。
#define RUN_SPEED_PERIOD_TICKS (2U)
#define RUN_DIRECTION_PERIOD_TICKS (10U)
#define RUN_VISION_TIMEOUT_TICKS (20U) // 方向任务计数：视觉 200ms 未更新即判视觉超时
#define RUN_STALL_GRACE_TICKS (150U)   // 速度任务计数：启动后 300ms 内不做堵转判断
#define RUN_STALL_CONFIRM_TICKS (150U) // 速度任务计数：堵转条件连续成立 300ms 才确认
#define RUN_STALL_TARGET_MIN (20)      // 目标速度大于 20cm/s 才谈堵转
#define RUN_STALL_SPEED_MAX (5)        // 实际速度绝对值小于 5cm/s 视为停转
#define RUN_SPEED_LIMIT (1000)         // 目标速度限幅
#define RUN_SPEED_ACCEL_STEP (2)       // 10ms 加速步长，保守初值，需实车标定
#define RUN_SPEED_DECEL_STEP (6)       // 10ms 减速步长，必须大于加速步长，需实车标定
#define SPEED_MID (170)                // 调试层默认目标速度
#define STEER_OUTPUT_MAP_SCALE (0.25)  // 转向输出换算成舵机角的缩放系数

// Speed Planner 保守初值：Track_Slope_x1000 是弯道趋势而非严格几何曲率，以下参数均需实车标定。
#define RUN_STRAIGHT_SLOPE_THRESHOLD (120) // 小于此值且 straight_long 有效，才累计直道确认
#define RUN_CURVE_SLOPE_LOW (150)          // 轻弯起点 S1
#define RUN_CURVE_SLOPE_MID (500)          // 中弯分段点 S2
#define RUN_CURVE_SLOPE_HIGH (1000)        // 急弯分段点 S3
#define RUN_SPEED_CURVE_MILD (130)         // 未确认长直道/中等弯道的速度上限
#define RUN_SPEED_CURVE_MIN (90)           // 急弯最低速度上限
#define RUN_SPEED_UNKNOWN_LIMIT (100)      // slope 无效时的保守速度上限
#define RUN_SPEED_CROSS_LIMIT (100)        // 十字速度上限
#define RUN_SPEED_RING_LIMIT (90)          // 环岛速度上限
#define RUN_SPEED_LINE_LOST_LIMIT (70)     // 丢线时的安全速度上限
#define RUN_STRAIGHT_CONFIRM_FRAMES (4U)   // 连续有效视觉帧数，约 80ms（按 50fps 估算）

// 原始速度与滤波后速度：只在 2ms 速度任务里更新
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
    uint8 mid_valid;       // 当前控制中线是否有效
    uint8 line_lost;       // 视觉丢线标志
    uint8 zebra;           // 斑马线标志，置位时停车
    int16 track_slope;     // 远近控制中线得到的赛道倾斜趋势（放大 1000 倍）
    uint8 track_slope_valid;
    uint8 straight_long;   // 仅作为恢复高速的许可
    uint8 cross;
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
static uint8 run_straight_confirmed = 0;
static uint32 run_speed_planner_frame_sequence = 0;

// 三路 PID：左右轮速度环为增量式，转向环为位置式
static PID_Controller_t pid_left;
static PID_Controller_t pid_right;
static PID_Controller_t pid_gap;

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
static void run_stop_outputs(void);                // 停输出：电机停转、舵机回中
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

// 上电调用一次：初始化硬件、三路 PID、编码器滤波，注册调试引用，
// 进入空闲态后再启动 1ms 基础调度中断与 10ms 调试中断。
void run_init(void)
{
    run_debug_control_ref_t debug_control;

    hardware_init();

    // 保持当前左右轮速度 PID 与方向 PID 参数不变，本次 Speed Planner 只调整目标速度。
    PID_Controller_Init(&pid_left, PID_ALGORITHM_INC, 0.09, 0.03, 0.0, 3000, -3000, 0, 0);
    PID_Controller_Init(&pid_right, PID_ALGORITHM_INC, 0.09, 0.03, 0.0, 3000, -3000, 0, 0);
    PID_Controller_Init(&pid_gap, PID_ALGORITHM_POS, 1.4, 0.0, 0.5, 90, -90, 0, 0);

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
    debug_control.planned_speed = &run_planned_speed;
    debug_control.ramped_speed = &speed;
    debug_control.track_slope = &run_vision.track_slope;
    debug_control.track_slope_valid = &run_vision.track_slope_valid;
    debug_control.straight_long = &run_vision.straight_long;
    debug_control.cross = &run_vision.cross;
    debug_control.ring_active = &run_vision.ring_active;
    debug_control.ring_stage = &run_vision.ring_stage;
    debug_control.line_lost = &run_vision.line_lost;
    run_debug_init(&debug_control);

    // run_debug_init() 不绘屏；IPS200 初始化完成后，主循环才执行绘屏。
    run_enter_state(RUN_STATE_IDLE);

    // WiFi 初始化等待已结束，控制对象也全部就绪后，再启动 1ms 基础调度和 10ms 调试中断。
    pit_ms_init(CCU60_CH0, 1);
    pit_ms_init(CCU60_CH1, 10);
}

// 主循环入口：控制运算全部在 1ms 基础调度中断里，这里只跑调试层的低频任务。
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
    run_held_mid_error = 0;
    run_direction_gap = 0;
    run_planned_speed = 0;
    run_curve_speed_limit = 0;
    run_element_speed_limit = 0;
    run_safety_speed_limit = 0;
    run_straight_confirm_count = 0;
    run_straight_confirmed = 0;
    run_speed_planner_frame_sequence = 0;
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

    filtered_mid_error = filter_rc_low_pass(&steer_input_filter, (float)run_held_mid_error, 1);
    run_direction_gap = PID_Controller_Calc(&pid_gap, 0, (double)filtered_mid_error);
}

// 返回最近一个 10ms 方向周期产生的方向输出，供 2ms 速度任务复用。
double run_get_gap(void)
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
    copy.mid_valid = ControlMidValid;
    copy.line_lost = LineLostFlag;
    copy.zebra = ImageFlag.Zebra_Flag;
    copy.track_slope = Track_Slope_x1000;
    copy.track_slope_valid = Track_SlopeValid;
    copy.straight_long = ImageFlag.straight_long;
    copy.cross = ImageFlag.Cross_Flag;
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

// 普通赛道限速：有效 slope 使用连续分段映射；无效 slope 绝不按直道放行。
// straight_long 只负责允许恢复用户最高速度，未确认时最高限制在轻弯速度。
static int16 run_calc_curve_speed_limit(const run_vision_struct *vision, int16 user_limit)
{
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
}

// 环岛和十字的补线会影响 slope，因此使用独立上限，并与其他限制取最小值。
static int16 run_calc_element_speed_limit(const run_vision_struct *vision)
{
    int16 limit = RUN_SPEED_LIMIT;

    if (vision->cross && RUN_SPEED_CROSS_LIMIT < limit)
        limit = RUN_SPEED_CROSS_LIMIT;
    if (vision->ring_active && RUN_SPEED_RING_LIMIT < limit)
        limit = RUN_SPEED_RING_LIMIT;
    return limit;
}

// 丢线期间方向环仍保持最后一次有效误差，因此必须同时降低基础速度。
static int16 run_calc_safety_speed_limit(const run_vision_struct *vision)
{
    return vision->line_lost ? RUN_SPEED_LINE_LOST_LIMIT : RUN_SPEED_LIMIT;
}

// 综合用户上限、弯道、特殊元素和安全限制；只产生基础目标速度，不操作执行器或 PID。
static int16 run_speed_planner(const run_vision_struct *vision)
{
    int16 planned = speed_setting;
    int32 slope = vision->track_slope;

    if (planned > RUN_SPEED_LIMIT)
        planned = RUN_SPEED_LIMIT;
    if (planned < -RUN_SPEED_LIMIT)
        planned = -RUN_SPEED_LIMIT;

    // 反向/停车命令保留原语义；当前前向赛道规划仅处理正速度。
    if (planned <= 0)
    {
        run_curve_speed_limit = planned;
        run_element_speed_limit = planned;
        run_safety_speed_limit = planned;
        return planned;
    }

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
    uint8 keys = run_debug_take_keys();
    run_vision_struct snapshot;

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
        // 斑马线保护在方向任务中立即停车，不等待下一次 2ms 速度任务。
        if (run_vision.zebra)
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
        run_stop_outputs();
}

// 2ms 速度任务：采样/滤波编码器并执行双轮速度 PID。方向 gap 只读最近一次新帧结果。
static void run_speed_tick_2ms(void)
{
    hardware_feedback_t feedback;

    hardware_update_2ms();
    hardware_get_feedback(&feedback);

    // raw_* 保留未滤波原始速度，encoder_* 为滤波后速度，速度 PID 只使用 encoder_*。
    raw_left = feedback.raw_left_speed;
    encoder_left = raw_left;
    filter_ring_write(&encoder_left_ring, encoder_left);
    encoder_left = filter_moving_average(&encoder_left_ring);
    encoder_left = filter_rc_low_pass(&encoder_left_filter, encoder_left, 0.4);

    raw_right = feedback.raw_right_speed;
    encoder_right = raw_right;
    filter_ring_write(&encoder_right_ring, encoder_right);
    encoder_right = filter_moving_average(&encoder_right_ring);
    encoder_right = filter_rc_low_pass(&encoder_right_filter, encoder_right, 0.4);

    if (RUN_STATE_RUNNING == run_status.state)
    {
        if (run_state_ticks < 65535U)
            run_state_ticks++;
        run_direct(speed, (int16)run_get_gap(), run_vision.zebra);
    }
    else
    {
        run_stop_outputs();
    }
}

// 1ms 基础调度：10ms 与 2ms 同时到期时，严格先方向、后速度。
static void run_control_tick_1ms(void)
{
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

// CCU60 CH0 中断：1ms 基础调度，先开高优先级嵌套再清标志
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    run_control_tick_1ms();
}

// CCU60 CH1 中断：10ms 调试周期，只做按键扫描与刷新、发送节拍
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    run_debug_tick_10ms();
}
