#include "run.h"
#include "pid.h"
#include "filter.h"
#include "camera.h"
#include "run_ui.h"
#if RUN_UART_PID_DEBUG_ENABLE
#include "uart_cmd.h"
#endif

// ------------------ PID 调试串口 ------------------
#define PID_DEBUG_UART        (UART_2)
#define PID_DEBUG_UART_BAUD   (115200U)
#define PID_DEBUG_UART_TX_PIN (UART2_TX_P10_5)
#define PID_DEBUG_UART_RX_PIN (UART2_RX_P10_6)

// ------------------ WiFi 通信配置 ------------------
#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
#define WIFI_IMAGE_SSID        "KY"
#define WIFI_IMAGE_PASSWORD    "asdfghjkl"
#define WIFI_IMAGE_TARGET_IP   "10.176.40.16"
#define WIFI_IMAGE_TARGET_PORT "8086"
#define WIFI_IMAGE_LOCAL_PORT  "6666"
#endif

// ------------------ WiFi PID 调试 ------------------
#if RUN_WIFI_PID_DEBUG_ENABLE
#define WIFI_PID_OSCILLOSCOPE_CHANNEL_NUM (9)

typedef enum
{
    WIFI_PARAM_SPEED = 0,
    WIFI_PARAM_PID_GROUP,
    WIFI_PARAM_KP,
    WIFI_PARAM_KI,
    WIFI_PARAM_KD,
    WIFI_PARAM_COUNT
} wifi_debug_param_index_enum;

typedef enum
{
    WIFI_PID_GROUP_LEFT = 0,
    WIFI_PID_GROUP_RIGHT,
    WIFI_PID_GROUP_STEER,
    WIFI_PID_GROUP_COUNT
} wifi_pid_group_enum;
#endif

// 图传只在 CPU0 的 run_while() 中执行，CPU1 仅负责发布处理完成的稳定帧
#if RUN_WIFI_IMAGE_ENABLE
static uint8 wifi_image_ready = 0;
static uint32 wifi_image_last_sequence = 0;
static uint8 wifi_image_buffer[LCDH][LCDW];
static seekfree_assistant_camera_struct wifi_image_information;
static seekfree_assistant_camera_boundary_uint8_struct wifi_left_boundary[LCDH];
static seekfree_assistant_camera_boundary_uint8_struct wifi_right_boundary[LCDH];
static seekfree_assistant_camera_boundary_uint8_struct wifi_mid_boundary[LCDH];
static seekfree_assistant_camera_boundary_struct wifi_left_information;
static seekfree_assistant_camera_boundary_struct wifi_right_information;
static seekfree_assistant_camera_boundary_struct wifi_mid_information;
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
static seekfree_assistant_oscilloscope_struct wifi_pid_oscilloscope;
static float wifi_pid_oscilloscope_buffer[WIFI_PID_OSCILLOSCOPE_CHANNEL_NUM];
static seekfree_assistant_debug_param_struct wifi_debug_param;
static uint8 wifi_pid_group = WIFI_PID_GROUP_LEFT;
#endif

// ------------------ 电机相关 ------------------
#define DIR_R2 (P21_2) // 实际使用的电机组
#define PWM_R2 (ATOM0_CH1_P21_3)
#define DIR_L2 (P21_4)
#define PWM_L2 (ATOM0_CH3_P21_5)

// ------------------ 编码器相关 ------------------
#define ENCODER_1 (TIM2_ENCODER)
#define ENCODER_1_A (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_1_B (TIM2_ENCODER_CH2_P33_6)

#define ENCODER_2 (TIM6_ENCODER)
#define ENCODER_2_A (TIM6_ENCODER_CH1_P20_3)
#define ENCODER_2_B (TIM6_ENCODER_CH2_P20_0)

// 编码器->实际速度转化，（M法：输入采样窗口内脉冲数 pulse_cnt）
#define ENCODER_MULTIPLE 1.0f                                                             // 编码器倍频，1倍频
#define PULSE_PRE_ROUND 1024.0f                                                           // 编码器【电机轴每转一圈】脉冲数，1024线
#define WHEEL_TO_ENCODER_RATIO (68.0f / 30.0f)                                            // 减速比：车轮齿数/编码器齿数，加浮点小数点
#define PULSE_PER_WHEEL_REV (WHEEL_TO_ENCODER_RATIO * PULSE_PRE_ROUND * ENCODER_MULTIPLE) // 车轮转完整一圈，编码器总共产生的脉冲数据
#define RADIUS_OF_TYRE 3.2f                                                               // 轮子半径 cm
#define LINE_SPEED_CM_REV (RADIUS_OF_TYRE * 2.0f * 3.1415926f)                            // 轮子周长 cm

#define SAMPLE_TIME_S 0.005f // M法采样周期：5ms = 0.005秒

#define CALC_WHEEL_RPS(pulse_cnt) ((pulse_cnt) / (PULSE_PER_WHEEL_REV * SAMPLE_TIME_S)) // 输出车轮转速 rps（圈/秒）
#define CALC_LINE_SPEED_CM_S(pulse_cnt) (CALC_WHEEL_RPS(pulse_cnt) * LINE_SPEED_CM_REV) // 输出线速度 cm/s
#define CALC_LINE_SPEED_M_S(pulse_cnt) (CALC_LINE_SPEED_CM_S(pulse_cnt) / 100.0f)       // 输出线速度 m/s

// ------------------ 舵机相关 ------------------
#define SERVO_MOTOR_PWM (ATOM1_CH1_P33_9) // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)             // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX (78) // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX (102)

// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif

// ------------------ 速度决策参数 ------------------
#define SPEED_LOW (80)   // 低速
#define SPEED_MID (120)   // 中速
#define SPEED_HIGH (160) // 高速
#define LOST_STEER_RECENTER_STEP (2.0)

// ------------------ 方向环输出映射参数 ------------------
#define STEER_OUTPUT_MAP_SCALE (0.8) // 映射比例：PID 原始输出乘以该系数
#define STEER_INPUT_FILTER_ALPHA (0.2f) // 方向环输入一阶低通系数，越小滤波越强

// ------------------ 运行状态与控制器 ------------------
volatile int16 raw_left = 0;
volatile int16 raw_right = 0;
volatile int16 encoder_left = 0;
volatile int16 encoder_right = 0;
int16 speed = SPEED_MID;
static int16 speed_setting = SPEED_MID;
#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
static volatile uint8 pid_debug_send_flag = 0;
#endif
static volatile uint8 run_enabled = 0;
static volatile uint8 run_start_pending = 0;
#if RUN_UART_PID_DEBUG_ENABLE
static ucmd_ctx_t pid_debug_cmd;
static char pid_debug_text[128];
#endif

PID_Controller_t pid_left;
PID_Controller_t pid_right;
PID_Controller_t pid_gap; // PD差速位置环

// 编码器滑动平均滤波
#define ENCODER_FILTER_WINDOW 32 // 滑动窗口大小，最大64
static filter_ring_t encoder_left_ring;
static filter_ring_t encoder_right_ring;
static float encoder_left_buffer[ENCODER_FILTER_WINDOW];
static float encoder_right_buffer[ENCODER_FILTER_WINDOW];

// 编码器 RC 低通滤波器
static filter_rc_state_t encoder_left_filter;
static filter_rc_state_t encoder_right_filter;

// 方向环输入 RC 低通滤波器
static filter_rc_state_t steer_input_filter;

static void run_stop_outputs(void);
static void run_request_start(void);

#if RUN_UART_PID_DEBUG_ENABLE
static void pid_debug_uart_receive(void);
static void pid_debug_uart_reply(const char *text);
static void pid_debug_send(void);
#endif

#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
static void wifi_module_init(void);
#endif

#if RUN_WIFI_IMAGE_ENABLE
static void wifi_image_while(void);
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
static void wifi_debug_param_init(void);
static void wifi_debug_param_while(void);
static void wifi_debug_param_load_group(uint8 group);
static void wifi_pid_oscilloscope_send(void);
#endif

// ------------------ 初始化与主循环 ------------------

// run 层总初始化函数，仅在 CPU0 上电初始化阶段调用一次
void run_init(void)
{
    // 串口 PID 调试由 RUN_UART_PID_DEBUG_ENABLE 独立控制
#if RUN_UART_PID_DEBUG_ENABLE
    uart_init(PID_DEBUG_UART, PID_DEBUG_UART_BAUD,
              PID_DEBUG_UART_TX_PIN, PID_DEBUG_UART_RX_PIN);
    ucmd_init(&pid_debug_cmd, PID_DEBUG_UART, pid_debug_uart_reply);

    // 紧凑调参命令：spd10、lkp1.2、rki0.1、dkd0.05，以回车或换行结束
    (void)ucmd_reg_int16(&pid_debug_cmd, "spd", &speed_setting);
    (void)ucmd_reg_double(&pid_debug_cmd, "lkp", &pid_left.controller.inc.Kp);
    (void)ucmd_reg_double(&pid_debug_cmd, "lki", &pid_left.controller.inc.Ki);
    (void)ucmd_reg_double(&pid_debug_cmd, "lkd", &pid_left.controller.inc.Kd);
    (void)ucmd_reg_double(&pid_debug_cmd, "rkp", &pid_right.controller.inc.Kp);
    (void)ucmd_reg_double(&pid_debug_cmd, "rki", &pid_right.controller.inc.Ki);
    (void)ucmd_reg_double(&pid_debug_cmd, "rkd", &pid_right.controller.inc.Kd);
    (void)ucmd_reg_double(&pid_debug_cmd, "dkp", &pid_gap.controller.pos.Kp);
    (void)ucmd_reg_double(&pid_debug_cmd, "dki", &pid_gap.controller.pos.Ki);
    (void)ucmd_reg_double(&pid_debug_cmd, "dkd", &pid_gap.controller.pos.Kd);
#endif

    // 电机驱动初始化
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R2, 17000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出
    pwm_init(PWM_L2, 17000, 0);

    // 编码器初始化
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 舵机初始化
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // 电机PID初始化
    PID_Controller_Init(&pid_left, PID_ALGORITHM_INC, 0.01, 0.05, 0.0, 2000, -2000, 0, 0);
    PID_Controller_Init(&pid_right, PID_ALGORITHM_INC, 0.01, 0.05, 0.0, 2000, -2000, 0, 0);

    // 转向位置环：当前只使用横向误差 PID
    PID_Controller_Init(&pid_gap, PID_ALGORITHM_POS, 0.4, 0.0, 0.1, 40, -40, 0, 0);

    // 编码器平均滤波初始化
    filter_ring_init(&encoder_left_ring, encoder_left_buffer, ENCODER_FILTER_WINDOW);
    filter_ring_init(&encoder_right_ring, encoder_right_buffer, ENCODER_FILTER_WINDOW);

    // 编码器低通滤波滤波初始化
    filter_rc_state_reset(&encoder_left_filter);
    filter_rc_state_reset(&encoder_right_filter);
    filter_rc_state_reset(&steer_input_filter);

    // WiFi 图传与 WiFi PID 调试共用模块和 TCP 连接
#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
    wifi_module_init();
#endif
#if RUN_WIFI_PID_DEBUG_ENABLE
    wifi_debug_param_init();
#endif

    // 界面初始化后保持停车，必须收到 KEY1 发车请求才允许输出
    run_ui_init();
    run_enabled = 0;
    run_start_pending = 0;
    run_stop_outputs();

    // 所有控制对象准备完成后再启动周期中断
    pit_ms_init(CCU60_CH0, 5);  // 5ms：车辆闭环控制
    pit_ms_init(CCU60_CH1, 10); // 10ms：按键扫描和界面刷新计时
}

// CPU0 主循环调用函数，不执行电机闭环，避免图传和界面处理改变控制周期
void run_while(void)
{
    // 串口收字节与完整命令解析都放在主循环，避免占用控制中断
#if RUN_UART_PID_DEBUG_ENABLE
    pid_debug_uart_receive();
    ucmd_poll(&pid_debug_cmd);
    speed = speed_setting;
#endif

    // WiFi 接收解析只在 CPU0 主循环进行，不占用 5ms 控制中断
#if RUN_WIFI_PID_DEBUG_ENABLE
    wifi_debug_param_while();
#endif

    // 界面层只返回一次性 KEY1 发车事件，具体清状态流程由 run 层完成
    if (run_ui_while(run_enabled, &pid_left, &pid_right, &pid_gap))
    {
        run_request_start();
    }

    // KEY1 先请求 CPU1 清空视觉状态；复位完成且控制中线有效后才真正发车
    if (run_start_pending && !Camera_RuntimeResetPending() && ControlMidValid && !ImageFlag.Zebra_Flag)
    {
        uint32 interrupt_state = interrupt_global_disable();
        run_start_pending = 0;
        run_enabled = 1;
        interrupt_global_enable(interrupt_state);
    }

#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
    if (pid_debug_send_flag == 1)
    {
        pid_debug_send_flag = 0;
        #if RUN_UART_PID_DEBUG_ENABLE
                pid_debug_send();
        #endif

        #if RUN_WIFI_PID_DEBUG_ENABLE
                wifi_pid_oscilloscope_send();
        #endif
    }
#endif

    // 图传位于主循环末尾，即使发送等待，5ms 控制中断仍可抢占执行
#if RUN_WIFI_IMAGE_ENABLE
    wifi_image_while();
#endif
}

// ------------------ 启停与状态清理 ------------------

// 立即关闭电机和舵机输出，同时清空三个 PID 与编码器累计值
static void run_stop_outputs(void)
{
    PID_Controller_Reset(&pid_left);
    PID_Controller_Reset(&pid_right);
    PID_Controller_Reset(&pid_gap);
    run_motor_left_speed(0);
    run_motor_right_speed(0);
    run_servo_motor_angle(0);
    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
}

// 响应 KEY1 发车事件：先停车清状态，再通知 CPU1 清空视觉运行状态
static void run_request_start(void)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    run_enabled = 0;
    run_start_pending = 0;
    run_stop_outputs();

    // 清空速度采样滤波器，避免沿用上一次运行留下的历史数据
    filter_ring_init(&encoder_left_ring, encoder_left_buffer, ENCODER_FILTER_WINDOW);
    filter_ring_init(&encoder_right_ring, encoder_right_buffer, ENCODER_FILTER_WINDOW);
    filter_rc_state_reset(&encoder_left_filter);
    filter_rc_state_reset(&encoder_right_filter);

    // 清空摄像头输入方向环RC低通滤波器
    filter_rc_state_reset(&steer_input_filter);

    raw_left = 0;
    raw_right = 0;
    encoder_left = 0;
    encoder_right = 0;
    speed = speed_setting;

    // CPU1 在下一帧图像处理中完成复位；run_while() 收到完成状态后才置运行标志
    Camera_RequestRuntimeReset();
    run_start_pending = 1;
    interrupt_global_enable(interrupt_state);
}

// ------------------ 执行器输出 ------------------

// 占空比duty:0-10000
void run_motor_left_speed(int16 duty)
{
    if (duty >= 0) // 正转
    {
        gpio_set_level(DIR_L2, GPIO_HIGH);                   // DIR输出高电平
        pwm_set_duty(PWM_L2, duty * (PWM_DUTY_MAX / 10000)); // 计算占空比
    }
    else // 反转
    {
        gpio_set_level(DIR_L2, GPIO_LOW);                       // DIR输出低电平
        pwm_set_duty(PWM_L2, (-duty) * (PWM_DUTY_MAX / 10000)); // 计算占空比
    }
}

// 占空比duty:0-10000
void run_motor_right_speed(int16 duty)
{
    if (duty >= 0) // 正转
    {
        gpio_set_level(DIR_R2, GPIO_LOW);                    // DIR输出低电平
        pwm_set_duty(PWM_R2, duty * (PWM_DUTY_MAX / 10000)); // 计算占空比
    }
    else // 反转
    {
        gpio_set_level(DIR_R2, GPIO_HIGH);                      // DIR输出高电平
        pwm_set_duty(PWM_R2, (-duty) * (PWM_DUTY_MAX / 10000)); // 计算占空比
    }
}

// angle：正值右转，负值左转，输入范围为（12~12）度
void run_servo_motor_angle(int16 angle)
{
    if (90 - angle < SERVO_MOTOR_L_MAX)
    {
        angle = 90 - SERVO_MOTOR_L_MAX;
    }
    else if (90 - angle > SERVO_MOTOR_R_MAX)
    {
        angle = 90 - SERVO_MOTOR_R_MAX;
    }
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(90 - angle));
}

// ------------------ 控制决策 ------------------

// 通过视觉获取小车差速值和舵机转向值
double run_get_gap(void)
{
    static double held_gap = 0;
    static uint8 previous_lost_count = 0;

    if (!run_enabled || VisionStopFlag || ImageFlag.Stop)
    {
        held_gap = 0;
        previous_lost_count = 0;
        return 0;
    }

    if (ControlMidValid)
    {
        // 重新找到有效中线后复位转向 PID，防止丢线期间的历史误差突变
        if (previous_lost_count > 0)
        {
            PID_Controller_Reset(&pid_gap);
        }
        // 保留方向 PID 原始输出，后轮差速直接使用该值
        held_gap = PID_Controller_Calc(&pid_gap, 0, (double)ControlMidError);

        previous_lost_count = 0;
        return held_gap;
    }

    if (LineLostCount != previous_lost_count)
    {
        previous_lost_count = LineLostCount;
        if (LineLostFlag)
        {
            // 持续丢线时分步回正，避免舵机从原打角瞬间跳回中位
            if (held_gap > LOST_STEER_RECENTER_STEP)
            {
                held_gap -= LOST_STEER_RECENTER_STEP;
            }
            else if (held_gap < -LOST_STEER_RECENTER_STEP)
            {
                held_gap += LOST_STEER_RECENTER_STEP;
            }
            else
            {
                held_gap = 0;
            }
        }
    }

    return held_gap;
}

// 速度决策函数
void run_speed_decision(void)
{
    // 未发车或视觉要求停车时，目标速度强制为零
    if (!run_enabled || VisionStopFlag || ImageFlag.Stop)
    {
        speed = 0;
    }
    else if (ImageFlag.Zebra_Flag == 1 || LineLostFlag)
    {
        // 斑马线与丢线阶段降速，不在此处直接切断运行状态
        speed = (speed_setting < SPEED_LOW) ? speed_setting : SPEED_LOW;
    }
    else
    {
        speed = speed_setting;
    }
}

// speed：速度值，正值前进，负值后退
// gap: 左右电机速度差值，正值右转，负值左转
void run_direct(int16 speed, int16 gap)
{
    // 读取左右编码器并依次完成滑动平均、RC 低通滤波
    raw_left = CALC_LINE_SPEED_CM_S(-encoder_get_count(ENCODER_1)); // 获取编码器计数
    encoder_left = raw_left;
    filter_ring_write(&encoder_left_ring, encoder_left); // 写入新数据
    encoder_left = filter_moving_average(&encoder_left_ring);
    encoder_left = filter_rc_low_pass(&encoder_left_filter, encoder_left, 0.2);
    encoder_clear_count(ENCODER_1); // 清空编码器计数

    raw_right = CALC_LINE_SPEED_CM_S(encoder_get_count(ENCODER_2)); // 获取编码器计数
    encoder_right = raw_right;
    filter_ring_write(&encoder_right_ring, encoder_right); // 写入新数据
    encoder_right = filter_moving_average(&encoder_right_ring);
    encoder_right = filter_rc_low_pass(&encoder_right_filter, encoder_right, 0.2);
    encoder_clear_count(ENCODER_2); // 清空编码器计数

    int16 left_speed = (int16)PID_Controller_Calc(&pid_left, speed + gap, encoder_left);
    int16 right_speed = (int16)PID_Controller_Calc(&pid_right, speed - gap, encoder_right);

    // 舵机和后轮差速共同转向，提高入弯速度和弯中跟随能力
    run_motor_left_speed(left_speed);
    run_motor_right_speed(right_speed);
}

// ------------------ 周期中断 ------------------

// 5ms 控制中断：本工程唯一执行电机闭环和舵机输出的位置
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    double gap;

    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    // 根据运行状态和视觉标志更新目标速度
    run_speed_decision();

    // 未按键发车时保持无输出；运行中只有识别到斑马线才停车
    if (!run_enabled || ImageFlag.Zebra_Flag)
    {
        if (ImageFlag.Zebra_Flag)
        {
            run_enabled = 0;
        }
        run_stop_outputs();
        return;
    }

    gap = run_get_gap();
    run_direct(speed, (int16)gap);                               // 后轮差速直接使用 PID 原始输出
    run_servo_motor_angle((int16)(gap * STEER_OUTPUT_MAP_SCALE)); // 舵机使用映射后的 -12 ～ +12
}

// 10ms 辅助中断：只扫描按键并产生低频任务标志，不做屏幕绘制和图传
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);

    run_ui_tick_10ms();
#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
    pid_debug_send_flag = 1; // 主循环中发送 PID 调试数据
#endif
}

// 串口 PID 调试函数：主循环轮询接收，不与 user/isr.c 的串口 ISR 重复定义
#if RUN_UART_PID_DEBUG_ENABLE
static void pid_debug_uart_receive(void)
{
    uint8 data;

    while (uart_query_byte(PID_DEBUG_UART, &data))
    {
        ucmd_isr_feed_byte(&pid_debug_cmd, data);
    }
}

// uart_cmd 的统一发送回调
static void pid_debug_uart_reply(const char *text)
{
    uart_write_string(PID_DEBUG_UART, text);
}

// 依次输出左电机、右电机、方向 PID 的 target、actual、output
static void pid_debug_send(void)
{
    (void)snprintf(pid_debug_text, sizeof(pid_debug_text),
                   "s:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                   pid_left.controller.inc.target,
                   pid_left.controller.inc.actual,
                   pid_left.controller.inc.output,
                   pid_right.controller.inc.target,
                   pid_right.controller.inc.actual,
                   pid_right.controller.inc.output,
                   pid_gap.controller.pos.target,
                   pid_gap.controller.pos.actual,
                   pid_gap.controller.pos.output);
    ucmd_send_text(&pid_debug_cmd, pid_debug_text);
}
#endif

// ------------------ WiFi 模块 ------------------

// WiFi 图传和 WiFi PID 调试任一开启时，共用此模块与 TCP 初始化
#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
static void wifi_module_init(void)
{
    // 初始化阶段允许等待连接成功；车辆控制中断此时尚未启动
    while (wifi_spi_init(WIFI_IMAGE_SSID, WIFI_IMAGE_PASSWORD))
    {
        printf("\r\n connect wifi failed.\r\n");
        system_delay_ms(100);
    }

    if (1 != WIFI_SPI_AUTO_CONNECT)
    {
        // 未启用驱动自动连接时，主动连接上位机图像端口
        while (wifi_spi_socket_connect(
                    "TCP",
                    WIFI_IMAGE_TARGET_IP,
                    WIFI_IMAGE_TARGET_PORT,
                    WIFI_IMAGE_LOCAL_PORT))
        {
            printf("\r\n connect image TCP server failed.\r\n");
            system_delay_ms(100);
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

#if RUN_WIFI_IMAGE_ENABLE
    // 图像尺寸使用视觉算法处理后的 LCDW × LCDH，而不是摄像头原始尺寸
    seekfree_assistant_camera_config(
        &wifi_image_information,
        SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
        LCDW,
        LCDH,
        wifi_image_buffer[0]);
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
    // 示波器通道：左电机 target/actual/output，右电机 target/actual/output，方向 target/actual/output
    seekfree_assistant_oscilloscope_config(
        &wifi_pid_oscilloscope,
        WIFI_PID_OSCILLOSCOPE_CHANNEL_NUM,
        wifi_pid_oscilloscope_buffer);
#endif

#if RUN_WIFI_IMAGE_ENABLE
    wifi_image_ready = 1;
#endif
    printf("\r\n wifi ready: version=%s, module=%s, server=%s:%s\r\n",
        wifi_spi_version,
        wifi_spi_ip_addr_port,
        WIFI_IMAGE_TARGET_IP,
        WIFI_IMAGE_TARGET_PORT);
}
#endif

// 将当前车速和左电机 PID 装入逐飞助手参数通道
#if RUN_WIFI_PID_DEBUG_ENABLE
static void wifi_debug_param_init(void)
{
    memset(&wifi_debug_param, 0, sizeof(wifi_debug_param));
    wifi_debug_param.data[WIFI_PARAM_SPEED] = (float)speed_setting;
    wifi_debug_param.data[WIFI_PARAM_PID_GROUP] = (float)wifi_pid_group;
    wifi_debug_param_load_group(wifi_pid_group);
}

// 把选中 PID 组的 Kp/Ki/Kd 同步到参数通道：0=左电机，1=右电机，2=方向
static void wifi_debug_param_load_group(uint8 group)
{
    if (WIFI_PID_GROUP_LEFT == group)
    {
        wifi_debug_param.data[WIFI_PARAM_KP] = (float)pid_left.controller.inc.Kp;
        wifi_debug_param.data[WIFI_PARAM_KI] = (float)pid_left.controller.inc.Ki;
        wifi_debug_param.data[WIFI_PARAM_KD] = (float)pid_left.controller.inc.Kd;
    }
    else if (WIFI_PID_GROUP_RIGHT == group)
    {
        wifi_debug_param.data[WIFI_PARAM_KP] = (float)pid_right.controller.inc.Kp;
        wifi_debug_param.data[WIFI_PARAM_KI] = (float)pid_right.controller.inc.Ki;
        wifi_debug_param.data[WIFI_PARAM_KD] = (float)pid_right.controller.inc.Kd;
    }
    else
    {
        wifi_debug_param.data[WIFI_PARAM_KP] = (float)pid_gap.controller.pos.Kp;
        wifi_debug_param.data[WIFI_PARAM_KI] = (float)pid_gap.controller.pos.Ki;
        wifi_debug_param.data[WIFI_PARAM_KD] = (float)pid_gap.controller.pos.Kd;
    }
}

// 解析上位机参数帧，再原子更新控制中断使用的参数
static void wifi_debug_param_while(void)
{
    uint8 index;

    seekfree_assistant_debug_param_analysis(&wifi_debug_param);

    for (index = 0; index < WIFI_PARAM_COUNT; index++)
    {
        if (wifi_debug_param.update_flag[index])
        {
            float value = wifi_debug_param.data[index];
            uint32 interrupt_state = interrupt_global_disable();

            if (WIFI_PARAM_SPEED == index)
            {
                speed_setting = (int16)value;
                speed = speed_setting;
            }
            else if (WIFI_PARAM_PID_GROUP == index)
            {
                int16 group = (int16)value;
                if (group < WIFI_PID_GROUP_LEFT)
                {
                    group = WIFI_PID_GROUP_LEFT;
                }
                else if (group >= WIFI_PID_GROUP_COUNT)
                {
                    group = WIFI_PID_GROUP_STEER;
                }
                wifi_pid_group = (uint8)group;
                wifi_debug_param.data[WIFI_PARAM_PID_GROUP] = (float)wifi_pid_group;
                // 若同一批数据已携带新增益，保留新值；否则加载该组当前参数
                if (!wifi_debug_param.update_flag[WIFI_PARAM_KP]
                    && !wifi_debug_param.update_flag[WIFI_PARAM_KI]
                    && !wifi_debug_param.update_flag[WIFI_PARAM_KD])
                {
                    wifi_debug_param_load_group(wifi_pid_group);
                }
            }
            else if (WIFI_PID_GROUP_LEFT == wifi_pid_group)
            {
                if (WIFI_PARAM_KP == index) pid_left.controller.inc.Kp = (double)value;
                if (WIFI_PARAM_KI == index) pid_left.controller.inc.Ki = (double)value;
                if (WIFI_PARAM_KD == index) pid_left.controller.inc.Kd = (double)value;
            }
            else if (WIFI_PID_GROUP_RIGHT == wifi_pid_group)
            {
                if (WIFI_PARAM_KP == index) pid_right.controller.inc.Kp = (double)value;
                if (WIFI_PARAM_KI == index) pid_right.controller.inc.Ki = (double)value;
                if (WIFI_PARAM_KD == index) pid_right.controller.inc.Kd = (double)value;
            }
            else
            {
                if (WIFI_PARAM_KP == index) pid_gap.controller.pos.Kp = (double)value;
                if (WIFI_PARAM_KI == index) pid_gap.controller.pos.Ki = (double)value;
                if (WIFI_PARAM_KD == index) pid_gap.controller.pos.Kd = (double)value;
            }

            wifi_debug_param.update_flag[index] = 0;
            interrupt_global_enable(interrupt_state);
        }
    }
}

// 10ms 发送一次三组 PID 的目标、实际值和输出
static void wifi_pid_oscilloscope_send(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    wifi_pid_oscilloscope_buffer[0] = (float)pid_left.controller.inc.target;
    wifi_pid_oscilloscope_buffer[1] = (float)pid_left.controller.inc.actual;
    wifi_pid_oscilloscope_buffer[2] = (float)pid_left.controller.inc.output;
    wifi_pid_oscilloscope_buffer[3] = (float)pid_right.controller.inc.target;
    wifi_pid_oscilloscope_buffer[4] = (float)pid_right.controller.inc.actual;
    wifi_pid_oscilloscope_buffer[5] = (float)pid_right.controller.inc.output;
    wifi_pid_oscilloscope_buffer[6] = (float)pid_gap.controller.pos.target;
    wifi_pid_oscilloscope_buffer[7] = (float)pid_gap.controller.pos.actual;
    wifi_pid_oscilloscope_buffer[8] = (float)pid_gap.controller.pos.output;

    interrupt_global_enable(interrupt_state);
    seekfree_assistant_oscilloscope_send(&wifi_pid_oscilloscope);
}
#endif

// 在 CPU0 主循环中发送一帧稳定图像及其左右边线、控制中线
#if RUN_WIFI_IMAGE_ENABLE
static void wifi_image_while(void)
{
    uint32 sequence_before;
    uint32 sequence_after;
    uint16 left_count = 0;
    uint16 right_count = 0;
    uint16 mid_count = 0;
    uint16 row;
    int point_x;

    if (!wifi_image_ready)
    {
        return;
    }

    sequence_before = VisionFrameSequence;
    // 序号为奇数表示 CPU1 正在更新；相同序号表示本帧已经发送过
    if ((0 == sequence_before) || (sequence_before & 1U) || (sequence_before == wifi_image_last_sequence))
    {
        return;
    }

    memcpy(wifi_image_buffer[0], Pixle[0], sizeof(wifi_image_buffer));

    // 同一稳定帧内复制二值图、左右边线以及最终用于控制的中线
    for (row = 0; row < LCDH; row++)
    {
        point_x = ImageDeal[row].LeftBorder;
        if (ImageDeal[row].IsLeftFind && point_x >= 0 && point_x < LCDW)
        {
            wifi_left_boundary[left_count].x = (uint8)point_x;
            wifi_left_boundary[left_count].y = (uint8)row;
            left_count++;
        }

        point_x = ImageDeal[row].RightBorder;
        if (ImageDeal[row].IsRightFind && point_x >= 0 && point_x < LCDW)
        {
            wifi_right_boundary[right_count].x = (uint8)point_x;
            wifi_right_boundary[right_count].y = (uint8)row;
            right_count++;
        }

        point_x = ControlMidLine[row];
        if (point_x >= 0 && point_x < LCDW)
        {
            wifi_mid_boundary[mid_count].x = (uint8)point_x;
            wifi_mid_boundary[mid_count].y = (uint8)row;
            mid_count++;
        }
    }

    sequence_after = VisionFrameSequence;
    // 复制期间若 CPU1 开始处理下一帧，则丢弃本次快照，避免发送撕裂数据
    if ((sequence_before != sequence_after) || (sequence_after & 1U))
    {
        return;
    }

    wifi_image_last_sequence = sequence_after;

    if (left_count)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_left_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_BLUE,
            left_count,
            wifi_left_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_left_information);
    }

    if (right_count)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_right_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_GREEN,
            right_count,
            wifi_right_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_right_information);
    }

    if (mid_count)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_mid_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_RED,
            mid_count,
            wifi_mid_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_mid_information);
    }

    seekfree_assistant_camera_send(&wifi_image_information);
}
#endif
