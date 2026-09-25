#include "run.h"
#include "pid.h"
#include "filter.h"
#include "camera.h"

// 电机相关
#define DIR_R2 (P21_2) // 实际使用的电机组
#define PWM_R2 (ATOM0_CH1_P21_3)
#define DIR_L2 (P21_4)
#define PWM_L2 (ATOM0_CH3_P21_5)

// 编码器相关
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

// 舵机相关
#define SERVO_MOTOR_PWM (ATOM1_CH1_P33_9) // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)             // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX (70) // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX (110)

// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif

// 速度决策宏定义
#define SPEED_LOW (20)  // 低速
#define SPEED_MID (40)  // 中速
#define SPEED_HIGH (60) // 高速

// 全局变量
int16 raw_left = 0;
int16 raw_right = 0;
int16 encoder_left = 0;
int16 encoder_right = 0;
int16 speed = 40;
uint8 uart_flag = 0;

PID_Controller_t pid_left;
PID_Controller_t pid_right;
PID_Controller_t pid_gap; // PD差速位置环

// 编码器滤波
// 滑动平均滤波
#define ENCODER_FILTER_WINDOW 32 // 滑动窗口大小，最大64
static filter_ring_t encoder_left_ring;
static filter_ring_t encoder_right_ring;
static float encoder_left_buffer[ENCODER_FILTER_WINDOW];
static float encoder_right_buffer[ENCODER_FILTER_WINDOW];

// rc低通滤波器
static filter_rc_state_t encoder_left_filter;
static filter_rc_state_t encoder_right_filter;

void run_init(void)
{
    pit_ms_init(CCU60_CH1, 10); // 10ms产生一次中断

    // 电机驱动初始化
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R2, 17000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出
    pwm_init(PWM_L2, 17000, 0);

    // 编码器初始化
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式
    pit_ms_init(CCU60_CH0, 5);                             // 5ms产生一次中断

    // 舵机初始化
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // 电机PID初始化
    PID_Controller_Init(&pid_left, PID_ALGORITHM_INC, 1, 0.1, 0.0, 1500, -1500, 0, 0);
    PID_Controller_Init(&pid_right, PID_ALGORITHM_INC, 1, 0.1, 0.0, 1500, -1500, 0, 0);

    // 转向位置环
    PID_Controller_Init(&pid_gap, PID_ALGORITHM_POS, 0.3, 0.0, 0.0, 50, -50, 0, 0);

    // 编码器平均滤波初始化
    filter_ring_init(&encoder_left_ring, encoder_left_buffer, ENCODER_FILTER_WINDOW);
    filter_ring_init(&encoder_right_ring, encoder_right_buffer, ENCODER_FILTER_WINDOW);

    // 编码器低通滤波滤波初始化
    filter_rc_state_reset(&encoder_left_filter);
    filter_rc_state_reset(&encoder_right_filter);
}

void run_while(void)
{
    if (uart_flag == 1)
    {
        uart_flag = 0;
        printf("Speed: %d,%d,%d,%d\r\n", raw_left, raw_right, encoder_left, encoder_right);
    }
}

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

// angle：正值右转，负值左转，输入范围为（-20~20）度
void run_servo_motor_angle(int8 angle)
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

// 通过视觉获取小车差速值和舵机转向值
double run_get_gap(void)
{
    double gap = 0;

    // 这里读取视觉数据，并做出滤波，PD调节输出后轮差速值
    if (NormalMidValid && ImageFlag.Zebra_Flag != 1) // 未识别到斑马线时
    {
        gap = PID_Controller_Calc(&pid_gap, 0, (double)NormalMidError);
    }

    return gap;
}

// 速度决策函数
void run_speed_decision(void)
{
    if (ImageFlag.Zebra_Flag == 1)
    {
        speed = 0;
    }
    else
    {
        // if (ImageFlag.flag == 1) // 直道加速
        // {
        //     speed = SPEED_HIGH;
        // }
        speed = 40; // 默认中速
    }
}

// speed：速度值，正值前进，负值后退
// gap: 左右电机速度差值，正值右转，负值左转
void run_direct(int16 speed, int16 gap)
{
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

    int16 left_speed = (int16)PID_Controller_Calc(&pid_left, speed, encoder_left);
    int16 right_speed = (int16)PID_Controller_Calc(&pid_right, speed, encoder_right);

    run_motor_left_speed(left_speed);
    run_motor_right_speed(right_speed);
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    // 速度决策函数
    run_speed_decision();

    run_direct(speed, (int16)run_get_gap()); // 5ms周期执行一次 run_direct() 函数
    run_servo_motor_angle((int8)run_get_gap());
}
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);

    uart_flag = 1; // 设置标志位，表示可以打印数据
}

