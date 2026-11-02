#include "hardware.h"

// 硬件层只做三件事：驱动外设、采样换算、执行器输出。
// 周期节拍由 run 层的 2ms 速度任务给定，本层不含滤波、PID 和状态判断。

// ------------------ 电机 ------------------
#define DIR_R2 (P21_2)             // 右电机方向脚
#define PWM_R2 (ATOM0_CH1_P21_3)   // 右电机 PWM 输出
#define DIR_L2 (P21_4)             // 左电机方向脚
#define PWM_L2 (ATOM0_CH3_P21_5)   // 左电机 PWM 输出

// ------------------ 编码器 ------------------
#define ENCODER_1 (TIM2_ENCODER)   // 左轮编码器，TIM2 正交解码
#define ENCODER_1_A (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_1_B (TIM2_ENCODER_CH2_P33_6)

#define ENCODER_2 (TIM6_ENCODER)   // 右轮编码器，TIM6 正交解码
#define ENCODER_2_A (TIM6_ENCODER_CH1_P20_3)
#define ENCODER_2_B (TIM6_ENCODER_CH2_P20_0)

// ------------------ 速度换算：脉冲数 -> 转速 -> 线速度 ------------------
#define ENCODER_MULTIPLE 1.0f                                          // 编码器倍频数
#define PULSE_PRE_ROUND 1024.0f                                        // 编码器每转一圈输出的脉冲数
#define WHEEL_TO_ENCODER_RATIO (68.0f / 30.0f)                         // 减速比：电机侧 68 齿 / 轮侧 30 齿
#define PULSE_PER_WHEEL_REV (WHEEL_TO_ENCODER_RATIO * PULSE_PRE_ROUND * ENCODER_MULTIPLE) // 轮子转一圈对应的脉冲数
#define RADIUS_OF_TYRE 3.2f                                            // 轮半径，单位 cm
#define LINE_SPEED_CM_REV (RADIUS_OF_TYRE * 2.0f * 3.1415926f)        // 轮子转一圈的周长，单位 cm
#define SAMPLE_TIME_S 0.002f                                           // 采样周期，必须与 2ms 速度任务一致
#define CALC_WHEEL_RPS(pulse_cnt) ((pulse_cnt) / (PULSE_PER_WHEEL_REV * SAMPLE_TIME_S))  // 脉冲数 -> 转速 r/s
#define CALC_LINE_SPEED_CM_S(pulse_cnt) (CALC_WHEEL_RPS(pulse_cnt) * LINE_SPEED_CM_REV)  // 脉冲数 -> 线速度 cm/s

// ------------------ 舵机 ------------------
#define SERVO_MOTOR_PWM (ATOM1_CH1_P33_9)  // 舵机 PWM 引脚
#define SERVO_MOTOR_FREQ (50)              // 舵机控制频率，单位 Hz
#define SERVO_MOTOR_L_MAX (77)             // 左极限：90-angle 的下限
#define SERVO_MOTOR_R_MAX (103)            // 右极限：90+angle 的上限
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0)) // 舵机角 -> 占空比：0 度=0.5ms，90 度=1.5ms，180 度=2.5ms

// 编译期校验：舵机频率超出 50~300Hz 直接报错
#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif

static void hardware_set_left_motor(int16 duty);
static void hardware_set_right_motor(int16 duty);

// 初始化电机方向脚与 17kHz PWM、两路正交编码器、50Hz 舵机，并清空编码器计数。
// 必须在控制中断启动之前调用一次，之后不再重复初始化。
void hardware_init(void)
{
    // 上电默认不转：方向脚置高、占空比给 0
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_R2, 17000, 0);
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_L2, 17000, 0);

    // 正交解码，A/B 相相位差决定正负方向，计数器由 update 读取后清零
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);

    // 舵机先给 0 占空比，等控制层第一次写入角度
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);
    hardware_reset_feedback();
}

// 固定由 2ms 速度任务调用：采样并返回本周期反馈，随后清空编码器计数器。
// 左轮编码器安装方向与右轮相反，取负号后统一为“正数=前进”。
void hardware_sample_2ms(hardware_feedback_t *feedback)
{
    feedback->raw_left_speed = CALC_LINE_SPEED_CM_S(-encoder_get_count(ENCODER_1));
    encoder_clear_count(ENCODER_1);

    feedback->raw_right_speed = CALC_LINE_SPEED_CM_S(-encoder_get_count(ENCODER_2));
    encoder_clear_count(ENCODER_2);
}

// 清零编码器脉冲；软件反馈和滤波状态由 run 统一复位，避免两层各留一份缓存。
void hardware_reset_feedback(void)
{
    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
}

// 左右电机独立给定：正数前进、负数后退，duty 范围 0~10000 对应 0~100% 占空比。
// 本函数不做限幅，输出值由控制层的 PID 限幅结果决定。
void hardware_set_motors(int16 left_duty, int16 right_duty)
{
    hardware_set_left_motor(left_duty);
    hardware_set_right_motor(right_duty);
}

// 左电机：正转方向脚为高，反转为低；PWM_DUTY_MAX = 10000，duty 即为千分比。
static void hardware_set_left_motor(int16 duty)
{
    if (duty >= 0)
    {
        gpio_set_level(DIR_L2, GPIO_HIGH);
        pwm_set_duty(PWM_L2, duty * (PWM_DUTY_MAX / 10000));
    }
    else
    {
        // 反转：方向脚取反，占空比取绝对值
        gpio_set_level(DIR_L2, GPIO_LOW);
        pwm_set_duty(PWM_L2, (-duty) * (PWM_DUTY_MAX / 10000));
    }
}

// 右电机与左电机镜像安装，前进方向的电平相反：正转方向脚为低。
static void hardware_set_right_motor(int16 duty)
{
    if (duty >= 0)
    {
        gpio_set_level(DIR_R2, GPIO_LOW);
        pwm_set_duty(PWM_R2, duty * (PWM_DUTY_MAX / 10000));
    }
    else
    {
        // 反转：方向脚取反，占空比取绝对值
        gpio_set_level(DIR_R2, GPIO_HIGH);
        pwm_set_duty(PWM_R2, (-duty) * (PWM_DUTY_MAX / 10000));
    }
}

// angle 是相对中位（0）的转角：先限幅，再换算成 90±12 度的舵机角输出占空比。
void hardware_set_servo(int16 angle)
{
    // 限幅：保证 90-angle 落在 [SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX] 内，即输入角约 ±12 度
    if (90 - angle < SERVO_MOTOR_L_MAX)
    {
        angle = 90 - SERVO_MOTOR_L_MAX;
    }
    else if (90 - angle > SERVO_MOTOR_R_MAX)
    {
        angle = 90 - SERVO_MOTOR_R_MAX;
    }
    // 90 度中位对应 1.5ms 脉宽
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(90 - angle));
}

// 停车输出：两轮占空比归零、舵机回中位，故障和状态切换时统一调用。
void hardware_stop(void)
{
    hardware_set_motors(0, 0);
    hardware_set_servo(0);
}

// 厂家等价原始接口：设备协议与 Mode3 首帧处理全部留在库内。
void hardware_imu_init(void)
{
    (void)icm20602_init_raw(); // 初始化可失败；后续逐次采样报告真实通信状态，不永久锁存失败
}

// 映射库数据到硬件反馈，运行层不依赖设备库的类型或全局变量。
void hardware_sample_imu(hardware_imu_feedback_t *feedback)
{
    icm20602_raw_data_struct raw;
    uint8 sample_failed;

    memset(feedback, 0, sizeof(*feedback));
    sample_failed = icm20602_get_raw(&raw);
    feedback->device_id = raw.device_id;
    feedback->identity_verified = raw.identity_verified;
    feedback->power_management = raw.power_management;
    if (sample_failed)
    {
        return;
    }
    feedback->acc_x = raw.acc_x;
    feedback->acc_y = raw.acc_y;
    feedback->acc_z = raw.acc_z;
    feedback->gyro_x = raw.gyro_x;
    feedback->gyro_y = raw.gyro_y;
    feedback->gyro_z = raw.gyro_z;
    feedback->available = 1;
}
