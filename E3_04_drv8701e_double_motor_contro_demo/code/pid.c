/**
 ******************************************************************************
 * @file    pid.c
 * @brief   通用PID调节核心模块 - 浮点/定点双模式
 * @note    通过 pid_types.h 中的 PID_USE_FIXED_POINT 切换模式
 ******************************************************************************
 */
#include "pid.h"
#include <stddef.h>

/* ========================= 公共函数实现 ========================= */

/**
 * @brief  初始化一个位置式 PID 控制器参数
 * @param  pid: PID结构体指针
 * @param  kp/ki/kd: 比例/积分/微分系数（浮点语义，内部自动转换）
 * @param  outMax/outMin: 输出限幅（浮点语义）
 * @param  intMax/intMin: 积分限幅（浮点语义）
 * @retval None
 */
void PID_Pos_Init(PID_Pos_t *pid, double kp, double ki, double kd,
                  double outMax, double outMin, double intMax, double intMin)
{
    if (pid == NULL)
        return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->outputMax = outMax;
    pid->outputMin = outMin;
    pid->errorIntMax = intMax;
    pid->errorIntMin = intMin;

    PID_Pos_Reset(pid);
}

/**
 * @brief  重置位置式 PID 运行状态
 * @param  pid: PID结构体指针
 * @retval None
 */
void PID_Pos_Reset(PID_Pos_t *pid)
{
    if (pid == NULL)
        return;

    pid->actual = 0;
    pid->target = 0;
    pid->output = 0;
    pid->Error0 = 0;
    pid->Error1 = 0;
    pid->ErrorInt = 0;
}

/**
 * @brief  位置式 PID 核心计算
 * @param  pid:     PID结构体指针
 * @param  target:  目标值（double 类型）
 * @param  actual:  实际值（double 类型）
 * @retval 调节输出值（double 类型）
 */
double PID_Pos_Calc(PID_Pos_t *pid, double target, double actual)
{
    if (pid == NULL)
        return 0;

    pid->target = target;
    pid->actual = actual;
    pid->Error1 = pid->Error0;
    pid->Error0 = target - actual;

    if (pid->Ki != 0)
    {
        pid->ErrorInt += pid->Error0;
        if (pid->ErrorInt > pid->errorIntMax)
        {
            pid->ErrorInt = pid->errorIntMax;
        }
        else if (pid->ErrorInt < pid->errorIntMin)
        {
            pid->ErrorInt = pid->errorIntMin;
        }
    }
    else
    {
        pid->ErrorInt = 0;
    }

    pid->output = pid->Kp * pid->Error0 + pid->Ki * pid->ErrorInt + pid->Kd * (pid->Error0 - pid->Error1);

    if (pid->output > pid->outputMax)
    {
        pid->output = pid->outputMax;
    }
    else if (pid->output < pid->outputMin)
    {
        pid->output = pid->outputMin;
    }

    return pid->output;
}

/* ========================= 增量式 PID ========================= */

/**
 * @brief  初始化一个增量式 PID 控制器参数
 * @param  pid: PID结构体指针
 * @param  kp/ki/kd: 比例/积分/微分系数）
 * @param  outMax/outMin: 输出限幅
 * @retval None
 */
void PID_Inc_Init(PID_Inc_t *pid, double kp, double ki, double kd,
                  double outMax, double outMin)
{
    if (pid == NULL)
        return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->outputMax = outMax;
    pid->outputMin = outMin;

    PID_Inc_Reset(pid);
}

/**
 * @brief  重置增量式 PID 运行状态
 * @param  pid: PID结构体指针
 * @retval None
 */
void PID_Inc_Reset(PID_Inc_t *pid)
{
    if (pid == NULL)
        return;

    pid->actual = 0;
    pid->target = 0;
    pid->output = 0;
    pid->Error0 = 0;
    pid->Error1 = 0;
    pid->Error2 = 0;
}

/**
 * @brief  增量式 PID 核心计算
 * @param  pid:     PID结构体指针
 * @param  target:  目标值
 * @param  actual:  实际值
 * @retval 调节输出值
 */
double PID_Inc_Calc(PID_Inc_t *pid, double target, double actual)
{
    if (pid == NULL)
        return 0;

    pid->target = target;
    pid->actual = actual;
    pid->Error2 = pid->Error1;
    pid->Error1 = pid->Error0;
    pid->Error0 = target - actual;

    pid->output += pid->Kp * (pid->Error0 - pid->Error1) + pid->Ki * pid->Error0 + pid->Kd * (pid->Error0 - 2 * pid->Error1 + pid->Error2);

    if (pid->output > pid->outputMax)
    {
        pid->output = pid->outputMax;
    }
    else if (pid->output < pid->outputMin)
    {
        pid->output = pid->outputMin;
    }

    return pid->output;
}

/* ========================= 统一 PID 控制器 ========================= */

/**
 * @brief  统一控制器的位置式 PID 适配函数
 * @param  controller: PID_Pos_t 控制器指针
 * @param  target:     目标值
 * @param  actual:     实际值
 * @retval 调节输出值
 */
static double PID_Controller_Pos_Calc(void *controller, double target,
                                      double actual)
{
    return PID_Pos_Calc((PID_Pos_t *)controller, target, actual);
}

/**
 * @brief  统一控制器的增量式 PID 适配函数
 * @param  controller: PID_Inc_t 控制器指针
 * @param  target:     目标值
 * @param  actual:     实际值
 * @retval 调节输出值
 */
static double PID_Controller_Inc_Calc(void *controller, double target,
                                      double actual)
{
    return PID_Inc_Calc((PID_Inc_t *)controller, target, actual);
}

/**
 * @brief  初始化统一位置式 PID 控制器
 * @param  controller: 统一 PID 控制器指针
 * @param  kp/ki/kd:    比例、积分、微分系数
 * @param  outMax/outMin: 输出限幅
 * @param  intMax/intMin: 积分限幅
 * @retval None
 */
void PID_Controller_InitPos(PID_Controller_t *controller, double kp, double ki,
                            double kd, double outMax, double outMin,
                            double intMax, double intMin)
{
    if (controller == NULL)
        return;

    controller->algorithm = PID_ALGORITHM_POS;
    controller->calc = PID_Controller_Pos_Calc;
    PID_Pos_Init(&controller->controller.pos, kp, ki, kd, outMax, outMin,
                 intMax, intMin);
}

/**
 * @brief  初始化统一增量式 PID 控制器
 * @param  controller: 统一 PID 控制器指针
 * @param  kp/ki/kd:    比例、积分、微分系数
 * @param  outMax/outMin: 输出限幅
 * @retval None
 */
void PID_Controller_InitInc(PID_Controller_t *controller, double kp, double ki,
                            double kd, double outMax, double outMin)
{
    if (controller == NULL)
        return;

    controller->algorithm = PID_ALGORITHM_INC;
    controller->calc = PID_Controller_Inc_Calc;
    PID_Inc_Init(&controller->controller.inc, kp, ki, kd, outMax, outMin);
}

/**
 * @brief  重置统一 PID 控制器运行状态
 * @param  controller: 统一 PID 控制器指针
 * @retval None
 */
void PID_Controller_Reset(PID_Controller_t *controller)
{
    if (controller == NULL)
        return;

    if (controller->algorithm == PID_ALGORITHM_POS)
    {
        PID_Pos_Reset(&controller->controller.pos);
    }
    else if (controller->algorithm == PID_ALGORITHM_INC)
    {
        PID_Inc_Reset(&controller->controller.inc);
    }
}

/**
 * @brief  统一 PID 控制器核心计算
 * @param  controller: 统一 PID 控制器指针
 * @param  target:     目标值
 * @param  actual:     实际值
 * @retval 调节输出值
 */
double PID_Controller_Calc(PID_Controller_t *controller, double target,
                           double actual)
{
    if (controller == NULL || controller->calc == NULL)
        return 0;

    if (controller->algorithm == PID_ALGORITHM_POS)
    {
        return controller->calc(&controller->controller.pos, target, actual);
    }
    else if (controller->algorithm == PID_ALGORITHM_INC)
    {
        return controller->calc(&controller->controller.inc, target, actual);
    }

    return 0;
}

/* ========================= Bang-Bang 控制 ========================= */

/**
 * @brief  初始化一个 Bang-Bang 控制器参数
 * @param  pid:       控制器结构体指针
 * @param  outMax:    正向最大输出
 * @param  outMin:    反向最小输出（单向控制时可设为 0）
 * @param  deadband:  目标值与实际值之间的控制死区
 * @retval None
 */
void PID_BangBang_Init(PID_BangBang_t *pid, double outMax, double outMin,
                       double deadband)
{
    if (pid == NULL)
        return;

    pid->outputMax = outMax;
    pid->outputMin = outMin;
    pid->deadband = (deadband >= 0) ? deadband : -deadband;
    pid->controller = NULL;

    PID_BangBang_Reset(pid);
}

/**
 * @brief  初始化带 PID 死区控制的 Bang-Bang 控制器
 * @param  pid:        Bang-Bang 控制器结构体指针
 * @param  outMax:     正向最大输出
 * @param  outMin:     反向最小输出
 * @param  deadband:   进入 PID 控制的误差死区
 * @param  controller: 已通过 PID_Controller_InitPos 或 PID_Controller_InitInc 初始化的控制器指针
 * @retval None
 */
void PID_BangBang_InitWithPID(PID_BangBang_t *pid, double outMax,
                              double outMin, double deadband,
                              PID_Controller_t *controller)
{
    if (pid == NULL)
        return;

    PID_BangBang_Init(pid, outMax, outMin, deadband);
    pid->controller = controller;
}

/**
 * @brief  重置 Bang-Bang 控制器运行状态
 * @param  pid: 控制器结构体指针
 * @retval None
 */
void PID_BangBang_Reset(PID_BangBang_t *pid)
{
    if (pid == NULL)
        return;

    PID_Controller_Reset(pid->controller);
}

/**
 * @brief  Bang-Bang 控制核心计算
 * @param  pid:    控制器结构体指针
 * @param  target: 目标值
 * @param  actual: 实际值
 * @retval 调节输出值
 */
double PID_BangBang_Calc(PID_BangBang_t *pid, double target, double actual)
{
    double output = 0;
    double error = target - actual;

    if (pid == NULL)
        return 0;

    if (error > pid->deadband)
    {
        output = pid->outputMax;
    }
    else if (error < -pid->deadband)
    {
        output = pid->outputMin;
    }
    else
    {
        output = PID_Controller_Calc(pid->controller, target, actual);
    }

    if (pid->controller != NULL)
    {
        if (pid->controller->algorithm == PID_ALGORITHM_POS)
        {
            pid->controller->controller.pos.output = output;
        }
        else if (pid->controller->algorithm == PID_ALGORITHM_INC)
        {
            pid->controller->controller.inc.output = output;
        }
    }

    return output;
}
