/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用PID调节核心模块头文件
  * @note    double 类型由 pid_types.h 定义，支持浮点 / Q12 定点双模式
  ******************************************************************************
  */

#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif


/* ========================= 类型定义 ========================= */

/**
  * @brief 位置式 PID 控制器结构体
  */
typedef struct {
    double Kp;
    double Ki;
    double Kd;

    double actual;
    double target;
    double output;

    double Error0;
    double Error1;
    double ErrorInt;

    double outputMax;
    double outputMin;
    double errorIntMax;
    double errorIntMin;
} PID_Pos_t;

/**
  * @brief 增量式 PID 控制器结构体
  */
typedef struct {
    double Kp;
    double Ki;
    double Kd;

    double actual;
    double target;
    double output;

    double Error0;
    double Error1;
    double Error2;

    double outputMax;
    double outputMin;
} PID_Inc_t;

/**
  * @brief PID 控制算法类型
  */
typedef enum {
    PID_ALGORITHM_NONE = 0,
    PID_ALGORITHM_POS,
    PID_ALGORITHM_INC
} PID_Algorithm_t;

/**
  * @brief PID 控制算法函数指针类型
  */
typedef double (*PID_CalcFunc_t)(void *controller,
                                 double target, double actual);

/**
  * @brief 统一 PID 控制器
  * @note 由 algorithm 选择 union 中实际生效的控制器
  */
typedef struct {
    PID_Algorithm_t algorithm;
    PID_CalcFunc_t calc;
    union {
        PID_Pos_t pos;
        PID_Inc_t inc;
    } controller;
} PID_Controller_t;

/**
  * @brief Bang-Bang 控制器结构体
  */
typedef struct {
    double outputMax;
    double outputMin;
    double deadband;

    PID_Controller_t *controller;
} PID_BangBang_t;

/* ========================= 公共函数声明 ========================= */

void PID_Pos_Init(PID_Pos_t *pid, double kp, double ki, double kd,
                  double outMax, double outMin, double intMax, double intMin);
void PID_Pos_Reset(PID_Pos_t *pid);
double PID_Pos_Calc(PID_Pos_t *pid, double target, double actual);

void PID_Inc_Init(PID_Inc_t *pid, double kp, double ki, double kd,
                  double outMax, double outMin);
void PID_Inc_Reset(PID_Inc_t *pid);
double PID_Inc_Calc(PID_Inc_t *pid, double target, double actual);


void PID_Controller_InitPos(PID_Controller_t *controller, double kp, double ki,
                            double kd, double outMax, double outMin,
                            double intMax, double intMin);
void PID_Controller_InitInc(PID_Controller_t *controller, double kp, double ki,
                            double kd, double outMax, double outMin);
void PID_Controller_Reset(PID_Controller_t *controller);
double PID_Controller_Calc(PID_Controller_t *controller, double target,
                            double actual);

void PID_BangBang_Init(PID_BangBang_t *pid, double outMax, double outMin,
                       double deadband);
void PID_BangBang_InitWithPID(PID_BangBang_t *pid, double outMax,
                              double outMin, double deadband,
                              PID_Controller_t *controller);
void PID_BangBang_Reset(PID_BangBang_t *pid);
double PID_BangBang_Calc(PID_BangBang_t *pid, double target, double actual);

#ifdef __cplusplus
}
#endif


#endif
