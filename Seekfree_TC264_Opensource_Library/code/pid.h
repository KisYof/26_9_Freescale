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
    double deadband;
    PID_Controller_t *controller;
} PID_BangBang_t;

// ------------------ 模糊 PID ------------------
// E 与 EC 各分 7 档做三角隶属度模糊化，查 3 张 7x7 规则表得到 Kp/Ki/Kd 的
// 偏移量档位，基础增益 + 偏移量 = 本周期生效增益，再按位置式 PID 计算输出。
#define FUZZY_PID_LEVEL_COUNT (7U) // 模糊子集档数：论域、偏移量与规则表共用

/**
  * @brief 完整模糊 PID 配置
  * @note  E 和 EC 只模糊化一次，三张规则表分别生成 Kp/Ki/Kd 在线修正量
  */
typedef struct {
    double error_points[FUZZY_PID_LEVEL_COUNT];          // 误差 E 的 7 档论域采样点，必须升序
    double delta_error_points[FUZZY_PID_LEVEL_COUNT];    // 误差变化率 EC 的 7 档论域采样点，必须升序
    double kp_offsets[FUZZY_PID_LEVEL_COUNT];            // Kp 各档偏移量，与基础 Kp 相加
    double ki_offsets[FUZZY_PID_LEVEL_COUNT];            // Ki 各档偏移量，与基础 Ki 相加
    double kd_offsets[FUZZY_PID_LEVEL_COUNT];            // Kd 各档偏移量，与基础 Kd 相加
    unsigned char kp_rules[FUZZY_PID_LEVEL_COUNT][FUZZY_PID_LEVEL_COUNT]; // Kp 规则表：行=E 档，列=EC 档
    unsigned char ki_rules[FUZZY_PID_LEVEL_COUNT][FUZZY_PID_LEVEL_COUNT]; // Ki 规则表：行=E 档，列=EC 档
    unsigned char kd_rules[FUZZY_PID_LEVEL_COUNT][FUZZY_PID_LEVEL_COUNT]; // Kd 规则表：行=E 档，列=EC 档
    double kp_min;                                       // 推理出的 Kp 下限
    double kp_max;                                       // 推理出的 Kp 上限
    double ki_min;                                       // 推理出的 Ki 下限
    double ki_max;                                       // 推理出的 Ki 上限
    double kd_min;                                       // 推理出的 Kd 下限
    double kd_max;                                       // 推理出的 Kd 上限
    double delta_error_limit;                            // >0: clamp EC for inference and D; 0: unlimited
    double integral_min;                                 // 积分累加下限，防积分饱和
    double integral_max;                                 // 积分累加上限
    double output_min;                                   // 输出下限
    double output_max;                                   // 输出上限
} FuzzyPID_Config_t;

/**
  * @brief 完整模糊在线整定 PID 控制器
  */
typedef struct {
    FuzzyPID_Config_t config; // 配置副本，Init 时整体拷入，改配置需重新 Init
    double base_kp;           // 基础 Kp，推理结果 = base_kp + 档位偏移量
    double base_ki;           // 基础 Ki
    double base_kd;           // 基础 Kd
    double active_kp;         // 本周期推理得到的生效 Kp，直接参与运算
    double active_ki;         // 本周期推理得到的生效 Ki
    double active_kd;         // 本周期推理得到的生效 Kd
    double target;            // 目标值
    double actual;            // 实际值
    double error;             // 本次误差 E = target - actual
    double delta_error;       /* EC may be limited by config.delta_error_limit. */ // 误差变化率 EC = error - last_error
    double last_error;        // 上次误差，供下周期计算 EC
    double error_integral;    // 积分累加值，已按 integral_min/max 限幅
    double output;            // 本周期输出，已按 output_min/max 限幅
} FuzzyPID_Controller_t;

/* ========================= 公共函数声明 ========================= */
void PID_Controller_Init(PID_Controller_t *controller, PID_Algorithm_t algorithm,
                         double kp, double ki, double kd, double outMax,
                         double outMin, double intMax, double intMin);
void PID_Controller_Reset(PID_Controller_t *controller);
double PID_Controller_Calc(PID_Controller_t *controller, double target,
                            double actual);

void PID_BangBang_Init(PID_BangBang_t *pid, double deadband,
                       PID_Controller_t *controller);
void PID_BangBang_Reset(PID_BangBang_t *pid);
double PID_BangBang_Calc(PID_BangBang_t *pid, double target, double actual);

// ------------------ 模糊 PID 接口 ------------------
void FuzzyPID_Config_InitReference(FuzzyPID_Config_t *config); // 载入参考论域、偏移量、3 张规则表与默认限幅
void FuzzyPID_Init(FuzzyPID_Controller_t *controller,          // 拷贝配置、写入基础增益并复位
                   const FuzzyPID_Config_t *config,
                   double base_kp, double base_ki, double base_kd);
void FuzzyPID_Reset(FuzzyPID_Controller_t *controller);        // 生效增益退回基础增益，清空误差、积分与输出
void FuzzyPID_SetBaseGains(FuzzyPID_Controller_t *controller,  // 在线修改基础增益，下个周期推理后生效
                           double base_kp, double base_ki, double base_kd);
double FuzzyPID_Calc(FuzzyPID_Controller_t *controller,        // 一次模糊 PID 计算，返回本周期输出
                     double target, double actual);

#ifdef __cplusplus
}
#endif


#endif
