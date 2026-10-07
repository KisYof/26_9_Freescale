/**
 ******************************************************************************
 * @file    pid.c
 * @brief   通用PID调节核心模块 - 浮点/定点双模式
 * @note    通过 pid_types.h 中的 PID_USE_FIXED_POINT 切换模式
 ******************************************************************************
 */
#include "pid.h"
#include <stddef.h>

/* ========================= 私有函数声明 ========================= */
static void PID_Pos_Init(PID_Pos_t *pid, double kp, double ki, double kd,
                         double outMax, double outMin, double intMax,
                         double intMin);
static void PID_Pos_Reset(PID_Pos_t *pid);
static double PID_Pos_Calc(PID_Pos_t *pid, double target, double actual);
static void PID_Inc_Init(PID_Inc_t *pid, double kp, double ki, double kd,
                         double outMax, double outMin);
static void PID_Inc_Reset(PID_Inc_t *pid);
static double PID_Inc_Calc(PID_Inc_t *pid, double target, double actual);

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

static double PID_Controller_Inc_Calc(void *controller, double target,
                                      double actual)
{
    return PID_Inc_Calc((PID_Inc_t *)controller, target, actual);
}

/**
 * @brief  初始化统一 PID 控制器
 * @param  controller: 统一 PID 控制器指针
 * @param  algorithm: PID_ALGORITHM_POS 或 PID_ALGORITHM_INC
 * @param  kp/ki/kd: 比例、积分、微分系数
 * @param  outMax/outMin: 输出限幅
 * @param  intMax/intMin: 位置式 PID 积分限幅，增量式 PID 忽略
 * @retval None
 */
void PID_Controller_Init(PID_Controller_t *controller, PID_Algorithm_t algorithm,
                         double kp, double ki, double kd, double outMax,
                         double outMin, double intMax, double intMin)
{
    if (controller == NULL)
        return;

    if (algorithm == PID_ALGORITHM_POS)
    {
        controller->algorithm = algorithm;
        controller->calc = PID_Controller_Pos_Calc;
        PID_Pos_Init(&controller->controller.pos, kp, ki, kd, outMax, outMin,
                     intMax, intMin);
    }
    else if (algorithm == PID_ALGORITHM_INC)
    {
        controller->algorithm = algorithm;
        controller->calc = PID_Controller_Inc_Calc;
        PID_Inc_Init(&controller->controller.inc, kp, ki, kd, outMax, outMin);
    }
    else
    {
        controller->algorithm = PID_ALGORITHM_NONE;
        controller->calc = NULL;
    }
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
 * @brief  初始化 Bang-Bang PID 控制器
 * @param  pid:        Bang-Bang PID 控制器结构体指针
 * @param  deadband:   目标值与实际值之间的控制死区
 * @param  controller: 已通过 PID_Controller_Init 初始化的控制器指针
 * @retval None
 */
void PID_BangBang_Init(PID_BangBang_t *pid, double deadband,
                       PID_Controller_t *controller)
{
    if (pid == NULL || controller == NULL)
        return;

    pid->deadband = (deadband >= 0) ? deadband : -deadband;
    pid->controller = controller;

    PID_BangBang_Reset(pid);
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
        if (pid->controller->algorithm == PID_ALGORITHM_POS)
            output = pid->controller->controller.pos.outputMax;
        else if (pid->controller->algorithm == PID_ALGORITHM_INC)
            output = pid->controller->controller.inc.outputMax;
    }
    else if (error < -pid->deadband)
    {
        if (pid->controller->algorithm == PID_ALGORITHM_POS)
            output = pid->controller->controller.pos.outputMin;
        else if (pid->controller->algorithm == PID_ALGORITHM_INC)
            output = pid->controller->controller.inc.outputMin;
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

/* ========================= 私有函数实现 ========================= */

/**
 * @brief  初始化一个位置式 PID 控制器参数
 * @param  pid: PID结构体指针
 * @param  kp/ki/kd: 比例/积分/微分系数（浮点语义，内部自动转换）
 * @param  outMax/outMin: 输出限幅（浮点语义）
 * @param  intMax/intMin: 积分限幅（浮点语义）
 * @retval None
 */
static void PID_Pos_Init(PID_Pos_t *pid, double kp, double ki, double kd,
                         double outMax, double outMin, double intMax,
                         double intMin)
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
static void PID_Pos_Reset(PID_Pos_t *pid)
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
static double PID_Pos_Calc(PID_Pos_t *pid, double target, double actual)
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
static void PID_Inc_Init(PID_Inc_t *pid, double kp, double ki, double kd,
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
static void PID_Inc_Reset(PID_Inc_t *pid)
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
static double PID_Inc_Calc(PID_Inc_t *pid, double target, double actual)
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

// ------------------ 模糊 PID ------------------
// 输入 E、EC 各分 7 档，先模糊化成相邻两档的隶属度，再查 3 张 7x7 规则表
// 得到 Kp/Ki/Kd 的偏移量档位，基础增益 + 偏移量 = 本周期生效增益，最后走位置式 PID。

// 单个输入量的模糊化结果：落在 [low_index, low_index+1] 两个相邻档位之间，
// 区间内线性插值，所以 low_membership + high_membership 恒为 1
typedef struct
{
    unsigned char low_index;  // 所在区间的下档位索引，0~5
    double low_membership;    // 下档位隶属度，1.0 表示完全落在该档
    double high_membership;   // 上档位隶属度，与下档位互补
} FuzzyPID_Membership_t;

// 通用限幅：超出 [minimum, maximum] 就取边界值
static double fuzzy_pid_clamp(double value, double minimum, double maximum)
{
    if (value > maximum)
        return maximum;
    if (value < minimum)
        return minimum;
    return value;
}

// 三角隶属度模糊化：输入落在 [points[i], points[i+1]] 时按线性比例拆成相邻两档；
// 小于首档或大于末档时归到最边缘的档位，保证任何输入都有可用的隶属度
static FuzzyPID_Membership_t fuzzy_pid_fuzzify(
    double value,
    const double points[FUZZY_PID_LEVEL_COUNT])
{
    FuzzyPID_Membership_t membership;
    unsigned char index;

    // 低于论域下限：完全属于第 0 档
    if (value <= points[0])
    {
        membership.low_index = 0;
        membership.low_membership = 1.0;
        membership.high_membership = 0.0;
        return membership;
    }

    // 高于论域上限：完全属于最后一档
    if (value >= points[FUZZY_PID_LEVEL_COUNT - 1U])
    {
        membership.low_index = FUZZY_PID_LEVEL_COUNT - 2U;
        membership.low_membership = 0.0;
        membership.high_membership = 1.0;
        return membership;
    }

    for (index = 0; index < FUZZY_PID_LEVEL_COUNT - 1U; index++)
    {
        // 找到输入所在的档间区间 [points[index], points[index + 1]]
        if (value <= points[index + 1U])
        {
            double width = points[index + 1U] - points[index];
            double high;

            // 档位点重复或逆序（配置错误）：按完全属于当前档处理
            if (width <= 0.0)
            {
                membership.low_index = index;
                membership.low_membership = 1.0;
                membership.high_membership = 0.0;
                return membership;
            }

            // 归一化到 0~1，值越大表示越靠向上档位
            high = (value - points[index]) / width;
            membership.low_index = index;
            membership.low_membership = 1.0 - high; // 与上档位互补
            membership.high_membership = high;
            return membership;
        }
    }

    // 循环未命中（不应走到这里）：兜底取最后一个区间
    membership.low_index = FUZZY_PID_LEVEL_COUNT - 2U;
    membership.low_membership = 0.0;
    membership.high_membership = 1.0;
    return membership;
}

// 单路增益推理：用 E、EC 各自命中的两个档位交叉出 4 条规则，
// 强度取两输入隶属度的较小值，相同后件合并只留最大强度，
// 再按强度加权平均；档位索引经 offsets 换算成实际增益修正量
static double fuzzy_pid_infer_gain(
    const unsigned char rules[FUZZY_PID_LEVEL_COUNT][FUZZY_PID_LEVEL_COUNT],
    const double offsets[FUZZY_PID_LEVEL_COUNT],
    const FuzzyPID_Membership_t *error_membership,
    const FuzzyPID_Membership_t *delta_membership,
    double base_gain,
    double gain_min,
    double gain_max)
{
    unsigned char consequents[4];
    double strengths[4];
    double numerator = 0.0;
    double denominator = 0.0;
    unsigned char index;
    unsigned char previous;

    // 4 个交点规则：(低,低) (高,低) (低,高) (高,高)，下标与 strengths 一一对应
    consequents[0] = rules[error_membership->low_index]
                           [delta_membership->low_index];
    consequents[1] = rules[error_membership->low_index + 1U]
                           [delta_membership->low_index];
    consequents[2] = rules[error_membership->low_index]
                           [delta_membership->low_index + 1U];
    consequents[3] = rules[error_membership->low_index + 1U]
                           [delta_membership->low_index + 1U];

    // 规则触发强度 = 两个输入的隶属度取小（Mamdani 取小推理）
    strengths[0] = (error_membership->low_membership < delta_membership->low_membership)
                       ? error_membership->low_membership
                       : delta_membership->low_membership;
    strengths[1] = (error_membership->high_membership < delta_membership->low_membership)
                       ? error_membership->high_membership
                       : delta_membership->low_membership;
    strengths[2] = (error_membership->low_membership < delta_membership->high_membership)
                       ? error_membership->low_membership
                       : delta_membership->high_membership;
    strengths[3] = (error_membership->high_membership < delta_membership->high_membership)
                       ? error_membership->high_membership
                       : delta_membership->high_membership;

    // 相同后件只保留最大隶属度，与参考模糊 PD 的聚合方式一致
    for (index = 0; index < 4U; index++)
    {
        // 档位越界时回退到最后一档
        if (consequents[index] >= FUZZY_PID_LEVEL_COUNT)
            consequents[index] = FUZZY_PID_LEVEL_COUNT - 1U;

        for (previous = 0; previous < index; previous++)
        {
            if (consequents[previous] == consequents[index])
            {
                if (strengths[index] > strengths[previous])
                    strengths[previous] = strengths[index];
                strengths[index] = 0.0;
                break;
            }
        }
    }

    // 按触发强度加权：分子 = Σ强度×增益，分母 = Σ强度
    for (index = 0; index < 4U; index++)
    {
        if (strengths[index] > 0.0)
        {
            double gain = fuzzy_pid_clamp(
                base_gain + offsets[consequents[index]], gain_min, gain_max);
            numerator += strengths[index] * gain;
            denominator += strengths[index];
        }
    }

    // 强度全为 0（不应出现）：退回限幅后的基础增益
    if (denominator <= 0.0)
        return fuzzy_pid_clamp(base_gain, gain_min, gain_max);
    return numerator / denominator;
}

// 载入参考模糊表：E/EC 的 7 档论域、Kp/Ki/Kd 的 7 档偏移量和 3 张 7x7 规则表，
// 最后补齐增益、积分与输出的默认限幅；config 为空时直接返回
void FuzzyPID_Config_InitReference(FuzzyPID_Config_t *config)
{
    // 论域采样点必须升序，档距可以不等（边缘档放宽以覆盖大误差）
    static const double error_points[FUZZY_PID_LEVEL_COUNT] =
        {-60.0, -40.0, -20.0, 0.0, 20.0, 40.0, 60.0};
    static const double delta_error_points[FUZZY_PID_LEVEL_COUNT] =
        {-4.0, -2.0, -1.0, 0.0, 1.0, 2.0, 4.0};
    // 各档增益偏移量：下标 0 最负、下标 6 最正，推理结果 = 基础增益 + 该偏移量
    static const double kp_offsets[FUZZY_PID_LEVEL_COUNT] =
        {-0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8};
    static const double ki_offsets[FUZZY_PID_LEVEL_COUNT] =
        {-0.04, -0.02, -0.01, 0.0, 0.01, 0.02, 0.04};
    static const double kd_offsets[FUZZY_PID_LEVEL_COUNT] =
        {-4.0, -2.0, 0.0, 2.0, 5.0, 8.0, 10.0};
    // Kp 规则表：行 = E 档位，列 = EC 档位，值 = 偏移量档位；
    // E 与 EC 同号（误差还在扩大）时给大 Kp，反号（误差被拉回）时给小 Kp
    static const unsigned char kp_rules[FUZZY_PID_LEVEL_COUNT][FUZZY_PID_LEVEL_COUNT] =
    {
        {6, 5, 4, 3, 2, 1, 0},
        {5, 4, 3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3, 4, 5},
        {0, 1, 2, 3, 4, 5, 6}
    };
    // Ki 规则表：E 越正（EC 越正）给越大的积分偏移量，E 为负时压小积分
    static const unsigned char ki_rules[FUZZY_PID_LEVEL_COUNT][FUZZY_PID_LEVEL_COUNT] =
    {
        {0, 0, 1, 1, 2, 3, 3},
        {0, 0, 1, 2, 2, 3, 3},
        {0, 1, 2, 2, 3, 4, 4},
        {1, 1, 2, 3, 4, 5, 5},
        {1, 2, 3, 4, 4, 5, 6},
        {3, 3, 4, 4, 5, 6, 6},
        {3, 3, 4, 5, 5, 6, 6}
    };
    // Kd 规则表：误差被拉回或接近稳态（E、EC 相互抵消）时加大微分阻尼，
    // 误差仍在扩大时给小微分，避免误差快速变化时微分项跟着抖动
    static const unsigned char kd_rules[FUZZY_PID_LEVEL_COUNT][FUZZY_PID_LEVEL_COUNT] =
    {
        {0, 1, 2, 3, 4, 5, 6},
        {1, 2, 3, 4, 5, 6, 5},
        {2, 3, 4, 5, 6, 5, 4},
        {3, 4, 5, 6, 5, 4, 3},
        {4, 5, 6, 5, 4, 3, 2},
        {5, 6, 5, 4, 3, 2, 1},
        {6, 5, 4, 3, 2, 1, 0}
    };
    unsigned char row;
    unsigned char column;

    if (config == NULL)
        return;

    // 论域、偏移量按档位逐项拷贝，规则表按行、列逐格拷贝
    for (row = 0; row < FUZZY_PID_LEVEL_COUNT; row++)
    {
        config->error_points[row] = error_points[row];
        config->delta_error_points[row] = delta_error_points[row];
        config->kp_offsets[row] = kp_offsets[row];
        config->ki_offsets[row] = ki_offsets[row];
        config->kd_offsets[row] = kd_offsets[row];
        for (column = 0; column < FUZZY_PID_LEVEL_COUNT; column++)
        {
            config->kp_rules[row][column] = kp_rules[row][column];
            config->ki_rules[row][column] = ki_rules[row][column];
            config->kd_rules[row][column] = kd_rules[row][column];
        }
    }

    // 默认限幅：三路增益 0~100，积分 ±10000，输出 ±10000
    config->kp_min = 0.0;
    config->kp_max = 100.0;
    config->ki_min = 0.0;
    config->ki_max = 100.0;
    config->kd_min = 0.0;
    config->kd_max = 100.0;
    config->integral_min = -10000.0;
    config->integral_max = 10000.0;
    config->output_min = -10000.0;
    config->output_max = 10000.0;
}

// 初始化：拷贝配置、写入基础增益，最后复位一次让生效增益对齐到基础增益
void FuzzyPID_Init(FuzzyPID_Controller_t *controller,
                   const FuzzyPID_Config_t *config,
                   double base_kp, double base_ki, double base_kd)
{
    if (controller == NULL || config == NULL)
        return;

    controller->config = *config;
    controller->base_kp = base_kp;
    controller->base_ki = base_ki;
    controller->base_kd = base_kd;
    FuzzyPID_Reset(controller);
}

// 复位：三路生效增益退回限幅后的基础增益，清空目标、实际、误差、积分与输出
void FuzzyPID_Reset(FuzzyPID_Controller_t *controller)
{
    if (controller == NULL)
        return;

    controller->active_kp = fuzzy_pid_clamp(
        controller->base_kp, controller->config.kp_min, controller->config.kp_max);
    controller->active_ki = fuzzy_pid_clamp(
        controller->base_ki, controller->config.ki_min, controller->config.ki_max);
    controller->active_kd = fuzzy_pid_clamp(
        controller->base_kd, controller->config.kd_min, controller->config.kd_max);
    controller->target = 0.0;
    controller->actual = 0.0;
    controller->error = 0.0;
    controller->delta_error = 0.0;
    controller->last_error = 0.0;
    controller->error_integral = 0.0;
    controller->output = 0.0;
}

// 在线修改基础增益：只改基础值，要等下一个控制周期推理后才反映到生效增益
void FuzzyPID_SetBaseGains(FuzzyPID_Controller_t *controller,
                           double base_kp, double base_ki, double base_kd)
{
    if (controller == NULL)
        return;

    controller->base_kp = base_kp;
    controller->base_ki = base_ki;
    controller->base_kd = base_kd;
}

// 一个控制周期：求 E、EC -> 各模糊化一次 -> 查 3 张规则表得到三路生效增益
// -> 积分累加限幅 -> 位置式 PID 输出限幅 -> 保存本次误差供下周期差分
double FuzzyPID_Calc(FuzzyPID_Controller_t *controller,
                     double target, double actual)
{
    FuzzyPID_Membership_t error_membership;
    FuzzyPID_Membership_t delta_membership;

    if (controller == NULL)
        return 0.0;

    controller->target = target;
    controller->actual = actual;
    controller->error = target - actual;                  // E = 目标 - 实际
    controller->delta_error = controller->error - controller->last_error; // EC = 本次 E - 上次 E

    // E 与 EC 在一次计算中只模糊化一次，三个增益共享结果
    error_membership = fuzzy_pid_fuzzify(
        controller->error, controller->config.error_points);
    delta_membership = fuzzy_pid_fuzzify(
        controller->delta_error, controller->config.delta_error_points);

    // 同一份隶属度分别查 Kp、Ki、Kd 三张规则表，互不影响
    controller->active_kp = fuzzy_pid_infer_gain(
        controller->config.kp_rules,
        controller->config.kp_offsets,
        &error_membership,
        &delta_membership,
        controller->base_kp,
        controller->config.kp_min,
        controller->config.kp_max);
    controller->active_ki = fuzzy_pid_infer_gain(
        controller->config.ki_rules,
        controller->config.ki_offsets,
        &error_membership,
        &delta_membership,
        controller->base_ki,
        controller->config.ki_min,
        controller->config.ki_max);
    controller->active_kd = fuzzy_pid_infer_gain(
        controller->config.kd_rules,
        controller->config.kd_offsets,
        &error_membership,
        &delta_membership,
        controller->base_kd,
        controller->config.kd_min,
        controller->config.kd_max);

    // 积分带上下限防饱和；Ki 被推理到 0 时清空积分，避免残余积分继续累积
    if (controller->active_ki > 0.0)
    {
        controller->error_integral += controller->error;
        controller->error_integral = fuzzy_pid_clamp(
            controller->error_integral,
            controller->config.integral_min,
            controller->config.integral_max);
    }
    else
    {
        controller->error_integral = 0.0;
    }

    // 位置式 PID：Kp*e + Ki*∫e + Kd*Δe，三项都用本周期推理出的生效增益
    controller->output = controller->active_kp * controller->error
                       + controller->active_ki * controller->error_integral
                       + controller->active_kd * controller->delta_error;
    // 输出限幅，防止超出执行器量程
    controller->output = fuzzy_pid_clamp(
        controller->output,
        controller->config.output_min,
        controller->config.output_max);
    controller->last_error = controller->error; // 保存本次误差，供下周期算 EC

    return controller->output; // 返回本周期控制量
}
