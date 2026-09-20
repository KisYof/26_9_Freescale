/**
 ******************************************************************************
 * @file    filter.c
 * @brief   基于逐飞 FIFO 的环形缓冲与数字滤波模块
 ******************************************************************************
 */
#include "filter.h"

/* ========================= 内部函数实现 ========================= */

/**
 * @brief  将 float 数据转换为 FIFO 使用的 uint32 数据
 * @param  value: float 数据
 * @retval 对应的 uint32 位模式
 */
static uint32 filter_float_to_uint32(float value)
{
    uint32 data;
    memcpy(&data, &value, sizeof(data));
    return data;
}

/**
 * @brief  将 FIFO 中的 uint32 数据转换为 float 数据
 * @param  data: uint32 位模式
 * @retval 对应的 float 数据
 */
static float filter_uint32_to_float(uint32 data)
{
    float value;
    memcpy(&value, &data, sizeof(value));
    return value;
}

/**
 * @brief  快速排序
 * @param  array: 待排序数组
 * @param  left: 排序区间左边界
 * @param  right: 排序区间右边界
 * @retval None
 */
static void filter_quick_sort(float *array, int32 left, int32 right)
{
    int32 i = left;
    int32 j = right;
    float pivot = array[(left + right) / 2];

    while (i <= j)
    {
        while (array[i] < pivot) i++;
        while (array[j] > pivot) j--;
        if (i <= j)
        {
            float temporary = array[i];
            array[i] = array[j];
            array[j] = temporary;
            i++;
            j--;
        }
    }

    if (left < j) filter_quick_sort(array, left, j);
    if (i < right) filter_quick_sort(array, i, right);
}

/**
 * @brief  限制 RC 滤波系数范围
 * @param  alpha: 原始滤波系数
 * @retval 限制在 0 到 1 范围内的滤波系数
 */
static float filter_clamp_alpha(float alpha)
{
    if (alpha < 0.0f) return 0.0f;
    if (alpha > 1.0f) return 1.0f;
    return alpha;
}

/* ========================= 环形缓冲区实现 ========================= */

/**
 * @brief  初始化环形滤波数据缓冲区
 * @param  ring: 环形缓冲区结构体指针
 * @param  buffer: 用户提供的 float 数组
 * @param  capacity: 环形缓冲区容量，最大为 FILTER_MAX_WINDOW_SIZE
 * @retval 1 初始化成功，0 初始化失败
 */
int8 filter_ring_init(filter_ring_t *ring, float *buffer, uint32 capacity)
{
    if ((ring == NULL) || (buffer == NULL) ||
        (capacity == 0U) || (capacity > FILTER_MAX_WINDOW_SIZE)) return 0;

    ring->buffer = buffer;
    ring->capacity = capacity;

    if (fifo_init(&ring->fifo, FIFO_DATA_32BIT, buffer, capacity) != FIFO_SUCCESS)
    {
        ring->buffer = NULL;
        ring->capacity = 0U;
        return 0;
    }

    ring->initialized = 1U;
    return 1;
}

/**
 * @brief  向环形缓冲区写入一个数据
 * @param  ring: 环形缓冲区结构体指针
 * @param  value: 待写入的数据
 * @retval 1 写入成功，0 写入失败
 * @note   缓冲区已满时自动覆盖最早写入的数据
 * @note   非中断安全，并发写入同一 ring 需自行加锁
 */
int8 filter_ring_write(filter_ring_t *ring, float value)
{
    uint32 discard;

    if ((ring == NULL) || (!ring->initialized) ||
        (ring->buffer == NULL) || (ring->capacity == 0U)) return 0;

    if (fifo_used(&ring->fifo) >= ring->capacity)
    {
        if (fifo_read_element(&ring->fifo, &discard, FIFO_READ_AND_CLEAN) != FIFO_SUCCESS)
        {
            return 0;
        }
    }

    return (fifo_write_element(&ring->fifo, filter_float_to_uint32(value)) == FIFO_SUCCESS);
}

/**
 * @brief  获取环形缓冲区当前数据数量
 * @param  ring: 环形缓冲区结构体指针
 * @retval 当前数据数量
 */
uint32 filter_ring_size(filter_ring_t *ring)
{
    if ((ring == NULL) || (!ring->initialized)) return 0U;
    return fifo_used(&ring->fifo);
}

/**
 * @brief  按时间先后顺序复制环形缓冲区数据
 * @param  ring: 环形缓冲区结构体指针
 * @param  output: 数据输出数组
 * @param  output_size: 输出数组容量
 * @retval 1 复制成功，0 复制失败
 */
int8 filter_ring_copy(filter_ring_t *ring, float *output, uint32 output_size)
{
    uint32 raw_values[FILTER_MAX_WINDOW_SIZE];
    uint32 length;
    uint32 index;

    if ((ring == NULL) || (!ring->initialized) || (output == NULL)) return 0;

    length = filter_ring_size(ring);
    if ((length == 0U) || (length > FILTER_MAX_WINDOW_SIZE) || (output_size < length)) return 0;

    if (fifo_read_buffer(&ring->fifo, raw_values, &length, FIFO_READ_ONLY) != FIFO_SUCCESS)
    {
        return 0;
    }

    if ((length == 0U) || (length > FILTER_MAX_WINDOW_SIZE) || (output_size < length)) return 0;

    for (index = 0U; index < length; index++)
    {
        output[index] = filter_uint32_to_float(raw_values[index]);
    }
    return 1;
}

/* ========================= 常用滤波算法实现 ========================= */

/**
 * @brief  计算环形缓冲区数据的中值
 * @param  ring: 环形缓冲区结构体指针
 * @retval 中值，缓冲区为空或异常时返回 0
 */
float filter_median(filter_ring_t *ring)
{
    float values[FILTER_MAX_WINDOW_SIZE];
    uint32 count;

    if ((ring == NULL) || (!ring->initialized)) return 0.0f;

    count = filter_ring_size(ring);
    if ((count == 0U) || (count > FILTER_MAX_WINDOW_SIZE) ||
        !filter_ring_copy(ring, values, count)) return 0.0f;

    filter_quick_sort(values, 0, (int32)count - 1);
    if ((count & 1U) != 0U) return values[count / 2U];
    return (values[count / 2U - 1U] + values[count / 2U]) * 0.5f;
}

/**
 * @brief  计算环形缓冲区数据的滑动均值
 * @param  ring: 环形缓冲区结构体指针
 * @retval 均值，缓冲区为空或异常时返回 0
 */
float filter_moving_average(filter_ring_t *ring)
{
    float values[FILTER_MAX_WINDOW_SIZE];
    float sum = 0.0f;
    uint32 count;
    uint32 index;

    if ((ring == NULL) || (!ring->initialized)) return 0.0f;

    count = filter_ring_size(ring);
    if ((count == 0U) || (count > FILTER_MAX_WINDOW_SIZE) ||
        !filter_ring_copy(ring, values, count)) return 0.0f;

    for (index = 0U; index < count; index++) sum += values[index];
    return sum / (float)count;
}

/**
 * @brief  执行一次 RC 低通滤波
 * @param  state: RC 滤波器状态指针
 * @param  input: 当前输入数据
 * @param  alpha: 滤波系数，范围为 0 到 1
 * @retval 当前低通滤波输出
 */
float filter_rc_low_pass(filter_rc_state_t *state, float input, float alpha)
{
    if (state == NULL) return input;

    alpha = filter_clamp_alpha(alpha);
    if (!state->low_pass_initialized)
    {
        state->low_pass_output = input;
        state->low_pass_initialized = 1U;
    }
    else
    {
        state->low_pass_output += alpha * (input - state->low_pass_output);
    }
    return state->low_pass_output;
}

/**
 * @brief  执行一次 RC 高通滤波
 * @param  state: RC 滤波器状态指针
 * @param  input: 当前输入数据
 * @param  alpha: 滤波系数，范围为 0 到 1
 * @retval 当前高通滤波输出
 */
float filter_rc_high_pass(filter_rc_state_t *state, float input, float alpha)
{
    float output;

    if (state == NULL) return input;

    alpha = filter_clamp_alpha(alpha);
    if (!state->high_pass_initialized)
    {
        state->previous_input = input;
        state->high_pass_output = 0.0f;
        state->high_pass_initialized = 1U;
    }
    output = alpha * (state->high_pass_output + input - state->previous_input);
    state->previous_input = input;
    state->high_pass_output = output;
    return output;
}

/**
 * @brief  执行一次 RC 带通滤波
 * @param  state: RC 滤波器状态指针
 * @param  input: 当前输入数据
 * @param  high_pass_alpha: 高通滤波系数，范围为 0 到 1
 * @param  low_pass_alpha: 低通滤波系数，范围为 0 到 1
 * @retval 当前带通滤波输出
 */
float filter_rc_band_pass(filter_rc_state_t *state, float input,
                          float high_pass_alpha, float low_pass_alpha)
{
    float high_pass_output;

    if (state == NULL) return input;

    high_pass_output = filter_rc_high_pass(state, input, high_pass_alpha);

    if (!state->low_pass_initialized)
    {
        state->low_pass_output = high_pass_output;
        state->low_pass_initialized = 1U;
    }
    else
    {
        state->low_pass_output += filter_clamp_alpha(low_pass_alpha) *
                                  (high_pass_output - state->low_pass_output);
    }
    return state->low_pass_output;
}

/**
 * @brief  重置 RC 滤波器运行状态
 * @param  state: RC 滤波器状态指针
 * @retval None
 */
void filter_rc_state_reset(filter_rc_state_t *state)
{
    if (state != NULL) memset(state, 0, sizeof(filter_rc_state_t));
}
