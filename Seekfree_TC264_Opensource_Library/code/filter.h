/**
  ******************************************************************************
  * @file    filter.h
  * @brief   基于逐飞 FIFO 的环形缓冲与数字滤波模块头文件
  *
  * 使用约束:
  *   1. filter_ring_t 使用前必须调用 filter_ring_init，否则行为未定义
  *   2. filter_ring_write 非中断安全，并发写入同一 ring 需自行加锁
  *   3. 同一个 filter_rc_state_t 可安全混用低通/高通/带通（已拆分初始化标志）
  *   4. FILTER_MAX_WINDOW_SIZE 同时决定环形缓冲上限与内部栈数组大小
  ******************************************************************************
  */

#ifndef _FILTER_H_
#define _FILTER_H_

#include "zf_common_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FILTER_MAX_WINDOW_SIZE (32U)

/* ========================= 类型定义 ========================= */

/**
  * @brief 环形滤波数据缓冲区
  * @note  使用前必须调用 filter_ring_init 进行初始化
  */
typedef struct {
    fifo_struct fifo;
    float *buffer;
    uint32 capacity;
    uint8 initialized;
} filter_ring_t;

/**
  * @brief RC 滤波器运行状态
  * @note  低通/高通/带通各自使用独立的初始化标志，可安全混用
  */
typedef struct {
    float low_pass_output;
    float high_pass_output;
    float previous_input;
    uint8 low_pass_initialized;
    uint8 high_pass_initialized;
} filter_rc_state_t;

/* ========================= 环形缓冲区函数声明 ========================= */

int8 filter_ring_init(filter_ring_t *ring, float *buffer, uint32 capacity);
int8 filter_ring_write(filter_ring_t *ring, float value);
uint32 filter_ring_size(filter_ring_t *ring);
int8 filter_ring_copy(filter_ring_t *ring, float *output, uint32 output_size);

/* ========================= 滤波算法函数声明 ========================= */

float filter_median(filter_ring_t *ring);
float filter_moving_average(filter_ring_t *ring);
float filter_rc_low_pass(filter_rc_state_t *state, float input, float alpha);
float filter_rc_high_pass(filter_rc_state_t *state, float input, float alpha);
float filter_rc_band_pass(filter_rc_state_t *state, float input,
                          float high_pass_alpha, float low_pass_alpha);
void filter_rc_state_reset(filter_rc_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
