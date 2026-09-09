/**
  ******************************************************************************
  * @file    filter.h
  * @brief   基于逐飞 FIFO 的环形缓冲与数字滤波模块头文件
  ******************************************************************************
  */

#ifndef _FILTER_H_
#define _FILTER_H_

#include "zf_common_fifo.h"

#define FILTER_MAX_WINDOW_SIZE (32U)

/* ========================= 类型定义 ========================= */

/**
  * @brief 环形滤波数据缓冲区
  */
typedef struct {
    fifo_struct fifo;
    float *buffer;
    uint32 capacity;
} filter_ring_t;

/**
  * @brief RC 滤波器运行状态
  */
typedef struct {
    float low_pass_output;
    float high_pass_output;
    float previous_input;
    uint8 initialized;
} filter_rc_state_t;

/* ========================= 公共函数声明 ========================= */

int8 filter_ring_init(filter_ring_t *ring, float *buffer, uint32 capacity);
int8 filter_ring_write(filter_ring_t *ring, float value);
uint32 filter_ring_size(filter_ring_t *ring);
int8 filter_ring_copy(filter_ring_t *ring, float *output, uint32 output_size);


float filter_median(filter_ring_t *ring);
float filter_moving_average(filter_ring_t *ring);
float filter_rc_low_pass(filter_rc_state_t *state, float input, float alpha);
float filter_rc_high_pass(filter_rc_state_t *state, float input, float alpha);
float filter_rc_band_pass(filter_rc_state_t *state, float input,
                          float high_pass_alpha, float low_pass_alpha);
void filter_rc_state_reset(filter_rc_state_t *state);

#endif
