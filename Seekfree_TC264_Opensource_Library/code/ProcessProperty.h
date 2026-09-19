/*
 * ProcessProperty.h
 *
 *  Created on: 2024年1月18日
 *      Author: 周文烽
 */

#ifndef CODE_PROCESSPROPERTY_H_
#define CODE_PROCESSPROPERTY_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "Encoder.h"
#include "camera.h"
#define STATUS_LENTH 30
#define ELEMENT_LENTH 6
#define Ring_LENTH 6

extern int8_t SpDe_Flag;
extern int16_t HSpeed;
extern int16_t LSpeed;
extern uint8_t No_fuya_flag;
typedef struct ProcessProperty{
        uint8 min_speed;
        uint8 max_speed;
        uint16 integral;
        uint8 speed_detaction_flag;
}ProcessProperty;

typedef struct ElementData{
        uint16 basic_speed;//元素基础速度
        uint16 dynamic_L_speed;//动态速度最大值
        uint16 dynamic_H_speed;//动态速度最小值
        uint16 integral;//编码器或陀螺仪积分的阈值
        uint16 prospect_line;//元素前瞻行
        uint16 addbasic_speed;
        uint16 adddynamic_H_speed;
        uint16 adddynamic_L_speed;
}ElementData;

typedef struct RingData{
        uint16 half_wide_offset_big;//大环半宽偏移量
        uint16 half_wide_offset_small;//小环半宽偏移量
        uint16 ring_size_threshold;//大小换阈值判断
        uint16 ring_inter_speed;//环内速度
        uint16 ring_exter_speed;//环外速度
        uint16 inter_prospect_line;//环内前瞻行
        uint16 exter_prospect_line;//环外前瞻行
        uint16 addring_exter_speed;
        uint16 addring_inter_speed;
}RingData;

extern ElementData element_data[ELEMENT_LENTH];
extern RingData ring_data[Ring_LENTH];

void Element_Data_Init(void);
void Speed_Decision(void);

#endif /* CODE_PROCESSPROPERTY_H_ */
