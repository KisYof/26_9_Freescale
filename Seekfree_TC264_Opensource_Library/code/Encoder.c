/*
 * Encoder.c
 *
 *  Created on: 2023骞�9鏈�24鏃�
 *      Author: 23913
 */
#include "zf_common_headfile.h"
#include "pid.h"
#include "FIliter.h"
#include "camera.h"
#include"control.h"
#define ENCODER_2                   (TIM6_ENCODER)
#define ENCODER_2_DIR               (TIM6_ENCODER_CH2_P20_0)
#define ENCODER_2_LSB               (TIM6_ENCODER_CH1_P20_3)

#define ENCODER_1                   (TIM2_ENCODER)
#define ENCODER_1_DIR               (TIM2_ENCODER_CH2_P33_6)
#define ENCODER_1_LSB               (TIM2_ENCODER_CH1_P33_7)
//int16 ENCO_Wish_L=70;
//int16 ENCO_Wish_R=70;

int16 ENCO_Wish_L=100;
int16 ENCO_Wish_R=100;

uint16 stop_distance=150;
int16 Basic_Speed=145;//400
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_sum_1 =0;
int16 encoder_sum_2 =0;
float encofloat_1;
float encofloat_2;
int16* point1=0;
int16* point2=0;
float speed1=0;
float speed2=0;
pid_param_t Steer_PID;
pid_param_t Motor_Left_PID;
pid_param_t Motor_Right_PID;
pid_param_t Current_PID_L;
pid_param_t Current_PID_R;
pid_param_t turn;
uint8 jifen_Stop;

/**********************************************
* @brief :缂栫爜鍣ㄥ垵濮嬪寲锛屽紑鍚畾鏃跺櫒5涓柇
* @param :鏃�bangbang
* @return:鏃�
* @date  :231008
* @author:T
************************************************/
void Encoder_init(void){
//    encoder_dir_init(ENCODER_1, ENCODER_1_DIR, ENCODER_1_LSB);                  // 鍒濆鍖栫紪鐮佸櫒妯″潡涓庡紩鑴� 鏂瑰悜瑙ｇ爜缂栫爜鍣ㄦā寮�
//    encoder_dir_init(ENCODER_2, ENCODER_2_DIR, ENCODER_2_LSB);
    encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);//右轮
    encoder_quad_init(TIM6_ENCODER,TIM6_ENCODER_CH1_P20_3,TIM6_ENCODER_CH2_P20_0 );//左轮

}


float fdata[6]={0};
uint8 tail[4]={0x00, 0x00, 0x80, 0x7f};
void uart_read_buffer(uint8* buff, uint32 len) {
    while (len)
    {
        static uint8_t i = 0;
        if (uart_query_byte(UART_2, buff)) {
            len--;
            buff++;
            i++;
        }
        else { break; }
    }
}
float R_speed=0;
float L_speed=0;
float Distance=0;
float Distance_start=0;

int tim_five=0;
int16 pwm_l;
int16 pwm_r;
//int yang=1;
float myadc;
void Motor_control(void)
{

    encoder_data_1 = -SecondOrderLagFilter_L(encoder_get_count(TIM6_ENCODER));//二阶滞后滤波
    encoder_data_2 = -SecondOrderLagFilter_R(encoder_get_count(TIM2_ENCODER));

    encoder_clear_count (TIM2_ENCODER);
    encoder_clear_count (TIM6_ENCODER);

    R_speed=(((float)encoder_data_2/1024)*0.4412*0.2011)/0.002;//左右轮速度
    L_speed=(((float)encoder_data_1/1024)*0.4412*0.2011)/0.002;


    if(ImageFlag.Stop==1)
               {
        Distance=Distance+((R_speed+L_speed)/2)*100*0.002;//总距离
                   if(Distance>=stop_distance){
                       jifen_Stop=1;
                   }
                }

//    difspeed();

//    /*pid算法
    pwm_l = motorL_pid(Motor_Left_PID, 2*encoder_data_1, ENCO_Wish_L);
    pwm_r = motorR_pid(Motor_Right_PID, 2*encoder_data_2, ENCO_Wish_R);


    set_left_speed(pwm_l);
    set_right_speed(pwm_r);

}




