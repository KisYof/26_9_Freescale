/*
 * Encoder.h
 *
 *  Created on: 2023年9月24日
 *      Author: 23913
 */

#ifndef ENCODER_H_
#define ENCODER_H_
void Encoder_init(void);
void Motor_control (void);
extern float encofloat_1;
extern float encofloat_2;
extern int16 encoder_data_1 ;
extern int16 encoder_data_2 ;//值做全局变量
extern int16 ENCO_Wish_L;
extern int16 ENCO_Wish_R;
extern int16 Basic_Speed;
extern int16 encoder_sum_1 ;
extern int16 encoder_sum_2 ;
extern float speed1;
extern float speed2;
extern float R_speed;
extern float L_speed;
extern float Distance;
extern float Distance_start;
extern uint8 jifen_Stop;
extern uint8 tail[4];
extern uint16 stop_distance;
extern float fdata[6];
extern int tim_five;
extern int16 pwm_l;
extern int16 pwm_r;
extern pid_param_t turn;
extern float
myadc;
#endif /* ENCODER_H_ */
