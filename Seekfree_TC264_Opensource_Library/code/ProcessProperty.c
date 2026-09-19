/*
 * ProcessProperty.c
 *
 *  Created on: 2024年1月18日
 *      Author: 周
 */
#include "ProcessProperty.h"
#include "Fuya.h"

/*1:直道 2：弯道 3：十字 4：斑马线*/
ProcessProperty process_property[STATUS_LENTH];
ElementData element_data[ELEMENT_LENTH];//1:直道 2：弯道 3：坡道 4:基础
RingData ring_data[Ring_LENTH];//1:小圆环2：大圆环 3：大小环共有参数
int8_t SpDe_Flag=0;
int16_t HSpeed=150;
int16_t LSpeed=130;
int16_t RefSpeed=0;
uint8_t No_fuya_flag=0;

//圆环参数初始化 1:小圆环2：大圆环 3：大小环共有参数
void Ring_Data_Init(void)
{
    //大小环判断阈值初始化
    ring_data[3].ring_size_threshold=200;//30
    ring_data[3].half_wide_offset_big=8;
    ring_data[3].half_wide_offset_small=25;

    if(gpio_get_level(P11_3)==0&&gpio_get_level(P11_6)==0)//拨码23--00低速
    {
        //大小环速度初始换
        ring_data[2].ring_exter_speed=235+ring_data[2].addring_exter_speed;//260;//大圆环环外速度
        ring_data[2].ring_inter_speed=260+ring_data[2].addring_inter_speed;//330;//大圆环环内速度
        ring_data[1].ring_exter_speed=235+ring_data[1].addring_exter_speed;//260;//小圆环环外速度
        ring_data[1].ring_inter_speed=260+ring_data[1].addring_inter_speed;//小圆环环内速度
    }
    else if(gpio_get_level(P11_3)==1&&gpio_get_level(P11_6)==0)//23--01中速
    {
        //大小环速度初始换
        ring_data[2].ring_exter_speed=245+ring_data[2].addring_exter_speed;//260;//大圆环环外速度
        ring_data[2].ring_inter_speed=300+ring_data[2].addring_inter_speed;//330;//大圆环环内速度
        ring_data[1].ring_exter_speed=245+ring_data[1].addring_exter_speed;//260;//小圆环环外速度
        ring_data[1].ring_inter_speed=300+ring_data[1].addring_inter_speed;//小圆环环内速度
    }
    else if(gpio_get_level(P11_3)==0&&gpio_get_level(P11_6)==1)//23--10高速
    {
        //大小环速度初始换
        ring_data[2].ring_exter_speed=250+ring_data[2].addring_exter_speed;//260;//大圆环环外速度
        ring_data[2].ring_inter_speed=315+ring_data[2].addring_inter_speed;//330;//大圆环环内速度
        ring_data[1].ring_exter_speed=250+ring_data[1].addring_exter_speed;//260;//小圆环环外速度
        ring_data[1].ring_inter_speed=315+ring_data[1].addring_inter_speed;//小圆环环内速度
    }
    else if(gpio_get_level(P11_3)==1&&gpio_get_level(P11_6)==1)//无负压测试
    {
        //大小环速度初始换
        ring_data[2].ring_exter_speed=150+ring_data[2].addring_exter_speed;//260;//大圆环环外速度
        ring_data[2].ring_inter_speed=160+ring_data[2].addring_inter_speed;//330;//大圆环环内速度
        ring_data[1].ring_exter_speed=150+ring_data[1].addring_exter_speed;//260;//小圆环环外速度
        ring_data[1].ring_inter_speed=160+ring_data[1].addring_inter_speed;//小圆环环内速度
    }

    //前瞻行初始化
    ring_data[2].inter_prospect_line=50;//大环环内前瞻    入环的前瞻比环内远是因为药避免圆环凸出来的部分造成的抖动 这样给分段会比较丝滑
    ring_data[1].inter_prospect_line=50;//小环环内前瞻
    ring_data[2].exter_prospect_line=40;//大环环外前瞻
    ring_data[1].exter_prospect_line=44;//小环环外前瞻

}


//元素数据初始化 1:直道 2：弯道 3：坡道 4:基础 5：斑马线
void Element_Data_Init(void)
{

    //圆环信息初始化
    Ring_Data_Init();

    if(gpio_get_level(P11_3)==0&&gpio_get_level(P11_6)==0)//拨码23--00低速
    {
        //元素基础速度初始化
        element_data[1].basic_speed=300+element_data[1].addbasic_speed;//330;//直道速度
        element_data[2].basic_speed=230+element_data[2].addbasic_speed;//265;//弯道速度
        element_data[3].basic_speed=200+element_data[3].addbasic_speed;//坡道速度

        //全局基础速度初始化
        element_data[4].dynamic_H_speed=280+element_data[4].adddynamic_H_speed;//300;//
        element_data[4].dynamic_L_speed=230+element_data[4].adddynamic_L_speed;//s265;//
    }
    else if(gpio_get_level(P11_3)==1&&gpio_get_level(P11_6)==0)//拨码23--01中速
    {
        //元素基础速度初始化
        element_data[1].basic_speed=315+element_data[1].addbasic_speed;//330;//直道速度
        element_data[2].basic_speed=250+element_data[2].addbasic_speed;//265;//弯道速度
        element_data[3].basic_speed=200+element_data[3].addbasic_speed;//坡道速度

        //全局基础速度初始化
        element_data[4].dynamic_H_speed=290+element_data[4].adddynamic_H_speed;//300;//
        element_data[4].dynamic_L_speed=250+element_data[4].adddynamic_L_speed;//s265;//
    }
    else if(gpio_get_level(P11_3)==0&&gpio_get_level(P11_6)==1)//拨码23--10高速
    {
        //元素基础速度初始化
        element_data[1].basic_speed=330+element_data[1].addbasic_speed;//330;//直道速度
        element_data[2].basic_speed=265+element_data[2].addbasic_speed;//265;//弯道速度
        element_data[3].basic_speed=200+element_data[3].addbasic_speed;//坡道速度

        //全局基础速度初始化
        element_data[4].dynamic_H_speed=305+element_data[4].adddynamic_H_speed;//300;//
        element_data[4].dynamic_L_speed=265+element_data[4].adddynamic_L_speed;//s265;//
    }
    else if(gpio_get_level(P11_3)==1&&gpio_get_level(P11_6)==1)//无负压测试
    {
        //元素基础速度初始化
        element_data[1].basic_speed=180+element_data[1].addbasic_speed;//330;//直道速度
        element_data[2].basic_speed=150+element_data[2].addbasic_speed;//265;//弯道速度
        element_data[3].basic_speed=100+element_data[3].addbasic_speed;//坡道速度

        //全局基础速度初始化
        element_data[4].dynamic_H_speed=180+element_data[4].adddynamic_H_speed;//300;//
        element_data[4].dynamic_L_speed=150+element_data[4].adddynamic_L_speed;//s265;//
    }

    //传感器阈值初始化
    element_data[3].integral=30000;//坡道积分路程

    //前瞻行初始化
//    element_data[1].prospect_line=26;//直道
//    element_data[2].prospect_line=26;//弯道
//    element_data[3].prospect_line=40;//坡道
//    element_data[4].prospect_line=26;//基础
//    element_data[5].prospect_line=30;//斑马线


    element_data[1].prospect_line=25;//直道
    element_data[2].prospect_line=25;//弯道
    element_data[3].prospect_line=40;//坡道
    element_data[4].prospect_line=25;//基础
    element_data[5].prospect_line=30;//斑马线



}

//速度决策
void Speed_Decision(void)
{
//        if(ImageFlag.straight_long==1||LongestLine_Found_flag==1)
    if(ImageFlag.straight_long==1)
        {
            Basic_Speed=HSpeed;
        }
    else if(ImageFlag.image_element_rings_flag==7)
        Basic_Speed=HSpeed;
//        else if(ImageFlag.Bend_Road !=0)
//         {
    else
            Basic_Speed=LSpeed;
//         }
//         else if(ImageFlag.ring_big_small==1)
//         {
//             if(ImageFlag.image_element_rings_flag==7)
//             {
//                 Basic_Speed=HSpeed;

//             }
////             else if(ImageFlag.image_element_rings_flag==9||ImageFlag.image_element_rings_flag==8)
////                 Basic_Speed=190;
//             else
//             {
//                 Basic_Speed=LSpeed;
//             }
//         }
////         else if(ImageFlag.straight_long==1||LongestLine_Found_flag==1)
////         {
////             Basic_Speed=200;
////         }
//         else
//
//         {
////             HSpeed=210;
////             LSpeed=190;
////             RefSpeed=HSpeed-ImageStatus.Det_True*ImageStatus.Det_True*(HSpeed-LSpeed)/1800*2;
//             RefSpeed=HSpeed-ImageStatus.Det_True*ImageStatus.Det_True*(HSpeed-LSpeed)/900;
//             Basic_Speed=RefSpeed;
//             if(Basic_Speed<LSpeed)
//             {
//                 Basic_Speed=LSpeed;
//             }
//         }
}
