/*
 * camera_tool.c
 *
 *  Created on: 2024年1月12日
 *      Author: 周文烽
 */
#include "Camera.h"
#include "Camera_tool.h"
#include "Encoder.h"
#include "zf_common_headfile.h"

uint8 right_line[LCDH];             //二值化原图的左线
uint8 left_line[LCDH];              //二值化原图的右线
uint8 mid_line[LCDH];               //二值化原图的中线
uint8 right_disline[LCDH][2];       //屏幕左线的坐标点
uint8 left_disline[LCDH][2];        //屏幕右线的左边点
uint8 mid_disline[LCDH][2];         //0-x 1-y

uint32 Time_counter=0;


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief         将每行的边界存储到数组方便画点
//  @brief
//  @brief
//  @parameter
//  @return
//  @time
//  @Author
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void put_line_to_array(void)
{
    uint8 i;
    for(i=0;i<LCDH;i++)
    {
        right_line[i]=ImageDeal[i].RightBorder;
        left_line[i]=ImageDeal[i].LeftBorder;
        //mid_line[i]=ImageDeal[i].Center;

        mid_line[i] = NormalMidLine[i];//显示是 Get_AllLine() 之后、元素处理之前的正常中线
    }
}
void array_to_disarray(void)
{

    uint8 i;
    uint8 dis_height;
    for(i=0;i<LCDH;i++)
    {
        dis_height=(uint8)(i*Dis_y/LCDH);

        right_disline[i][0]=(uint8)(right_line[i]*Dis_x/LCDW);
        left_disline[i][0]=(uint8)(left_line[i]*Dis_x/LCDW);
        mid_disline[i][0]=(uint8)(mid_line[i]*Dis_x/LCDW);

        right_disline[i][1]=dis_height;
        left_disline[i][1]=dis_height;
        mid_disline[i][1]=dis_height;

    }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief         将每行的边界存储到数组方便画点
//  @brief          //x——0~239(ips200_width_max) y——0~319(ips200_height_max)
//  @brief          //边界的就画一个点，不是边界的画周围九个
//  @parameter
//  @return
//  @time
//  @Author
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void ips200_draw_rgb_point(uint8 x,uint8 y,uint8 color)
{
    uint8 i,j;
    if(color==RED)
    {
        if(x>0&&x<(ips200_width_max-1)&&y>0&&y<(ips200_height_max-1))
        {
            for(i=x-1;i<=x+1;i++)
            {
                for(j=y-1;j<=y+1;j++)
                {
                    ips200_draw_point(i,j,RGB565_RED);
                }
            }
        }
        else if(x==0||x==ips200_width_max-1||y==0||y==ips200_height_max-1)
        {
            ips200_draw_point(x,y,RGB565_RED);
        }
    }
    else if(color==BLUE)
    {
        if(x>0&&x<(ips200_width_max-1)&&y>0&&y<(ips200_height_max-1))
        {
            for(i=x-1;i<=x+1;i++)
            {
                for(j=y-1;j<=y+1;j++)
                {
                    ips200_draw_point(i,j,RGB565_BLUE);
                }
            }
        }
        else if(x==0||x==ips200_width_max-1||y==0||y==ips200_height_max-1)
        {
            ips200_draw_point(x,y,RGB565_BLUE);
        }
    }
}





//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief         屏幕画左右边线
//  @brief
//  @brief
//  @parameter
//  @return
//  @time
//  @Author
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void ips200_draw_side_line(uint8 line_r[],uint8 line_l[])
{
    uint8 i;
    for(i=0;i<LCDH-1;i++)
    {
        ips200_draw_rgb_point(line_r[i],i,RED);
        ips200_draw_rgb_point(line_l[i],i,RED);
    }
}

void ips200_draw_mid_line(uint8 line_mid[])
{
    uint8 i;
    for(i=0;i<LCDH-1;i++)
    {
        ips200_draw_rgb_point(line_mid[i],i,BLUE);
    }
}

void ips_draw_center(void)
{
    int i;
    for(i=LCDH-1;i>=20;i--)
    {
        ips200_draw_point(ImageDeal[i].Center,i,RGB565_BLUE);
        ips200_draw_point(ImageDeal[i].LeftBoundary,i,RGB565_GREEN);
        ips200_draw_point(ImageDeal[i].RightBorder,i,RGB565_RED);
    }
}





void ips_show_data(void)
{
//    ips200_show_int(LCDW+5,0,a,3);
//    ips200_show_int(LCDW+5,30,ImageStatus.OFFLine,3);
//    ips200_show_float(LCDW+5,150,turn.Kp,2,3);
//    ips200_show_float(LCDW+5,180,turn.Kd,2,3);
//    ips114_show_int(LCDW+5,30,ImageStatus.Cross_Miss_Right_lines,3);
//    ips200_show_float(LCDW+5,240,pid_value,2,3);
    ips200_show_int(LCDW+20,120,ImageStatus.Det_True,3);
}



/*===================================================================================
//在 IPS200 屏幕上显示二值化图像
void draw_tool_process(void)
{
//    system_start();
    myadc=adc_mean_filter_convert(ADC0_CH5_A5,10);//10次滤波
//    int time=system_getval_us();
//    if(go==0)
//    {
    put_line_to_array();
    array_to_disarray();
    my_ips200_show_gray_image(0,0,Pixle[0],LCDW,LCDH,200,120,127);
//    ips200_show_gray_image(0, 125, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    ips200_show_string(0,260,"ImageStatus.OFFLine:");
    ips200_show_int(200,260,ImageStatus.OFFLine,3);
    ips200_show_string(0,280,"Miss_Left_lines:");
    ips200_show_int(200,280,ImageStatus.Miss_Left_lines,3);
    ips200_show_string(0,300,"Miss_Right_lines:");
    ips200_show_int(200,300,ImageStatus.Miss_Right_lines,3);
//    ips200_show_string(0,240,"element_rings_flag:");
//    ips200_show_int(200,240,ImageFlag.image_element_rings_flag,3);
    ips200_show_string(0,240,"ImageFlag.Bend_Road:");
        ips200_show_int(200,240,ImageFlag.Bend_Road,3);
    ips200_show_string(0,220,"ImageStatus.Det_True:");
    ips200_show_int(200,220,ImageStatus.Det_True,4);
//    ips200_show_string(0,200,"Distance_start:");
//    ips200_show_int(160,200,Distance_start,6);
    ips200_show_string(0,200,"LongestLine_Found:");
    ips200_show_int(200,200,LongestLine_Found_flag,4);
    ips200_show_string(0,180,"straight_long:");
    ips200_show_int(200,180,ImageFlag.straight_long,4);
    ips200_show_string(0,160,"pid_value:");
    ips200_show_int(200,160,pid_value,4);
    ips200_show_string(0,140,"right_ring_num:");
    ips200_show_int(200,140,right_ring_num,4);
    ips200_show_string(0,120,"num:");
    ips200_show_int(150,120,num,4);
//


//    }
//    ips114_show_gray_image (10, 10, (const uint8 *)mt9v03x_image, 188, 120,188, 120,0);
//    ips114_show_gray_image(0,0,Pixle[0],126,70,126,70,0);
//    ips200_show_gray_image (10, 10, (const uint8 *)mt9v03x_image, 188, 120,188, 120,0);
//    ips_draw_center();
//    ips_show_data();
}
*/

/*------------------------------------------------------------------------------------------------------------------
 * @brief          在IPS200上显示二值图、正常中线和调试数据
 * @note           显示内容：
 *                 上方：二值化图像
 *                 图像上：蓝色正常中线
 *                 下方：MV、ME、V、DET
 *------------------------------------------------------------------------------------------------------------------*/

void draw_tool_process(void)
{
    static uint8 update_cnt = 0;
    uint16 i;
    uint16 x;
    uint16 y;


    myadc = adc_mean_filter_convert(ADC0_CH5_A5, 10);


    if (++update_cnt < 2)
    {
        return;
    }

    update_cnt = 0;

    // 1. 显示二值化图像
  ips200_show_gray_image(0,0,Pixle[0],LCDW,LCDH,200,120,127);

//    ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);//原始图

    // 2. 在二值图上画蓝色正常中线
    for (i = 0; i < LCDH; i++)
    {
        // 图像坐标映射到IPS200显示区域
        x = (uint16)(NormalMidLine[i] * 200 / LCDW);
        y = (uint16)(i * 120 / LCDH);

        ips200_draw_point(x, y, RGB565_BLUE);
    }

    // 3. 显示正常中线位置
    ips200_show_string(0, 140, "MV:");
    ips200_show_int(40, 140, NormalMidValue, 4);

    // 4. 显示正常中线误差
    ips200_show_string(0, 160, "ME:");
    ips200_show_int(40, 160, NormalMidError, 4);

    // 5. 显示正常中线是否有效
    ips200_show_string(0, 180, "V:");
    ips200_show_int(40, 180, NormalMidValid, 2);

}




