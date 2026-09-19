/*
 * camera_tool.c
 *
 *  Created on: 2024年1月12日
 *      Author: 周文烽
 */
#include "Camera.h"
#include "Camera_tool.h"
#include "zf_common_headfile.h"

/*------------------------------------------------------------------------------------------------------------------
 * @brief          在IPS200上显示二值图、正常中线和调试数据
 * @note           显示内容：
 *                 上方：二值化图像
 *                 图像上：蓝色正常中线
 *                 下方：MV、ME、V
 *------------------------------------------------------------------------------------------------------------------*/

void draw_tool_process(void)
{
    static uint8 update_cnt = 0;
    uint16 i;
    uint16 x;
    uint16 y;


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




