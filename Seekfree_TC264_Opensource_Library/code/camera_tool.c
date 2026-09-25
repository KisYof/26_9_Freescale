/*
 * camera_tool.c
 *
 *  Created on: 2024年1月12日
 *      Author: 周文烽
 */
#include "Camera.h"
#include "Camera_tool.h"
#include "zf_common_headfile.h"

#define DISPLAY_FRAME_DIVIDER (2U)

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


    if (++update_cnt < DISPLAY_FRAME_DIVIDER)
    {
        return;
    }

    update_cnt = 0;

    // 1. 显示二值化图像
  ips200_show_gray_image(0,0,Pixle[0],LCDW,LCDH,200,120,127);

//    ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);//原始图

    // 2. 在二值图上画蓝色正常中线

//  for (i = 0; i < LCDH; i++)
//      {
//          // 跳过异常的图像坐标，避免越界
//          if (NormalMidLine[i] < 0 || NormalMidLine[i] >= LCDW)
//          {
//              continue;
//          }
//
//          // 图像坐标映射到 IPS200 显示区域
//          x = (uint16)(NormalMidLine[i] * 200 / LCDW);
//          y = (uint16)(i * 120 / LCDH);
//
//          ips200_draw_point(x, y, RGB565_BLUE);
//      }
  if (NormalMidValid)//只有中线合理时才画中线
  {
      for (i = 0; i < LCDH; i++)
      {
          if (NormalMidLine[i] < 0 || NormalMidLine[i] >= LCDW)
          {
              continue;
          }

          x = (uint16)(NormalMidLine[i] * 200 / LCDW);
          y = (uint16)(i * 120 / LCDH);

          ips200_draw_point(x, y, RGB565_BLUE);
      }
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

    // 直线判断测试
    ips200_show_string(0, 200, "Straight:");
    ips200_show_int(80, 200, ImageFlag.straight_long, 1);

    // 十字判断测试
    ips200_show_string(0, 220, "Cross:");
    ips200_show_int(80, 220, ImageFlag.Cross_Flag, 1);

    // 斑马线判断测试
    ips200_show_string(0, 240, "Zebra:");
    ips200_show_int(80, 240, ImageFlag.Zebra_Flag, 1);

    // 圆环判断测试
    ips200_show_string(0, 260, "Ring:");//0：无圆环 1：左圆环 2：右圆环
    ips200_show_int(80, 260, ImageFlag.image_element_rings, 1);

    ips200_show_string(0, 280, "RState:");//圆环状态
    ips200_show_int(80, 280, ImageFlag.image_element_rings_flag, 1);

    ips200_show_string(0, 300, "L/R:");//左边数字 = 左圆环检测次数  右边数字 = 右圆环检测次数
    ips200_show_int(80, 300, left_ring_num, 2);
    ips200_show_int(120, 300, right_ring_num, 2);

    //轨道斜率测试
    ips200_show_string(120, 140, "Slope:");
    ips200_show_int(140, 140, Track_Slope_x1000, 5);

    ips200_show_string(120, 160, "SValid:");
    ips200_show_int(140, 160, Track_SlopeValid, 1);

}


