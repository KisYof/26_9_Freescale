/*
 * camera_tool.h
 *
 *  Created on: 2024年1月12日
 *      Author: 周文烽
 */

#ifndef CODE_CAMERA_TOOL_H_
#define CODE_CAMERA_TOOL_H_

#include "zf_common_headfile.h"
#include "Camera.h"
#include "Encoder.h"

#define RED 0
#define BLUE 1
#define GREEN 2
#define Dis_x 200    //屏幕上看到的宽度
#define Dis_y 120    //屏幕上看到的长度

extern uint8 right_line[LCDH];
extern uint8 left_line[LCDH];
extern uint8 mid_line[LCDH];
extern uint8 right_disline[LCDH][2];
extern uint8 left_disline[LCDH][2];
extern uint8 mid_disline[LCDH][2];

void draw_tool_process(void);


#endif /* CODE_CAMERA_TOOL_H_ */
