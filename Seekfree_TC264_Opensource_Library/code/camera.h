/*
 * Camera.h
 *
 *  Created on: 2023年11月16日
 *      Author: 周文烽
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#include "zf_common_headfile.h"

#define LCDH 70                                              //用于图像处理的图像高度（行）
#define LCDW 160                                             //用于图像处理的图像宽度（列）

#define Cut_high_bottom 30                                   //裁剪原图的底部
#define Cut_high_top 20                                      //裁剪原图的顶部
#define Cut_width_half (188-LCDW)/2                          //裁剪原图的左右 (188-LCDW)/2

#define ImageSensorMid    LCDW/2-1                           //图像的屏幕中点
#define LimitL(L) (L = ((L < 1) ? 1 : L))                    //限幅限幅
#define LimitH(H) (H = ((H > (LCDW-2)) ? (LCDW-2) : H))      //限幅限幅
#define BeeOn       pwm_init(ATOM1_CH3_P10_3,3800,5000);       //蜂鸣器叫
#define BeeOff      pwm_init(ATOM1_CH3_P10_3,3800,0);          //蜂鸣器不叫
extern uint8 Mid_line[LCDH];                                 //最开始写的Get_SideLine搜边线函数里面用来存放中线的数组
extern uint8 Left_line[LCDH];                                //最开始写的Get_SideLine搜边线函数里面用来存放左边线的数组
extern uint8 Right_line[LCDH];                               //最开始写的Get_SideLine搜边线函数里面用来存放右边线的数组

extern uint8 Garage_Location_Flag;                           //暂时没用到
extern int32 Image_Scan_Length;                              //暂时没用到
extern uint8 Threshold;                                      //每一帧图像的二值化阈值
extern uint8 Pixle[LCDH][LCDW];                              //二值化后的图像
extern volatile uint32 VisionFrameSequence;                  // Odd: processing, even: stable processed frame
extern uint8* Image_Use[LCDH][LCDW];                         //裁剪后图像各像素的地址
extern uint16 Ramp_cnacel;                                   //坡道标志取消
extern uint8 ramp_finish_flag;                               //坡道状态结束
extern int Black;                                            //判断大小圆环时用的黑点数量
extern uint16 distance;                                      //坡道用来接收距离数据
extern uint8 Prospect_line;                                      //图像前瞻控制行
extern uint8 Image_iError_Flag;
extern int8 Error_change_flag;
extern uint16 position;
extern uint32 Ramp_num;
extern float Cross_white_line;                              //十字双丢边行阈值条件
extern uint8 NO_zhidao_add_flag;
extern int right_ring_num;
extern int left_ring_num;
extern uint8 num;
extern uint8 numchujie;


extern int NormalMidLine[];
extern int NormalMidValue;
extern int NormalMidError;
extern uint8 NormalMidValid;

void NormalMid_Update(void);

extern volatile int16 Track_Slope_x1000;
extern volatile uint8 Track_SlopeValid;

void Track_Slope_Update(void);

extern int ControlMidLine[];
extern volatile int ControlMidValue;
extern volatile int ControlMidError;
extern volatile uint8 ControlMidValid;

void ControlMid_Update(void);

extern volatile uint8 LineLostCount;
extern volatile uint8 LineLostFlag;
extern volatile uint8 VisionStopFlag;

void LineLost_Protection_Update(void);
void Camera_RequestRuntimeReset(void);
uint8 Camera_RuntimeResetPending(void);

extern uint8 ControlProspectType;
extern uint8 ControlProspectRow;
//每一行的属性
typedef struct {
  //梯度扫线数据
  uint8 IsRightFind;                                         //右边有边标志
  uint8 IsLeftFind;                                          //左边有边标志
  int Wide;                                                  //边界宽度
  int LeftBorder;                                            //左边线 梯度扫线
  int RightBorder;                                           //右边线
  int Center;                                                //中线
  int TopBorder;                                             //顶部边线

  int LeftBorder_Inner;
int LeftBorder_Outer;
int RightBorder_Inner;
int RightBorder_Outer;
  
  //左右手法则扫线数据
  int LeftBoundary_First;                                    //左边界 保存第一次数据
  int RightBoundary_First;                                   //右边界 保存第一次数据
  uint8 LeftBoundary;                                        //左边界 保存最后一次数据
  uint8 RightBoundary;                                       //右边界 保存最后一次数据
} ImageDealDatatypedef;

typedef struct {
  //图像信息
  int TowPoint;                                              //控制行，也就是我说的前瞻
  uint8 OFFLine;                                             //图像顶边，扫线截止行
  uint8 WhiteLine;                                           //双边丢边数
  uint8 OFFLineBoundary;                                     //扶墙截止行
  uint8 Rings_OFFLine;                                        //圆环半宽补线截止行
  uint8 Miss_Left_lines;                                     //左线丢失
  uint8 Miss_Right_lines;                                    //右线丢失
  uint8 Cross_Miss_Left_lines;                               //十字左线丢失
  uint8 Cross_Miss_whitelines;                               //十字双边丢失
  uint8 Cross_Miss_Right_lines;                              //十字右线丢失
  uint8 Bend_Miss_Left_lines;
  uint8 Bend_Miss_Right_lines;
  int Det_True;                                              //前瞻
  float MU_P;                                                //
  float MU_D;                                                //

  //左右手法则扫线数据
  uint8 WhiteLine_L;                                         //左边丢线数
  uint8 WhiteLine_R;                                         //右边丢线数
} ImageStatustypedef;

typedef struct {
  int point;                                                 //梯度扫线扫到的边线点
  uint8 type;                                                //梯度扫线扫到的点对应的类型
} JumpPointtypedef;

typedef struct {
    uint8 Search_error_flag;                                 /*0:种子寻找正常      1：种子寻找不正常*/
    uint8 Bend_Road;                                         /*0:无            1:右弯道       2:左弯道*/
    uint8 image_element_rings;                               /*0:无圆环         1:左圆环       2:右圆环*/
    uint8 ring_big_small;                                    /*0:无            1:小圆环       2:大圆环*/
    uint8 image_element_rings_flag;                          /*圆环进程 0 1 2 3 4 5 6 7 8 9 */
    uint8 straight_long;                                     /*长直道标志位 0：不是长直道 1：是长直道*/
    uint8 straight_xie;                                      /*中线长直道标志位 0：不是长直道 1：是长直道*/
    uint8 Zebra_Flag;                                        /*0 :无斑马线       1 :斑马线*/
    uint8 Zebra_Counter;                                     /*斑马线计数器 0：无斑马线 1：第一次经过斑马线 2：第二次经过斑马线*/
    uint8 Ramp;                                              /*0 :无坡道        1：坡道*/
    uint8 Ramp_Flag;
    uint8 Cross_Flag;                                        /*0:十字有       1：十字无*/
    uint8 Stop;                                              /*0:正常行驶      1：刹车*/
    uint8 chujie_Stop_1;
    uint8 SmallBlock;
} ImageFlagtypedef;

extern ImageDealDatatypedef ImageDeal[LCDH];
extern ImageStatustypedef ImageStatus;
extern ImageFlagtypedef ImageFlag;
extern uint8 LongestLine_Found_flag;
extern int a;
extern int ImageScanInterval;

uint8 Protect_Without_Road(uint8 StartLine,uint8 EndLine,uint8 StartRow,uint8 EndRow);
void Image_CutInit(void);
uint8 Image_Process(void);
void Get_BinaryImage(void);
uint8_t GetOSTU(uint8 image[MT9V03X_H][MT9V03X_W],int height,int wight);
void Get_Border_And_SideType_MidWhite(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q) ;
void Get_BaseLine(void);
void Get_AllLine(void);
void Get_ExtensionLine(void);
void Flag_init(void);
void Data_init(void);
extern uint8 get_Threshold(uint8* image[][LCDW],uint16 col, uint16 row);
extern uint32 break_road(uint8 line_start);

#endif /* CODE_CAMERA_H_ */
