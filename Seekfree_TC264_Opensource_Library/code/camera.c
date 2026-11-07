/*
 * Camera.c
 *
 *  Created on: 2023年11月16日
 *      Author: 周文烽
 */
//sobel
#include "zf_common_headfile.h"
#include "isr_config.h"
#include "camera.h"
#include "math.h"

#include <stdio.h>

/*图像基础信息的变量*/
//uint8* Image_Use;                                   //用来存储压缩之后灰度图像的二维数组
uint8 Pixle[LCDH][LCDW];                                        //图像处理时真正处理的二值化图像数组
volatile uint32 VisionFrameSequence = 0;                         // Odd while CPU1 updates Pixle, even when stable
float Mh=MT9V03X_H;                                             //压缩图像时用来接原图的高
float Mw=MT9V03X_W;                                             //压缩图像时用来接原图的宽
float Lh=LCDH;                                                  //压缩图像后的高
float Lw=LCDW;                                                  //压缩图像后的宽
uint8 Threshold;                                                //通过大津法计算出来的二值化阈值
int ImageScanInterval=5;                                        //扫边的范围
uint8 ImageScanInterval_corret=0;                               //W行的边界修正（暂时赋值为0，不用先）
static uint8* PicTemp;                                          //一个保存单行图像像素值的指针变量
static int IntervalLow = 0, IntervalHigh = 0;                   //梯度扫描区间的上下限变量
static int Ysite = 0, Xsite = 0;                                //Ysite就是图像的行，Xsite就是图像的列。
static int BottomBorderRight = LCDW-1,                          //69行的右边界
BottomBorderLeft = 0,                                           //69行的左边界
BottomCenter = 0;                                               //69行的中点
static int ytemp = 0;                                           //存放行的临时变量
#define Middle_Center LCDW/2-1                                  //屏幕中心
ImageDealDatatypedef ImageDeal[LCDH];                           //记录单行信息的结构体数组
ImageDealDatatypedef ImageDeal1[LCDH];                          //记录单列信息的结构体数组
ImageStatustypedef ImageStatus;                                 //图像处理的的全局变量
ImageFlagtypedef ImageFlag;                                     //图像元素的标志位


/*midblack重扫线(自己写的那个扫线时中间是黑像素重新扫线）相关变量*/
int ExterMid_Row=0;                                             //midblack重定义的行数
float ExterMid_Det=0;                                           //midbalck重定义中线斜率
float Midblack_Det=0;
int Midblack_aim_point=0;
int ExterMid_num=0;                                             //midblack重定义的次数



/*中线相关变量 */
int NormalMidLine[LCDH];//保存当前帧的“正常中线”
int NormalMidValue = 0;//当前帧经过近端和远端加权后得到的正常中线位置
int NormalMidError = 0;//正常中线误差
uint8 NormalMidValid = 0;//正常中线有效标志

/* 提供给控制组的轨道斜率相关变量 */
volatile int16 Track_Slope_x1000 = 0;   // 斜率乘以1000  >0：右轨道 <0：左轨道
volatile uint8 Track_SlopeValid = 0;    // 1：有效，0：无效
/* 提供给控制组的中线相关变量 */
int ControlMidLine[LCDH];
volatile int ControlMidValue = 0;
volatile int ControlMidError = 0;
volatile uint8 ControlMidValid = 0;

/*直道相关变量*/
uint8 Prospect_line=26;                                         //图像前瞻控制行

/*斑马线相关变量*/
uint32 Zebra_length=0;                                          //斑马线赛道长度
uint32 Zebra_num=2500;                                         //斑马线赛道长度阈值
uint8 Zebra_Bottom_flag=0;                                  //斑马线到一帧图像底部的标志位
uint8 Zebra_Miss_flag=0;                                    //斑马线从一帧图像消失的标志位

/*十字相关变量*/
uint8 ExtenLFlag = 0;                                           //左边线是否需要补线的标志变量（十字）
uint8 ExtenRFlag = 0;                                           //右边线是否需要补线的标志变量（十字）
int left_difference_num;                                        //十字左边线与标准中线39作差的和（40-20行）
int right_difference_num;                                       //十字右边线与标准中线39作差的和（40-20行）
int Cross_Corner_L=0;                                           //十字右底部的拐点
int Cross_Corner_R=0;                                           //十字左底部的拐点
static int TFSite = 0,left_FTSite = 0,right_FTSite = 0;         //补线计算斜率的时候需要用的存放行的变量
static float DetR = 0, DetL = 0;                                //存放补线斜率的变量
//uint8 ExterMidFlag = 0;

uint8 Buxian_Flag_R=0;
uint8 Buxian_Flag_L=0;

/*圆环相关变量*/
uint8 Ring_Help_Flag = 0;                                       //进环辅助标志（开启圆环进程的标志）
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //右圆环判断的两点纵坐标
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //左圆环判断的两点纵坐标
int Point_Xsite,Point_Ysite;                                    //拐点横纵坐标
int Repair_Point_Xsite,Repair_Pont_Ysite;                      //补线点横纵坐标
int Point_Ysite1;                                               //判断大小圆环时用的纵坐标
int Black;                                                      //判断大小圆环时用的黑点数量
int Less_Big_Small_Num;                                         //判断大小圆环时用的
uint16 circle_num=0;                                            //判断圆环大小的计数器
uint16 position=37;//正15 反38                                   //大环判断的阈值
float Big_Small_Help_Gradient;                                  //大小圆环的辅助判断斜率
int Small_Rings_HalfWide_Offset=20;                            //小环半宽偏移
int right_ring_num=0;
int left_ring_num=0;

uint8 Half_Road_Wide[LCDH] =                                    //直道赛道半宽
{

        12,13,14,14,15,15,16,17,18,19,
        19,20,20,21,21,22,23,23,24,25,
        25,26,26,27,28,28,29,30,30,31,
        32,32,33,34,34,35,36,36,37,38,
        38,39,40,40,41,42,42,43,43,44,
        45,45,46,46,47,48,48,49,49,50,
        51,51,52,53,53,54,54,55,56,56
};

int8 All_Road_Wide[LCDH]=                                      //直道赛道全宽
{
        24,26,28,28,30,31,32,34,36,38,
        39,40,41,42,43,45,46,47,49,50,
        51,53,53,55,56,57,59,60,61,62,
        64,65,66,68,68,70,72,72,74,76,
        76,78,80,80,82,84,84,86,86,88,
        90,90,92,93,94,96,97,98,99,101,
        102,103,104,106,107,108,109,110,112,113
};
uint8 Half_Bend_Wide[LCDH] =                                    //弯道赛道半宽32  53//25个0
{
    32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
    32, 32, 32, 32, 32, 32, 31, 30, 30, 30,
    29, 30, 30, 30, 29, 29, 29, 29,30, 30,
    30, 30, 30, 30, 30, 30, 31, 31, 31, 31,
    31, 31, 31, 32, 33, 33, 33, 34, 34, 34,
    34, 35, 36, 36, 36, 36, 37, 37, 38, 38,
};

//圆环半宽
uint8 Half_Ring_Wide[LCDH] =//50行有半宽值
{
        20,20,20,20,20,20,20,20,20,20,
        7,13,16,18,20,22,23,25,26,27,
        28,29,30,31,32,32,33,34,35,36,
        36,37,38,38,39,39,40,41,41,42,
        42,43,43,44,44,45,45,46,46,47,
        47,48,48,49,49,50,50,50,51,51,
        52,52,53,53,53,54,54,54,55,55
};

//小圆环半宽
uint8 Half_Ring_Wide_small[LCDH]=
{
        20,20,20,20,20,20,20,20,20,20,
        20,20,20,20,20,20,20,20,9,14,
        17,19,21,23,24,26,27,28,29,30,
        31,32,33,34,35,36,36,37,38,39,
        39,40,41,41,42,43,43,44,45,45,
        46,46,47,47,48,48,49,49,50,50,
        51,51,52,52,53,53,54,54,55,55
};

#define VP_NORMAL             0
#define VP_CROSS              1
#define VP_ZEBRA              2
#define VP_RING_ENTRY         3
#define VP_RING_INSIDE        4
#define VP_RING_INSIDE_BIG    5
#define VP_RING_EXIT          6
#define VP_COUNT              7

#define VP_NORMAL_ROW          27  // 普通赛道固定前瞻行
#define VP_DYNAMIC_NEAR_ROW    35  // 普通赛道最近前瞻行
#define VP_DYNAMIC_SLOPE_START 120  // 斜率绝对值（放大1000倍），超过该值开始收近
#define VP_DYNAMIC_SLOPE_FULL  800  // 达到此值时目标行收至最近
#define VP_DYNAMIC_STEP_NEAR   2    // 每帧向近处最多移动的行数
#define VP_DYNAMIC_STEP_FAR    1    // 每帧向远处最多移动的行数
//建立元素前瞻行参数表
static const uint8 Vision_ProspectRow[VP_COUNT] =
{
    VP_NORMAL_ROW,  // 普通赛道
    40,     // 十字：看中间
    35,     // 斑马线
    30,     // 圆环入口
    47,     // 圆环内部
    50,     // 大圆环内部
    30      // 圆环出口
};

uint8 ControlProspectType = VP_NORMAL;
uint8 ControlProspectRow  = 26;
static uint8 Vision_DynamicProspectRow = VP_NORMAL_ROW;

/*其他*/
#define LimitP_L(x) (x=((x>ImageStatus.OFFLine)?x:(ImageStatus.OFFLine+2)))//对图像的前瞻进行限制，不要超过扫线截止行（后续用动态前瞻控制行的时候那里限制，这里删掉）
#define Scan_type 1                                             //选择扫线类型 1：梯度   2：梯度加重定义

static volatile uint8 image_processing = 0;
static volatile uint8 vision_runtime_reset_request = 0;

static void Camera_ClearRuntimeState(void)
{
    memset(&ImageFlag, 0, sizeof(ImageFlag));
    memset(&ImageStatus, 0, sizeof(ImageStatus));

    Zebra_Bottom_flag = 0;
    Zebra_Miss_flag = 0;
    NO_zhidao_add_flag = 0;
    LongestLine_Found_flag = 0;
    Ring_Help_Flag = 0;
    left_ring_num = 0;
    right_ring_num = 0;
    num = 0;
    numchujie = 0;

    NormalMidValue = 0;
    NormalMidError = 0;
    NormalMidValid = 0;
    ControlMidValue = 0;
    ControlMidError = 0;
    ControlMidValid = 0;
    Track_Slope_x1000 = 0;
    Track_SlopeValid = 0;
    Vision_DynamicProspectRow = VP_NORMAL_ROW;

    LineLostCount = 0;
    LineLostFlag = 0;
    VisionStopFlag = 0;
}

void Camera_RequestRuntimeReset(void)
{
    vision_runtime_reset_request = 1;
}

uint8 Camera_RuntimeResetPending(void)
{
    return vision_runtime_reset_request;
}

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);

    if (exti_flag_get(ERU_CH3_REQ6_P02_0))
    {
        if (image_processing || mt9v03x_finish_flag)
        {
            exti_flag_clear(ERU_CH3_REQ6_P02_0);
        }
        else
        {
            camera_vsync_handler();
        }
    }

    if (exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}

IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();
}

//出界保护
uint8 num=0;
uint8 numchujie=0;
uint8 Protect_Without_Road(uint8 StartLine,uint8 EndLine,uint8 StartRow,uint8 EndRow)
{
    if(ImageFlag.chujie_Stop_1==1)
    {
        return 1;
    }
    int i,j;
    int BlackDotNum=0;                                                                        //原定义是uint16类型，似乎一上电就卡在这个的死循环了
    for(i=StartLine;i<EndLine;i++)
    {
        for(j=StartRow;j<EndRow;j++)
        {
            if(Pixle[i][j]==0)
            {
                BlackDotNum++;
            }
        }
    }
    if(BlackDotNum >= (int)(0.75*(EndLine-StartLine)*(EndRow-StartRow)))                        //上面原写法：if(BlackDotNum >= (int)0.7*(EndLine-StartLine)*(EndRow-StartRow))，少了个括号因为在强转之前没有括起来，所以吧0.7强转为0，最后的结果就是陷入死循环
    {
        num++;
        if(num>numchujie)
        ImageFlag.chujie_Stop_1=1;

        return 1;
    }
    else
    {
        return 0;
    }
}
//大津法（二值化处理） 一种能够算出一幅图像最佳的那个分割阈值的一种算法
uint8_t GetOSTU(uint8 image[MT9V03X_H][MT9V03X_W],int height,int wight)                                                       //透视没毛病版本（不是余xia的   ）
{
    int16_t i,j;
    uint32_t Amount = 0;
    uint32_t PixelBack = 0;
    uint32_t PixelIntegralBack = 0;
    uint32_t PixelIntegral = 0;
    uint32_t PixelIntegralFore = 0;
    uint32_t PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma;
    int16_t MinValue, MaxValue;
    uint8_t Threshold = 0;
    uint8_t HistoGram[256];

    for (j = 0; j < 256; j++)  HistoGram[j] = 0;

    for (j = Cut_high_top; j < height-Cut_high_bottom; j++)
    {
        for (i = Cut_width_half; i < wight-Cut_width_half; i++)
        {
            HistoGram[ image[j][i]]++;
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ;

    if (MaxValue == MinValue)     return MaxValue;
    if (MinValue + 1 == MaxValue)  return MinValue;

    for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];

    PixelIntegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        PixelIntegral += HistoGram[j] * j;
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];
        PixelFore = Amount - PixelBack;
        OmegaBack = (double)PixelBack / Amount;
        OmegaFore = (double)PixelFore / Amount;
        PixelIntegralBack += HistoGram[j] * j;
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;
        MicroBack = (double)PixelIntegralBack / PixelBack;
        MicroFore = (double)PixelIntegralFore / PixelFore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
  return Threshold;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           get_Threshold  //指针
//  @brief          优化之后的的大津法。大津法就是一种能够算出一幅图像最佳的那个分割阈值的一种算法。
//  @brief          这个东西你们可以如果实在不能理解就直接拿来用，什么参数都不用修改，只要没有光照影响，那么算出来的这个阈值就一定可以得到一幅效果还不错的二值化图像。
//  @parameter      image  原始的灰度图像数组
//  @parameter      clo    图像的宽（图像的列）
//  @parameter      row    图像的高（图像的行）
//  @return         uint8
//  @time           2022年2月17日
//  @Author
//  Sample usage:   Threshold = Threshold_deal(Image_Use[0], 80, 60); 把存放60行80列的二维图像数组Image_Use传进来，求出这幅图像的阈值，并将这个阈值赋给Threshold。
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8 get_Threshold(uint8* image[][LCDW],uint16 col, uint16 row)
{
  #define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8 threshold = 0;
  for (i = 0; i < GrayScale; i++)                    //先把pixelCount和pixelPro两个数组元素全部赋值为0
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;

  /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/
  for (i = 0; i < height; i += 1)                   //遍历图像的每一行，从第零行到第59行。
  {
    for (j = 0; j < width; j += 1)                  //遍历图像的每一列，从第零列到第79列。
    {
      pixelCount[*image[i][j]]++;       //将当前的像素点的像素值（灰度值）作为计数数组的下标。
      gray_sum += *image[i][j];         //计算整幅灰度图像的灰度值总和。
    }
  }

  /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/



  /**************************************计算每个像素值（灰度值）在整幅灰度图像中所占的比例*************************************************/
  for (i = 0; i < GrayScale; i++)
  {
      pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  /**************************************计算每个像素值（灰度值）在整幅灰度图像中所占的比例**************************************************/



  /**************************************开始遍历整幅图像的灰度值（0-255），这一步也是大津法最难理解的一步***************************/
  /*******************为什么说他难理解？因为我也是不理解！！反正好像就是一个数学问题，你可以理解为数学公式。***************************/
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < GrayScale; j++)
  {
    w0 += pixelPro[j];                          //求出背景部分每个灰度值的像素点所占的比例之和，即背景部分的比例。
    u0tmp += j * pixelPro[j];

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;                            //背景平均灰度
    u1 = u1tmp / w1;                            //前景平均灰度
    u = u0tmp + u1tmp;                          //全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = (uint8)j;
    }
    if (deltaTmp < deltaMax)
    {
      break;
    }
  }
  /**************************************开始遍历整幅图像的灰度值（0-255），这一步也是大津法最难理解的一步***************************/
  /*******************为什么说他难理解？因为我也是不理解！！反正好像就是一个数学问题，你可以理解为数学公式。***************************/

  return threshold;                             //把上面这么多行代码算出来的阈值给return出去。
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BinaryImage
//  @brief          灰度图像二值化处理
//  @brief          整体思路就是：先调用Get_Threshold（）函数得到阈值，然后遍历原始灰度图像的每一个像素点，用每一个像素点的灰度值来跟阈值计较。
//  @brief          大于阈值的你就把它那个像素点的值赋值为1（记为白点），否则就赋值为0（记为黑点）。当然你可以把这个赋值反过来，只要你自己清楚1和0谁代表黑谁代表白就行。
//  @brief          所以我前面提到的60*80现在你们就应该明白是什么意思了吧！就是像素点嘛，一行有80个像素点，一共60行，也就是压缩后的每一幅图像有4800个像素点。
//  @parameter      void
//  @return         void
//  @time           2022年2月17日
//  @Author
//  Sample usage:   Get_BinaryImage();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BinaryImage(void)
{

  Threshold = GetOSTU(mt9v03x_image,MT9V03X_H,MT9V03X_W);  //这里是一个函数调用，通过该函数可以计算出一个效果很不错的二值化阈值。
  uint8 i, j = 0;
  int line_dif=-2;
//  if(ImageFlag.Cross_Flag==0)
//  {
      for (i = 0; i <LCDH; i++)                                //遍历二维数组的每一行
      {
        for (j = 0; j <LCDW; j++)                              //遍历二维数组的每一列
        {
          if (mt9v03x_image[i+Cut_high_top][j+Cut_width_half+line_dif] > Threshold)                     //如果这个点的灰度值大于阈值Threshold - lightadd(光线补偿)
              Pixle[i][j] = 255;                                //那么这个像素点就记为白点
          else                                                  //如果这个点的灰度值小于阈值Threshold
              Pixle[i][j] = 0;                                  //那么这个像素点就记为黑点
        }
      }
//  }
//  else
//  {
//      for (i = 0; i < LCDH; i++)                                //遍历二维数组的每一行
//      {
//        for (j = 0; j < LCDW; j++)                              //遍历二维数组的每一列
//        {
//          if (*Image_Use[i][j] > Threshold&&j>=17&&j<LCDW-17)  ///17                   //如果这个点的灰度值大于阈值Threshold
//              Pixle[i][j] = 255;                                //那么这个像素点就记为白点
//          else                                                  //如果这个点的灰度值小于阈值Threshold
//              Pixle[i][j] = 0;                                  //那么这个像素点就记为黑点
//        }
//      }

//  }
}



//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Border_And_SideType
//  @brief          得到边线和边线的类型，我在这里给边线分为了三种类型，T类型、W类型和H类型。分别代表正常边线、无边边线和大跳变边线。
//  @brief          和上面一样，要想看懂我的逻辑，前提是要搞懂T、W、H三种类型的边线到底是什么样子的。
//  @parameter      p        指向传进来数组的一个指针变量。
//  @parameter      type     只能是L或者是R，分别代表扫左边线和扫右边线。
//  @parameter      L        扫描的区间下限 ，也就是从哪一列开始扫。
//  @parameter      H        扫描的区间上限 ，也就是一直扫到哪一列。
//  @parameter      Q        是一个结构体指针变量，自己跳过去看看这个结构体里面的成员。
//  @time           2022年2月20日
//  @Author
//  Sample usage:   Get_SideType_And_Border(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);
//  Sample usage:   从PicTemp(PicTemp是个指针，指向一个数组)的IntervalLow列开始扫，扫到IntervalHigh列，然后把得到的边线所在的列和边线类型记录到JumpPoint结构体中。
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//T类型、W类型和H类型。分别代表正常边线、无边边线和大跳变边线。
void Get_Border_And_SideType_MidWhite(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)
{

  int i = 0;
  if (type == 'L')                                      //如果Type是L(Left),则扫描左边线。
  {
    for (i = H; i >= L; i--)                            //从右往左扫
    {
      if (*(p + i) == 255 && *(p + i - 1) != 255)       //如果有黑白跳变    1是白 0是黑
      {
        Q->point = i;                                   //那就把这个列记录下来作为左边线
        Q->type = 'T';                                  //并且把这一行当作是正常跳变，边线类型记为T，即边线正常。
        break;                                          //找到了就跳出循环不找了
      }
      else if (i == (L + 1))                            //要是扫到最后都没找到
      {
          if(*(p+(L+H)/2)!=0)
          {
              Q->point = (H+L)/2;                       //那么就认为这一行的左边线是传进来扫描区间的中点。
              Q->type = 'W';                            //并且把这一行当作是非正常跳变，边线类型记为W，即无边行。
              break;
          }
          else
          {
              Q->point = H;
              Q->type = 'H';
              break;
          }
      }
    }
  }
  else if (type == 'R')                                 //如果Type是R(Right),则扫描右边线。
  {
    for (i = L; i <= H; i++)                            //从左往右扫
    {
      if (*(p + i) == 255 && *(p + i + 1) != 255)       //如果有黑白跳变    1是白 0是黑
      {
        Q->point = i;                                   //那就把这个列记录下来作为右边线
        Q->type = 'T';                                  //并且把这一行当作是正常跳变，边线类型记为T，即边线正常。
        break;                                          //找到了就跳出循环不找了
      }
      else if (i == (H - 1))                            //要是扫到最后都没找到
      {
          if(*(p+(L+H)/2)!=0)
          {
              Q->point = (L+H)/2;                       //那么就认为这一行的右边线是传进来扫描区间的中点。
              Q->type = 'W';                            //并且把这一行当作是非正常跳变，边线类型记为W，即无边行。
              break;
          }
          else
          {
              Q->point = L;
              Q->type = 'H';
              break;
          }
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BaseLine
//  @brief          用遍历的方法得到图像最底下五行的边线和中线信息。这五行边线和中线信息的准确度非常的重要，直接影响到整幅图像的处理结果。
//  @brief          Get_BaseLine函数是下面Get_AllLine函数的基础和前提，Get_AllLine函数是以Get_BaseLine为基础的。看名字应该也能看出对吧，一个叫得到基础线，一个叫得到所有线。
//  @brief          Get_BaseLine函数和Get_All Line函数加在一起，也就组成了我优化之后的搜边线算法。
//  @parameter      void
//  @time           2022年2月21日
//  @Author
//  Sample usage:   Get_BaseLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    /**************************************遍历搜索图像最底行（79行）左右边线从而确定中线的过程 ********************************************************************/
    /****************************************************Begin*****************************************************************************/

    PicTemp = Pixle[LCDH-1];                                                //让PicTemp这个指针变量指向图像数组的Pixle[79]
    for (Xsite = ImageSensorMid; Xsite < LCDW-1; Xsite++)                   //假设83是中心列，从中心列开始一列一列的往右边搜索右边线
    {
      if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)           //如果连续出现了两个黑点，说明找到了边线。
      {
        BottomBorderRight = Xsite;                                          //把这一列记录下来作为这一行的右边线
        break;                                                              //跳出循环
      }
      else if (Xsite == LCDW-2)                                             //如果找到了第166列都还没出现黑点，说明这一行的边线有问题。
      {
        BottomBorderRight = LCDW-1;                                         //所以我这里的处理就是，直接假设图像最右边的那一列（第167列）就是这一行的右边线。
        break;                                                              //跳出循环
      }
    }

    for (Xsite = ImageSensorMid; Xsite > 0; Xsite--)                        //假设83是中心列，从中心列开始一列一列的往左边搜索左边线
    {
      if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)           //如果连续出现了两个黑点，说明找到了边线。
      {
        BottomBorderLeft = Xsite;                                           //把这一列记录下来作为这一行的左边线
        break;                                                              //跳出循环
      }
      else if (Xsite == 1)                                                  //如果找到了第1列都还没出现黑点，说明这一行的边线有问题。
      {
        BottomBorderLeft = 0;                                               //所以我这里的处理就是，直接假设图像最左边的那一列（第0列）就是这一行的左边线。
        break;                                                              //跳出循环
      }
    }

    BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;               //根据左右边界计算出第79行的中线
    ImageDeal[LCDH-1].LeftBorder = BottomBorderLeft;                        //把第79行的左边界存储进数组，注意看ImageDeal这个数字的下标，是不是正好对应79。
    ImageDeal[LCDH-1].RightBorder = BottomBorderRight;                      //把第79行的右边界存储进数组，注意看ImageDeal这个数字的下标，是不是正好对79。
    ImageDeal[LCDH-1].Center = BottomCenter;                                //把第79行的中线存储进数组，    注意看ImageDeal这个数字的下标，是不是正好对应79。
    ImageDeal[LCDH-1].Wide = BottomBorderRight - BottomBorderLeft;          //把第79行的赛道宽度存储数组，注意看ImageDeal这个数字的下标，是不是正好对应79。
    ImageDeal[LCDH-1].IsLeftFind = 'T';                                     //记录第79行的左边线类型为T，即正常找到左边线。
    ImageDeal[LCDH-1].IsRightFind = 'T';                                    //记录第79行的右边线类型为T，即正常找到右边线。

    /****************************************************End*******************************************************************************/
    /**************************************遍历搜索图像最底行（79行）左右边线从而确定中线的过程 ********************************************************************/



    /**************************************在第79行中线已经确定的情况下确定78-75这四行中线的过程 ******************************************/
    /****************************************************Begin*****************************************************************************/
    /*
         * 下面几行的的搜线过程我就不再赘述了，根据我的注释把第79行的搜线过程理解好，
         * 那么78到75行的搜线就完全没问题，是一模一样的逻辑和过程。
     */
    for (Ysite = LCDH-2; Ysite > LCDH-6; Ysite--)
    {
        PicTemp = Pixle[Ysite];
        for(Xsite = ImageDeal[Ysite + 1].Center; Xsite < LCDW-1;Xsite++)
        {
          if(*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            break;
          }
          else if (Xsite == LCDW-2)
          {
            ImageDeal[Ysite].RightBorder = LCDW-1;
            break;
          }
        }

        for (Xsite = ImageDeal[Ysite + 1].Center; Xsite > 0;Xsite--)
        {
          if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)
          {
            ImageDeal[Ysite].LeftBorder = Xsite;
            break;
          }
          else if (Xsite == 1)
          {
            ImageDeal[Ysite].LeftBorder = 0;
            break;
          }
        }

        ImageDeal[Ysite].IsLeftFind  = 'T';
        ImageDeal[Ysite].IsRightFind = 'T';
        ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2;
        ImageDeal[Ysite].Wide   = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    }

    /****************************************************End*****************************************************************************/
    /**************************************在第69行中线已经确定的情况下确定68-64这四行中线的过程 ****************************************/
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_AllLine
//  @brief          在Get_BaseLine的基础上，针对部分特殊情况，利用一些特殊的处理算法得到剩余行的边线和中线信息。
//  @brief          这个函数应该是目前为止我代码里面最难理解的一部分了，一定要花大量时间静下心来去看，看的时候脑子里面要有那个二值化黑白图像。
//  @brief          如果你能一边想着二值化黑白图像，一边来思考我代码的逻辑的话，很多地方你就能慢慢理解了，不要光盯着那个代码一直看啊，那样是没用的，切忌！
//  @brief          多动脑思考，我能想出来，你们肯定也可以的。这个过程会很枯燥，但是你都理解清楚了之后，你的车车就已经可以跑直道和弯道了。
//  @parameter      void
//  @time           2023年2月21日
//  @Author
//  Sample usage:   Get_AllLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_AllLine(void)
{
  uint8 L_Found_T  = 'F';                                                //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_L_line = 'F';                                                //找到这一帧图像的基准左斜率，为什么这里要置为F，看了下面的代码就知道了。
  uint8 R_Found_T  = 'F';                                                //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_R_line = 'F';                                                //找到这一帧图像的基准右斜率，为什么这里要置为F，看了下面的代码就知道了。
  float D_L = 0;                                                         //左边线延长线的斜率
  float D_R = 0;                                                         //右边线延长线的斜率
  int ytemp_W_L;                                                         //记住首次左丢边行
  int ytemp_W_R;                                                         //记住首次右丢边行
//  ExterMidFlag = 0;                                                    //midblack重定义的标志位
  ExterMid_Row=0;                                                        //midblack重定义的行数
  ExterMid_Det=0;                                                        //midblack底边几行计算出来的斜率
  ExterMid_num=0;                                                        //记录midblack判断次数
  ExtenRFlag = 0;                                                        //标志位清0
  ExtenLFlag = 0;                                                        //标志位清0
  Cross_Corner_L = 0;                                                   //下拐点只使用本帧找到的位置
  Cross_Corner_R = 0;
  Midblack_aim_point=0;                                                  //通过ExterMid_Det计算出来的在ExterMid_Row的标准点
  ImageStatus.OFFLine=3;                                                 //这个结构体成员我之所以在这里赋值，是因为我ImageStatus结构体里面的成员太多了，但是暂时又只用到了OFFLine，所以我在哪用到它就在哪赋值。
  ImageStatus.Miss_Right_lines = 0;
  ImageStatus.WhiteLine = 0;
  ImageStatus.Miss_Left_lines = 0;
//  int temp_mid=0;
  for (Ysite = LCDH-6 ; Ysite > ImageStatus.OFFLine; Ysite--)            //前5行在Get_BaseLine()中已经处理过了，现在从55行处理到自己设定的不处理行OFFLine。
  {                                                                      //因为太前面的图像可靠性不搞，所以OFFLine的设置很有必要，没必要一直往上扫到第0行。
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                       // JumpPoint[0]代表左边线，JumpPoint[1]代表右边线。
    /******************************扫描本行的右边线******************************/
    IntervalLow  = ImageDeal[Ysite + 1].RightBorder-ImageScanInterval;//从上一行的右边线加减Interval对应的列开始扫描本行，Interval一般取5，当然你为了保险起见可以把这个值改的大一点。
    IntervalHigh = ImageDeal[Ysite + 1].RightBorder+ImageScanInterval;//正常情况下只需要在上行边线左右5的基础上（差不多10列的这个区间）去扫线，一般就能找到本行的边线了，所以这个值其实不用太大。
    LimitL(IntervalLow);                                             //这里就是对传给GetJumpPointFromDet()函数的扫描区间进行一个限幅操作。
    LimitH(IntervalHigh);                                            //假如上一行的边线是第2列，那你2-5=-3，-3是不是就没有实际意义了？怎么会有-3列呢？
    Get_Border_And_SideType_MidWhite(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);  //扫线用的一个子函数，自己跳进去看明白逻辑。
  /******************************扫描本行的右边线******************************/

  /******************************扫描本行的左边线******************************/
    IntervalLow =ImageDeal[Ysite + 1].LeftBorder-ImageScanInterval;  //从上一行的左边线加减Interval对应的列开始扫描本行，Interval一般取5，当然你为了保险起见可以把这个值改的大一点。
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder+ImageScanInterval; //正常情况下只需要在上行边线左右5的基础上（差不多10列的这个区间）去扫线，一般就能找到本行的边线了，所以这个值其实不用太大。
    LimitL(IntervalLow);                                             //这里就是对传给GetJumpPointFromDet()函数的扫描区间进行一个限幅操作。
    LimitH(IntervalHigh);                                            //假如上一行的边线是第2列，那你2-5=-3，-3是不是就没有实际意义了？怎么会有-3列呢？
    Get_Border_And_SideType_MidWhite(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);  //扫线用的一个子函数，自己跳进去看明白逻辑。
  /******************************扫描本行的左边线******************************/

    if (JumpPoint[0].type =='W')   //改成l也                                           //如果本行的左边线属于不正常跳变，即这10个点都是白点。
    {
//        if(Ysite%3==0&&ImageFlag.Cross_Flag)
//            ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder+1;//那么本行的左边线就采用上一行的边线。
//        else
            ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;
    }
    else                                                                             //如果本行的左边线属于T或者是H类别
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                              //那么扫描到的边线是多少，我就记录下来是多少。
    }

    if (JumpPoint[1].type == 'W')                                                    //如果本行的右边线属于不正常跳变，即这10个点都是白点。
    {
//        if(Ysite%3==0&&ImageFlag.Cross_Flag)
//            ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder-1;//那么本行的右边线就采用上一行的边线。
//        else
            ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;    }
    else                                                                             //如果本行的右边线属于T或者是H类别
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                             //那么扫描到的边线是多少，我就记录下来是多少。//H这样的话就能够逐渐趋近于边线
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                  //记录本行找到的左边线类型，是T？是W？还是H？这个类型后面是有用的，因为我还要进一步处理。
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;                                //记录本行找到的右边线类型，是T？是W？还是H？这个类型后面是有用的，因为我还要进一步处理。


  /*
        下面就开始对W和H类型的边线分别进行处理， 为什么要处理？
        如果你看懂了GetJumpPointFromDet函数逻辑，明白了T W H三种类型分别对应什么情况，
        那你就应该知道W和H类型的边线都属于非正常类型，那我是不是要处理？
        这一部分的处理思路需要自己花大量时间好好的去琢磨，我在注释这里没法给你说清楚的。，
        实在想不通就来问我吧！
  */
    if (( ImageDeal[Ysite].IsLeftFind == 'H' || ImageDeal[Ysite].IsRightFind == 'H'))
      {
        /**************************处理左边线的大跳变***************************/
        if (ImageDeal[Ysite].IsLeftFind == 'H')
        {
          for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);Xsite <= (ImageDeal[Ysite].RightBorder - 1);Xsite++)//从左往右                                                           //左右边线之间重新扫描
          {
            if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0))//黑白跳变
            {
              ImageDeal[Ysite].LeftBorder =Xsite;
              ImageDeal[Ysite].IsLeftFind = 'T';
              break;
            }
            else if(*(PicTemp + Xsite) != 0)//如果找的第一个点就是白点，就你懂的
            {
                ImageDeal[Ysite].IsLeftFind = 'T';
                break;
            }//??为什么这里不加ImageDeal[Ysite].IsLeftFind = 'T';还是想保留大跳变线后续处理
            else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))//注意没有修改边界的值
            {
                  ImageDeal[Ysite].IsLeftFind = 'T';
                  break;
            }
          }
        }
        /**************************处理左边线的大跳变***************************/


        /**************************处理右边线的大跳变***************************/
        if (ImageDeal[Ysite].IsRightFind == 'H')
        {
          for (Xsite = (ImageDeal[Ysite].RightBorder - 1);Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--)//从右往左
          {
            if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0))//黑白跳变
            {
              ImageDeal[Ysite].RightBorder =Xsite;
              ImageDeal[Ysite].IsRightFind = 'T';
              break;
            }
            else if (*(PicTemp + Xsite) != 0)
            {
                ImageDeal[Ysite].IsLeftFind = 'T';
                break;
            }
            else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
            {
  //            ImageDeal[Ysite].RightBorder = Xsite;
              ImageDeal[Ysite].IsRightFind = 'T';
              break;
            }
           }
         }
       }
      /**************************处理右边线的大跳变***************************/



 /************************************重新确定无边行（即W类）的边界****************************************************************/
    int ysite = 0;
//    int i=0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;
    int ytemp_W_R_Border=0;
    int ytemp_W_L_Border=0;
    uint8 Block_miss_left_num=0;
    uint8 Block_miss_right_num=0;
    /**************************处理右边线的无边行***************************/
    if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite > 10&&Ysite < LCDH-5)        //控制行夹在这个范围才进行处理
    {
      if (Get_R_line == 'F')
      {
        Get_R_line = 'T';

        ytemp_W_R_Border=ImageDeal[Ysite].RightBorder;
        ytemp_W_R = Ysite;
        for(ysite = Ysite;ysite<=LCDH-2;ysite++)//从W行开始往下找                                //打擂台求出突出的尖尖的横坐标  第一次出现无边行才要哦
        {
            if(ImageDeal[ysite+1].RightBorder>ImageDeal[ysite].RightBorder)//理想情况，即正常情况
            {
                ytemp_W_R_Border=ImageDeal[ysite+1].RightBorder;                //拐点横坐标
                ytemp_W_R=ysite+1;                                              //拐点纵坐标
                break;
            }
        }

        Cross_Corner_R=ytemp_W_R;                                               //记录拐点纵坐标

//        if(ImageDeal[ytemp_W_R-2].IsRightFind == 'T'&&ImageDeal[ytemp_W_R-1].IsRightFind == 'T'
//           &&(ImageDeal[ytemp_W_R-2].RightBorder==ImageDeal[ytemp_W_R-1].RightBorder))//基于观察图像给出的条件，加这个是为了限制弯道也进行下面的判断,只能是缩小误判概率，不能完全
//        {
//            Continue_R_flag=0;
//        }

//        if(Continue_R_flag)
//        {
            for (ysite = ytemp_W_R + 1; ysite < ytemp_W_R + 15; ysite++)
            {//zwf
              if(ysite==LCDH)continue;  //为啥不直接break                                            //加到80时，超过数组大小，进入下次循环
              if (ImageDeal[ysite].IsRightFind =='T')
              {
                  R_found_point++;
              }
            }
//        }

        if (R_found_point >8)                                                   //限制车不在太近或者太远进行斜率计算，适合得位置进行计算/zwf
        {
          D_R = ((float)(ImageDeal[ytemp_W_R + R_found_point].RightBorder - ImageDeal[ytemp_W_R + 3].RightBorder)) /((float)(R_found_point - 3));//记录在Ysite+2位置，所以这里是3？
          if (D_R > 0/*&&Straight_Judge(2,ytemp_W_R,ytemp_W_R+8)<3*/)
          {
            R_Found_T ='T';
            for(int i=Ysite;i<ytemp_W_R;i++)
            {
//                ImageDeal[i].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - i);
//                ImageDeal[i].RightBorder=ImageDeal[ytemp_W_R].RightBorder;
                ImageDeal[i].IsRightFind = 'W';
            }
          }
          else
          {
            R_Found_T = 'F';
            if (D_R < 0)
            {
                ExtenRFlag = 'F';
            }
          }
        }
        if(Cross_Corner_R<50)//十字右下拐点纵坐标
        {
            for(int ysite=Cross_Corner_R;ysite<Cross_Corner_R+5;ysite++)
            {
                if(ImageDeal[ysite].IsLeftFind =='W')
                {
                    Block_miss_left_num++;
                }
            }
        }
      }
      if (R_Found_T == 'T'&&Block_miss_left_num>1)       //正常是要十字才不线，不加ImageFlag.image_element_rings==0的话圆环会误判
      {
//        ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //如果找到了 那么以基准行做延长线//这里补线了的画后面的无边行就能稳定的找到

      }
      LimitL(ImageDeal[Ysite].RightBorder);//限幅
      LimitH(ImageDeal[Ysite].RightBorder);//限幅
    }
    /**************************处理右边线的无边行***************************/


    /**************************处理左边线的无边行***************************/
    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < LCDH-5 )
    {
      if (Get_L_line == 'F')
      {
        Get_L_line = 'T';

        ytemp_W_L_Border=ImageDeal[Ysite].LeftBorder;
        ytemp_W_L = Ysite;
        for(ysite = Ysite;ysite<=LCDH-2;ysite++)                //打擂台求出突出的尖尖的横坐标
        {
            if(ImageDeal[ysite+1].LeftBorder<ImageDeal[ysite].LeftBorder)
            {
                ytemp_W_L_Border=ImageDeal[ysite+1].LeftBorder; //拐点横坐标
                ytemp_W_L=ysite+1;                              //拐点纵坐标
                break;
            }
        }
        Cross_Corner_L=ytemp_W_L;                               //记录拐点纵坐标

//        if(ImageDeal[ytemp_W_L-2].IsLeftFind == 'T'&&ImageDeal[ytemp_W_L-1].IsLeftFind == 'T'
//           &&(ImageDeal[ytemp_W_L-2].LeftBorder==ImageDeal[ytemp_W_L-1].LeftBorder))//基于观察图像给出的条件，加这个是为了限制弯道也进行下面的判断,只能是缩小误判概率，不能完全
//        {
//            Continue_L_flag=0;
//        }

//        if(Continue_L_flag)
//        {
            for (ysite = ytemp_W_L + 1; ysite < ytemp_W_L + 15; ysite++)
            {
    //          if(ysite>79)break;
              if(ysite==LCDH)continue;
              if (ImageDeal[ysite].IsLeftFind == 'T')
                {
                  L_found_point++;
                }
            }
//        }

        if (L_found_point > 8)                                  //找到基准斜率边  做延长线重新确定无边
        {
          D_L = ((float)(ImageDeal[ytemp_W_L + 3].LeftBorder -ImageDeal[ytemp_W_L + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
          if (D_L > 0/*&&Straight_Judge(1,ytemp_W_R,ytemp_W_R+8)<3*/)
          {
            L_Found_T = 'T';

            for(int i=Ysite;i<ytemp_W_L;i++)
            {
//                ImageDeal[i].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - i);
//                ImageDeal[i].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder;
                ImageDeal[i].IsLeftFind = 'W';                //要不要补充进来作为十字是否要处理的判别呢
            }

          }
          else
          {
            L_Found_T = 'F';
            if (D_L < 0)
            {
                ExtenLFlag = 'F';
            }
          }
        }
        if(Cross_Corner_L<55)//十字左底部拐点纵坐标
        {
            for(int ysite=Cross_Corner_L;ysite<Cross_Corner_L+5;ysite++)
            {
                if(ImageDeal[ysite].IsRightFind =='W')
                {
                    Block_miss_right_num++;
                }
            }
        }
      }

      if (L_Found_T == 'T'&&Block_miss_right_num>1)//正常是要十字才不线，不加ImageFlag.image_element_rings==0的话圆环会误判
      {
//          ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);
      }

      LimitL(ImageDeal[Ysite].LeftBorder);//限幅
      LimitH(ImageDeal[Ysite].LeftBorder);//限幅
    }

    /**************************处理左边线的无边行***************************/


    /************************************都处理完之后，其他的一些数据整定操作*************************************************/
//      if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
//      {
//          ImageStatus.WhiteLine++;                              //要是左右都无边，丢边数+1
//      }
     if (ImageDeal[Ysite].IsLeftFind == 'W'&&Ysite<LCDH-5)
     {
          ImageStatus.Miss_Left_lines++;//左数
     }
     if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite<LCDH-5)
     {
          ImageStatus.Miss_Right_lines++;//右数
     }

      LimitL(ImageDeal[Ysite].LeftBorder);//限幅
      LimitH(ImageDeal[Ysite].LeftBorder);//限幅
      LimitL(ImageDeal[Ysite].RightBorder);//限幅
      LimitH(ImageDeal[Ysite].RightBorder);//限幅

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

      if (ImageDeal[Ysite].Wide <= 22)//赛道宽度<12时就结束搜线
      {
          ImageStatus.OFFLine = Ysite + 1;                      //图像顶边加1即跳出循环，搜线结束
          break;
      }
      else if (ImageDeal[Ysite].RightBorder <= 10||ImageDeal[Ysite].LeftBorder >= LCDW-10)//左右边线的位置太过异常也结束搜线
      {
          ImageStatus.OFFLine = Ysite + 1;                      //记录更新到的offLine
          break;
      }
//      LimitP_L(Prospect_line);
      /************************************都处理完之后，其他的一些数据整定操作*************************************************/
  }//该括号结束整个搜线过程
  for(int Ysite=LCDH-6;Ysite>ImageStatus.OFFLine+1;Ysite--)//不要放这里吧，放斜入判断那
  {
      if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
      {
          ImageStatus.WhiteLine++;                              //要是左右都无边，丢边数+1
      }
  }
  //    ips200_show_string(10,160,"B:");
  //    ips200_show_string(30,160,"R:");
  //    ips200_show_string(50,160,"L:");

  //  ips200_show_uint(10,180,ImageStatus.WhiteLine,3);
  //  ips200_show_uint(30,180,ImageStatus.Miss_Right_lines,3);
  //  ips200_show_uint(50,180,ImageStatus.Miss_Left_lines,3);
}
/***************限制在区间取值**************/
int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Flag_init
//  @brief          标志位清0
//  @parameter      void
//  @time           2023年2月19日
//  @Author
//  Sample usage:   Flag_init();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void Flag_init(void)
{
    ImageFlag.Search_error_flag=0;
    ImageFlag.Bend_Road = 0;
//    ImageFlag.Ramp = 0;
    ImageFlag.Cross_Flag=0;
    ImageFlag.Zebra_Flag = 0;
    ImageFlag.Zebra_Counter =0;
    ImageFlag.image_element_rings = 0;
    ImageFlag.image_element_rings_flag = 0;
    ImageFlag.straight_xie = 0;
    ImageFlag.straight_long = 0;
    ImageFlag.ring_big_small = 0;
    ImageFlag.Stop=0;
//    ImageFlag.LongestLine_Found_flag=0;
}

/*------------------------------------------------------------------------------------------------------------------
 * @name           NormalMid_Update
 *
 * @brief          计算正常中线（
 *
 * @details        这个函数是在 Get_AllLine() 已经算出ImageDeal[y].Center 的基础上，提取出一份“正常中线”。
 *                 
 *
 *                 主要做四件事：
 *                 1. 在元素处理之前，把每一行的普通中线保存到 NormalMidLine[]；
 *                 2. 取图像底部附近若干行，计算近端中线，保证稳定性；
 *                 3. 取前瞻行附近若干行，计算远端中线，提供前方弯道预判；
 *                 4. 将近端和远端中线按比例加权，得到 NormalMidValue，
 *                    再计算 NormalMidError，供控制部分使用。
 *
 * @note           
 *
 * @parameter      void
 * @return         void
 *------------------------------------------------------------------------------------------------------------------*/
#define NORMAL_MID_EDGE_MARGIN     2
#define NORMAL_MID_MIN_WIDTH       20
#define NORMAL_MID_MAX_WIDTH       140
#define NORMAL_MID_FAR_DIFF_LIMIT  40
void NormalMid_Update(void)
{
    int i;
    int x, y;
    int white_count = 0;
    int valid_start;

    /* 近端中线累加值和有效行数 */
    int near_sum = 0;
    int near_cnt = 0;

    /* 远端中线累加值和有效行数 */
    int far_sum = 0;
    int far_cnt = 0;

    /* 远端搜索的起始行和结束行 */
    int far_start;
    int far_end;

    /* 近端和远端的平均中线值 */
    int near_value = 0;
    int far_value = 0;

    int far_diff;

    NormalMidValid = 0;
    NormalMidValue = ImageSensorMid;
    NormalMidError = 0;

    for (i = 0; i < LCDH; i++)
    {
        NormalMidLine[i] = -1;
    }

    /* 1. 保存普通中线
    * 必须在元素处理之前保存，避免 Center 被十字、环岛等逻辑修改
    */
    // 统计图像下方12行的白色像素数量
    for (y = LCDH - 12; y < LCDH; y++)
    {
        for (x = 0; x < LCDW; x++)
        {
            if (Pixle[y][x] == 255)
            {
                white_count++;
            }
        }
    }

    // 下方几乎没有白点，认为没有赛道
    if (white_count < 150)
    {
        return;
    }

    /* OFFLine 以上没有经过 Get_AllLine() 填充，不能据此判整帧无效。 */
    valid_start = (int)ImageStatus.OFFLine + 1;

    // 只保存扫线有效区域内的合法中点，异常行保持为 -1
    for (i = valid_start; i < LCDH; i++)
    {
        int left = ImageDeal[i].LeftBorder;
        int right = ImageDeal[i].RightBorder;
        int wide = right - left;
        int center = (left + right) / 2;

        // 排除贴到图像边缘的假边界
        if (left > NORMAL_MID_EDGE_MARGIN &&
            right < LCDW - NORMAL_MID_EDGE_MARGIN &&
            right > left &&
            wide >= NORMAL_MID_MIN_WIDTH &&
            wide <= NORMAL_MID_MAX_WIDTH &&
            center >= 0 &&
            center < LCDW)
        {
            NormalMidLine[i] = center;
        }
    }

    /* 2. 计算近端中线
     * 取图像底部5行，靠近车头的部分通常最稳定
     */
    // 近端取图像底部5行
    for (i = LCDH - 5; i < LCDH; i++)
    {
        if (NormalMidLine[i] >= 0)
        {
            near_sum += NormalMidLine[i];
            near_cnt++;
        }
    }
    // 底部5行至少要有3行有效
    if (near_cnt < 3)
    {
        NormalMidValid = 0;
        NormalMidError = 0;
        return;
    }

    near_value = near_sum / near_cnt;

    /* 3. 计算远端中线
    远端取前瞻附近5行
     */
    far_start = Prospect_line;
    far_end = Prospect_line + 2;

    if (far_end >= LCDH)
    {
        far_end = LCDH - 1;
    }

    if (far_start < 0)
    {
        far_start = 0;
    }


    if (ImageStatus.OFFLine < far_start)
    {
        for (i = far_start; i <= far_end; i++)
        {
            if (NormalMidLine[i] >= 0)
            {
                far_sum += NormalMidLine[i];
                far_cnt++;
            }
        }
    }

    // /* 4. 近端和远端加权，得到最终正常中线 */
    // // 底部5行至少要有3行有效
    // if (near_cnt >= 3)
    // {
    //     near_value = near_sum / near_cnt;

    //     if (far_cnt > 0)
    //     {
    //         far_value = far_sum / far_cnt;
    //         NormalMidValue = (7 * near_value + 3 * far_value) / 10;
    //     }
    //     else
    //     {
    //         NormalMidValue = near_value;
    //     }

    //     NormalMidError = ImageSensorMid - NormalMidValue;
    //     NormalMidValid = 1;
    // }
    // else
    // {
    //     NormalMidValid = 0;
    //     NormalMidError = 0;
    // }
    if (far_cnt >= 3)
    {
        far_value = far_sum / far_cnt;

        // 限制远端对近端的影响，防止中线突然飘
        far_diff = far_value - near_value;

        if (far_diff > NORMAL_MID_FAR_DIFF_LIMIT)
        {
            far_diff = NORMAL_MID_FAR_DIFF_LIMIT;
        }
        else if (far_diff < -NORMAL_MID_FAR_DIFF_LIMIT)
        {
            far_diff = -NORMAL_MID_FAR_DIFF_LIMIT;
        }

        NormalMidValue = near_value + (3 * far_diff) / 10;
    }
    else
    {
        NormalMidValue = near_value;
    }

    NormalMidError = ImageSensorMid - NormalMidValue;
    NormalMidValid = 1;
    
}
/*------------------------------------------------------------------------------------------------------------------
 * @name           Track_Slope_Sample
 *
 * @brief          计算轨道斜率
 *
 * @details
 *
 * @note
 *
 * @parameter      void
 * @return         void
 *------------------------------------------------------------------------------------------------------------------*/
#define SLOPE_NEAR_START   60
#define SLOPE_NEAR_END     66

#define SLOPE_FAR_START    32
#define SLOPE_FAR_END      38


static uint8 Track_Slope_Sample(int16 *sample)
{
    int y;

    int near_sum = 0;
    int near_cnt = 0;

    int far_sum = 0;
    int far_cnt = 0;

    int near_x;
    int far_x;
    int dy;

    int32_t slope;

    // 计算近端平均中线
    for (y = SLOPE_NEAR_START; y <= SLOPE_NEAR_END; y++)
    {
        if (ControlMidLine[y] >= 0 && ControlMidLine[y] < LCDW)
        {
            near_sum += ControlMidLine[y];
            near_cnt++;
        }
    }

    // 计算远端平均中线
    for (y = SLOPE_FAR_START; y <= SLOPE_FAR_END; y++)
    {
        if (ControlMidLine[y] >= 0 && ControlMidLine[y] < LCDW)
        {
            far_sum += ControlMidLine[y];
            far_cnt++;
        }
    }

    // 有效中线点太少，不计算斜率
    if (near_cnt < 2 || far_cnt < 2)
    {
        return 0;
    }

    near_x = near_sum / near_cnt;
    far_x  = far_sum / far_cnt;

    dy = ((SLOPE_NEAR_START + SLOPE_NEAR_END) / 2)
       - ((SLOPE_FAR_START + SLOPE_FAR_END) / 2);

    if (dy <= 0)
    {
        return 0;
    }

    // 斜率放大1000倍，避免控制层频繁用浮点
    slope = (int32_t)(far_x - near_x) * 1000 / dy;

    // 防止异常值把速度控制带飞
    if (slope > 5000)
    {
        slope = 5000;
    }
    else if (slope < -5000)
    {
        slope = -5000;
    }

    *sample = (int16)slope;
    return 1;
}

static void Track_Slope_Publish(uint8 sample_valid, int16 slope)
{
    /* 控制中线有效时才给控制组使用本帧斜率 */
    uint8 valid = ControlMidValid && sample_valid;
    Track_Slope_x1000 = valid ? slope : 0;
    Track_SlopeValid = valid;
}

void Track_Slope_Update(void)
{
    int16 slope = 0;
    uint8 sample_valid = ControlMidValid && Track_Slope_Sample(&slope);

    Track_Slope_Publish(sample_valid, slope);
}



//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Bottom_Line_OTSU
//  @brief          获取底层左右边线
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @return         Bottonline                              底边行选择
//  @time
//  @Author
//  Sample usage:   Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{//Search_Border_OTSU(Pixle,LCDH,LCDW,LCDH-2);

    //寻找左边边界
    for (int Xsite = Col / 2-2; Xsite > 0; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 255 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;//获取底边左边线
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < LCDW-1; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 255 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;//获取底边右边线
            break;
        }
    }
    ImageFlag.Search_error_flag=0;
    if(ImageDeal[Bottonline].RightBoundary-ImageDeal[Bottonline].LeftBoundary<40)//说明斑马线在图像底部了
    {
        ImageFlag.Search_error_flag=1;
    }

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Left_and_Right_Lines
//  @brief          通过sobel提取左右边线
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         无
//  @time
//  @Author
//  Sample usage:   Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------
#define MAX_BOUNDARY_JUMP 12
void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    //定义小人的当前行走状态位置为 上 左 下 右 一次要求 上：左边为黑色 左：上边为褐色 下：右边为色  右：下面有黑色
/*  前进方向定义：
                *   0
                * 3   1
                *   2
*/
/*寻左线坐标规则*/
    int Left_Rule[2][8] = {
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
    };
    /*寻右线坐标规则*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
    };
      int num=0;
    int16_t Left_Ysite = Bottonline;
    int16_t Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    int16_t Left_Rirection = 0;//左边方向
    int16_t Pixel_Left_Ysite = Bottonline;
    int16_t Pixel_Left_Xsite = 0;

    int16_t Right_Ysite = Bottonline;
    int16_t Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    int16_t Right_Rirection = 0;//右边方向
    int16_t Pixel_Right_Ysite = Bottonline;
    int16_t Pixel_Right_Xsite = 0;

    uint8 Ysite = Bottonline;
    ImageStatus.OFFLineBoundary = 5;
    while (1)
    {
            num++;
            if(num>400)
            {
                 ImageStatus.OFFLineBoundary = Ysite;//400次的时候还没扫到截止行就停止扶墙，记录当前的截止行
                break;
            }
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {
            if (Ysite < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;//自减开启扶墙
            }
        }
        /*********左边巡线*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//前方是黑色
            {
                //顺时针旋转90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//前方是白色
            {
                /*计算左前方坐标*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0)
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Left_Rirection  逆时针90度
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }

            }
        }
        /*********右边巡线*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//前方是黑色
            {
                //逆时针旋转90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//前方是白色
            {
                /*计算右前方坐标*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == LCDW-1 )
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Right_Rirection  逆时针90度
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == LCDW-1)
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }

            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 5)//Ysite<80是为了放在底部是斑马线扫描结束  3 && Ysite < 30
        {

            ImageStatus.OFFLineBoundary = Ysite;
            break;
        }

    }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Border_OTSU
//  @brief          通过OTSU获取边线 和信息
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         无
//  @time           2022年10月7日
//  @Author
//  Sample usage:   Search_Border_OTSU(mt9v03x_image, IMAGE_ROW, IMAGE_COL, IMAGE_ROW-8);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{//Search_Border_OTSU(Pixle,LCDH,LCDW,LCDH-2);
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /*封上下边界处理*/
    for (int Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*封左右边界处理*/
    for (int Ysite = 0; Ysite < LCDH; Ysite++)
    {
            ImageDeal[Ysite].LeftBoundary_First = 0;
            ImageDeal[Ysite].RightBoundary_First = LCDW-1;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][LCDW - 1] = 0;
    }
    /********获取底部边线*********/
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    /********获取左右边线*********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);



    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if (ImageDeal[Ysite].RightBoundary > LCDW - 3)
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_ExtensionLine
//  @brief          这个函数的作用就是补线！
//  @brief          十字路口那里，按理来说车得直行对吧，但是这种情况摄像头扫线的时候，是不是会出现扫不到边线的情况？因为那几行都是白色的嘛，找不到黑白跳变点。
//  @brief          所以按照上面的搜边线算法，如果我们不对这种情况做算法处理的话，那我那些行的左右边界是不是就不对了？对应的中线是不是也不对了？那你能保证小车还直行嘛？
//  @brief          显然保证不了，所以这个时候小车可能就会根据算出来的中线，直接左转或者右转了，是不是违背比赛规则了，那是不是就寄了？所以说补线是非常重要的一环。
//  @parameter      void
//  @time           2023年2月21日
//  @Author
//  Sample usage:   Get_ExtensionLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
/* 十字补线方式：1使用原来的上下端点补线，0使用上方拟合延长。 */
#ifndef CROSS_USE_LEGACY_EXTENSION
#define CROSS_USE_LEGACY_EXTENSION 0
#endif

#if CROSS_USE_LEGACY_EXTENSION
void Get_ExtensionLine(void)
{
    int Left_Corner = Cross_Corner_L;
    int Right_Corner = Cross_Corner_R;

    if (ImageFlag.Cross_Flag != 1 || ImageFlag.image_element_rings != 0 ||
        ImageFlag.Zebra_Flag != 0)
    {
        return;
    }

    /* 首个丢边行可能沿用了下一行的坐标，补线用下一行的实测点。 */
    if (Left_Corner > ImageStatus.OFFLine && Left_Corner < LCDH-1 &&
        ImageDeal[Left_Corner].IsLeftFind == 'W' &&
        ImageDeal[Left_Corner + 1].IsLeftFind == 'T' &&
        ImageDeal[Left_Corner].LeftBorder == ImageDeal[Left_Corner + 1].LeftBorder)
    {
        Left_Corner++;
    }
    if (Right_Corner > ImageStatus.OFFLine && Right_Corner < LCDH-1 &&
        ImageDeal[Right_Corner].IsRightFind == 'W' &&
        ImageDeal[Right_Corner + 1].IsRightFind == 'T' &&
        ImageDeal[Right_Corner].RightBorder == ImageDeal[Right_Corner + 1].RightBorder)
    {
        Right_Corner++;
    }

    //ImageStatus.OFFLine=5;                                                  //这个结构体成员我之所以在这里赋值，是因为我ImageStatus结构体里面的成员太多了，但是暂时又只用到了OFFLine，所以我在哪用到它就在哪赋值。
    /************************************左边线的补线处理*************************************************/
//    if (ImageStatus.WhiteLine >= 8)   //可给严苛一点                            //如果丢边行的数量大于8
    TFSite = LCDH-5;                                                          //那就给TFSite赋值为55，这个变量是待会算补线斜率的一个变量。
    left_FTSite=0;
    right_FTSite=0;
//    ImageFlag.Cross_Flag=0;

    if (ExtenLFlag != 'F' && Left_Corner > ImageStatus.OFFLine &&
        Left_Corner < LCDH && ImageDeal[Left_Corner].IsLeftFind == 'T' &&
        ImageDeal[Left_Corner].LeftBorder > 1 &&
        ImageDeal[Left_Corner].LeftBorder < LCDW-2)
        for (Ysite = LCDH-10; Ysite >= (ImageStatus.OFFLine + 4);Ysite--)       //从第54开始往上扫，一直扫到顶边下面几行。
        {
            PicTemp = Pixle[Ysite];
            if (ImageDeal[Ysite].IsLeftFind =='W'&&Ysite<60)                             //如果本行的左边线类型是W类型，也就是无边行类型。
            {
                if (ImageDeal[Ysite + 1].LeftBorder >= LCDW-10)                //如果左边界到了第70列右边去了，那大概率就是极端情况，说明已经快寄了。
                {
                  ImageStatus.OFFLine = Ysite + 1;                             //这种情况最好的处理方法就是不处理，直接跳出循环。,记录offline
                  break;
                }
                while (Ysite >= (ImageStatus.OFFLine + 4))                     //如果左边界正常，那就进入while循环卡着，直到满足循环结束条件。
                {
                    Ysite--;                                                   //行数减减
                    if (ImageDeal[Ysite].IsLeftFind == 'T'
                            &&ImageDeal[Ysite - 1].IsLeftFind == 'T'
                         &&ImageDeal[Ysite - 2].IsLeftFind == 'T'
                          && Ysite - 5 > ImageStatus.OFFLine + 5    //先检查实际使用的上端点行号
                          && Ysite - 5 < Left_Corner
                          && ImageDeal[Ysite - 5].IsLeftFind == 'T'
                          && ImageDeal[Ysite - 5].LeftBorder > 1
                          && ImageDeal[Ysite - 5].LeftBorder < LCDW-2

                      /*&&ImageDeal[Ysite - 2].LeftBorder > 0
                      &&ImageDeal[Ysite - 2].LeftBorder <LCDW-10*/
                      )                                                        //如果扫到的无边行的上面连续三行都是正常边线
                    {
                      left_FTSite = Ysite - 5; //3给大点会增加对锯齿拐角的抵抗力         //那就把扫到的这一行的上面两行存入FTsite变量
//                      if(Buxian_Flag_L)
//                      {
//                          left_FTSite=Ysite;//找十字左上拐点
//                      }
                      break;                                                   //跳出while循环
                    }
                }
               if (left_FTSite > ImageStatus.OFFLine+5)                        //如果FTSite存储的那一行在图像顶边OFFline的下面
               {
                    DetL =((float)(ImageDeal[left_FTSite].LeftBorder -ImageDeal[Left_Corner].LeftBorder)) /((float)(left_FTSite - Left_Corner));  //计算左边线的补线斜率
                   for (ytemp = Left_Corner; ytemp >= left_FTSite; ytemp--)//那么我就从第一次扫到的左边界的下面第二行的位置开始往上一直补线，补到FTSite行。
                   {
                   ImageDeal[ytemp].LeftBorder =(int)(DetL * ((float)(ytemp - Left_Corner))) +ImageDeal[Left_Corner].LeftBorder;//这里就是具体的补线操作了
                   }
                       Ysite=(ImageStatus.OFFLine + 4)-1;                        //减一停止外层的循环，因为有可能遇到双十字，以底部十字补线一次即可
               }
            }                                         //这里为什么要Ysite+2，我没法在注释里面讲清楚，自己领会吧。//应该就是在找到无边行的下两行来计算斜率
        }
    /************************************左边线的补线处理*************************************************/
//    wireless_uart_send_byte(0x00);

    /************************************右边线的补线处理（跟左边线处理思路一模一样）注释略*************************************************/
    if (ExtenRFlag != 'F' && Right_Corner > ImageStatus.OFFLine &&
        Right_Corner < LCDH && ImageDeal[Right_Corner].IsRightFind == 'T' &&
        ImageDeal[Right_Corner].RightBorder > 1 &&
        ImageDeal[Right_Corner].RightBorder < LCDW-2)
        for (Ysite = LCDH-10; Ysite >= (ImageStatus.OFFLine + 4);Ysite--)
        {
          PicTemp = Pixle[Ysite];//存当前行
          if (ImageDeal[Ysite].IsRightFind =='W'&&Ysite<60)
          {
            if (ImageDeal[Ysite + 1].RightBorder <= 10)
            {
              ImageStatus.OFFLine =Ysite + 1;
              break;
            }
            while (Ysite >= (ImageStatus.OFFLine + 4))
            {
              Ysite--;
              if (ImageDeal[Ysite].IsRightFind == 'T'
                  &&ImageDeal[Ysite - 1].IsRightFind == 'T'
                  &&ImageDeal[Ysite - 2].IsRightFind == 'T'
                          && Ysite - 5 > ImageStatus.OFFLine + 5    //先检查实际使用的上端点行号
                          && Ysite - 5 < Right_Corner
                          && ImageDeal[Ysite - 5].IsRightFind == 'T'
                          && ImageDeal[Ysite - 5].RightBorder > 1
                          && ImageDeal[Ysite - 5].RightBorder < LCDW-2
                  /*&&ImageDeal[Ysite - 2].RightBorder < LCDW-1
                  &&ImageDeal[Ysite - 2].RightBorder > 10*/
                  )
              {
                  right_FTSite = Ysite - 5;
//                  if(Buxian_Flag_R==1)
//                  {
//                      right_FTSite=Ysite;
//                  }
                break;
              }
            }
           if (right_FTSite > ImageStatus.OFFLine+5)
           {
                  DetR =((float)(ImageDeal[right_FTSite].RightBorder -ImageDeal[Right_Corner].RightBorder)) /((float)(right_FTSite - Right_Corner));
                 for (ytemp = Right_Corner; ytemp >= right_FTSite;ytemp--)
                 {
                   ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - Right_Corner))) +ImageDeal[Right_Corner].RightBorder;
                 }
                 Ysite=(ImageStatus.OFFLine + 4)-1;                        //减一停止外层的循环，因为有可能遇到双十字，以底部十字补线一次即可
           }
          }
        }
      /************************************右边线的补线处理（跟左边线处理思路一模一样）注释略*************************************************/


   /*****************************************所有的补线处理结束，把中线和赛道宽度信息再更新一遍******************************************************/
        if(left_FTSite != 0 && right_FTSite != 0)
        {
            for (Ysite = LCDH-1; Ysite >= ImageStatus.OFFLine; Ysite--)
            {
                ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;
                ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder ;

            }

        }

        if((right_FTSite-left_FTSite) > 5)
        {
//            BeeOn;
            for (Ysite = LCDH-1; Ysite >= ImageStatus.OFFLine; Ysite--)
            {
                ImageDeal[Ysite].Center =ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
            }
        }
        if((left_FTSite-right_FTSite) > 5)
        {
            for (Ysite = LCDH-1; Ysite >= ImageStatus.OFFLine; Ysite--)
            {
                ImageDeal[Ysite].Center =ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
            }
        }

//    ImageFlag.Cross_Flag=0;
//    BeeOff;
//    /*****************************************所有的补线处理结束，把中线和赛道宽度信息再更新一遍******************************************************/
}

#else

/* 新十字补线：上方连续实测边线拟合，向下延长到第LCDH-2行。 */
/* Upper-road fitting parameters for the current 70 x 160 image. */
#define CROSS_EXT_FIT_ROWS       8       // 拟合所需的连续有效边线行数
#define CROSS_EXT_GAP_ROWS       3       // 连续出现多少行W/H，才确认经过断口
#define CROSS_EXT_STEP_MAX       3       // 连续拟合点相邻两行允许的最大列坐标差，单位：像素
#define CROSS_EXT_ROW_GAP_MAX    8       // 左右拟合区下端允许的最大行号差，单位：行
#define CROSS_EXT_SLOPE_MAX      1.5f    // 拟合斜率绝对值上限，单位：列像素/行
#define CROSS_EXT_MSE_MAX        2.25f   // 拟合均方残差上限，单位：像素平方，越小越严格
#define CROSS_EXT_EDGE_MIN      3       // 有效实测点及补线结果允许的最小列坐标
#define CROSS_EXT_EDGE_MAX      (LCDW - 3)          // 有效实测点及补线结果允许的最大列坐标
#define CROSS_EXT_WIDTH_MIN     20.0f              // 补线后每行允许的最小道路宽度，单位：像素
#define CROSS_EXT_WIDTH_MAX     ((float)(LCDW - 5)) // 补线后每行允许的最大道路宽度，单位：像素


#define CROSS_EXT_INNER_ROWS     2      /* 连续两行发现更靠内的边线，就重新选边 */
#define CROSS_EXT_RECOVER_BLACK  4      /* 重搜时忽略较短黑点 */
#define CROSS_EXT_RECOVER_AGREE  5      /* 两种重搜结果的允许列差 */
#define CROSS_EXT_LEFT           0
#define CROSS_EXT_RIGHT          1
#define CROSS_EXT_RECOVER_LEFT   2
#define CROSS_EXT_RECOVER_RIGHT  3

typedef struct
{
    int far_row;
    int near_row;
    float k;
    float b;
} CrossExtFit;

/* Return a measured transition, normalized to its white-side pixel. */
static uint8 CrossExt_GetMeasuredPoint(int side, int y, int *point_x)
{
    int x;
    uint8 found;

    if (y <= ImageStatus.OFFLine || y >= LCDH - 1)
        return 0;

    x = side == 0 ? ImageDeal[y].LeftBorder : ImageDeal[y].RightBorder;
    found = side == 0 ? ImageDeal[y].IsLeftFind : ImageDeal[y].IsRightFind;

    if (found != 'T' || x < CROSS_EXT_EDGE_MIN || x > CROSS_EXT_EDGE_MAX)
        return 0;

    if (side == 0)
    {
        if (Pixle[y][x] == 255 && Pixle[y][x - 1] == 0)
            *point_x = x;
        else if (Pixle[y][x] == 0 && Pixle[y][x + 1] == 255)
            *point_x = x + 1;
        else
            return 0;
    }
    else
    {
        if (Pixle[y][x] == 255 && Pixle[y][x + 1] == 0)
            *point_x = x;
        else if (Pixle[y][x] == 0 && Pixle[y][x - 1] == 255)
            *point_x = x - 1;
        else
            return 0;
    }
    return 1;
}

/* 从白路上的已知点向另一侧搜，返回第一段连续黑区前的白点。 */
static int CrossExt_FindInnerBorder(int Ysite, int Start_Xsite, int Step, int Scan_Width)
{
    int Xsite, Distance;
    int Black_Num = 0;

    for (Distance = 1; Distance <= Scan_Width; Distance++)
    {
        Xsite = Start_Xsite + Step * Distance;
        if (Xsite <= 0 || Xsite >= LCDW - 1)
        {
            break;
        }

        if (Pixle[Ysite][Xsite] == 255)
        {
            Black_Num = 0;
        }
        else if (++Black_Num >= CROSS_EXT_RECOVER_BLACK)
        {
            return Start_Xsite + Step * (Distance - Black_Num);
        }
    }
    return -1;                                                //这段范围内没有找到边界
}

/* 十字断口上方重搜边线，只在该行的合理路宽范围内取点。 */
static uint8 CrossExt_GetPoint(int source, int y, int *point_x)
{
    int anchor, x, step;
    int width, min_width, max_width;

    if (source == CROSS_EXT_LEFT || source == CROSS_EXT_RIGHT)
        return CrossExt_GetMeasuredPoint(source, y, point_x);

    if (!CrossExt_GetMeasuredPoint(source == CROSS_EXT_RECOVER_RIGHT ?
                                  CROSS_EXT_LEFT : CROSS_EXT_RIGHT, y, &anchor))
        return 0;

    step = source == CROSS_EXT_RECOVER_RIGHT ? 1 : -1;
    width = 2 * Half_Road_Wide[y];
    min_width = width / 2;
    if (min_width < CROSS_EXT_WIDTH_MIN)
        min_width = (int)CROSS_EXT_WIDTH_MIN;
    max_width = width + width / 2;

    x = CrossExt_FindInnerBorder(y, anchor, step, max_width + CROSS_EXT_RECOVER_BLACK);
    if (x < CROSS_EXT_EDGE_MIN || x > CROSS_EXT_EDGE_MAX)
        return 0;

    width = (x - anchor) * step;
    if (width < min_width || width > max_width)
        return 0;

    *point_x = x;
    return 1;
}

/* Fit x = k*y + b using every measured row in the inclusive interval. */
static uint8 CrossExt_FitRange(int side, int far_row, int near_row,
                               CrossExtFit *fit)
{
    int y, x, n = 0, previous_x = 0;
    float sum_y = 0.0f, sum_x = 0.0f;
    float sum_yy = 0.0f, sum_yx = 0.0f;
    float denominator, k, b, error, error_sum = 0.0f;

    for (y = far_row; y <= near_row; y++)
    {
        if (!CrossExt_GetPoint(side, y, &x))
            return 0;
        if (n > 0 && (x - previous_x > CROSS_EXT_STEP_MAX ||
                      previous_x - x > CROSS_EXT_STEP_MAX))
            return 0;
        previous_x = x;
        sum_y += (float)y;
        sum_x += (float)x;
        sum_yy += (float)y * (float)y;
        sum_yx += (float)y * (float)x;
        n++;
    }
    if (n < CROSS_EXT_FIT_ROWS)
        return 0;

    denominator = (float)n * sum_yy - sum_y * sum_y;
    if (denominator <= 0.0f)
        return 0;

    k = ((float)n * sum_yx - sum_y * sum_x) / denominator;
    b = (sum_x - k * sum_y) / (float)n;
    if (!(k >= -CROSS_EXT_SLOPE_MAX && k <= CROSS_EXT_SLOPE_MAX))
        return 0;

    for (y = far_row; y <= near_row; y++)
    {
        if (!CrossExt_GetPoint(side, y, &x))
            return 0;
        error = (float)x - (k * (float)y + b);
        error_sum += error * error;
    }
    if (!(error_sum / (float)n <= CROSS_EXT_MSE_MAX))
        return 0;

    fit->far_row = far_row;
    fit->near_row = near_row;
    fit->k = k;
    fit->b = b;
    return 1;
}

/* Walk upward through a gap, then select one continuous measured segment. */
static uint8 CrossExt_FindUpper(int side, CrossExtFit *fit)
{
    int y, x, previous_x = 0, difference;
    int gap_count = 0, gap_seen = 0;
    int run_count = 0, run_near = 0;
    uint8 found;

    /* Get_AllLine starts here; exclude the bottom five baseline rows. */
    for (y = LCDH - 6; y > ImageStatus.OFFLine; y--)
    {
        found = side == 0 ? ImageDeal[y].IsLeftFind : ImageDeal[y].IsRightFind;
        if (!gap_seen)
        {
            if (found == 'W' || found == 'H')
            {
                if (++gap_count >= CROSS_EXT_GAP_ROWS)
                    gap_seen = 1;
            }
            else
                gap_count = 0;
            continue;
        }

        if (!CrossExt_GetPoint(side, y, &x))
        {
            run_count = 0;
            continue;
        }
        if (run_count > 0)
        {
            difference = x - previous_x;
            if (difference < 0)
                difference = -difference;
            if (difference > CROSS_EXT_STEP_MAX)
                run_count = 0;
        }
        if (run_count == 0)
            run_near = y;
        previous_x = x;
        run_count++;

        /* Reject a poor first candidate rather than fit across another branch. */
        if (run_count >= CROSS_EXT_FIT_ROWS)
            return CrossExt_FitRange(side, y, run_near, fit);
    }
    return 0;
}

/* Validate a complete candidate without changing the original scan. */
static uint8 CrossExt_ValidatePair(const CrossExtFit *left_fit,
                                  const CrossExtFit *right_fit, int *start_row)
{
    int first_row, overlap_end, row_difference, prospect;
    int y, left, right;
    float left_x, right_x, width;

    row_difference = left_fit->near_row - right_fit->near_row;
    if (row_difference < 0)
        row_difference = -row_difference;
    if (row_difference > CROSS_EXT_ROW_GAP_MAX)
        return 0;

    first_row = left_fit->far_row > right_fit->far_row ?
                left_fit->far_row : right_fit->far_row;
    overlap_end = left_fit->near_row < right_fit->near_row ?
                  left_fit->near_row : right_fit->near_row;
    if (first_row > overlap_end)
        return 0;

    prospect = (int)Vision_ProspectRow[VP_CROSS];
    if (prospect - 1 < first_row || prospect + 1 > LCDH - 2)
        return 0;

    /* Validate the complete float and integer output before any writes. */
    for (y = first_row; y <= LCDH - 2; y++)
    {
        left_x = left_fit->k * (float)y + left_fit->b;
        right_x = right_fit->k * (float)y + right_fit->b;
        width = right_x - left_x;
        if (!(left_x >= CROSS_EXT_EDGE_MIN && left_x <= CROSS_EXT_EDGE_MAX &&
              right_x >= CROSS_EXT_EDGE_MIN && right_x <= CROSS_EXT_EDGE_MAX &&
              width >= CROSS_EXT_WIDTH_MIN && width <= CROSS_EXT_WIDTH_MAX))
            return 0;
        left = (int)(left_x + 0.5f);
        right = (int)(right_x + 0.5f);
        if (right <= left || right - left < 20 || right - left > LCDW - 5)
            return 0;

    }

    *start_row = first_row;
    return 1;
}

/* 如果白路在实测边线之前已经结束，外侧的边线不能拿来配对。 */
static uint8 CrossExt_HasInnerBorder(int Start_Ysite, int End_Ysite)
{
    int Ysite, Left_Xsite, Right_Xsite, Inner_Xsite, Wide;
    int Left_Inner_Num = 0;
    int Right_Inner_Num = 0;

    for (Ysite = Start_Ysite; Ysite <= End_Ysite; Ysite++)
    {
        if (!CrossExt_GetMeasuredPoint(CROSS_EXT_LEFT, Ysite, &Left_Xsite) ||
            !CrossExt_GetMeasuredPoint(CROSS_EXT_RIGHT, Ysite, &Right_Xsite))
        {
            Left_Inner_Num = 0;
            Right_Inner_Num = 0;
            continue;
        }

        Wide = Right_Xsite - Left_Xsite;
        Inner_Xsite = CrossExt_FindInnerBorder(Ysite, Left_Xsite, 1, Wide);
        if (Inner_Xsite >= 0 && Right_Xsite - Inner_Xsite > CROSS_EXT_RECOVER_AGREE)
            Right_Inner_Num++;
        else
            Right_Inner_Num = 0;

        Inner_Xsite = CrossExt_FindInnerBorder(Ysite, Right_Xsite, -1, Wide);
        if (Inner_Xsite >= 0 && Inner_Xsite - Left_Xsite > CROSS_EXT_RECOVER_AGREE)
            Left_Inner_Num++;
        else
            Left_Inner_Num = 0;

        if (Left_Inner_Num >= CROSS_EXT_INNER_ROWS || Right_Inner_Num >= CROSS_EXT_INNER_ROWS)
        {
            return 1;
        }
    }
    return 0;
}

/* 十字断口上方，沿着左边线右侧的白路找右边线。 */
static uint8 CrossExt_FindPairFromLeft(CrossExtFit *Left_Fit,
                                      CrossExtFit *Right_Fit, int *First_Row)
{
    CrossExtFit Left_Candidate, Right_Candidate;
    int Ysite, Near_Ysite, Start_Ysite;
    int Left_Xsite, Right_Xsite, Old_Right_Xsite;
    int Last_Left = 0, Last_Right = 0;
    int Left_Gap = 0, Right_Gap = 0;
    int Line_Num = 0, Inner_Ysite = -1, Last_Inner_Ysite = -1;
    uint8 Gap_Found = 0, Pair_Found = 0, Inner_Found;

    for (Ysite = LCDH - 6; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        if (!Gap_Found)
        {
            if (ImageDeal[Ysite].IsLeftFind == 'W' || ImageDeal[Ysite].IsLeftFind == 'H')
                Left_Gap++;
            else
                Left_Gap = 0;

            if (ImageDeal[Ysite].IsRightFind == 'W' || ImageDeal[Ysite].IsRightFind == 'H')
                Right_Gap++;
            else
                Right_Gap = 0;

            if (Left_Gap >= CROSS_EXT_GAP_ROWS || Right_Gap >= CROSS_EXT_GAP_ROWS)
                Gap_Found = 1;
            continue;
        }

        if (!CrossExt_GetMeasuredPoint(CROSS_EXT_LEFT, Ysite, &Left_Xsite) ||
            !CrossExt_GetPoint(CROSS_EXT_RECOVER_RIGHT, Ysite, &Right_Xsite))
        {
            Line_Num = 0;
            continue;
        }

        if (Line_Num > 0 &&
            (abs(Left_Xsite - Last_Left) > CROSS_EXT_STEP_MAX ||
             abs(Right_Xsite - Last_Right) > CROSS_EXT_STEP_MAX))
        {
            Line_Num = 0;                                      //跳到另一条边线了，从本行重新数
        }
        Last_Left = Left_Xsite;
        Last_Right = Right_Xsite;
        Line_Num++;

        if (CrossExt_GetMeasuredPoint(CROSS_EXT_RIGHT, Ysite, &Old_Right_Xsite) &&
            Old_Right_Xsite - Right_Xsite > CROSS_EXT_RECOVER_AGREE)
        {
            Last_Inner_Ysite = Inner_Ysite;
            Inner_Ysite = Ysite;
        }

        if (Line_Num < CROSS_EXT_FIT_ROWS)
            continue;

        Near_Ysite = Ysite + CROSS_EXT_FIT_ROWS - 1;
        Inner_Found = Inner_Ysite >= Ysite && Last_Inner_Ysite <= Near_Ysite &&
                      Last_Inner_Ysite == Inner_Ysite + 1;
        if (Pair_Found && !Inner_Found)
            continue;

        if (CrossExt_FitRange(CROSS_EXT_LEFT, Ysite, Near_Ysite, &Left_Candidate) &&
            CrossExt_FitRange(CROSS_EXT_RECOVER_RIGHT, Ysite, Near_Ysite, &Right_Candidate) &&
            CrossExt_ValidatePair(&Left_Candidate, &Right_Candidate, &Start_Ysite))
        {
            *Left_Fit = Left_Candidate;
            *Right_Fit = Right_Candidate;
            *First_Row = Start_Ysite;
            Pair_Found = 1;

            /* 先留最近的可用配对，找到被原右线漏掉的内侧边就优先用内侧。 */
            if (Inner_Found)
                break;
        }
    }
    return Pair_Found;
}

static uint8 CrossExt_FitsAgree(const CrossExtFit *a, const CrossExtFit *b, int y)
{
    float difference = (a->k - b->k) * (float)y + a->b - b->b;

    return difference >= -CROSS_EXT_RECOVER_AGREE &&
           difference <= CROSS_EXT_RECOVER_AGREE;
}

void Get_ExtensionLine(void)
{
    CrossExtFit left_fit, right_fit, recovered_left, recovered_right;
    uint8 left_found, right_found, from_left, from_right;
    int first_row, start_from_left, start_from_right;
    int y, left, right, prospect;

    if (ImageFlag.Cross_Flag != 1 || ImageFlag.image_element_rings != 0 ||
        ImageFlag.Zebra_Flag != 0)
        return;

    /* 优先按左边线、白路、右侧黑块的顺序配对。 */
    if (!CrossExt_FindPairFromLeft(&left_fit, &right_fit, &first_row))
    {
        left_found = CrossExt_FindUpper(CROSS_EXT_LEFT, &left_fit);
        right_found = CrossExt_FindUpper(CROSS_EXT_RIGHT, &right_fit);

        /* 两侧几何合格后，还要看共同拟合区里是否存在更靠内的边线。 */
        if (!(left_found && right_found &&
              CrossExt_ValidatePair(&left_fit, &right_fit, &first_row) &&
              !CrossExt_HasInnerBorder(first_row, left_fit.near_row < right_fit.near_row ?
                                                left_fit.near_row : right_fit.near_row)))
        {
            /* 两个方向独立重搜，使用锚点拟合区相同的行。 */
            from_left = left_found &&
                        CrossExt_FitRange(CROSS_EXT_RECOVER_RIGHT, left_fit.far_row,
                                          left_fit.near_row, &recovered_right) &&
                        CrossExt_ValidatePair(&left_fit, &recovered_right,
                                              &start_from_left);
            from_right = right_found &&
                         CrossExt_FitRange(CROSS_EXT_RECOVER_LEFT, right_fit.far_row,
                                           right_fit.near_row, &recovered_left) &&
                         CrossExt_ValidatePair(&recovered_left, &right_fit,
                                               &start_from_right);

            if (!from_left && !from_right)
                return;

            /* 两边各自找到不同白路时，当前帧缺少可确定的配对。 */
            if (from_left && from_right)
            {
                prospect = (int)Vision_ProspectRow[VP_CROSS];
                if (!CrossExt_FitsAgree(&left_fit, &recovered_left, prospect) ||
                    !CrossExt_FitsAgree(&right_fit, &recovered_right, prospect) ||
                    !CrossExt_FitsAgree(&left_fit, &recovered_left, LCDH - 2) ||
                    !CrossExt_FitsAgree(&right_fit, &recovered_right, LCDH - 2))
                    return;
            }

            if (from_left)
            {
                right_fit = recovered_right;
                first_row = start_from_left;
            }
            else
            {
                left_fit = recovered_left;
                first_row = start_from_right;
            }
        }
    }

    /* Commit both sides together; retain original measured T/W/H flags. */
    for (y = first_row; y <= LCDH - 2; y++)
    {
        left = (int)(left_fit.k * (float)y + left_fit.b + 0.5f);
        right = (int)(right_fit.k * (float)y + right_fit.b + 0.5f);
        ImageDeal[y].LeftBorder = left;
        ImageDeal[y].RightBorder = right;
        ImageDeal[y].Wide = right - left;
        ImageDeal[y].Center = (left + right) / 2;
    }
}

#endif

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Straight_Judge
//  @brief          *****************直线判断******************
//  @parameter
//  @note           //在[start, end]这些行里，把某一组边线点拟合成一条直线，然后计算实际边线点和这条理想直线的偏差。
//  @note           返回值越小，越接近直线
//  @note           可以用于直道、弯道、环岛入口等判断
//  @time
//  @Author
//  Sample usage:   Straight_Judge
//---------------------------------------------------------------------------------------------------------------------------------------------------------------


float Straight_Judge(uint8 dir, uint8 start, uint8 end)         //返回实际边界与拟合直线的方差和
//在[start, end]这些行里，把某一组边线点拟合成一条直线，然后计算实际边线点和这条理想直线的偏差。
//返回值越小，越接近直线
//可以用于直道、弯道、环岛入口等判断
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;//每往下一行，边线理论上应该移动多少像素 也就是理想直线的斜率。
    switch (dir)//1：左边界 LeftBorder 2：右边界 RightBorder 3：中线 Center
    {
    case 1:k = (float)(ImageDeal[start].LeftBorder - ImageDeal[end].LeftBorder) / (start - end);
        for (i = 0; i < end - start; i++)//计算每行的误差平方和
        {
            Err = (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder) * (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder);
            Sum += Err;//正常情况下，Sum应该比较小
        }
        S = Sum / (end - start);//所有行误差平方的平均值
        break;
    case 2:k = (float)(ImageDeal[start].RightBorder - ImageDeal[end].RightBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder) * (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 3:k = (float)(ImageDeal[start].Center - ImageDeal[end].Center) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].Center + k * i - ImageDeal[i + start].Center) * (ImageDeal[start].Center + k * i - ImageDeal[i + start].Center);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    }
    return S;
}


uint8 NO_zhidao_add_flag=0;//作用：暂时禁止长直道判断 触发：斑马线阶段


/***************长直线判断*******************/
void Straight_long_judge(void)                                  //返回结果小于3即为直线
{
    if((ImageFlag.Zebra_Counter==1&&ImageFlag.Zebra_Flag==1)||NO_zhidao_add_flag==1)
        return;
 //左右边线误差均小于5 -> 直道判断成立
    if((Straight_Judge(1,ImageStatus.OFFLine+2,65) < 5) && (Straight_Judge(2,ImageStatus.OFFLine+2,65) < 5)
            && ImageStatus.OFFLine < 8 && ImageStatus.Miss_Left_lines < 2 && ImageStatus.Miss_Right_lines < 2)
    {
        ImageFlag.straight_long = 1;
    }

}
/***************长直线处理*******************/
void Straight_long_handle(void)                                  //返回结果小于1即为直线
{

    if(ImageFlag.straight_long)
    {
        if(
                (Straight_Judge(1,ImageStatus.OFFLine+2,65) >= 5)
            || (Straight_Judge(2,ImageStatus.OFFLine+2,65) >= 5)
//            || ImageStatus.OFFLine >= 15
            ||ImageStatus.WhiteLine_L>=3
            ||ImageStatus.WhiteLine_R>=3)
        {
            ImageFlag.straight_long = 0;
            BeeOff;
            //左边线不直了或右边线不直了或一侧出现明显    → 不再是长直道   → 关闭蜂鸣器
        }
    }
}

//用于变参数的直道检测  也就是用中线来判断直道 放在十字处理后进行判断
float sum1=0,sum2=0;

uint8 Bai_point_num=0;//当前这一列统计到的白点数量
uint8 Bai_point_max_num=0;//所有扫描列中，白点数量最多的那一列
uint8 LongestLine_Found_flag=0;//是否找到长白列的标志位

//长直道判断  图像中间是否存在一整列很长的白色区域 标志：LongestLine_Found_flag
void LongestLine_Judge(void)
{
    //如果是环岛阶段，就不判断长直道
    if(ImageFlag.image_element_rings_flag!=0)
    {
        LongestLine_Found_flag=0;
        return;
    }

//    for(uint8 Xsite=20;Xsite<106;Xsite++)
    for(uint8 Xsite=40;Xsite<120;Xsite++)
      {
        if(Pixle[68][Xsite]==0)
            {continue;}
          Bai_point_num=0;
          for(uint8 Ysite=69;Ysite>0;Ysite--)
          {
              if(Pixle[Ysite][Xsite]==255)
              {
                  Bai_point_num++;
              }
              else if(Pixle[Ysite][Xsite]==255&&Pixle[Ysite-1][Xsite]==0)
              {
                  break;
              }
              else if(Ysite==1)
              {
                  break;
              }
          }
          if(Bai_point_num>Bai_point_max_num)
          {
              Bai_point_max_num=Bai_point_num;
          }
          if(Bai_point_max_num>67||(ImageStatus.OFFLine<6&&ImageStatus.Det_True<20))
          {
              Bai_point_max_num=0;
              LongestLine_Found_flag=1;
              break;
          }
          if(LongestLine_Found_flag)
          {
              if(ImageStatus.OFFLine>10)
                  LongestLine_Found_flag=0;
          }
//          else
//              LongestLine_Found_flag=0;
      }
}

//长直道处理 持续确认长白列 如果白列消失了，就取消标志
void LongestLind_Handle(void)
{
    if(LongestLine_Found_flag==1)
    {
        for(uint8 Xsite=20;Xsite<140;Xsite++)
          {
            if(Pixle[68][Xsite]==0)
                {continue;}
              Bai_point_num=0;
              for(uint8 Ysite=69;Ysite>0;Ysite--)
              {
                  if(Pixle[Ysite][Xsite]==255)
                  {
                      Bai_point_num++;
                  }
                  else if(Pixle[Ysite][Xsite]==255&&Pixle[Ysite-1][Xsite]==0)
                  {
                      break;
                  }
                  else if(Ysite==1)
                  {
                      break;
                  }
              }
              if(Bai_point_num>Bai_point_max_num)
              {
                  Bai_point_max_num=Bai_point_num;
              }
          }
        if(Bai_point_max_num<=67||ImageFlag.image_element_rings_flag!=0)
        {
            LongestLine_Found_flag=0;
        }
    }
}

//--------------------------------------------------------------
//  @name          Element_Judgment_Zebra()
//  @brief         //斑马线判断 通过扫描黑白跳变次数
//  @parameter      void
//  @note         ImageFlag.Zebra_Flag 当前是否处于斑马线状态
//  @note         ImageFlag.Zebra_Counter 斑马线计数器，记录斑马线出现的次数
//  @note         Zebra_Bottom_flag 斑马线是否到达图像底部
//  @note         Zebra_Miss_flag 斑马线是否消失
//  @time
//  @Author
//  Sample usage:
//--------------------------------------------------------------
void Element_Judgment_Zebra()
{
    if (ImageFlag.Zebra_Flag)
        return;

    int NUM = 0, net = 0;
    if(ImageStatus.OFFLineBoundary<20)
    {
        for (int Ysite = 69; Ysite >29; Ysite--)
        {
            net = 0;
            for (int Xsite =ImageDeal[Ysite].LeftBoundary ; Xsite < ImageDeal[Ysite].RightBoundary-1; Xsite++)
            {
                if ((Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 255)||(Pixle[Ysite][Xsite] == 255 && Pixle[Ysite][Xsite + 1] == 0))
                {
                    net++;
                    if (net > 8)
                    {
                        NUM++;
                    }
                }
            }
        }
    }

    if (NUM >= 3 && ImageFlag.Zebra_Flag == 0)
    {
        ImageFlag.Zebra_Flag = 1;
        if(ImageFlag.Zebra_Counter==1)
        {
            NO_zhidao_add_flag=1;
        }
//        BeeOn;
    }
}

//斑马线处理 Zwf 2024-2-20
/****一般ImageFlag.Zebra_Flag = 1时斑马线还在图像中间部分，等ImageFlag.Search_error_flag==1时说明斑马线在图像底部了，这时挂起斑马线到底部的标志位Zebra_Bottom_flag=1
 ,在Zebra_Bottom_flag=1条件下检测Search_error_flag=0条件成立时，则此次斑马线状态结束*****/
void Element_Handle_Zebra()
{
    if(ImageFlag.Search_error_flag==1)
    {
        Search_Border_OTSU(Pixle,LCDH,LCDW,25);             //由于图像底部时斑马线，迷宫找种子没找到,故在图像中间找种子避免生长错误
        Zebra_Bottom_flag=1;                                //斑马线消失
//        BeeOn;
    }
    if(Zebra_Bottom_flag==1)
    {
        uint8 x_R=0;
        uint8 x_L=0;
        Zebra_Miss_flag=1;
        /****这个for是为了确保斑马线消失****/
        for(int Ysite=68;Ysite>29;Ysite--)//68是因为封边
        {
            for (int Xsite = LCDW / 2-2; Xsite > 0; Xsite--)
            {
                if (Pixle[Ysite][Xsite] == 255 && Pixle[Ysite][Xsite - 1] == 0)
                {
                    x_L = Xsite;//获取底边左边线
                    break;
                }
            }
            for (int Xsite = LCDW / 2+2; Xsite < LCDW; Xsite++)
            {
                if (Pixle[Ysite][Xsite] == 255 && Pixle[Ysite][Xsite + 1] == 0)
                {
                    x_R = Xsite;//获取底边右边线
                    break;
                }
            }
            if(x_R-x_L<40)
            {
                Zebra_Miss_flag=0;
                break;
            }
        }

        /**结束斑马线进程**/
        if(Zebra_Miss_flag==1)
        {
            Get_BaseLine();
            Get_AllLine();

            ImageFlag.Zebra_Flag=0;
            ImageFlag.Zebra_Counter++;
            Zebra_Bottom_flag=0;
//            BeeOff;
            if(ImageFlag.Zebra_Counter==2)                                                    //第二次识别到斑马线才挂起停车标志位
            {
                ImageFlag.Stop=1;
//                ImageFlag.Zebra_Counter=0;
            }
        }
    }
}
//--------------------------------------------------------------
//  @name          
//  @brief         元素判断: 路障判断
//  @parameter      void
//  @note
//  @time
//  @Author
//  Sample usage:
//--------------------------------------------------------------
uint8 small_block_ystart=0;
void Block_Sub_Judge(uint8 ystart,uint8 yend)
{
    small_block_ystart=0;
    uint8 net=0;
    for(int Ysite=ystart;Ysite>yend;Ysite--)
    {
        for (int Xsite =ImageDeal[Ysite].LeftBoundary+1 ; Xsite < ImageDeal[Ysite].RightBoundary; Xsite++)
        {
            if(Pixle[Ysite][Xsite] == 0)
            {
                net++;
                if(net>6)
                {
                    small_block_ystart=Ysite;
                    Ysite=yend;
                    //找到这行的路障边边的xsite
                    break;
                }
            }
            else
            {
                net=0;
            }
        }
    }
}

//左/右路障判断：先判断对侧边线是否是直线，如果是直线，再判断赛道是否明显变窄，如果变窄，则说明左/右侧有路障
//当前版本先统一统计变窄行数，再分别检查两侧直线度。
static uint8 Element_Judgement_SmallBlock(void)
{
    uint8 hits = 0;
    int first_row = clip(ImageStatus.OFFLine + 1, 20, LCDH - 1);

    if (ImageFlag.Ramp || ImageFlag.Zebra_Flag || ImageFlag.Cross_Flag ||
        ImageFlag.straight_long || ImageFlag.image_element_rings ||
        first_row > 48)
    {
        return 0;
    }

    //检查第20–48行。
    for (int y = first_row; y <= 48; y++)
    {
        int width = ImageDeal[y].RightBorder - ImageDeal[y].LeftBorder;

        //赛道明显变窄
        if (ImageDeal[y].IsLeftFind == 'T' && ImageDeal[y].IsRightFind == 'T' &&
            width > 30 && All_Road_Wide[y] - width > 10)
        {
            hits++;
        }
    }

    if (hits < 11)
    {
        return 0;
    }

    //左是直线
    uint8 right_hit = Straight_Judge(1, first_row, LCDH - 1) < 3.0f;
    //右是直线
    uint8 left_hit = Straight_Judge(2, first_row, LCDH - 1) < 3.0f;

    if (left_hit == right_hit)
    {
        return 0;
    }

    //右路障：1；左路障：2
    return right_hit ? 1 : 2;
}

//------------------------路障中线处理----------------------------
static void Element_Handle_SmallBlock(void)
{
    //修改第20–64行的中线Center（现在两端渐变到0）。
    const int first_row = 20;
    const int last_row = 64;
    const int taper_rows = 5;
    const int max_offset = 10;
    //左侧有障碍就给中线+10，让目标往右移。
    //右侧有障碍就给中线-10，让目标往左移。
    //当前版本中段保留10像素最大偏移。
    const int direction = ImageFlag.SmallBlock == 2 ? 1 : -1;

    for (int y = first_row + 1; y < last_row; y++)
    {
        int taper = clip(y - first_row, 0, taper_rows);
        int distance_to_end = last_row - y;

        if (distance_to_end < taper)
        {
            taper = distance_to_end;
        }

        ImageDeal[y].Center += direction * max_offset * taper / taper_rows;
        LimitL(ImageDeal[y].Center);
        LimitH(ImageDeal[y].Center);
    }
}

//--------------------------------------------------------------
//  @name          Element_Judgement_Cross()
//  @brief         //元素判断: 十字判断
//  @parameter      void
//  @note
//  @time
//  @Author
//  Sample usage:
//--------------------------------------------------------------
float Cross_white_line=10;

void Element_Judgement_Cross()
{
    //每帧先清标志 默认当前没有十字  左右计数先清零
    ImageFlag.Cross_Flag=0;
    ImageStatus.Cross_Miss_Left_lines=0;
    ImageStatus.Cross_Miss_Right_lines=0;

    //统计左右 从第 40 行开始，一直往上看，统计左/右边线找不到的行数
    for(int ysite=40;ysite>ImageStatus.OFFLine+1;ysite--)
    {
        if(ImageDeal[ysite].IsLeftFind=='W')//这一行左边线搜索异常，或者没有找到左边线
        {
            ImageStatus.Cross_Miss_Left_lines++;
        }
        if(ImageDeal[ysite].IsRightFind=='W')
        {
            ImageStatus.Cross_Miss_Right_lines++;
        }
    }
//    ips200_show_int(50,220,ImageStatus.WhiteLine,4);

    /*其他边界搜索算法发现白色区域或白线数量异常 或者 左边线和右边线同时大面积丢失*/
    /*并且当前不是环岛 */
    if((ImageStatus.WhiteLine >= Cross_white_line||(ImageStatus.Cross_Miss_Left_lines>12&&ImageStatus.Cross_Miss_Right_lines>12))&&ImageFlag.image_element_rings==0) //十字处理
    {
//        BeeOn;
        ImageFlag.Cross_Flag=1;
    }
//    if(ImageFlag.Cross_Flag==0){
//        BeeOff;
//    }

}
//十字补线函数 根据十字之前的左右边线 用已经知道的斜率延长出丢失的边线 重新生成十字区域的中线
void Element_Handle_Cross()
{
    Get_ExtensionLine();
}

//--------------------------------------------------------------
//  @name           Element_Judgment_Left_Rings()
//  @brief          整个图像判断的子函数，用来判断左圆环类型.
//  @parameter      void
//  @note           如果判断成立，就设置：
//  @note           ImageFlag.image_element_rings = 1;
//  @note           ImageFlag.image_element_rings_flag = 1;
//  @note           left_ring_num++;
//  @time
//  @Author
//  Sample usage:   Element_Judgment_Left_Rings();
//--------------------------------------------------------------
void Element_Judgement_Left_Rings()
{
    //前置排除条件
    if(  ImageStatus.Miss_Right_lines>5 || ImageStatus.Miss_Left_lines<10
       ||ImageStatus.OFFLine>10 || Straight_Judge(2,10,60)>3 || ImageStatus.WhiteLine>4
       ||ImageDeal[62].IsLeftFind == 'W'
       ||ImageDeal[63].IsLeftFind == 'W'
       ||ImageDeal[64].IsLeftFind == 'W'
       ||ImageFlag.image_element_rings == 1
               )return;

    int ring_ysite = 35;
        Left_RingsFlag_Point1_Ysite = 0;
        Left_RingsFlag_Point2_Ysite = 0;
    //第一个环岛特征点   寻找左边线第一次出现明显跳变的位置(相邻两行左边界突然变化超过4个像素)
    for (int Ysite = 60; Ysite > ring_ysite; Ysite--)//这个60阈值是为了防止在斑马线的时候误判，不能给到68
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    //第二个环岛特征点
    for (int Ysite = 60; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    //判断左边界是否有圆环形状
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        if (ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite+3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
            && ImageDeal[Ysite+3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder
            )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    //区分直角和圆环
    //直角边 Point2 和 Point1 的距离比较小
    //圆环 Point2 通常比 Point1 更靠上
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0)//这里的话，其实就是区别于直角边，直角的话，Left_RingsFlag_Point2_Ysite < Left_RingsFlag_Point1_Ysite+3
    {
        if(ImageStatus.Miss_Left_lines>10)
            Ring_Help_Flag = 1;
    }

    //最终确认条件：1.Point2 明显在 Point1 上面,并且 Ring_Help_Flag = 1
    //2.右边线基本正常 左边线丢失超过20行 顶部扫描截止行较小 中线误差较小(说明左侧大面积，但车还在正常赛道方向，也可能是左环岛入口。)

    /*if ((Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1)
            ||(ImageStatus.Miss_Right_lines<3&&ImageStatus.Miss_Left_lines>20&&ImageStatus.OFFLine < 10&&ImageStatus.Det_True<10))*/
        if (Left_RingsFlag_Point1_Ysite > 0 && Left_RingsFlag_Point2_Ysite > 0 && Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 3 && Ring_Help_Flag == 1)
    {
        ImageFlag.image_element_rings = 1;         // 左环岛
        ImageFlag.image_element_rings_flag = 1;    // 进入环岛流程
        left_ring_num++;                           // 左环岛计数
    }
    Ring_Help_Flag = 0;
}
//--------------------------------------------------------------
//  @name           Element_Judgment_Right_Rings()
//  @brief          整个图像判断的子函数，用来判断右圆环类型.
//  @parameter      void
//  @time
//  @Author
//  Sample usage:   Element_Judgment_Right_Rings();
//--------------------------------------------------------------
void Element_Judgment_Right_Rings()
{
    if (   ImageStatus.Miss_Left_lines > 4 || ImageStatus.Miss_Right_lines < 10
        || ImageStatus.OFFLine > 10 || Straight_Judge(1, 10, 60) > 5 || ImageStatus.WhiteLine>4   //这些条件限制了十字，弯道等元素；防止下面判断时在这些元素也误判
        || ImageDeal[62].IsRightFind == 'W'
        || ImageDeal[63].IsRightFind == 'W'
        || ImageDeal[64].IsRightFind == 'W'
        || ImageFlag.image_element_rings == 2//加这几个W判别是为了在圆环比较远的时候得知圆环已经到来了，不至于圆环很近的时候//也就是当快到圆弧时，这个判断函数不用再执行了,正常情况下已经进入圆环进程了
        )return;

    int ring_ysite = 20;//35                                                                //待调
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 60; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;                                               //找到圆环和直道相交的底部尖尖的点
            break;
        }
    }
    for (int Ysite = 60; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;                                               //圆环底部黑色边界与图像边框的交点
            break;
        }
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {
        if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite].RightBorder
           )                                                                                    //ImageDeal[Ysite + 3]这个点向左边突出，也就是有个圆弧顶
        {
            Ring_Help_Flag = 1;                                                                 //找到圆弧的弧顶
            break;
        }
    }
    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.Miss_Right_lines>10)
            Ring_Help_Flag = 1;
    }
//    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1)
    if ((Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1)
            ||(ImageStatus.Miss_Left_lines<3&&ImageStatus.Miss_Right_lines>20&&ImageStatus.OFFLine < 10&&ImageStatus.Det_True<10))
    {
            ImageFlag.image_element_rings = 2;
            ImageFlag.image_element_rings_flag = 1;
//            BeeOn;
            //开启圆环进程状态                                                        //小环
            right_ring_num++;
    }
    Ring_Help_Flag = 0;
}


uint8 LRings_Size_get_Flag=0;

//--------------------------------------------------------------
//  @name           Element_Handle_Left_Rings()
//  @brief          整个图像处理的子函数，用来处理左圆环类型.
//  @parameter      void
//  @note           圆环状态机：0：无环岛 1~4：准备进环 5~6：进环 7：环内 8：准备出环 9：已出环
//  @time
//  @Author
//  Sample usage:   Element_Handle_Left_Rings();
//-------------------------------------------------------------
void Element_Handle_Left_Rings()
{
    ImageFlag.ring_big_small=1;
    /***************************************判断**************************************/
    int num = 0;
    for (int Ysite = 65; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')
        {
            num++;
        }
        if(    ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
            && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T' )
            break;
    }
    //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>15)          //圆环底部黑色还没消失
    {
        ImageFlag.image_element_rings_flag = 2;//左环岛入口处左边线丢失很多，进入状态2
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<5)           //圆环底部黑色已经消失
    {
        ImageFlag.image_element_rings_flag = 5;//左边线丢失数明显减少，车已经进入环岛区域，状态5
    }

            Small_Rings_HalfWide_Offset=13;
    //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Miss_Right_lines>15)//
    {
        ImageFlag.image_element_rings_flag = 6;//右边线丢失明显增加，说明正在往环岛里面走
    }
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Miss_Right_lines<7)
    {
        ImageFlag.image_element_rings_flag = 7;//右边线恢复后，进入环内部状态
    }
    //环内定位
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 45; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if (    ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 4].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 4].RightBorder
               )                                                    //此点向左边凸
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;         //出环时要补线的那个尖尖的角
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 25)
          ImageFlag.image_element_rings_flag = 8;//准备出环
    }
    //出环后
    if (ImageFlag.image_element_rings_flag == 8)
    {
        if (    Straight_Judge(2, ImageStatus.OFFLine+15, 55) < 5
             && ImageStatus.Miss_Right_lines < 8
             && ImageStatus.OFFLine < 10)                            //右边为直线且截止行（前瞻值）很小
        {
            ImageFlag.image_element_rings_flag = 9;//右边线重新变直，右减少，图像扫描截止行也正常，就进入状态9
        }
    }
    //结束圆环进程
    if (ImageFlag.image_element_rings_flag == 9/* && ImageFlag.ring_big_small==1*/)
    {
        int num=0;
        for (int Ysite = 65; Ysite > 30; Ysite--)
        {
            if(ImageDeal[Ysite].IsLeftFind == 'W' )//左边线恢复正常，并且左边线比较直，就退出环岛
                num++;
        }
//        ips200_show_uint(20,180,num,3);
        if( num<5 &&Straight_Judge(1, 30, 69)<4)
        {
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            LRings_Size_get_Flag=0;
//            Front_Ring_Continue_Count++;
            BeeOff;
        }
    }


    /***************************************处理**************************************/
    //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            int left = ImageDeal[Ysite].LeftBorder;
            int right = ImageDeal[Ysite].RightBorder;
            int wide = right - left;
            int left_valid = (ImageDeal[Ysite].IsLeftFind == 'T' &&
                              left > 2 && left < LCDW - 2);
            int right_valid = (ImageDeal[Ysite].IsRightFind == 'T' &&
                               right > 2 && right < LCDW - 2);
            int center = ImageDeal[Ysite].Center;

            /* 必须实际找到两侧边线，才允许使用双边平均。 */
            if (left_valid && right_valid && wide >= 20 && wide <= 140)
            {
                center = (left + right) / 2;
            }
            else if (right_valid)
            {
                center = right - Half_Road_Wide[Ysite];
            }
            else if (left_valid)
            {
                center = left + Half_Road_Wide[Ysite];
            }
            /* 两侧都无效时，保留本帧普通搜线生成的中线。 */

            LimitL(center);
            LimitH(center);
            ImageDeal[Ysite].Center = center;
        }
    }

    //进环  半宽处理
    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6
        )
    {
        if(ImageFlag.ring_big_small==1)
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Ring_Wide_small[Ysite]-Small_Rings_HalfWide_Offset;
                    LimitL(ImageDeal[Ysite].Center);
            }
        }
        else
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Ring_Wide[Ysite]-Small_Rings_HalfWide_Offset;
                    LimitL(ImageDeal[Ysite].Center);
            }
        }
    }
    //环内  半宽处理
    if (ImageFlag.image_element_rings_flag == 7)
    {
        if(ImageFlag.ring_big_small==1)
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Ring_Wide_small[Ysite]-Small_Rings_HalfWide_Offset;
                    LimitL(ImageDeal[Ysite].Center);
            }
        }
        else
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Ring_Wide[Ysite]-Small_Rings_HalfWide_Offset;
                    LimitL(ImageDeal[Ysite].Center);
            }
        }
    }
    //出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 8)                                      //大圆环
    {
        if(ImageFlag.ring_big_small==1)
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Ring_Wide_small[Ysite]-Small_Rings_HalfWide_Offset+10;
                    LimitL(ImageDeal[Ysite].Center);
            }
        }
        else
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Ring_Wide[Ysite]-Small_Rings_HalfWide_Offset;
                    LimitL(ImageDeal[Ysite].Center);
            }
        }
    }
    //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 69; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}

uint8 RRings_Size_get_Flag=0;
//--------------------------------------------------------------
//  @name           Element_Handle_Right_Rings()
//  @brief          整个图像处理的子函数，用来处理右圆环类型.
//  @parameter      void
//  @time
//  @Author
//  Sample usage:   Element_Handle_Right_Rings();
//-------------------------------------------------------------
void Element_Handle_Right_Rings()
{
    //直接赋值小环
    ImageFlag.ring_big_small = 1;
    /****************判断*****************/
    int num =0 ;
    for (int Ysite = 65; Ysite > 30; Ysite--)
    {
        if(ImageDeal[64].IsRightFind == 'W')
        {
            if(ImageDeal[Ysite].IsRightFind == 'W')
           {
                num++;
           }
        }
        else
            num=0;
        if(    ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T' )
            break;
    }
    //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>20)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<5)
    {
        ImageFlag.image_element_rings_flag = 5;
    }
    //大小圆环判断
    if(ImageFlag.image_element_rings_flag == 5)
    {
        Small_Rings_HalfWide_Offset=13;
    }
//    进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Miss_Left_lines>15)
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Miss_Left_lines<7)
    {
        ImageFlag.image_element_rings_flag = 7;
        //Stop=1;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 45; Ysite > ImageStatus.OFFLine+3; Ysite--)
        {
            if (    ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder
                )

            {
                        Point_Xsite = ImageDeal[Ysite].LeftBorder;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 25)
        {
            //Stop = 1;
            ImageFlag.image_element_rings_flag = 8;
        }
    }
//    出环后
    if (ImageFlag.image_element_rings_flag == 8)
    {
        int help=0;
        float k=100.0;

         if (
                (Straight_Judge(1, ImageStatus.OFFLine+15, 60) < 5//左边线比较直
             && (ImageStatus.Miss_Left_lines < 8)
             && ImageStatus.OFFLine < 10)
//             ||(k<1.0&&ImageStatus.OFFLine < 13)
             )    //右边为直线且截止行（前瞻值）很小
            {ImageFlag.image_element_rings_flag = 9;}
    }
    if(ImageFlag.image_element_rings_flag == 9 )
    {
        int num=0;
        for (int Ysite = 65; Ysite > 30; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
            {
                num++;
            }
        }

        if(num < 5&&Straight_Judge(2, 30, 69)<4)
        {
//            BeeOff;
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            RRings_Size_get_Flag=0;
//            Front_Ring_Continue_Count++;
        }
    }


    /***************************************处理**************************************/
    //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            int left = ImageDeal[Ysite].LeftBorder;
            int right = ImageDeal[Ysite].RightBorder;
            int wide = right - left;
            int left_valid = (ImageDeal[Ysite].IsLeftFind == 'T' &&
                              left > 2 && left < LCDW - 2);
            int right_valid = (ImageDeal[Ysite].IsRightFind == 'T' &&
                               right > 2 && right < LCDW - 2);
            int center = ImageDeal[Ysite].Center;

            /* 必须实际找到两侧边线，才允许使用双边平均。 */
            if (left_valid && right_valid && wide >= 20 && wide <= 140)
            {
                center = (left + right) / 2;
            }
            else if (right_valid)
            {
                center = right - Half_Road_Wide[Ysite];
            }
            else if (left_valid)
            {
                center = left + Half_Road_Wide[Ysite];
            }
            /* 两侧都无效时，保留本帧普通搜线生成的中线。 */

            LimitL(center);
            LimitH(center);
            ImageDeal[Ysite].Center = center;
        }
    }

    //进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 5//入环
        || ImageFlag.image_element_rings_flag == 6//入环
       )
    {
        if(ImageFlag.ring_big_small==1)//小环半宽补线
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Ring_Wide_small[Ysite]+Small_Rings_HalfWide_Offset;//加5是为了提前入环
                    LimitH(ImageDeal[Ysite].Center);
            }
        }
        else//右环半宽补线
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Ring_Wide[Ysite]+Small_Rings_HalfWide_Offset;
                    LimitH(ImageDeal[Ysite].Center);
            }
        }
    }
    //环内  半宽处理
    if (ImageFlag.image_element_rings_flag == 7)
    {
        if(ImageFlag.ring_big_small==1)//小环
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Ring_Wide_small[Ysite]+Small_Rings_HalfWide_Offset;
                    LimitH(ImageDeal[Ysite].Center);
            }
        }
        else//大环
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Ring_Wide[Ysite]+Small_Rings_HalfWide_Offset;
                    LimitH(ImageDeal[Ysite].Center);
            }
        }
    }

    //出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 8)  //大圆环
{
        if(ImageFlag.ring_big_small==1)
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Ring_Wide_small[Ysite]+Small_Rings_HalfWide_Offset-10;
                    LimitH(ImageDeal[Ysite].Center);
            }
        }
        else
        {
            for (int Ysite = 69; Ysite > 20; Ysite--)
            {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Ring_Wide[Ysite]+Small_Rings_HalfWide_Offset;
                    LimitH(ImageDeal[Ysite].Center);
            }
        }
    }
    //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 69; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           ControlMid_Update
//  @brief
//  @parameter      void
//  @time
//  @Author
//  Sample usage:   ControlMid_Update
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8 Vision_SelectProspectType(void)
{
    /* 斑马线优先 */
    if (ImageFlag.Zebra_Flag == 1)
    {
        return VP_ZEBRA;
    }

    /* 圆环 */
    if (ImageFlag.image_element_rings != 0)
    {
        if (ImageFlag.image_element_rings_flag <= 2)
        {
            return VP_RING_ENTRY;
        }

        if (ImageFlag.image_element_rings_flag >= 8)
        {
            return VP_RING_EXIT;
        }

        if (ImageFlag.ring_big_small == 2)
        {
            return VP_RING_INSIDE_BIG;
        }

        return VP_RING_INSIDE;
    }

    /* 十字 */
    if (ImageFlag.Cross_Flag == 1)
    {
        return VP_CROSS;
    }

    return VP_NORMAL;
}
// @name  ControlMid_SelectNearbyReference
// @brief 固定前瞻行无可靠参考时，只在近侧两行内寻找当前帧连续白路。
// @note  必须连续三行、每行横移不超过4像素，平均行内不得跨黑区；不跨帧造线。
static uint8 ControlMid_SelectNearbyReference(int preferred_row, int *selected_row,
                                            int *selected_value)
{
    int row, x, a, b, c, low, high;
    for (row = preferred_row + 1; row <= preferred_row + 2 && row < LCDH - 1; row++)
    {
        if (row < 1)
            continue;
        a = ControlMidLine[row - 1];
        b = ControlMidLine[row];
        c = ControlMidLine[row + 1];
        if (a <= 2 || a >= LCDW - 2 || b <= 2 || b >= LCDW - 2 ||
            c <= 2 || c >= LCDW - 2)
            continue;
        if (b - a < -4 || b - a > 4 || c - b < -4 || c - b > 4)
            continue;
        if (Pixle[row - 1][a] != 255 || Pixle[row][b] != 255 ||
            Pixle[row + 1][c] != 255)
            continue;
        low = a < b ? a : b;
        if (c < low) low = c;
        high = a > b ? a : b;
        if (c > high) high = c;
        for (x = low; x <= high && Pixle[row][x] == 255; x++)
        {
            /* 三行中心在当前候选行必须属于同一连续白段。 */
        }
        if (x <= high)
            continue;
        *selected_row = row;
        *selected_value = (a + b + c) / 3;
        return 1;
    }
    return 0;
}

// @name  Vision_SelectDynamicProspectRow
// @brief 根据斜率绝对值调整普通赛道前瞻行，斜率越大越靠近车头。
// @note  行号越大越近，进弯收近和出弯回远分别限制每帧步长。
static int Vision_SelectDynamicProspectRow(int16 slope)
{
    const int slope_span = VP_DYNAMIC_SLOPE_FULL - VP_DYNAMIC_SLOPE_START;
    const int row_span = VP_DYNAMIC_NEAR_ROW - VP_NORMAL_ROW;
    int slope_abs = slope < 0 ? -(int)slope : slope;
    int slope_delta = slope_abs - VP_DYNAMIC_SLOPE_START;
    int target_row;

    if (slope_delta < 0)
    {
        slope_delta = 0;
    }
    else if (slope_delta > slope_span)
    {
        slope_delta = slope_span;
    }

    /* 整数线性插值，取最近的整数行 */
    target_row = VP_NORMAL_ROW + (row_span * slope_delta + slope_span / 2) / slope_span;
    if (target_row > Vision_DynamicProspectRow + VP_DYNAMIC_STEP_NEAR)
    {
        target_row = Vision_DynamicProspectRow + VP_DYNAMIC_STEP_NEAR;
    }
    else if (target_row < Vision_DynamicProspectRow - VP_DYNAMIC_STEP_FAR)
    {
        target_row = Vision_DynamicProspectRow - VP_DYNAMIC_STEP_FAR;
    }

    return target_row;
}

// @name  ControlMid_ReadReference
// @brief 读取指定前瞻行的中线参考，普通道路和十字要求当前白路支撑。
static uint8 ControlMid_ReadReference(int row, uint8 require_white_road, int *value)
{
    int i;
    int start;
    int end;
    int sum;
    int cnt;

    /* 3. 先取控制行附近三行 */
    start = row - 1;
    end   = row + 1;

    if (start < 0)
    {
        start = 0;
    }

    if (end >= LCDH)
    {
        end = LCDH - 1;
    }

    sum = 0;
    cnt = 0;

    for (i = start; i <= end; i++)
    {
        if (ControlMidLine[i] >= 0)
        {
            sum += ControlMidLine[i];
            cnt++;
        }
    }

    /* 4. 附近不够，再扩大到上下5行 */
    if (cnt == 0)
    {
        start = row - 5;
        end   = row + 5;

        if (start < 0)
        {
            start = 0;
        }

        if (end >= LCDH)
        {
            end = LCDH - 1;
        }

        for (i = start; i <= end; i++)
        {
            if (ControlMidLine[i] >= 0)
            {
                sum += ControlMidLine[i];
                cnt++;
            }
        }
    }

    if (cnt == 0 || (require_white_road && (cnt < 2 || Pixle[row][sum / cnt] != 255)))
    {
        return 0;
    }

    *value = sum / cnt;
    return 1;
}

void ControlMid_Update(void)
{
    int i;
    int center;
    int left;
    int right;
    int wide;
    int fixed_row;
    int row;
    int value;
    int16 slope = 0;

    uint8 ring_active;
    uint8 require_white_road;
    uint8 slope_valid;
    uint8 reference_supported = 0;

    ControlMidValid = 0;
    ControlMidError = 0;

    ring_active = (ImageFlag.image_element_rings != 0);
    require_white_road = !ring_active && !ImageFlag.Zebra_Flag;

    /* 1. 先生成每一行的控制中线 */
    for (i = 0; i < LCDH; i++)
    {
        center = ImageDeal[i].Center;
        left   = ImageDeal[i].LeftBorder;
        right  = ImageDeal[i].RightBorder;
        wide   = right - left;

        if (ring_active &&
            center > 2 &&
            center < LCDW - 2)
        {
            /* 圆环状态下优先相信元素补线 */
            ControlMidLine[i] = center;
        }
        else if (center > 2 &&
                 center < LCDW - 2 &&
                 right > left &&
                 wide >= 8 &&
                 wide <= LCDW - 5)
        {
            ControlMidLine[i] = center;
        }
        else if (NormalMidLine[i] > 2 &&
                 NormalMidLine[i] < LCDW - 2)
        {
            ControlMidLine[i] = NormalMidLine[i];
        }
        else
        {
            ControlMidLine[i] = -1;
        }
        /* 普通道路和十字必须有当前白路支撑，黑区中线不得参与控制。 */
        if (!ring_active && !ImageFlag.Zebra_Flag && ControlMidLine[i] >= 0 &&
            Pixle[i][ControlMidLine[i]] != 255)
            ControlMidLine[i] = -1;
    }

    /* 2. 根据元素状态选择控制行 */
    ControlProspectType = Vision_SelectProspectType();
    fixed_row = Vision_ProspectRow[ControlProspectType];

    if (fixed_row >= LCDH)
    {
        fixed_row = LCDH - 1;
    }

    /* 普通赛道用本帧斜率调整前瞻行，坡道和避障仍用固定行。 */
    row = fixed_row;
    slope_valid = Track_Slope_Sample(&slope);
    if (ControlProspectType == VP_NORMAL && slope_valid &&
        !ImageFlag.Ramp && !ImageFlag.SmallBlock)
    {
        row = Vision_SelectDynamicProspectRow(slope);
        if (row != fixed_row && ControlMid_ReadReference(row, require_white_road, &value))
        {
            Vision_DynamicProspectRow = (uint8)row;
            reference_supported = 1;
        }
    }

    /* 动态行无可靠参考时，回到固定行及其近侧白路。 */
    if (!reference_supported)
    {
        Vision_DynamicProspectRow = VP_NORMAL_ROW;
        row = fixed_row;
        reference_supported = ControlMid_ReadReference(row, require_white_road, &value);

        /* 原固定行优先；失效时只接受近侧连续三行支持的真实白路。 */
        if (require_white_road && !reference_supported)
        {
            reference_supported = ControlMid_SelectNearbyReference(row, &row, &value);
        }
    }

    ControlProspectRow = (uint8)row;
    if (reference_supported)
    {
        ControlMidValue = value;
        ControlMidError = ImageSensorMid - ControlMidValue;
        ControlMidValid = 1;
    }
    else
    {
        ControlMidValue = ImageSensorMid;
        ControlMidError = 0;
        ControlMidValid = 0;
    }

    Track_Slope_Publish(slope_valid, slope);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           LineLost_Protection_Update
//  @brief
//  @parameter      void
//  @note           每帧检查 ControlMidValid → 连续无效则增加 LineLostCount → 达到阈值后设置 LineLostFlag
//  @note           → 严重后设置 VisionStopFlag → ControlMidValid 恢复后清零计数
//  @time
//  @Author
//  Sample usage:   保护
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//丢线保护状态量：
volatile uint8 LineLostCount = 0;
volatile uint8 LineLostFlag  = 0;
volatile uint8 VisionStopFlag = 0;
void LineLost_Protection_Update(void)
{
    if (ControlMidValid)//若ControlMidValid = 1：LineLostCount 清零
    {
        LineLostCount = 0;
        LineLostFlag  = 0;
        return;
    }

    if (LineLostCount < 255)
    {
        LineLostCount++;
    }

    if (LineLostCount >= 5)//连续 5 帧没有有效控制中线时
    {
        LineLostFlag = 1;// LineLostFlag标志位置1 控制组看到它以后可以减速
    }

    if (LineLostCount >= 30)//连续 30 帧无效
    {
        VisionStopFlag = 1;//VisionStopFlag置1 控制组看到它以后可以停车
    }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Process
//  @brief          整个图像处理的主函数，里面包含了所有的图像处理子函数
//  @parameter      void
//  @time           2023年2月19日
//  @Author
//  Sample usage:   Image_Process();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Scan_Element(void)
{
    Element_Judgment_Zebra();

    Element_Judgement_Left_Rings();
    Element_Judgment_Right_Rings();  

    Element_Judgement_Cross();

    Straight_long_judge();

    ImageFlag.SmallBlock = Element_Judgement_SmallBlock();
}
/*元素处理函数*/
void Element_Handle()
{


    if(ImageFlag.straight_long == 1)
    {
        Straight_long_handle();
    }
    if(ImageFlag.Zebra_Flag == 1)//斑马线
    {
        Element_Handle_Zebra();
    }
    if(ImageFlag.image_element_rings == 2)
    {
        Element_Handle_Right_Rings();
    }
    if(ImageFlag.image_element_rings == 1)
    {
        Element_Handle_Left_Rings();
    }

    if (!ImageFlag.Cross_Flag &&
    !ImageFlag.Zebra_Flag &&
    !ImageFlag.image_element_rings &&
    !ImageFlag.Ramp &&
    (ImageFlag.SmallBlock == 1 || ImageFlag.SmallBlock == 2))//1：右障碍物，2：左障碍物
    {
        Element_Handle_SmallBlock();
    }  


    if(ImageFlag.Cross_Flag==1)
    {
        Element_Handle_Cross();
    }
    Protect_Without_Road(LCDH-6,LCDH-1,5,LCDW-6);

}
uint8 Image_Process(void)
{
    if (mt9v03x_finish_flag == 1)                         //如果一帧图像采集完了，那么就可以对这副图像进行处理了。摄像头完成一帧 ->二值化

    {
          VisionFrameSequence++;
          image_processing = 1;
          mt9v03x_finish_flag = 0;

          if (1 == vision_runtime_reset_request)
          {
              vision_runtime_reset_request = 2;
              Camera_ClearRuntimeState();
          }
//        camera_send_image(DEBUG_UART_INDEX, (const uint8 *)Pixle, (LCDH*LCDW));      //上位机显示
//        usar_image((const uint8 *)Pixle,(LCDH*LCDW));//串口图传
          memset(ImageDeal, 0, sizeof(ImageDeal));

          //把原始图像二值化
          Get_BinaryImage();                              //调用此函数时，记得在Get_Border_And_SideType把1改成255，不用的时候改回来

          //找底部基础边线
          Get_BaseLine();

          //找所有行的边线和中线
          Get_AllLine();

          //计算正常中线
          NormalMid_Update();

          Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);

          //元素判断
          Scan_Element();
          //元素处理
          Element_Handle();

          //
          ControlMid_Update();
          //保护计算
          LineLost_Protection_Update();


           //正常中线有效时，控制层使用正常中线误差
           if (ControlMidValid)
           {
              ImageStatus.Det_True = ControlMidError;
           }


          image_processing = 0;
          VisionFrameSequence++;
          if (2 == vision_runtime_reset_request)
          {
              vision_runtime_reset_request = 0;
          }
          return 1;
    }

    return 0;
}
