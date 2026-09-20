/*
 * Camera.c
 *
 *  Created on: 2023年11月16日
 *      Author: 周文烽
 */
//sobel
#include "Camera.h"
#include "math.h"

#include <stdio.h>

/*图像基础信息的变量*/
//uint8* Image_Use;                                   //用来存储压缩之后灰度图像的二维数组
uint8 Pixle[LCDH][LCDW];                                        //图像处理时真正处理的二值化图像数组
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

/* 正常中线使用的远端前瞻行 */
uint8 Prospect_line = 26;

/* Get_AllLine补线逻辑需要 */
uint8 ExtenRFlag = 0;
uint8 ExtenLFlag = 0;
int Cross_Corner_R = 0;
int Cross_Corner_L = 0;



/*其他*/
#define LimitP_L(x) (x=((x>ImageStatus.OFFLine)?x:(ImageStatus.OFFLine+2)))//对图像的前瞻进行限制，不要超过扫线截止行（后续用动态前瞻控制行的时候那里限制，这里删掉）
#define Scan_type 1                                             //选择扫线类型 1：梯度   2：梯度加重定义



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

        if (R_found_point >8)                                                   //限制车不在丢线太近或者太远进行斜率计算，适合得位置进行计算/zwf
        {
          D_R = ((float)(ImageDeal[ytemp_W_R + R_found_point].RightBorder - ImageDeal[ytemp_W_R + 3].RightBorder)) /((float)(R_found_point - 3));//丢线记录在Ysite+2位置，所以这里是3？
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
          ImageStatus.Miss_Left_lines++;//左丢线数
     }
     if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite<LCDH-5)
     {
          ImageStatus.Miss_Right_lines++;//右丢线数
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
void NormalMid_Update(void)
{
    int i;

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

    /* 1. 保存普通中线
     * 必须在元素处理之前保存，避免 Center 被十字、环岛等逻辑修改
     */
    for (i = 0; i < LCDH; i++)
    {
        NormalMidLine[i] = ImageDeal[i].Center;
    }

    /* 2. 计算近端中线
     * 取图像底部5行，靠近车头的部分通常最稳定
     */
    for (i = LCDH - 5; i < LCDH; i++)
    {
        if (ImageDeal[i].LeftBorder < ImageDeal[i].RightBorder)
        {
            near_sum += ImageDeal[i].Center;
            near_cnt++;
        }
    }

    /* 3. 计算远端中线
     * 使用前瞻行附近的3行，让控制提前感知前方弯道
     */
    far_start = Prospect_line;
    far_end = Prospect_line + 2;

    if (far_end >= LCDH)
    {
        far_end = LCDH - 1;
    }

    if (ImageStatus.OFFLine < far_start)
    {
        for (i = far_start; i <= far_end; i++)
        {
            if (ImageDeal[i].LeftBorder < ImageDeal[i].RightBorder)
            {
                far_sum += ImageDeal[i].Center;
                far_cnt++;
            }
        }
    }

    /* 4. 近端和远端加权，得到最终正常中线 */
    if (near_cnt > 0)
    {
        near_value = near_sum / near_cnt;

        if (far_cnt > 0)
        {
            far_value = far_sum / far_cnt;

            /* 近端占7成，远端占3成
             * 近端保证稳定性，远端提供弯道预判
             */
            NormalMidValue = (7 * near_value + 3 * far_value) / 10;
        }
        else
        {
            /* 远端无效时，只使用近端中线 */
            NormalMidValue = near_value;
        }

        /* 计算中线误差，供舵机 PID 使用 */
        NormalMidError = ImageSensorMid - NormalMidValue;

        /* 本帧正常中线有效 */
        NormalMidValid = 1;
    }
    else
    {
        /* 近端都找不到有效中线，本帧标记为无效 */
        NormalMidValid = 0;
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


void Image_Process(void)
{
    if (mt9v03x_finish_flag == 1)                         //如果一帧图像采集完了，那么就可以对这副图像进行处理了。摄像头完成一帧 ->二值化

    {
//        camera_send_image(DEBUG_UART_INDEX, (const uint8 *)Pixle, (LCDH*LCDW));      //上位机显示
//        usar_image((const uint8 *)Pixle,(LCDH*LCDW));//串口图传
          //把原始图像二值化
          Get_BinaryImage();                              //调用此函数时，记得在Get_Border_And_SideType把1改成255，不用的时候改回来

          //找底部基础边线
          Get_BaseLine();

          //找所有行的边线和中线
          Get_AllLine();

          //计算正常中线
          NormalMid_Update();
           
           //正常中线有效时，控制层使用正常中线误差
           if (NormalMidValid)
            {
                 ImageStatus.Det_True = NormalMidError;
            }
    
          
     mt9v03x_finish_flag=0;     //这一帧已经处理完了  可以开始下一帧


    }
}






