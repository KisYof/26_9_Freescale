/*
 *image_eight.c
 *mt9v03x_image
→ 复制灰度图
→ OTSU 二值化
→ 八邻域形态学滤波
→ 图像四周封黑边
→ 底部找左右种子
→ 八邻域左右爬线
→ 得到二维轮廓点
→ 每行提取左、右边线
→ 计算八邻域中线
→ 保存到 eight_frame
 * @version 1.0
 * @brief 8邻域图像处理

*/
#include <stddef.h>
#include "image_eight.h"


int16 eight_center_crop[LCDH];
uint8 eight_center_valid[LCDH];

int eight_diff_rows;
int eight_diff_average;
int eight_diff_max;
static int Eight_Abs(int value);
static int16 Eight_Limit(int16 x, int a, int b);
static uint8 Eight_OtsuThreshold(uint8 *image, uint16 col, uint16 row);
static void Eight_GetImage(const uint8 image[EIGHT_IMAGE_H][EIGHT_IMAGE_W]);
static void Eight_Binarize(void);
static void Eight_Filter(uint8 image[EIGHT_IMAGE_H][EIGHT_IMAGE_W]);
static void Eight_DrawRectangle(uint8 image[EIGHT_IMAGE_H][EIGHT_IMAGE_W]);
static uint8 Eight_GetStartPoint(uint8 start_row);
static void Eight_SearchLeftRight(
    uint16 break_flag,
    uint8 image[EIGHT_IMAGE_H][EIGHT_IMAGE_W],
    uint16 *l_stastic,
    uint16 *r_stastic,
    uint8 l_start_x,
    uint8 l_start_y,
    uint8 r_start_x,
    uint8 r_start_y,
    uint8 *hightest
);
static void Eight_GetLeft(uint16 total_l);
static void Eight_GetRight(uint16 total_r);
EightNeighborFrame eight_frame;

//static uint8 eight_original[EIGHT_IMAGE_H][EIGHT_IMAGE_W];
static uint8 eight_bin[EIGHT_IMAGE_H][EIGHT_IMAGE_W];//图像数组
static uint8 eight_threshold;

static uint8 eight_l_border[EIGHT_IMAGE_H];
static uint8 eight_r_border[EIGHT_IMAGE_H];

static float Eight_SlopeCalculate(
    uint8 begin,
    uint8 end,
    const uint8 *border
);

static void Eight_CalculateSlopeIntercept(
    uint8 start,
    uint8 end,
    const uint8 *border,
    float *slope_rate,
    float *intercept
);

static void Eight_CrossFill(
    uint8 (*image)[EIGHT_IMAGE_W],
    uint8 *left_border,
    uint8 *right_border,
    uint16 total_num_l,
    uint16 total_num_r,
    const uint16 *dir_l,
    const uint16 *dir_r,
    const uint16 (*points_l)[2],
    const uint16 (*points_r)[2]
);

static int16 eight_left_crop[LCDH];
static int16 eight_right_crop[LCDH];

static uint8 eight_left_valid_crop[LCDH];
static uint8 eight_right_valid_crop[LCDH];

static uint8 eight_left_raw_valid[EIGHT_IMAGE_H];
static uint8 eight_right_raw_valid[EIGHT_IMAGE_H];
/*
函数名称：Eight_Abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  Eight_Abs( x)；
 */
int Eight_Abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 Eight_Limit(int16 x, int a, int b)//坐标限幅
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------

static void Eight_GetImage(
    const uint8 image[EIGHT_IMAGE_H][EIGHT_IMAGE_W])
{
    for (uint16 y = 0; y < EIGHT_IMAGE_H; y++)
    {
        for (uint16 x = 0; x < EIGHT_IMAGE_W; x++)
        {
            eight_bin[y][x] = image[y][x];//这里的参数是摄像头采集到的图像
        }
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 Eight_OtsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};
	
	uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;
	
	
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      生成二值图，这里用的是大津法二值化。
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------

void Eight_Binarize(void)
{
  uint8 i,j;
 eight_threshold = Eight_OtsuThreshold(eight_bin[0], EIGHT_IMAGE_W, EIGHT_IMAGE_H);
  for(i = 0;i<EIGHT_IMAGE_H;i++)
  {
      for(j = 0;j<EIGHT_IMAGE_W;j++)
      {
          if(eight_bin[i][j]>eight_threshold)eight_bin[i][j] = EIGHT_WHITE_PIXEL;
          else eight_bin[i][j] = EIGHT_BLACK_PIXEL;
      }
  }
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255*5//此参数根据需求调节
#define threshold_min	255*2//此参数根据需求调节
void Eight_Filter(uint8(*eight_bin)[EIGHT_IMAGE_W])//八邻域形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < EIGHT_IMAGE_H - 1; i++)
	{
		for (j = 1; j < (EIGHT_IMAGE_W - 1); j++)
		{
			//统计八个方向的像素值
			num =
				eight_bin[i - 1][j - 1] + eight_bin[i - 1][j] + eight_bin[i - 1][j + 1]
				+ eight_bin[i][j - 1] + eight_bin[i][j + 1]
				+ eight_bin[i + 1][j - 1] + eight_bin[i + 1][j] + eight_bin[i + 1][j + 1];


			if (num >= threshold_max && eight_bin[i][j] == 0)
			{

				eight_bin[i][j] = 255;//白  可以搞成宏定义，方便更改

			}
			if (num <= threshold_min && eight_bin[i][j] == 255)
			{

				eight_bin[i][j] = 0;//黑

			}

		}
	}

}
/*
    函数名称：void Eight_DrawRectangle(uint8(*image)[EIGHT_IMAGE_W])
    功能说明：给图像画一个黑框
参数说明：uint8(*image)[EIGHT_IMAGE_W]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(eight_bin);
 */
void Eight_DrawRectangle(uint8(*image)[EIGHT_IMAGE_W])
{

	uint8 i = 0;
	for (i = 0; i < EIGHT_IMAGE_H; i++)
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][EIGHT_IMAGE_W - 1] = 0;
		image[i][EIGHT_IMAGE_W - 2] = 0;

	}
	for (i = 0; i < EIGHT_IMAGE_W; i++)
	{
		image[0][i] = 0;
		image[1][i] = 0;
		//image[EIGHT_IMAGE_H-1][i] = 0;

	}
}
/*
函数名称：void Eight_GetStartPoint(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
备    注：
example：  Eight_GetStartPoint(EIGHT_IMAGE_H-2)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 Eight_GetStartPoint(uint8 start_row)
{
	uint8 i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = EIGHT_IMAGE_W / 2; i > EIGHT_BORDER_MIN; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (eight_bin[start_row][i] == 255 && eight_bin[start_row][i - 1] == 0)
		{
			//printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = EIGHT_IMAGE_W / 2; i < EIGHT_BORDER_MAX; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (eight_bin[start_row][i] == 255 && eight_bin[start_row][i + 1] == 0)
		{
			//printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("未找到起点\n");
		return 0;
	} 
}

/*
函数名称：void Eight_SearchLeftRight(uint16 break_flag, uint8(*image)[EIGHT_IMAGE_W],uint16 *l_stastic, uint16 *r_stastic,
							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r			：最多需要循环的次数
(*image)[EIGHT_IMAGE_W]		：需要进行找点的图像数组，必须是二值图,填入数组名称即可
					   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic				：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic				：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x				：左边起点横坐标
l_start_y				：左边起点纵坐标
r_start_x				：右边起点横坐标
r_start_y				：右边起点纵坐标
hightest				：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
	Eight_SearchLeftRight((uint16)USE_num,eight_bin,&data_stastics_l, &data_stastics_r,start_point_l[0],
				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num	EIGHT_IMAGE_H*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 eight_points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 eight_points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 eight_points_l_num = 0;//统计左边找到点的个数
uint16 eight_points_r_num = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void Eight_SearchLeftRight(uint16 break_flag, uint8(*image)[EIGHT_IMAGE_W], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

	uint8 i = 0, j = 0;

	//左边变量
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;//统计左边
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针

	//右边变量
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };//中心坐标点
	uint8 index_r = 0;//索引下标
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//统计右边
	//定义八个邻域
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//开启邻域循环
	while (break_flag--)
	{

		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		eight_points_l[l_data_statics][0] = center_point_l[0];//x
		eight_points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		eight_points_r[r_data_statics][0] = center_point_r[0];//x
		eight_points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0];
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i);//记录生长方向
			}

			if (index_l)
			{
				//更新坐标点
				center_point_l[0] = temp_l[0][0];//x
				center_point_l[1] = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l[1] > temp_l[j][1])
					{
						center_point_l[0] = temp_l[j][0];//x
						center_point_l[1] = temp_l[j][1];//y
					}
				}
			}

		}
		if ((eight_points_r[r_data_statics][0]== eight_points_r[r_data_statics-1][0]&& eight_points_r[r_data_statics][0] == eight_points_r[r_data_statics - 2][0]
			&& eight_points_r[r_data_statics][1] == eight_points_r[r_data_statics - 1][1] && eight_points_r[r_data_statics][1] == eight_points_r[r_data_statics - 2][1])
			||(eight_points_l[l_data_statics-1][0] == eight_points_l[l_data_statics - 2][0] && eight_points_l[l_data_statics-1][0] == eight_points_l[l_data_statics - 3][0]
				&& eight_points_l[l_data_statics-1][1] == eight_points_l[l_data_statics - 2][1] && eight_points_l[l_data_statics-1][1] == eight_points_l[l_data_statics - 3][1]))
		{
			//printf("三次进入同一个点，退出\n");
			break;
		}
		if (Eight_Abs(eight_points_r[r_data_statics][0] - eight_points_l[l_data_statics - 1][0]) < 2
			&& Eight_Abs(eight_points_r[r_data_statics][1] - eight_points_l[l_data_statics - 1][1]) < 2
			)
		{
			//printf("\n左右相遇退出\n");	
			*hightest = (eight_points_r[r_data_statics][1] + eight_points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((eight_points_r[r_data_statics][1] < eight_points_l[l_data_statics - 1][1]))
		{
			//printf("\n如果左边比右边高了，左边等待右边\n");	
			continue;//如果左边比右边高了，左边等待右边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (eight_points_r[r_data_statics][1] > eight_points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			//printf("\n左边开始向下了，等待右边，等待中... \n");
			center_point_l[0] = eight_points_l[l_data_statics - 1][0];//x
			center_point_l[1] = eight_points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//索引加一
				dir_r[r_data_statics - 1] = (i);//记录生长方向
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
			if (index_r)
			{

				//更新坐标点
				center_point_r[0] = temp_r[0][0];//x
				center_point_r[1] = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r[1] > temp_r[j][1])
					{
						center_point_r[0] = temp_r[j][0];//x
						center_point_r[1] = temp_r[j][1];//y
					}
				}

			}
		}


	}


	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}
/*
函数名称：void Eight_GetLeft(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L	：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： Eight_GetLeft(eight_points_l_num);
 */
// uint8 l_border[EIGHT_IMAGE_H;//左线数组
// uint8 r_border[EIGHT_IMAGE_H];//右线数组
// uint8 center_line[EIGHT_IMAGE_H];//中线数组
#define EIGHT_EDGE_MARGIN 3
void Eight_GetLeft(uint16 total_L)
{
    uint16 j;
    uint8 y;
    uint8 x;

    for (y = 0; y < EIGHT_IMAGE_H; y++)
    {
        eight_l_border[y] = EIGHT_BORDER_MIN;
        eight_left_raw_valid[y] = 0;
    }

    for (j = 0; j < total_L; j++)
    {
        y = (uint8)eight_points_l[j][1];
        x = (uint8)(eight_points_l[j][0] + 1);

        if (y >= EIGHT_IMAGE_H || x >= EIGHT_IMAGE_W)
        {
            continue;
        }

        /* 过滤图像封边框 */
        if (x <= EIGHT_EDGE_MARGIN ||
            x >= EIGHT_IMAGE_W - 1 - EIGHT_EDGE_MARGIN)
        {
            continue;
        }

        /* 每行只取第一次出现的点 */
        if (!eight_left_raw_valid[y])
        {
            eight_l_border[y] = x;
            eight_left_raw_valid[y] = 1;
        }
    }
}
/*
函数名称：void Eight_GetRight(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：Eight_GetRight(eight_points_r_num);
 */
void Eight_GetRight(uint16 total_R)
{
    uint16 j;
    uint8 y;
    uint8 x;

    for (y = 0; y < EIGHT_IMAGE_H; y++)
    {
        eight_r_border[y] = EIGHT_BORDER_MAX;
        eight_right_raw_valid[y] = 0;
    }

    for (j = 0; j < total_R; j++)
    {
        y = (uint8)eight_points_r[j][1];
        x = (uint8)(eight_points_r[j][0] - 1);

        if (y >= EIGHT_IMAGE_H || x >= EIGHT_IMAGE_W)
        {
            continue;
        }

        /* 过滤图像封边框 */
        if (x <= EIGHT_EDGE_MARGIN ||
            x >= EIGHT_IMAGE_W - 1 - EIGHT_EDGE_MARGIN)
        {
            continue;
        }

        /* 参考工程思想：每行只取第一次出现的点 */
        if (!eight_right_raw_valid[y])
        {
            eight_r_border[y] = x;
            eight_right_raw_valid[y] = 1;
        }
    }
}
/** 
* @brief 最小二乘法斜率函数
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
*  @see CTest		Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
static float Eight_SlopeCalculate(
    uint8 begin,
    uint8 end,
    const uint8 *border)
{
    int i;
    int n;
    double xsum = 0.0;
    double ysum = 0.0;
    double xysum = 0.0;
    double x2sum = 0.0;
    double denom;

    if (border == NULL || end <= begin + 1)
    {
        return 0.0f;
    }

    n = (int)end - (int)begin;

    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += (double)i * border[i];
        x2sum += (double)i * i;
    }

    denom = (double)n * x2sum - xsum * xsum;

    if (denom > -1e-6 && denom < 1e-6)
    {
        return 0.0f;
    }

    return (float)(((double)n * xysum - xsum * ysum) / denom);
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
*  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
static void Eight_CalculateSlopeIntercept(
    uint8 start,
    uint8 end,
    const uint8 *border,
    float *slope_rate,
    float *intercept)
{
    uint16 i;
    uint16 num = 0;
    uint16 xsum = 0;
    uint16 ysum = 0;
    float x_average = 0.0f;
    float y_average = 0.0f;

    if (border == NULL || slope_rate == NULL || intercept == NULL)
    {
        return;
    }

    for (i = start; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }

    if (num > 0)
    {
        x_average = (float)xsum / (float)num;
        y_average = (float)ysum / (float)num;
    }

    *slope_rate = Eight_SlopeCalculate(start, end, border);
    *intercept = y_average - (*slope_rate) * x_average;
}
/** 
* @brief 十字补线函数
* @param uint8(*image)[image_w]		输入二值图像
* @param uint8 *l_border			输入左边界首地址
* @param uint8 *r_border			输入右边界首地址
* @param uint16 total_num_l			输入左边循环总次数
* @param uint16 total_num_r			输入右边循环总次数
* @param uint16 *dir_l				输入左边生长方向首地址
* @param uint16 *dir_r				输入右边生长方向首地址
* @param uint16(*points_l)[2]		输入左边轮廓首地址
* @param uint16(*points_r)[2]		输入右边轮廓首地址
*  @see CTest		cross_fill(image,l_border, r_border, data_statics_l, data_statics_r, dir_l, dir_r, points_l, points_r);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
static void Eight_CrossFill(
    uint8 (*image)[EIGHT_IMAGE_W],
    uint8 *left_border,
    uint8 *right_border,
    uint16 total_num_l,
    uint16 total_num_r,
    const uint16 *dir_l,
    const uint16 *dir_r,
    const uint16 (*points_l)[2],
    const uint16 (*points_r)[2])
{
    int i;
    int break_num_l = 0;
    int break_num_r = 0;
    int start_l;
    int end_l;
    int start_r;
    int end_r;
    int fill_start;
    float slope_l = 0.0f;
    float intercept_l = 0.0f;
    float slope_r = 0.0f;
    float intercept_r = 0.0f;

    if (image == NULL ||
        left_border == NULL ||
        right_border == NULL ||
        dir_l == NULL ||
        dir_r == NULL ||
        points_l == NULL ||
        points_r == NULL)
    {
        return;
    }

    /* 左边方向码判断 */
    for (i = 1; i + 7 < (int)total_num_l; i++)
    {
        if (dir_l[i - 1] == 4 &&
            dir_l[i] == 4 &&
            dir_l[i + 3] == 6 &&
            dir_l[i + 5] == 6 &&
            dir_l[i + 7] == 6)
        {
            break_num_l = points_l[i][1];
            break;
        }
    }

    /* 右边方向码判断 */
    for (i = 1; i + 7 < (int)total_num_r; i++)
    {
        if (dir_r[i - 1] == 4 &&
            dir_r[i] == 4 &&
            dir_r[i + 3] == 6 &&
            dir_r[i + 5] == 6 &&
            dir_r[i + 7] == 6)
        {
            break_num_r = points_r[i][1];
            break;
        }
    }

    if (break_num_l < 7 || break_num_r < 7)
    {
        return;
    }

    /* 底部左右两侧必须仍是白色赛道 */
    if (image[EIGHT_IMAGE_H - 1][4] == 0 ||
        image[EIGHT_IMAGE_H - 1][EIGHT_IMAGE_W - 4] == 0)
    {
        return;
    }

    /* 左边线拟合 */
    start_l = break_num_l - 15;
    end_l   = break_num_l - 5;

    if (start_l < 0)
    {
        start_l = 0;
    }

    if (end_l <= start_l + 1)
    {
        end_l = start_l + 2;
    }

    if (end_l >= EIGHT_IMAGE_H)
    {
        end_l = EIGHT_IMAGE_H - 1;
    }

    Eight_CalculateSlopeIntercept(
        (uint8)start_l,
        (uint8)end_l,
        left_border,
        &slope_l,
        &intercept_l
    );

    fill_start = break_num_l - 5;

    if (fill_start < 0)
    {
        fill_start = 0;
    }

    for (i = fill_start; i < EIGHT_IMAGE_H - 1; i++)
    {
        int16 x = (int16)(slope_l * i + intercept_l);

        left_border[i] = (uint8)Eight_Limit(
            x,
            EIGHT_BORDER_MIN,
            EIGHT_BORDER_MAX
        );
    }

    /* 右边线拟合 */
    start_r = break_num_r - 15;
    end_r   = break_num_r - 5;

    if (start_r < 0)
    {
        start_r = 0;
    }

    if (end_r <= start_r + 1)
    {
        end_r = start_r + 2;
    }

    if (end_r >= EIGHT_IMAGE_H)
    {
        end_r = EIGHT_IMAGE_H - 1;
    }

    Eight_CalculateSlopeIntercept(
        (uint8)start_r,
        (uint8)end_r,
        right_border,
        &slope_r,
        &intercept_r
    );

    fill_start = break_num_r - 5;

    if (fill_start < 0)
    {
        fill_start = 0;
    }

    for (i = fill_start; i < EIGHT_IMAGE_H - 1; i++)
    {
        int16 x = (int16)(slope_r * i + intercept_r);

        right_border[i] = (uint8)Eight_Limit(
            x,
            EIGHT_BORDER_MIN,
            EIGHT_BORDER_MAX
        );
    }
}
void EightNeighbor_ToLegacy(void)
{
    int y_old;

    for (y_old = 0; y_old < LCDH; y_old++)
    {
        int y_raw = y_old + Cut_high_top;
        int left_raw;
        int right_raw;
        int center_raw;

        int left_old;
        int right_old;
        int center_old;

        eight_center_crop[y_old] = -1;
        eight_center_valid[y_old] = 0;

        eight_left_crop[y_old] = -1;
        eight_right_crop[y_old] = -1;

        eight_left_valid_crop[y_old] = 0;
        eight_right_valid_crop[y_old] = 0;

        if (!eight_frame.valid ||
            y_raw < eight_frame.highest ||
            y_raw >= EIGHT_IMAGE_H)
        {
            continue;
        }

        left_raw   = eight_frame.left[y_raw];
        right_raw  = eight_frame.right[y_raw];
        center_raw = eight_frame.center[y_raw];

        left_old   = left_raw   - (Cut_width_half - 2);
        right_old  = right_raw  - (Cut_width_half - 2);
        center_old = center_raw - (Cut_width_half - 2);

        /* 左边线有效 */
        if (eight_left_raw_valid[y_raw] &&
    left_old >= 0 &&
    left_old < LCDW)
{
    eight_left_crop[y_old] = (int16)left_old;
    eight_left_valid_crop[y_old] = 1;
}

        /* 右边线有效 */
        if (eight_right_raw_valid[y_raw] &&
    right_old >= 0 &&
    right_old < LCDW)
{
    eight_right_crop[y_old] = (int16)right_old;
    eight_right_valid_crop[y_old] = 1;
}

        /* 中线有效 */
        if (eight_left_raw_valid[y_raw] &&
    eight_right_raw_valid[y_raw] &&
    center_old >= 0 &&
    center_old < LCDW)
{
    eight_center_crop[y_old] = (int16)center_old;
    eight_center_valid[y_old] = 1;
}
}
}
/* 八邻域主处理函数

复制原始灰度图
→ OTSU 二值化
→ 八邻域形态学滤波
→ 图像四周封黑边
→ 底部找左右种子
→ 左右八邻域爬线
→ 得到左右完整轮廓点
→ 每行提取左右边线
→ 计算八邻域中线
最终结果保存在：
eight_frame.left[]
eight_frame.right[]
eight_frame.center[]
eight_frame.points_l[]
eight_frame.points_r[]
eight_frame.valid
eight_frame.highest

*/
void EightNeighbor_Process(const uint8 image[EIGHT_IMAGE_H][EIGHT_IMAGE_W])
{
    uint8 highest = 0;

    eight_frame.valid = 0;
    eight_frame.points_l_num = 0;
    eight_frame.points_r_num = 0;

    Eight_GetImage(image);
    Eight_Binarize();
    Eight_Filter(eight_bin);
    Eight_DrawRectangle(eight_bin);

    eight_points_l_num = 0;
    eight_points_r_num = 0;

   uint8 start_row;
uint8 start_found = 0;

for (start_row = EIGHT_IMAGE_H - 2;
     start_row > 2;
     start_row--)
{
    if (Eight_GetStartPoint(start_row))
    {
        start_found = 1;
        break;
    }
}

if (!start_found)
{
    return;
}

    Eight_SearchLeftRight(
        (uint16)(EIGHT_IMAGE_H * 3),
        eight_bin,
        &eight_points_l_num,
        &eight_points_r_num,
        start_point_l[0],
        start_point_l[1],
        start_point_r[0],
        start_point_r[1],
        &highest
    );

    Eight_GetLeft(eight_points_l_num);
    Eight_GetRight(eight_points_r_num);

Eight_CrossFill(
    eight_bin,
    eight_l_border,
    eight_r_border,
    eight_points_l_num,
    eight_points_r_num,
    dir_l,
    dir_r,
    (const uint16 (*)[2])eight_points_l,
    (const uint16 (*)[2])eight_points_r
);

    eight_frame.highest = highest;
    eight_frame.points_l_num = eight_points_l_num;
    eight_frame.points_r_num = eight_points_r_num;

    for (uint16 i = 0; i < eight_points_l_num && i < EIGHT_MAX_POINTS; i++)
    {
        eight_frame.points_l[i][0] = eight_points_l[i][0];
        eight_frame.points_l[i][1] = eight_points_l[i][1];
    }

    for (uint16 i = 0; i < eight_points_r_num && i < EIGHT_MAX_POINTS; i++)
    {
        eight_frame.points_r[i][0] = eight_points_r[i][0];
        eight_frame.points_r[i][1] = eight_points_r[i][1];
    }

    for (int y = 0; y < EIGHT_IMAGE_H; y++)
    {
        eight_frame.left[y] = eight_l_border[y];
        eight_frame.right[y] = eight_r_border[y];
    }

    for (int y = highest; y < EIGHT_IMAGE_H - 1; y++)
    {
        eight_frame.center[y] =
            (eight_frame.left[y] + eight_frame.right[y]) / 2;
    }

    eight_frame.valid = 1;
}
void EightNeighbor_Compare(void)
{
    int sum = 0;
    int count = 0;
    int max_diff = 0;

    for (int y = 0; y < LCDH; y++)
    {
        if (!eight_center_valid[y] || NormalMidLine[y] < 0)
        {
            continue;
        }

        int diff = eight_center_crop[y] - NormalMidLine[y];

        if (diff < 0) diff = -diff;

        sum += diff;
        count++;

        if (diff > max_diff)
        {
            max_diff = diff;
        }
    }

    eight_diff_rows = count;
    eight_diff_average = count ? sum / count : -1;
    eight_diff_max = max_diff;
}
//以下是给调试显示用的数据搬运函数
// 蓝色 = 八邻域左边界
// 绿色 = 八邻域右边界
// 红色 = 八邻域控制中线
void EightNeighbor_PublishEdgesToImageDeal(void)
{
    int y_old;
    int y_raw;
    int left_raw;
    int right_raw;
    int left_old;
    int right_old;

    // /* 八邻域本帧无效，保留 Get_AllLine() 的边线用于调试 */
    // if (!eight_frame.valid)
    // {
    //     return;
    // }
   

    for (y_old = 0; y_old < LCDH; y_old++)
    {
        y_raw = y_old + Cut_high_top;
         ImageDeal[y_old].IsLeftFind  = 'W';
ImageDeal[y_old].IsRightFind = 'W';
ImageDeal[y_old].LeftBorder  = -1;
ImageDeal[y_old].RightBorder = -1;
ImageDeal[y_old].Center      = -1;
ImageDeal[y_old].Wide        = 0;

        if (y_raw < eight_frame.highest ||
            y_raw >= EIGHT_IMAGE_H)
        {
            continue;
        }

        left_raw  = eight_frame.left[y_raw];
        right_raw = eight_frame.right[y_raw];

        left_old  = left_raw  - (Cut_width_half - 2);
        right_old = right_raw - (Cut_width_half - 2);

        if (eight_left_raw_valid[y_raw] &&
            left_old >= 0 &&
            left_old < LCDW)
        {
            ImageDeal[y_old].LeftBorder = left_old;
            ImageDeal[y_old].IsLeftFind = 'T';
        }

        if (eight_right_raw_valid[y_raw] &&
            right_old >= 0 &&
            right_old < LCDW)
        {
            ImageDeal[y_old].RightBorder = right_old;
            ImageDeal[y_old].IsRightFind = 'T';
        }

        if (ImageDeal[y_old].IsLeftFind == 'T' &&
            ImageDeal[y_old].IsRightFind == 'T')
        {
            ImageDeal[y_old].Center =
                (ImageDeal[y_old].LeftBorder +
                 ImageDeal[y_old].RightBorder) / 2;

            ImageDeal[y_old].Wide =
                ImageDeal[y_old].RightBorder -
                ImageDeal[y_old].LeftBorder;
        }
    }
}
//圆环中线处理函数
void EightNeighbor_ApplyRingCenter(void)
{
    int y;
    int center;
    int left_ok;
    int right_ok;
    int wide;
    int half_width;

    if (ImageFlag.image_element_rings == 0)
    {
        return;
    }

    for (y = 0; y < LCDH; y++)
    {
        center = -1;
        left_ok = eight_left_valid_crop[y];
        right_ok = eight_right_valid_crop[y];

        if (left_ok && right_ok)
        {
            wide = eight_right_crop[y] - eight_left_crop[y];
        }
        else
        {
            wide = 0;
        }

        if (ImageFlag.image_element_rings == 1)
        {
            /* 左圆环 */
            if (ImageFlag.image_element_rings_flag >= 5 &&
                ImageFlag.image_element_rings_flag <= 8)
            {
                /* 进入/环内：使用左侧内线 */
                if (left_ok)
                {
                    if (ImageFlag.ring_big_small == 2)
                    {
                        half_width = Half_Ring_Wide[y];
                    }
                    else
                    {
                        half_width = Half_Ring_Wide_small[y];
                    }

                    center = eight_left_crop[y] +
                             half_width -
                             Small_Rings_HalfWide_Offset;

                    /* 准备出环，额外修正 */
                    if (ImageFlag.image_element_rings_flag == 8 &&
                        ImageFlag.ring_big_small != 2)
                    {
                        center += 10;
                    }
                }
            }
            else
            {
                /* 入口/出口：优先使用左右中点 */
                if (left_ok && right_ok &&
                    wide >= 20 &&
                    wide <= 140)
                {
                    center = (eight_left_crop[y] +
                              eight_right_crop[y]) / 2;
                }
                else if (right_ok)
                {
                    /* 只有右侧外线 */
                    center = eight_right_crop[y] -
                             Half_Road_Wide[y];
                }
                else if (left_ok)
                {
                    /* 只有左侧线 */
                    center = eight_left_crop[y] +
                             Half_Road_Wide[y];
                }
            }
        }
        else if (ImageFlag.image_element_rings == 2)
        {
            /* 右圆环 */
            if (ImageFlag.image_element_rings_flag >= 5 &&
                ImageFlag.image_element_rings_flag <= 8)
            {
                /* 进入/环内：使用右侧内线 */
                if (right_ok)
                {
                    if (ImageFlag.ring_big_small == 2)
                    {
                        half_width = Half_Ring_Wide[y];
                    }
                    else
                    {
                        half_width = Half_Ring_Wide_small[y];
                    }

                    center = eight_right_crop[y] -
                             half_width +
                             Small_Rings_HalfWide_Offset;

                    /* 准备出环，额外修正 */
                    if (ImageFlag.image_element_rings_flag == 8 &&
                        ImageFlag.ring_big_small != 2)
                    {
                        center -= 10;
                    }
                }
            }
            else
            {
                /* 入口/出口：优先使用左右中点 */
                if (left_ok && right_ok &&
                    wide >= 20 &&
                    wide <= 140)
                {
                    center = (eight_left_crop[y] +
                              eight_right_crop[y]) / 2;
                }
                else if (left_ok)
                {
                    /* 只有左侧外线 */
                    center = eight_left_crop[y] +
                             Half_Road_Wide[y];
                }
                else if (right_ok)
                {
                    /* 只有右侧线 */
                    center = eight_right_crop[y] -
                             Half_Road_Wide[y];
                }
            }
        }

        if (center >= 0 && center < LCDW)
        {
            eight_center_crop[y] = center;
            eight_center_valid[y] = 1;
        }
        else
        {
            eight_center_crop[y] = -1;
            eight_center_valid[y] = 0;
        }
    }
}