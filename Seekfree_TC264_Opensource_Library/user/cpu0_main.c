
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"


int core0_main(void)
{
    //系统基础初始化 
    clock_init();
    debug_init();

    //ips屏幕初始化
    ips200_set_dir(IPS200_PORTAIT); 
    ips200_init(IPS200_TYPE_SPI);

    //摄像头初始化
    mt9v03x_init(); 

    // //摄像头测试，先跳过启动按键
    // go = 1;
    // Motor_Flag = 0;
    while (TRUE)
    {
        draw_tool_process();

    }
}


#pragma section all restore


