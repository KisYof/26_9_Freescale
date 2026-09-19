
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

#define fuya_flag 1

int core0_main(void)
{
    clock_init();
    debug_init();
    adc_init(ADC0_CH5_A5,ADC_12BIT);
//        pwm_init(ATOM1_CH7_P00_8,100,1200);//测试电调专用
//        pwm_init(ATOM3_CH0_P33_4,100,1200);
//    pwm_init(ATOM1_CH1_P33_9, 50,775);                     //舵机中值初始化
    PID_Init();                                             //pid初始化


    ips200_set_dir(IPS200_PORTAIT);                         //ips屏幕初始化
    ips200_init(IPS200_TYPE_SPI);

    gpio_init(P20_10,GPI,1, GPI_PULL_UP);                   //启动按键初始化
    gpio_init(P11_2,GPI,1, GPI_PULL_UP);                    //拨码开关1初始化
    gpio_init(P11_3,GPI,1, GPI_PULL_UP);                    //拨码开关2初始化
    gpio_init(P11_6,GPI,1, GPI_PULL_UP);                    //拨码开关3初始化
    gpio_init(P33_9,GPO,0, GPO_PUSH_PULL);                  //初始化灯板IO为低电平
//    gpio_init(P20_8,GPO,0, GPO_PUSH_PULL);                  //初始化灯板IO为低电平

    mt9v03x_init();                                         //摄像头初始化
//    Image_CutInit();                                        //图像指针映射
    /************menu*******************/
//      Menu_Open();
    /************menu*******************/
    PWMInit();
//    imu660ra_init();
   // kalman1_init(&kalman_gyro,1,100);

    uart_init(UART_2,115200,UART2_TX_P10_5,UART2_RX_P10_6);
    uart_rx_interrupt(UART_2, 1);


    Encoder_init();                                         //编码器初始化
    pit_ms_init(CCU60_CH0,2);                               //编码器中断
    pit_ms_init(CCU60_CH1,8);                              //转向环中断
    pit_ms_init(CCU61_CH0,4);                               //出界保护
//    pit_ms_init(CCU61_CH1,2);                                    //陀螺仪积分

//    Mymenu_main();
    draw_tool_process();

    if(Speed_FS_Flag==1)
    {
        if(PWM_MAX==8000)
        {
            fuzzy_p=4.385;
            fuzzy_d=14.350;
        }
        else if(PWM_MAX==9000)


        {
            fuzzy_p=5.580;
            fuzzy_d=16.250;
        }
        else if(PWM_MAX==9999)
        {
            fuzzy_p=4.380;
            fuzzy_d=14.251;
        }
    }
    go=0;
    system_delay_ms(300);
//    mt9v03x_set_exposure_time(exposure_time);

    //视觉测试模式，先跳过启动按键
    go = 1;
    Motor_Flag = 0;

/*    while(go==0&&Fuya_Flag==1)
    {
        if(gpio_get_level(P20_10)==0){
            system_delay_ms(5);//延时10MS，消抖滤波
            if(gpio_get_level(P20_10)==0){
                    pwm_init(ATOM1_CH7_P00_8,100,1000);
                    pwm_init(ATOM3_CH0_P33_4,100,1000);
                    go=1;
            }
        }
    }*/




    system_delay_ms(1000);

    if(Fuya_Flag==1)
    Fuya_up();


    pit_ms_init(CCU60_CH1,8);                              //转向环中断
    system_delay_ms(300);
//    go=0;
//    Mymenu_Init();
    cpu_wait_event_ready();
    while (TRUE)
    {


        if(cc60_ch0_flag==1)
        {
//            printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",(float)go,(float)ImageStatus.Det_True,(float)ENCO_Wish_L,(float)ENCO_Wish_R,(float)encoder_data_1,(float)encoder_data_2);

                if(Distance_start<30)//计算开始发车时走的距离
                    Distance_start=Distance_start+((R_speed+L_speed)/2)*100*0.002;//总距离

                if((ImageFlag.Stop==1&&Distance>=stop_distance)||ImageFlag.chujie_Stop_1==1||Distance_start<2||go==0)//Distance_start<2当车走了2厘米才开灯
                {
                    gpio_set_level(P33_9,0);

//                    pwm_set_duty(ATOM1_CH7_P00_8,0);
//                    pwm_set_duty(ATOM3_CH0_P33_4,0);
                }
                else
                    gpio_set_level(P33_9,1);
                if(go==0)
                    ImageFlag.chujie_Stop_1==1;
                if(Motor_Flag==1)
                Motor_control();


            cc60_ch0_flag=0;
        }
        else if(cc60_ch1_flag==1)
        {


            cc60_ch1_flag=0;
        }
//        else
        else if(cc61_ch0_flag==1)
        {
            cc61_ch0_flag=0;
        }
    }
}

void uart_rx_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);
//    printf("%.2f,%.2f,%.2f,%.2f,%.2f",1,1,0,0,0);
//             printf("\n");// 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_2, &RxBuffer[0]);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    RxLine++;
        DataBuff[RxLine - 1] = RxBuffer[0];
//        printf("%.2f,%.2f,%.2f,%.2f,%.2f",RxBuffer[0],RxLine,1,0,0);
//                     printf("\n");
        if (RxBuffer[0] == '!')
             {
                 printf("RXLen=%d\r\n", RxLine);
                for (int i = 0; i < RxLine; i++)
                    printf("UART DataBuff[%d] = %c\r\n", i, DataBuff[i]);

                 USART_PID_Adjust(1);
                 memset(DataBuff, 0, sizeof(DataBuff));
                 RxLine = 0;
             }
}


#pragma section all restore


