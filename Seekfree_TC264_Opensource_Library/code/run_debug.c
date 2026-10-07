#include "run_debug.h"
#include "camera.h"
#if RUN_UART_PID_DEBUG_ENABLE
#include "uart_cmd.h"
#endif

// 调试层：按键、IPS200 显示、WiFi 图像与示波器、串口 PID 调参。
// 只通过 run_debug_control_ref_t 读写控制层参数，本身不参与滤波和控制运算。

#define RUN_DEBUG_REFRESH_TICKS (50U) // 500ms 刷新一次参数
#define RUN_KEY_DEBOUNCE_TICKS (3U)   // 按下、松开均稳定 30ms

// ------------------ 串口 PID 文本调参 ------------------
#if RUN_UART_PID_DEBUG_ENABLE
#define PID_DEBUG_UART        (UART_0)
#define PID_DEBUG_UART_BAUD   (115200U)
#define PID_DEBUG_UART_TX_PIN (UART0_TX_P14_0)
#define PID_DEBUG_UART_RX_PIN (UART0_RX_P14_1)
#endif

// ------------------ WiFi 模块连接参数 ------------------
#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
#define WIFI_IMAGE_SSID        "KY"
#define WIFI_IMAGE_PASSWORD    "asdfghjkl"
#define WIFI_IMAGE_TARGET_IP   "10.176.40.16"
#define WIFI_IMAGE_TARGET_PORT "8086"
#define WIFI_IMAGE_LOCAL_PORT  "6666"
#endif

// ------------------ WiFi 示波器与可调参数 ------------------
#if RUN_WIFI_PID_DEBUG_ENABLE
#define WIFI_PID_OSCILLOSCOPE_CHANNEL_NUM (16)

// 上位机参数表下标，与 wifi_debug_param.data[] 一一对应
typedef enum
{
    WIFI_PARAM_SPEED = 0,   // 目标速度
    WIFI_PARAM_PID_GROUP,   // 当前编辑的 PID 组号
    WIFI_PARAM_KP,
    WIFI_PARAM_KI,
    WIFI_PARAM_KD,
    WIFI_PARAM_COUNT
} wifi_debug_param_index_enum;

// 一次只编辑一组 PID，避免参数表过大
typedef enum
{
    WIFI_PID_GROUP_LEFT = 0,    // 左轮速度环
    WIFI_PID_GROUP_RIGHT,       // 右轮速度环
    WIFI_PID_GROUP_STEER,       // 转向环
    WIFI_PID_GROUP_COUNT
} wifi_pid_group_enum;
#endif

// 控制层引用，保存指针，调参直接改 run 层的 PID 和目标速度
static run_debug_control_ref_t run_debug_control;

// 按键事件与消抖状态：10ms 中断写入，主循环取出后清零
static const gpio_pin_enum run_key_pin[] = KEY_LIST;
static volatile uint8 run_debug_key_event = 0;
static volatile uint8 run_debug_refresh_request = 0;
static uint8 run_debug_refresh_count = 0;
static uint8 run_key_press_count = 0;
static uint8 run_key_release_count = 0;
static uint8 run_key_ready = 0;

// 屏幕状态：布局只画一次，状态或故障变化时强制刷新数值
static uint8 run_debug_drawn = 0;
static run_state_enum run_debug_last_state = RUN_STATE_IDLE;
static uint8 run_debug_last_fault = RUN_FAULT_NONE;

// 10ms 中断置位、主循环清零，保证每个周期最多发送一次
#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
static volatile uint8 pid_debug_send_flag = 0;
#endif

// 串口命令上下文与回显缓冲
#if RUN_UART_PID_DEBUG_ENABLE
static ucmd_ctx_t pid_debug_cmd;
static char pid_debug_text[128];
#endif

// WiFi 图像上传：像素缓冲与各图层的点集
#if RUN_WIFI_IMAGE_ENABLE
static uint8 wifi_image_ready = 0;
static uint32 wifi_image_last_sequence = 0;
static uint8 wifi_image_buffer[LCDH][LCDW];
static seekfree_assistant_camera_struct wifi_image_information;
static seekfree_assistant_camera_boundary_uint8_struct wifi_left_boundary[LCDH];
static seekfree_assistant_camera_boundary_uint8_struct wifi_right_boundary[LCDH];
static seekfree_assistant_camera_boundary_uint8_struct wifi_mid_boundary[LCDH];
static seekfree_assistant_camera_boundary_uint8_struct wifi_prospect_boundary[LCDW];
static seekfree_assistant_camera_boundary_uint8_struct wifi_reference_boundary[13];
static seekfree_assistant_camera_boundary_struct wifi_left_information;
static seekfree_assistant_camera_boundary_struct wifi_right_information;
static seekfree_assistant_camera_boundary_struct wifi_mid_information;
static seekfree_assistant_camera_boundary_struct wifi_prospect_information;
static seekfree_assistant_camera_boundary_struct wifi_reference_information;
#endif

// WiFi 示波器缓冲与上位机参数表
#if RUN_WIFI_PID_DEBUG_ENABLE
static seekfree_assistant_oscilloscope_struct wifi_pid_oscilloscope;
static float wifi_pid_oscilloscope_buffer[WIFI_PID_OSCILLOSCOPE_CHANNEL_NUM];
static seekfree_assistant_debug_param_struct wifi_debug_param;
static uint8 wifi_pid_group = WIFI_PID_GROUP_STEER;
#endif

// ------------------ 内部函数前向声明 ------------------
static void run_debug_draw_layout(void);                      // 画固定界面框架，只执行一次
static void run_debug_draw_values(const run_status_struct *status); // 刷新状态、故障与 PID 数值
static void run_debug_ui_while(const run_status_struct *status);    // 屏幕刷新调度

#if RUN_UART_PID_DEBUG_ENABLE
static void pid_debug_uart_receive(void);
static void pid_debug_uart_reply(const char *text);
static void pid_debug_send(void);
#endif

#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
static void wifi_module_init(void);
#endif

#if RUN_WIFI_IMAGE_ENABLE
static void wifi_image_while(void);
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
static void wifi_debug_param_init(void);
static void wifi_debug_param_while(void);
static void wifi_debug_param_load_group(uint8 group);
static void wifi_pid_oscilloscope_send(void);
#endif

// 初始化调试层：保存控制层引用，按开关注册串口调参命令、连接 WiFi，最后初始化按键并请求首次绘屏。
// 此时 5ms/10ms 控制中断尚未启动，因此允许在这里阻塞等待 WiFi 连接。
void run_debug_init(const run_debug_control_ref_t *control)
{
    run_debug_control = *control;

#if RUN_UART_PID_DEBUG_ENABLE
    uart_init(PID_DEBUG_UART, PID_DEBUG_UART_BAUD,
              PID_DEBUG_UART_TX_PIN, PID_DEBUG_UART_RX_PIN);
    ucmd_init(&pid_debug_cmd, PID_DEBUG_UART, pid_debug_uart_reply);

    // 注册可调参数：目标速度 + 三组 PID 各自的 Kp/Ki/Kd
    (void)ucmd_reg_int16(&pid_debug_cmd, "spd", (int16 *)run_debug_control.speed_setting);
    (void)ucmd_reg_double(&pid_debug_cmd, "lkp", &run_debug_control.left_pid->controller.inc.Kp);
    (void)ucmd_reg_double(&pid_debug_cmd, "lki", &run_debug_control.left_pid->controller.inc.Ki);
    (void)ucmd_reg_double(&pid_debug_cmd, "lkd", &run_debug_control.left_pid->controller.inc.Kd);
    (void)ucmd_reg_double(&pid_debug_cmd, "rkp", &run_debug_control.right_pid->controller.inc.Kp);
    (void)ucmd_reg_double(&pid_debug_cmd, "rki", &run_debug_control.right_pid->controller.inc.Ki);
    (void)ucmd_reg_double(&pid_debug_cmd, "rkd", &run_debug_control.right_pid->controller.inc.Kd);
    (void)ucmd_reg_double(&pid_debug_cmd, "dkp", &run_debug_control.gap_pid->controller.pos.Kp);
    (void)ucmd_reg_double(&pid_debug_cmd, "dki", &run_debug_control.gap_pid->controller.pos.Ki);
    (void)ucmd_reg_double(&pid_debug_cmd, "dkd", &run_debug_control.gap_pid->controller.pos.Kd);
#endif

#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
    wifi_module_init();
#endif
#if RUN_WIFI_PID_DEBUG_ENABLE
    wifi_debug_param_init();
#endif

    // 按键扫描周期 10ms，与 10ms 中断一致；先丢弃上电瞬间的毛刺按键
    key_init(10);
    run_debug_discard_keys();
    run_debug_refresh_request = 1;
    run_debug_refresh_count = 0;
    run_debug_drawn = 0;
}

// 主循环轮询的低频任务：收串口命令、收 WiFi 调参、刷屏、发示波器与图像。
// 只读运行状态（run_get_status），控制运算全部在 5ms 中断里完成。
void run_debug_while(void)
{
    run_status_struct status;

#if RUN_UART_PID_DEBUG_ENABLE
    pid_debug_uart_receive();
    ucmd_poll(&pid_debug_cmd);
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
    wifi_debug_param_while();
#endif

    run_get_status(&status);
    run_debug_ui_while(&status);

#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
    if (pid_debug_send_flag == 1)
    {
        pid_debug_send_flag = 0;
#if RUN_UART_PID_DEBUG_ENABLE
        pid_debug_send();
#endif
#if RUN_WIFI_PID_DEBUG_ENABLE
        wifi_pid_oscilloscope_send();
#endif
    }
#endif

#if RUN_WIFI_IMAGE_ENABLE
    wifi_image_while();
#endif
}

// 10ms 中断只扫描按键并设置低频任务标志。
void run_debug_tick_10ms(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    if (KEY_RELEASE_LEVEL == gpio_get_level(run_key_pin[0]))
    {
        run_key_press_count = 0;
        if (run_key_release_count < RUN_KEY_DEBOUNCE_TICKS)
            run_key_release_count++;
        if (run_key_release_count >= RUN_KEY_DEBOUNCE_TICKS)
            run_key_ready = 1;
    }
    else
    {
        run_key_release_count = 0;
        if (run_key_ready)
        {
            if (++run_key_press_count >= RUN_KEY_DEBOUNCE_TICKS)
            {
                run_key_ready = 0;
                run_debug_key_event |= RUN_KEY_START_STOP;
            }
        }
    }

    // 500ms 请求一次界面刷新，由主循环执行绘制
    if (++run_debug_refresh_count >= RUN_DEBUG_REFRESH_TICKS)
    {
        run_debug_refresh_count = 0;
        run_debug_refresh_request = 1;
    }
#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
    pid_debug_send_flag = 1;
#endif
    interrupt_global_enable(interrupt_state);
}

// 丢弃未处理的按键事件与消抖计数，状态切换（如故障）时调用，避免把停车键带进新状态。
void run_debug_discard_keys(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    run_debug_key_event = 0;
    run_key_ready = 0;
    run_key_press_count = 0;
    run_key_release_count = 0;
    interrupt_global_enable(interrupt_state);
}

// 原子取出并清空按键事件，返回 RUN_KEY_* 位标志；关中断保护，避免与 10ms 中断竞争。
uint8 run_debug_take_keys(void)
{
    uint8 events;
    uint32 interrupt_state = interrupt_global_disable();
    events = run_debug_key_event;
    run_debug_key_event = 0;
    interrupt_global_enable(interrupt_state);
    return events;
}

// 屏幕刷新调度：首次进入先画布局，之后按 500ms 刷新标志或状态/故障变化重画数值。
static void run_debug_ui_while(const run_status_struct *status)
{
    uint8 refresh;
    uint32 interrupt_state = interrupt_global_disable();
    refresh = run_debug_refresh_request;
    run_debug_refresh_request = 0;
    interrupt_global_enable(interrupt_state);

    if (!run_debug_drawn)
    {
        // 首次进入补画一次固定布局
        run_debug_draw_layout();
        run_debug_drawn = 1;
        refresh = 1;
    }
    // 状态或故障变化时立即刷新，不必等 500ms 周期
    if (status->state != run_debug_last_state || status->fault != run_debug_last_fault)
        refresh = 1;
    if (refresh)
    {
        run_debug_draw_values(status);
        run_debug_last_state = status->state;
        run_debug_last_fault = status->fault;
    }
}

// 绘制固定界面框架：标题、状态区、左右轮与转向三组 PID 标签、按键说明；只在首次进入执行。
static void run_debug_draw_layout(void)
{
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear();

    ips200_show_string(0, 0, "RUN CONTROL");
    ips200_draw_line(0, 18, 199, 18, RGB565_BLACK);
    ips200_show_string(0, 28, "STATE:");

    ips200_show_string(0, 64, "LEFT PID");
    ips200_show_string(0, 84, "KP:");
    ips200_show_string(0, 104, "KI:");
    ips200_show_string(0, 124, "KD:");

    ips200_show_string(112, 64, "RIGHT PID");
    ips200_show_string(112, 84, "KP:");
    ips200_show_string(112, 104, "KI:");
    ips200_show_string(112, 124, "KD:");

    ips200_show_string(0, 168, "STEER PID");
    ips200_show_string(0, 188, "KP:");
    ips200_show_string(0, 208, "KI:");
    ips200_show_string(0, 228, "KD:");

    ips200_show_string(0, 284, "KEY1: START / STOP");
}

// 刷新可变内容：运行状态、故障说明（按位标志转成文字）以及三组 PID 数值。
static void run_debug_draw_values(const run_status_struct *status)
{
    const char *state_text = "IDLE    ";
    const char *fault_text = "NONE           ";

    switch (status->state)
    {
        case RUN_STATE_RUNNING: state_text = "RUNNING "; break;
        case RUN_STATE_FAULT: state_text = "FAULT   "; break;
        default: break;
    }
    // 故障字段是位标志，左右轮堵转可能同时置位
    if (status->fault & RUN_FAULT_VISION_TIMEOUT) fault_text = "VISION TIMEOUT ";
    else if ((status->fault & 3U) == 3U) fault_text = "STALL BOTH     ";
    else if (status->fault & RUN_FAULT_STALL_LEFT) fault_text = "STALL LEFT     ";
    else if (status->fault & RUN_FAULT_STALL_RIGHT) fault_text = "STALL RIGHT    ";

    ips200_show_string(56, 28, state_text);
    ips200_show_string(0, 46, fault_text);

    ips200_show_float(32, 84, run_debug_control.left_pid->controller.inc.Kp, 2, 2);
    ips200_show_float(32, 104, run_debug_control.left_pid->controller.inc.Ki, 2, 2);
    ips200_show_float(32, 124, run_debug_control.left_pid->controller.inc.Kd, 2, 2);

    ips200_show_float(144, 84, run_debug_control.right_pid->controller.inc.Kp, 2, 2);
    ips200_show_float(144, 104, run_debug_control.right_pid->controller.inc.Ki, 2, 2);
    ips200_show_float(144, 124, run_debug_control.right_pid->controller.inc.Kd, 2, 2);

    ips200_show_float(32, 188, run_debug_control.gap_pid->controller.pos.Kp, 2, 2);
    ips200_show_float(32, 208, run_debug_control.gap_pid->controller.pos.Ki, 2, 2);
    ips200_show_float(32, 228, run_debug_control.gap_pid->controller.pos.Kd, 2, 2);
}

#if RUN_UART_PID_DEBUG_ENABLE
// 非阻塞取出串口收到的字节，逐字节喂给命令解析器（主循环调用）。
static void pid_debug_uart_receive(void)
{
    uint8 data;
    while (uart_query_byte(PID_DEBUG_UART, &data))
    {
        ucmd_isr_feed_byte(&pid_debug_cmd, data);
    }
}

// 命令执行结果通过串口回发给上位机。
static void pid_debug_uart_reply(const char *text)
{
    uart_write_string(PID_DEBUG_UART, text);
}

// 按文本格式输出三组 PID 的目标值/实际值/输出值，格式为 s:9 个浮点数。
static void pid_debug_send(void)
{
    (void)snprintf(pid_debug_text, sizeof(pid_debug_text),
                   "s:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                   run_debug_control.left_pid->controller.inc.target,
                   run_debug_control.left_pid->controller.inc.actual,
                   run_debug_control.left_pid->controller.inc.output,
                   run_debug_control.right_pid->controller.inc.target,
                   run_debug_control.right_pid->controller.inc.actual,
                   run_debug_control.right_pid->controller.inc.output,
                   run_debug_control.gap_pid->controller.pos.target,
                   run_debug_control.gap_pid->controller.pos.actual,
                   run_debug_control.gap_pid->controller.pos.output);
    ucmd_send_text(&pid_debug_cmd, pid_debug_text);
}
#endif

#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
// 连接 WiFi 与 TCP 服务器，并初始化上位机的图像、示波器、可调参数接口。
static void wifi_module_init(void)
{
    // 初始化阶段允许等待连接成功；车辆控制中断此时尚未启动。
    while (wifi_spi_init(WIFI_IMAGE_SSID, WIFI_IMAGE_PASSWORD))
    {
        printf("\r\n connect wifi failed.\r\n");
        system_delay_ms(100);
    }

    if (1 != WIFI_SPI_AUTO_CONNECT)
    {
        while (wifi_spi_socket_connect(
                    "TCP",
                    WIFI_IMAGE_TARGET_IP,
                    WIFI_IMAGE_TARGET_PORT,
                    WIFI_IMAGE_LOCAL_PORT))
        {
            printf("\r\n connect image TCP server failed.\r\n");
            system_delay_ms(100);
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

#if RUN_WIFI_IMAGE_ENABLE
    seekfree_assistant_camera_config(
        &wifi_image_information,
        SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
        LCDW,
        LCDH,
        wifi_image_buffer[0]);
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
    seekfree_assistant_oscilloscope_config(
        &wifi_pid_oscilloscope,
        WIFI_PID_OSCILLOSCOPE_CHANNEL_NUM,
        wifi_pid_oscilloscope_buffer);
#endif

#if RUN_WIFI_IMAGE_ENABLE
    wifi_image_ready = 1;
#endif
    printf("\r\n wifi ready: version=%s, module=%s, server=%s:%s\r\n",
        wifi_spi_version,
        wifi_spi_ip_addr_port,
        WIFI_IMAGE_TARGET_IP,
        WIFI_IMAGE_TARGET_PORT);
}
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
// 参数表初始化：写入当前目标速度与默认编辑组，并把该组 PID 参数同步到上位机。
static void wifi_debug_param_init(void)
{
    memset(&wifi_debug_param, 0, sizeof(wifi_debug_param));
    wifi_debug_param.data[WIFI_PARAM_SPEED] = (float)*run_debug_control.speed_setting;
    wifi_debug_param.data[WIFI_PARAM_PID_GROUP] = (float)wifi_pid_group;
    wifi_debug_param_load_group(wifi_pid_group);
}

// 按组号把对应 PID 的 Kp/Ki/Kd 填入上位机参数表，供界面显示和修改。
static void wifi_debug_param_load_group(uint8 group)
{
    if (WIFI_PID_GROUP_LEFT == group)
    {
        wifi_debug_param.data[WIFI_PARAM_KP] = (float)run_debug_control.left_pid->controller.inc.Kp;
        wifi_debug_param.data[WIFI_PARAM_KI] = (float)run_debug_control.left_pid->controller.inc.Ki;
        wifi_debug_param.data[WIFI_PARAM_KD] = (float)run_debug_control.left_pid->controller.inc.Kd;
    }
    else if (WIFI_PID_GROUP_RIGHT == group)
    {
        wifi_debug_param.data[WIFI_PARAM_KP] = (float)run_debug_control.right_pid->controller.inc.Kp;
        wifi_debug_param.data[WIFI_PARAM_KI] = (float)run_debug_control.right_pid->controller.inc.Ki;
        wifi_debug_param.data[WIFI_PARAM_KD] = (float)run_debug_control.right_pid->controller.inc.Kd;
    }
    else
    {
        wifi_debug_param.data[WIFI_PARAM_KP] = (float)run_debug_control.gap_pid->controller.pos.Kp;
        wifi_debug_param.data[WIFI_PARAM_KI] = (float)run_debug_control.gap_pid->controller.pos.Ki;
        wifi_debug_param.data[WIFI_PARAM_KD] = (float)run_debug_control.gap_pid->controller.pos.Kd;
    }
}

// 解析上位机下发的参数更新：速度直接写目标值，PID 按当前组号写回对应控制器。
// 写入过程关中断，避免与 1ms 基础调度中断读到半截数据。
static void wifi_debug_param_while(void)
{
    uint8 index;
    seekfree_assistant_debug_param_analysis(&wifi_debug_param);

    for (index = 0; index < WIFI_PARAM_COUNT; index++)
    {
        if (wifi_debug_param.update_flag[index])
        {
            float value = wifi_debug_param.data[index];
            uint32 interrupt_state = interrupt_global_disable();

            if (WIFI_PARAM_SPEED == index)
            {
                *run_debug_control.speed_setting = (int16)value;
            }
            else if (WIFI_PARAM_PID_GROUP == index)
            {
                int16 group = (int16)value;
                // 组号越界时钳位到合法范围
                if (group < WIFI_PID_GROUP_LEFT)
                {
                    group = WIFI_PID_GROUP_LEFT;
                }
                else if (group >= WIFI_PID_GROUP_COUNT)
                {
                    group = WIFI_PID_GROUP_STEER;
                }
                wifi_pid_group = (uint8)group;
                wifi_debug_param.data[WIFI_PARAM_PID_GROUP] = (float)wifi_pid_group;
                // 只切组、没带 PID 参数时，重新读该组当前值回显
                if (!wifi_debug_param.update_flag[WIFI_PARAM_KP]
                    && !wifi_debug_param.update_flag[WIFI_PARAM_KI]
                    && !wifi_debug_param.update_flag[WIFI_PARAM_KD])
                {
                    wifi_debug_param_load_group(wifi_pid_group);
                }
            }
            else if (WIFI_PID_GROUP_LEFT == wifi_pid_group)
            {
                if (WIFI_PARAM_KP == index) run_debug_control.left_pid->controller.inc.Kp = (double)value;
                if (WIFI_PARAM_KI == index) run_debug_control.left_pid->controller.inc.Ki = (double)value;
                if (WIFI_PARAM_KD == index) run_debug_control.left_pid->controller.inc.Kd = (double)value;
            }
            else if (WIFI_PID_GROUP_RIGHT == wifi_pid_group)
            {
                if (WIFI_PARAM_KP == index) run_debug_control.right_pid->controller.inc.Kp = (double)value;
                if (WIFI_PARAM_KI == index) run_debug_control.right_pid->controller.inc.Ki = (double)value;
                if (WIFI_PARAM_KD == index) run_debug_control.right_pid->controller.inc.Kd = (double)value;
            }
            else
            {
                if (WIFI_PARAM_KP == index) run_debug_control.gap_pid->controller.pos.Kp = (double)value;
                if (WIFI_PARAM_KI == index) run_debug_control.gap_pid->controller.pos.Ki = (double)value;
                if (WIFI_PARAM_KD == index) run_debug_control.gap_pid->controller.pos.Kd = (double)value;
            }

            wifi_debug_param.update_flag[index] = 0;
            interrupt_global_enable(interrupt_state);
        }
    }
}

// WiFi 示波器通道：0~8 = 左轮、右轮、方向环三组 target、actual、output；
// 9~11 = track slope、planned speed、斜坡后 speed；
// 12~15 = slope valid、straight_long、赛道元素编码、line lost。
// 赛道元素编码 = cross + (ring_active << 1)：0=无，1=十字，2=左环岛，4=右环岛，
// 3/5 表示十字标志与左/右环岛标志同时存在。ring stage 当前不参与速度规划，不占示波器通道。
// PID 参数仍通过参数表在线查看/修改，不再占用示波器通道。
static void wifi_pid_oscilloscope_send(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    uint8 track_element_state;

    wifi_pid_oscilloscope_buffer[0] = (float)run_debug_control.left_pid->controller.inc.target;
    wifi_pid_oscilloscope_buffer[1] = (float)run_debug_control.left_pid->controller.inc.actual;
    wifi_pid_oscilloscope_buffer[2] = (float)run_debug_control.left_pid->controller.inc.output;
    wifi_pid_oscilloscope_buffer[3] = (float)run_debug_control.right_pid->controller.inc.target;
    wifi_pid_oscilloscope_buffer[4] = (float)run_debug_control.right_pid->controller.inc.actual;
    wifi_pid_oscilloscope_buffer[5] = (float)run_debug_control.right_pid->controller.inc.output;
    wifi_pid_oscilloscope_buffer[6] = (float)run_debug_control.gap_pid->controller.pos.target;
    wifi_pid_oscilloscope_buffer[7] = (float)run_debug_control.gap_pid->controller.pos.actual;
    wifi_pid_oscilloscope_buffer[8] = (float)run_debug_control.gap_pid->controller.pos.output;
    wifi_pid_oscilloscope_buffer[9] = (float)*run_debug_control.track_slope;
    wifi_pid_oscilloscope_buffer[10] = (float)*run_debug_control.planned_speed;
    wifi_pid_oscilloscope_buffer[11] = (float)*run_debug_control.ramped_speed;
    wifi_pid_oscilloscope_buffer[12] = (float)*run_debug_control.track_slope_valid;
    wifi_pid_oscilloscope_buffer[13] = (float)*run_debug_control.straight_long;
    track_element_state = (uint8)(*run_debug_control.cross
                        + (uint8)(*run_debug_control.ring_active << 1));
    wifi_pid_oscilloscope_buffer[14] = (float)track_element_state;
    wifi_pid_oscilloscope_buffer[15] = (float)*run_debug_control.line_lost;

    interrupt_global_enable(interrupt_state);
    seekfree_assistant_oscilloscope_send(&wifi_pid_oscilloscope);
}
#endif

#if RUN_WIFI_IMAGE_ENABLE
// 上传一帧视觉数据：整幅图像 + 左右边线 + 中线 + 前瞻线 + 参考标记十字。
// 帧序号读取前后各校验一次，期间换帧或正在写入则放弃本次发送，避免上传撕裂图像。
static void wifi_image_while(void)
{
    uint32 sequence_before;
    uint32 sequence_after;
    uint16 left_count = 0;
    uint16 right_count = 0;
    uint16 mid_count = 0;
    uint16 reference_count = 0;
    uint16 row;
    uint16 column;
    uint8 prospect_row;
    uint8 reference_valid;
    int reference_x;
    int offset;
    int point_x;
    int point_y;

    if (!wifi_image_ready)
    {
        return;
    }

    sequence_before = VisionFrameSequence;
    if ((0 == sequence_before) || (sequence_before & 1U) || (sequence_before == wifi_image_last_sequence))
    {
        return;
    }

    memcpy(wifi_image_buffer[0], Pixle[0], sizeof(wifi_image_buffer));
    prospect_row = ControlProspectRow;
    reference_x = ControlMidValue;
    reference_valid = ControlMidValid;

    // 逐行挑出有效的左/右边线与中线坐标，转成上位机点集
    for (row = 0; row < LCDH; row++)
    {
        point_x = ImageDeal[row].LeftBorder;
        if (ImageDeal[row].IsLeftFind && point_x >= 0 && point_x < LCDW)
        {
            wifi_left_boundary[left_count].x = (uint8)point_x;
            wifi_left_boundary[left_count].y = (uint8)row;
            left_count++;
        }

        point_x = ImageDeal[row].RightBorder;
        if (ImageDeal[row].IsRightFind && point_x >= 0 && point_x < LCDW)
        {
            wifi_right_boundary[right_count].x = (uint8)point_x;
            wifi_right_boundary[right_count].y = (uint8)row;
            right_count++;
        }

        point_x = ControlMidLine[row];
        if (point_x >= 0 && point_x < LCDW)
        {
            wifi_mid_boundary[mid_count].x = (uint8)point_x;
            wifi_mid_boundary[mid_count].y = (uint8)row;
            mid_count++;
        }
    }

    // 拷贝与组包期间若换帧（或仍在写入），直接放弃本次发送
    sequence_after = VisionFrameSequence;
    if ((sequence_before != sequence_after) || (sequence_after & 1U))
    {
        return;
    }

    wifi_image_last_sequence = sequence_after;

    // 前瞻行横线，以及参考中线位置的十字标记
    if (prospect_row < LCDH)
    {
        for (column = 0; column < LCDW; column++)
        {
            wifi_prospect_boundary[column].x = (uint8)column;
            wifi_prospect_boundary[column].y = prospect_row;
        }

        if (reference_valid && reference_x >= 0 && reference_x < LCDW)
        {
            for (offset = -3; offset <= 3; offset++)
            {
                point_x = reference_x + offset;
                if (point_x >= 0 && point_x < LCDW)
                {
                    wifi_reference_boundary[reference_count].x = (uint8)point_x;
                    wifi_reference_boundary[reference_count].y = prospect_row;
                    reference_count++;
                }

                point_y = (int)prospect_row + offset;
                if (offset != 0 && point_y >= 0 && point_y < LCDH)
                {
                    wifi_reference_boundary[reference_count].x = (uint8)reference_x;
                    wifi_reference_boundary[reference_count].y = (uint8)point_y;
                    reference_count++;
                }
            }
        }
    }

    // 各图层分别配置颜色、点数后发送，最后再送整幅图像
    if (left_count)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_left_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_BLUE,
            left_count,
            wifi_left_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_left_information);
    }

    if (right_count)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_right_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_GREEN,
            right_count,
            wifi_right_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_right_information);
    }

    if (mid_count)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_mid_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_RED,
            mid_count,
            wifi_mid_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_mid_information);
    }

    if (prospect_row < LCDH)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_prospect_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_YELLOW,
            LCDW,
            wifi_prospect_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_prospect_information);
    }

    if (reference_count)
    {
        seekfree_assistant_camera_boundary_config(
            &wifi_reference_information,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8,
            RGB565_MAGENTA,
            reference_count,
            wifi_reference_boundary);
        seekfree_assistant_camera_boundary_send(&wifi_reference_information);
    }

    seekfree_assistant_camera_send(&wifi_image_information);
}
#endif
