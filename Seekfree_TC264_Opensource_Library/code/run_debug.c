#include "run_debug.h"
#include "camera.h"
#include <math.h>
#if RUN_UART_PID_DEBUG_ENABLE
#include "uart_cmd.h"
#endif

// 通信适配器：仅 WiFi 图传、示波器和串口调参。没有按键和屏幕代码。
// run 层提供一致快照和统一参数提交接口；run_ui 与本模块互不依赖。

// ------------------ 串口 PID 文本调参 ------------------
#if RUN_UART_PID_DEBUG_ENABLE
#define PID_DEBUG_UART        (UART_0)
#define PID_DEBUG_UART_BAUD   (115200U)
#define PID_DEBUG_UART_TX_PIN (UART0_TX_P14_0)
#define PID_DEBUG_UART_RX_PIN (UART0_RX_P14_1)
#endif

// ------------------ WiFi 模块连接参数 ------------------
#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE) && CHEN
#define WIFI_IMAGE_SSID        "Redmi Note 12 Turbo"
#define WIFI_IMAGE_PASSWORD    "nifcxsmvgr2fbp8"
#define WIFI_IMAGE_TARGET_IP   "192.168.9.49"
#define WIFI_IMAGE_TARGET_PORT "8086"
#define WIFI_IMAGE_LOCAL_PORT  "6666"
#elif (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE) && !CHEN
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

#endif

#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
static run_monitor_t debug_monitor;
#endif
#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
static uint8 wifi_connected;
#endif

// 10ms 中断置位、主循环清零，保证每个周期最多发送一次
#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
static volatile uint8 pid_debug_send_flag = 0;
#endif

// 串口命令上下文与回显缓冲
#if RUN_UART_PID_DEBUG_ENABLE
static ucmd_ctx_t pid_debug_cmd;
static run_params_t uart_params;
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
static run_pid_group_enum wifi_pid_group = RUN_PID_STEER;
#endif

// ------------------ 内部函数前向声明 ------------------
#if RUN_UART_PID_DEBUG_ENABLE
static void pid_debug_uart_receive(void);
static void pid_debug_uart_reply(const char *text);
static void pid_debug_send(void);
#endif

#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
static uint8 wifi_module_init(void);
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

// 初始化通信，基本控制和摄像头此时已就绪。网络失败后离线继续。
void run_debug_init(void)
{
#if RUN_UART_PID_DEBUG_ENABLE
    uart_init(PID_DEBUG_UART, PID_DEBUG_UART_BAUD,
              PID_DEBUG_UART_TX_PIN, PID_DEBUG_UART_RX_PIN);
    ucmd_init(&pid_debug_cmd, PID_DEBUG_UART, pid_debug_uart_reply);

    run_get_params(&uart_params);
    (void)ucmd_reg_int16_range(&pid_debug_cmd, "spd", &uart_params.speed, 0, RUN_PARAM_SPEED_MAX);
    (void)ucmd_reg_double_range(&pid_debug_cmd, "lkp", &uart_params.pid[RUN_PID_LEFT].kp, 0, RUN_PARAM_GAIN_MAX);
    (void)ucmd_reg_double_range(&pid_debug_cmd, "lki", &uart_params.pid[RUN_PID_LEFT].ki, 0, RUN_PARAM_GAIN_MAX);
    (void)ucmd_reg_double_range(&pid_debug_cmd, "lkd", &uart_params.pid[RUN_PID_LEFT].kd, 0, RUN_PARAM_GAIN_MAX);
    (void)ucmd_reg_double_range(&pid_debug_cmd, "rkp", &uart_params.pid[RUN_PID_RIGHT].kp, 0, RUN_PARAM_GAIN_MAX);
    (void)ucmd_reg_double_range(&pid_debug_cmd, "rki", &uart_params.pid[RUN_PID_RIGHT].ki, 0, RUN_PARAM_GAIN_MAX);
    (void)ucmd_reg_double_range(&pid_debug_cmd, "rkd", &uart_params.pid[RUN_PID_RIGHT].kd, 0, RUN_PARAM_GAIN_MAX);
    (void)ucmd_reg_double_range(&pid_debug_cmd, "dkp", &uart_params.pid[RUN_PID_STEER].kp, 0, RUN_PARAM_GAIN_MAX);
#if RUN_STEER_FUZZY_ENABLE
    (void)ucmd_reg_double_range(&pid_debug_cmd, "dki", &uart_params.pid[RUN_PID_STEER].ki, 0, 0);
#else
    (void)ucmd_reg_double_range(&pid_debug_cmd, "dki", &uart_params.pid[RUN_PID_STEER].ki, 0, RUN_PARAM_GAIN_MAX);
#endif
    (void)ucmd_reg_double_range(&pid_debug_cmd, "dkd", &uart_params.pid[RUN_PID_STEER].kd, 0, RUN_PARAM_GAIN_MAX);
#endif

#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
    wifi_connected = wifi_module_init();
#endif
#if RUN_WIFI_PID_DEBUG_ENABLE
    if (wifi_connected)
    {
        wifi_debug_param_init();
    }
#endif

}

void run_debug_while(void)
{
#if RUN_UART_PID_DEBUG_ENABLE
    run_params_t before;
    run_get_params(&uart_params);
    before = uart_params;
    pid_debug_uart_receive();
    ucmd_poll(&pid_debug_cmd);
    if (!run_params_equal(&before, &uart_params))
    {
        if (!run_apply_params(&uart_params))
        {
            ucmd_send_text(&pid_debug_cmd, "ERR: stop car; invalid or busy parameters\r\n");
        }
        run_get_params(&uart_params);
    }
#endif
#if RUN_WIFI_PID_DEBUG_ENABLE
    if (wifi_connected)
    {
        wifi_debug_param_while();
    }
#endif
#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
    if (pid_debug_send_flag)
    {
        pid_debug_send_flag = 0;
        // 先取一致快照，再恢复中断后格式化和发送，避免通信阻塞控制。
        run_get_monitor(&debug_monitor);
#if RUN_UART_PID_DEBUG_ENABLE
        pid_debug_send();
#endif
#if RUN_WIFI_PID_DEBUG_ENABLE
        if (wifi_connected)
        {
            wifi_pid_oscilloscope_send();
        }
#endif
    }
#endif
#if RUN_WIFI_IMAGE_ENABLE
    if (wifi_connected)
    {
        wifi_image_while();
    }
#endif
}

void run_debug_tick_10ms(void)
{
#if (RUN_WIFI_PID_DEBUG_ENABLE || RUN_UART_PID_DEBUG_ENABLE)
    pid_debug_send_flag = 1;
#endif
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
                   debug_monitor.pid_values[0],
                   debug_monitor.pid_values[1],
                   debug_monitor.pid_values[2],
                   debug_monitor.pid_values[3],
                   debug_monitor.pid_values[4],
                   debug_monitor.pid_values[5],
                   debug_monitor.pid_values[6],
                   debug_monitor.pid_values[7],
                   debug_monitor.pid_values[8]);
    ucmd_send_text(&pid_debug_cmd, pid_debug_text);
}
#endif

#if (RUN_WIFI_IMAGE_ENABLE || RUN_WIFI_PID_DEBUG_ENABLE)
// 连接 WiFi 与 TCP 服务器，并初始化上位机的图像、示波器、可调参数接口。
static uint8 wifi_module_init(void)
{
    // 只尝试连接一次，失败后离线继续；库函数内部仍有同步超时等待。
    // 两项 WiFi 宏均关闭时，编译期排除整条联网路径。
    if (wifi_spi_init(WIFI_IMAGE_SSID, WIFI_IMAGE_PASSWORD))
    {
        printf("\r\nWiFi unavailable: continuing offline.\r\n");
        return 0;
    }
    if (1 != WIFI_SPI_AUTO_CONNECT && wifi_spi_socket_connect(
            "TCP", WIFI_IMAGE_TARGET_IP, WIFI_IMAGE_TARGET_PORT, WIFI_IMAGE_LOCAL_PORT))
    {
        printf("\r\nTCP unavailable: continuing offline.\r\n");
        return 0;
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
    return 1;
}
#endif

#if RUN_WIFI_PID_DEBUG_ENABLE
// 与本地 UI 共用参数提交接口，仅空闲且无启动请求时接受更新。
static void wifi_debug_param_init(void)
{
    run_params_t p;
    run_get_params(&p);
    memset(&wifi_debug_param, 0, sizeof(wifi_debug_param));
    wifi_debug_param.data[WIFI_PARAM_SPEED] = (float)p.speed;
    wifi_debug_param.data[WIFI_PARAM_PID_GROUP] = (float)wifi_pid_group;
    wifi_debug_param_load_group(wifi_pid_group);
}

static void wifi_debug_param_load_group(uint8 group)
{
    run_params_t p;
    run_get_params(&p);
    wifi_debug_param.data[WIFI_PARAM_SPEED] = (float)p.speed;
    wifi_debug_param.data[WIFI_PARAM_KP] = (float)p.pid[group].kp;
    wifi_debug_param.data[WIFI_PARAM_KI] = (float)p.pid[group].ki;
    wifi_debug_param.data[WIFI_PARAM_KD] = (float)p.pid[group].kd;
}

static void wifi_debug_param_while(void)
{
    uint8 i;
    uint8 changed = 0;
    uint8 valid = 1;
    run_params_t p;
    float value;
    seekfree_assistant_debug_param_analysis(&wifi_debug_param);
    run_get_params(&p);
    if (wifi_debug_param.update_flag[WIFI_PARAM_PID_GROUP])
    {
        value = wifi_debug_param.data[WIFI_PARAM_PID_GROUP];
        if (isfinite(value) && value >= 0 && value < RUN_PID_GROUP_COUNT)
        {
            wifi_pid_group = (run_pid_group_enum)value;
        }
        else
        {
            valid = 0;
        }
    }
    for (i = 0; i < WIFI_PARAM_COUNT; ++i)
    {
        if (!wifi_debug_param.update_flag[i])
        {
            continue;
        }
        value = wifi_debug_param.data[i];
        if (!isfinite(value))
        {
            valid = 0;
        }
        else if (i == WIFI_PARAM_SPEED)
        {
            if (value < 0 || value > RUN_PARAM_SPEED_MAX)
            {
                valid = 0;
            }
            else
            {
                p.speed = (int16)value;
            }
            changed = 1;
        }
        else if (i >= WIFI_PARAM_KP)
        {
            if (i == WIFI_PARAM_KP)
            {
                p.pid[wifi_pid_group].kp = value;
            }
            if (i == WIFI_PARAM_KI)
            {
                p.pid[wifi_pid_group].ki = value;
            }
            if (i == WIFI_PARAM_KD)
            {
                p.pid[wifi_pid_group].kd = value;
            }
            changed = 1;
        }
        wifi_debug_param.update_flag[i] = 0;
    }
    if (changed && valid)
    {
        (void)run_apply_params(&p);
    }
    wifi_debug_param.data[WIFI_PARAM_PID_GROUP] = (float)wifi_pid_group;
    wifi_debug_param_load_group(wifi_pid_group);
}

// WiFi 示波器通道：0~8 = 左轮、右轮、方向环三组 target、actual、output；
// 9~11 = track slope、planned speed、斜坡后 speed；
// 12~15 = slope valid、straight_long、赛道元素编码、line lost。
// 赛道元素编码 = cross + (ring_active << 1)：0=无，1=十字，2=左环岛，4=右环岛，
// 3/5 表示十字标志与左/右环岛标志同时存在。ring stage 当前不参与速度规划，不占示波器通道。
// PID 参数仍通过参数表在线查看/修改，不再占用示波器通道。
static void wifi_pid_oscilloscope_send(void)
{
    uint8 track_element_state;

    wifi_pid_oscilloscope_buffer[0] = (float)debug_monitor.pid_values[0];
    wifi_pid_oscilloscope_buffer[1] = (float)debug_monitor.pid_values[1];
    wifi_pid_oscilloscope_buffer[2] = (float)debug_monitor.pid_values[2];
    wifi_pid_oscilloscope_buffer[3] = (float)debug_monitor.pid_values[3];
    wifi_pid_oscilloscope_buffer[4] = (float)debug_monitor.pid_values[4];
    wifi_pid_oscilloscope_buffer[5] = (float)debug_monitor.pid_values[5];
    wifi_pid_oscilloscope_buffer[6] = (float)debug_monitor.pid_values[6];
    wifi_pid_oscilloscope_buffer[7] = (float)debug_monitor.pid_values[7];
    wifi_pid_oscilloscope_buffer[8] = (float)debug_monitor.pid_values[8];
    wifi_pid_oscilloscope_buffer[9] = (float)debug_monitor.track_slope;
    wifi_pid_oscilloscope_buffer[10] = (float)debug_monitor.planned_speed;
    wifi_pid_oscilloscope_buffer[11] = (float)debug_monitor.ramped_speed;
    wifi_pid_oscilloscope_buffer[12] = (float)debug_monitor.track_slope_valid;
    wifi_pid_oscilloscope_buffer[13] = (float)debug_monitor.straight_long;
    track_element_state = (uint8)(debug_monitor.cross
                        + (uint8)(debug_monitor.ring_active << 1));
    wifi_pid_oscilloscope_buffer[14] = (float)track_element_state;
    wifi_pid_oscilloscope_buffer[15] = (float)debug_monitor.line_lost;

#if RUN_SPEED_PLANNER_DEBUG_ENABLE
    // V2 仍为 16 通道：保留双轮 target/actual/PWM、gap、原始 slope、计划和斜坡速度。
    // 编译期切换后须在会话元数据标明：6=risk, 7=curve limit, 13=straight confirmed。
    wifi_pid_oscilloscope_buffer[6] = (float)debug_monitor.curve_risk;
    wifi_pid_oscilloscope_buffer[7] = (float)debug_monitor.curve_speed_limit;
    wifi_pid_oscilloscope_buffer[13] = (float)debug_monitor.straight_confirmed;
#elif (RUN_STEER_FUZZY_ENABLE && RUN_STEER_FUZZY_DEBUG_ENABLE)
    // 可选观测模式：6=E, 7=受限 EC, 8=u, 9=Kp_eff, 12=Kd_eff, 13=饱和。
    // 0~5 双轮、10~11 速度、14~15 元素/丢线保持原顺序；默认模式完全兼容。
    wifi_pid_oscilloscope_buffer[6] = (float)debug_monitor.fuzzy_error;
    wifi_pid_oscilloscope_buffer[7] = (float)debug_monitor.fuzzy_delta;
    wifi_pid_oscilloscope_buffer[8] = (float)debug_monitor.fuzzy_output;
    wifi_pid_oscilloscope_buffer[9] = (float)debug_monitor.fuzzy_kp;
    wifi_pid_oscilloscope_buffer[12] = (float)debug_monitor.fuzzy_kd;
    wifi_pid_oscilloscope_buffer[13] =
        (float)debug_monitor.fuzzy_saturated;
#endif

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
