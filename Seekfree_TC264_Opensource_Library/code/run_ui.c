#include "run_ui.h"

// 界面每 500ms 定时刷新一次，运行状态变化时立即刷新
#define RUN_UI_REFRESH_TICKS    (50U)

// 界面状态机：首次绘制、停车显示、运行显示
typedef enum
{
    RUN_UI_SCREEN_INIT = 0,
    RUN_UI_SCREEN_PARK,
    RUN_UI_SCREEN_START,
} run_ui_screen_state_enum;

static volatile uint8 run_ui_start_request = 0;
static volatile uint8 run_ui_refresh_request = 0;
static uint8 run_ui_refresh_count = 0;
static run_ui_screen_state_enum run_ui_screen_state = RUN_UI_SCREEN_INIT;

static void run_ui_draw_layout(void);
static void run_ui_draw_values(
    uint8 running,
    const PID_Controller_t *left_pid,
    const PID_Controller_t *right_pid,
    const PID_Controller_t *gap_pid);

// 初始化按键与界面状态；此处不操作 IPS，避免早于 ips200_init() 绘图
void run_ui_init(void)
{
    key_init(10);
    key_clear_all_state();

    run_ui_start_request = 0;
    run_ui_refresh_request = 1;
    run_ui_refresh_count = 0;
    run_ui_screen_state = RUN_UI_SCREEN_INIT;
}

// 由 10ms 中断调用，只扫描按键和设置标志，不在中断中刷新 IPS
void run_ui_tick_10ms(void)
{
    key_scanner();

    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        // KEY1 当前只产生发车/复位请求，不实现再次按下停车
        key_clear_state(KEY_1);
        run_ui_start_request = 1;
        run_ui_refresh_request = 1;
    }

    run_ui_refresh_count++;
    if (run_ui_refresh_count >= RUN_UI_REFRESH_TICKS)
    {
        run_ui_refresh_count = 0;
        run_ui_refresh_request = 1;
    }
}

// 由 CPU0 主循环调用，消费按键事件并按需刷新界面
uint8 run_ui_while(
    uint8 running,
    const PID_Controller_t *left_pid,
    const PID_Controller_t *right_pid,
    const PID_Controller_t *gap_pid)
{
    uint8 start_request = 0;
    uint8 refresh_request = 0;
    uint32 interrupt_state;
    run_ui_screen_state_enum requested_state;

    // 与 10ms 按键中断交换事件标志时使用短临界区
    interrupt_state = interrupt_global_disable();
    if (run_ui_start_request)
    {
        run_ui_start_request = 0;
        start_request = 1;
    }
    if (run_ui_refresh_request)
    {
        run_ui_refresh_request = 0;
        refresh_request = 1;
    }
    interrupt_global_enable(interrupt_state);

    requested_state = running ? RUN_UI_SCREEN_START : RUN_UI_SCREEN_PARK;

    // 初次进入时绘制固定布局，之后只在状态变化时切换界面状态
    if (RUN_UI_SCREEN_INIT == run_ui_screen_state)
    {
        run_ui_draw_layout();
        run_ui_screen_state = requested_state;
        refresh_request = 1;
    }
    else if (requested_state != run_ui_screen_state)
    {
        run_ui_screen_state = requested_state;
        refresh_request = 1;
    }

    if (refresh_request)
    {
        // IPS 刷新始终留在主循环，避免延长 10ms 中断执行时间
        run_ui_draw_values(running, left_pid, right_pid, gap_pid);
    }

    return start_request;
}

static void run_ui_draw_layout(void)
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

    ips200_show_string(0, 284, "KEY1: START / RESET");
}

static void run_ui_draw_values(
    uint8 running,
    const PID_Controller_t *left_pid,
    const PID_Controller_t *right_pid,
    const PID_Controller_t *gap_pid)
{
    ips200_show_string(56, 28, running ? "START" : "PARK ");

    ips200_show_float(32, 84, left_pid->controller.inc.Kp, 2, 2);
    ips200_show_float(32, 104, left_pid->controller.inc.Ki, 2, 2);
    ips200_show_float(32, 124, left_pid->controller.inc.Kd, 2, 2);

    ips200_show_float(144, 84, right_pid->controller.inc.Kp, 2, 2);
    ips200_show_float(144, 104, right_pid->controller.inc.Ki, 2, 2);
    ips200_show_float(144, 124, right_pid->controller.inc.Kd, 2, 2);

    ips200_show_float(32, 188, gap_pid->controller.pos.Kp, 2, 2);
    ips200_show_float(32, 208, gap_pid->controller.pos.Ki, 2, 2);
    ips200_show_float(32, 228, gap_pid->controller.pos.Kd, 2, 2);
}
