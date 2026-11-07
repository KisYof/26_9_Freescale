#include "run_ui.h"
#include "camera.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// 按键消抖和短/长按判断全部复用逐飞驱动，1ms 扫描使库阈值 10 对应约 10ms。
// 短按在松开时产生；长按期间保留库状态，避免松开时再产生一次短按。
#define UI_QUEUE_SIZE 16U
#define UI_LONG 0x80U
#define UI_TEXT_MS 200U
#define UI_SPEED_TEST_STEP (5) // 检测速度调节步长，单位 cm/s
#define UI_DISPLAY_TYPE (IPS200_TYPE_SPI)
#define UI_IMAGE_X 8U
#define UI_IMAGE_Y 48U
// 显示尺寸直接跟随裁剪后的处理图，一像素对应屏幕一点，不再次缩放或裁剪。
#define UI_IMAGE_W (LCDW)
#define UI_IMAGE_H (LCDH)
#define UI_IMAGE_INFO_Y (UI_IMAGE_Y + UI_IMAGE_H + 8U)

// ------------------ 页面与 UI 自有状态 ------------------
enum
{
    UI_CAMERA = 0,           // 上电默认进入摄像头显示页
    UI_SPEED_TEST,
    UI_RUN,
    UI_LEFT,
    UI_RIGHT,
    UI_STEER,
    UI_SETTINGS,
    UI_PAGE_COUNT
};
static volatile uint32 ui_ms;
// 单生产者（1ms 中断）/单消费者（主循环）队列，菜单队列满时丢弃新事件。
// KEY1 不进入队列，因此菜单积压不会吞掉启停请求。
static volatile uint8 ui_queue[UI_QUEUE_SIZE];
static volatile uint8 ui_head;
static volatile uint8 ui_tail;
static uint8 ui_long_seen[KEY_NUMBER];
static volatile uint8 ui_page = UI_CAMERA; // 主循环切页，KEY1 中断按当前页选择启动模式
static uint8 ui_field;
static uint8 ui_edit;
static uint8 ui_step = 1;
static uint8 ui_screen_on = 1;
static uint8 ui_image_on = 1;
static uint8 ui_auto_pause;
static uint8 ui_image_rate = 1;
static uint8 ui_setting;
static uint8 ui_redraw = 1;
static uint8 ui_layout_dirty = 1;
static uint32 ui_last_text;
static uint32 ui_last_image;
static uint32 ui_image_sequence;
static run_state_enum ui_last_state = RUN_STATE_IDLE;
static uint8 ui_last_fault;
static uint8 ui_last_pending;
static run_params_t ui_draft;
static run_params_t ui_original;
static const char *ui_notice = "RAM only; K2 hold to edit";
static const uint16 ui_image_period[] = {500U, 200U, 100U};
static const double ui_gain_step[] = {0.001, 0.01, 0.1, 1.0};
static const int16 ui_speed_step[] = {1, 5, 10, 50};
static const char *ui_titles[] = {"CAMERA / CONTROL", "SPEED PID TEST", "RUN MONITOR", "LEFT WHEEL PID", "RIGHT WHEEL PID",
    "STEER FUZZY PD BASE",
    "DISPLAY SETTINGS"};

// CPU0 独立保存显示快照；CPU1 写帧或复制期间换帧时丢弃本次刷新。
// 图像复制和 SPI 绘屏均保持中断开启，不延长控制中断的临界区。
static uint8 ui_pixels[LCDH][LCDW];
static int16 ui_left[LCDH];
static int16 ui_right[LCDH];
static int16 ui_mid[LCDH];
static uint8 ui_prospect;
static uint8 ui_reference_valid;
static int16 ui_reference;

// 240 像素宽、8x16 字体每行最多 30 字符；补空格覆盖旧值，避免残字。
static void ui_text(uint16 y, const char *format, ...)
{
    char line[31];
    uint8 n;
    va_list ap;
    va_start(ap, format);
    (void)vsnprintf(line, sizeof(line), format, ap);
    va_end(ap);
    n = (uint8)strlen(line);
    while (n < 30U)
    {
        line[n++] = ' ';
    }
    line[30] = 0;
    ips200_show_string(0, y, line);
}

void run_ui_init(void)
{
    ips200_init(UI_DISPLAY_TYPE);
    key_init(1);
    key_clear_all_state();
    memset(ui_long_seen, 0, sizeof(ui_long_seen));
    ui_page = UI_CAMERA;
    ui_ms = 0;
    ui_head = ui_tail = 0;
    ui_redraw = 1;
    ui_layout_dirty = 1;
}

void run_ui_tick_1ms(void)
{
    uint8 i;
    uint8 event;
    uint8 next;
    key_state_enum state;
    ++ui_ms;
    key_scanner();
    for (i = 0; i < KEY_NUMBER; ++i)
    {
        event = 0;
        state = key_get_state((key_index_enum)i);
        if (KEY_RELEASE == state)
        {
            ui_long_seen[i] = 0;
        }
        else if (KEY_SHORT_PRESS == state)
        {
            event = (uint8)(i + 1U);
            key_clear_state((key_index_enum)i);
        }
        else if (!ui_long_seen[i])
        {
            ui_long_seen[i] = 1;
            event = (uint8)((i + 1U) | UI_LONG);
        }
        if (!event)
        {
            continue;
        }
        if (i == KEY_1)
        {
            // KEY1 直接投递启停请求，不排在菜单、刷屏或通信后面。
            if (ui_page == UI_SPEED_TEST)
            {
                run_request_speed_test_toggle();
            }
            else
            {
                run_request_toggle();
            }
            continue;
        }
        next = (uint8)((ui_head + 1U) % UI_QUEUE_SIZE);
        if (next != ui_tail)
        {
            ui_queue[ui_head] = event;
            ui_head = next;
        }
    }
}

// 主循环短暂关中断取一项事件，队列操作之后才执行菜单和绘屏。
static uint8 ui_take_event(void)
{
    uint8 event = 0;
    uint32 interrupt_state = interrupt_global_disable();
    if (ui_tail != ui_head)
    {
        event = ui_queue[ui_tail];
        ui_tail = (uint8)((ui_tail + 1U) % UI_QUEUE_SIZE);
    }
    interrupt_global_enable(interrupt_state);
    return event;
}

// 页码与控制组号显式对应，调整页面顺序不会改变所编辑的 PID 组。
static run_pid_group_enum ui_get_pid_group(uint8 page)
{
    switch (page)
    {
    case UI_LEFT:  return RUN_PID_LEFT;
    case UI_RIGHT: return RUN_PID_RIGHT;
    case UI_STEER: return RUN_PID_STEER;
    default:       return RUN_PID_GROUP_COUNT;
    }
}

static void ui_adjust(int8 direction)
{
    double *value;
    run_pid_group_enum group = ui_get_pid_group(ui_page);
    run_gains_t *g;
    if (group >= RUN_PID_GROUP_COUNT)
    {
        return;
    }
    g = &ui_draft.pid[group];
    if (ui_field == 4)
    {
        ui_step = (uint8)((ui_step + (direction > 0 ? 1 : 3)) % 4);
        return;
    }
    if (ui_field == 0)
    {
        int32 speed_value = ui_draft.speed + direction * ui_speed_step[ui_step];
        ui_draft.speed = (int16)(speed_value < 0 ? 0 : (speed_value > RUN_PARAM_SPEED_MAX ? RUN_PARAM_SPEED_MAX : speed_value));
        return;
    }
    if (ui_page == UI_STEER && ui_field == 2)
    {
        return;
    }
    value = ui_field == 1 ? &g->kp : (ui_field == 2 ? &g->ki : &g->kd);
    *value += direction * ui_gain_step[ui_step];
    if (*value < 0)
    {
        *value = 0;
    }
    if (*value > RUN_PARAM_GAIN_MAX)
    {
        *value = RUN_PARAM_GAIN_MAX;
    }
}

static void ui_handle(uint8 event, const run_monitor_t *m)
{
    uint8 key = event & 0x7fU;
    uint8 long_press = event & UI_LONG;
    uint8 pause_for_run = ui_auto_pause && m->status.state == RUN_STATE_RUNNING && !m->speed_test_active;
    run_params_t current;
    if (key == 4 && long_press && !ui_edit)
    {
        // 唤醒时同时取消运行自动停刷，保证长按 KEY4 能立即恢复显示。
        if (pause_for_run)
        {
            ui_auto_pause = 0;
            ui_screen_on = 1;
        }
        else
        {
            ui_screen_on = !ui_screen_on;
        }
        if (!ui_screen_on)
        {
            ui_text(256, "PAUSED: hold K4 to wake");
        }
        else
        {
            ui_layout_dirty = 1;
        }
        ui_redraw = 1;
        return;
    }
    if (!ui_screen_on || pause_for_run)
    {
        return;
    }
    if (ui_edit)
    {
        if (key == 3 && long_press)
        {
            ui_edit = 0;
            ui_notice = "Cancelled; values unchanged";
        }
        else if (key == 2 && long_press)
        {
            // 通信端可能在编辑期间改过参数；旧草稿不能覆盖新的有效值。
            run_get_params(&current);
            if (!run_params_equal(&current, &ui_original))
            {
                ui_notice = "Changed remotely; re-edit";
            }
            else if (run_apply_params(&ui_draft))
            {
                ui_notice = "Applied to RAM";
            }
            else
            {
                ui_notice = "Rejected: stop car first";
            }
            ui_edit = 0;
        }
        else if (!long_press)
        {
            if (key == 2)
            {
                ui_field = (uint8)((ui_field + 1) % 5);
            }
            if (key == 3)
            {
                ui_adjust(-1);
            }
            if (key == 4)
            {
                ui_adjust(1);
            }
        }
    }
    else if (key == 2 && !long_press)
    {
        ui_page = (uint8)((ui_page + 1) % UI_PAGE_COUNT);
        ui_layout_dirty = 1;
        ui_notice = "RAM only; K2 hold to edit";
    }
    else if (key == 2 && long_press && ui_get_pid_group(ui_page) < RUN_PID_GROUP_COUNT)
    {
        if (m->status.state == RUN_STATE_IDLE && !m->start_pending)
        {
            run_get_params(&ui_draft);
            ui_original = ui_draft;
            ui_field = 0;
            ui_edit = 1;
            ui_notice = "Editing; K3 hold = cancel";
        }
        else
        {
            ui_notice = "Stop car before editing";
        }
    }
    else if (!long_press && ui_page == UI_SPEED_TEST)
    {
        if (key == 3)
        {
            run_adjust_speed_test_target(-UI_SPEED_TEST_STEP);
        }
        if (key == 4)
        {
            run_adjust_speed_test_target(UI_SPEED_TEST_STEP);
        }
    }
    else if (!long_press && ui_page == UI_CAMERA)
    {
        if (key == 3)
        {
            ui_image_on = !ui_image_on;
        }
        if (key == 4)
        {
            ui_image_rate = (uint8)((ui_image_rate + 1) % 3);
        }
    }
    else if (!long_press && ui_page == UI_SETTINGS)
    {
        if (key == 3)
        {
            ui_setting = (uint8)((ui_setting + 1) % 4);
        }
        if (key == 4)
        {
            switch (ui_setting)
            {
            case 0:
                ui_screen_on = 0;
                ui_text(256, "PAUSED: hold K4 to wake");
                break;
            case 1:
                ui_image_on = !ui_image_on;
                break;
            case 2:
                ui_image_rate = (uint8)((ui_image_rate + 1) % 3);
                break;
            default:
                ui_auto_pause = !ui_auto_pause;
                break;
            }
        }
    }
    ui_redraw = 1;
}

static void ui_draw_status(const run_monitor_t *m)
{
    const char *state = !m->ready ? "INIT" : (m->start_pending ? "WAIT FRAME" :
        (m->status.state == RUN_STATE_RUNNING ? (m->speed_test_active ? "TESTING" : "RUNNING") :
         (m->status.state == RUN_STATE_FAULT ? "FAULT" : "IDLE")));
    ui_text(0, "%s", ui_titles[ui_page]);
    ui_text(20, "%s  fault:%u", state, (unsigned)m->status.fault);
}

static void ui_draw_values(const run_monitor_t *m)
{
    run_params_t p;
    const run_gains_t *g;
    const char *fault = "NONE";
    run_get_params(&p);
    ui_draw_status(m);
    if (ui_page == UI_SPEED_TEST)
    {
        const char *hint = "";
        ui_text(48, "Test set: %d cm/s", m->speed_test_target);
        ui_text(72, "Ramp: %d cm/s", m->speed_test_active ? m->ramped_speed : 0);
        ui_text(96, "     actual(cm/s)    PWM");
        if (m->speed_test_active)
        {
            ui_text(120, "L %11.0f %9.0f", m->pid_values[1], m->pid_values[2]);
            ui_text(144, "R %11.0f %9.0f", m->pid_values[4], m->pid_values[5]);
        }
        else
        {
            ui_text(120, "L          --        --");
            ui_text(144, "R          --        --");
        }
        if (m->status.fault)
        {
            ui_text(216, "Power cycle to clear fault");
        }
        else if (m->status.state != RUN_STATE_IDLE || m->start_pending)
        {
            ui_text(216, "K1 stop; adjust when idle");
        }
        else
        {
            ui_text(216, "K3 -%d / K4 +%d cm/s", UI_SPEED_TEST_STEP, UI_SPEED_TEST_STEP);
            hint = m->speed_test_target > 0 ? "K1 starts speed test" : "Set speed above 0 to start";
        }
        ui_text(240, "%s", hint);
    }
    else if (ui_page == UI_RUN)
    {
        ui_text(48, "Speed set: %d cm/s", p.speed);
        ui_text(72, "Plan:%d Ramp:%d", m->planned_speed, m->ramped_speed);
        ui_text(96, "     target actual PWM");
        ui_text(120, "L %6.0f %6.0f %5.0f", m->pid_values[0], m->pid_values[1], m->pid_values[2]);
        ui_text(144, "R %6.0f %6.0f %5.0f", m->pid_values[3], m->pid_values[4], m->pid_values[5]);
        ui_text(168, "Frame:%lu Age:%ums", (unsigned long)m->frame_sequence, (unsigned)m->vision_age_ms);
        ui_text(192, "Mid:%ld V:%u Lost:%u", (long)m->mid_error, m->mid_valid, m->line_lost);
        if (m->status.fault & RUN_FAULT_VISION_STOP)
        {
            fault = "VISION STOP";
        }
        else if (m->status.fault & RUN_FAULT_VISION_TIMEOUT)
        {
            fault = "VISION TIMEOUT";
        }
        else if ((m->status.fault & 3U) == 3U)
        {
            fault = "STALL BOTH";
        }
        else if (m->status.fault & RUN_FAULT_STALL_LEFT)
        {
            fault = "STALL LEFT";
        }
        else if (m->status.fault & RUN_FAULT_STALL_RIGHT)
        {
            fault = "STALL RIGHT";
        }
        ui_text(216, "Fault: %s", fault);
        ui_text(240, m->status.fault ? "Power cycle to clear fault" : "K1 start/stop; K2 next page");
    }
    else if (ui_get_pid_group(ui_page) < RUN_PID_GROUP_COUNT)
    {
        if (ui_edit)
        {
            p = ui_draft;
        }
        g = &p.pid[ui_get_pid_group(ui_page)];
        ui_text(48, "%c Speed: %d cm/s", ui_edit && ui_field == 0 ? '>' : ' ', p.speed);
        ui_text(76, "%c Kp: %.3f", ui_edit && ui_field == 1 ? '>' : ' ', g->kp);
        ui_text(104, "%c Ki: %.3f", ui_edit && ui_field == 2 ? '>' : ' ', g->ki);
        ui_text(132, "%c Kd: %.3f", ui_edit && ui_field == 3 ? '>' : ' ', g->kd);
        ui_text(160, "%c Step: %.3f / %d cm/s", ui_edit && ui_field == 4 ? '>' : ' ', ui_gain_step[ui_step], ui_speed_step[ui_step]);
        if (ui_page == UI_STEER)
        {
            ui_text(188, "Ki locked at zero");
            ui_text(212, "Eff P:%.3f D:%.3f", m->fuzzy_kp, m->fuzzy_kd);
        }
        ui_text(240, "%s", ui_notice);
        ui_text(260, ui_edit ? "K2 field/hold apply; K3-/K4+" : "K2 hold edit (IDLE only)");
    }
    else if (ui_page == UI_CAMERA)
    {
        ui_text(UI_IMAGE_INFO_Y, "%ux%u %s  %u FPS requested",
                (unsigned)UI_IMAGE_W, (unsigned)UI_IMAGE_H,
                ui_image_on ? "ON" : "OFF", 1000U / ui_image_period[ui_image_rate]);
        ui_text(UI_IMAGE_INFO_Y + 24U, "Frame:%lu Age:%ums", (unsigned long)ui_image_sequence, (unsigned)m->vision_age_ms);
        ui_text(UI_IMAGE_INFO_Y + 48U, "Mid:%ld V:%u", (long)m->mid_error, m->mid_valid);
        ui_text(UI_IMAGE_INFO_Y + 72U, "K3 image ON/OFF; K4 rate");
        ui_text(UI_IMAGE_INFO_Y + 96U, "Blue L / Green R / Red mid");
    }
    else
    {
        ui_text(52, "%c Screen refresh: ON", ui_setting == 0 ? '>' : ' ');
        ui_text(88, "%c Image refresh: %s", ui_setting == 1 ? '>' : ' ', ui_image_on ? "ON" : "OFF");
        ui_text(124, "%c Image rate: %u FPS", ui_setting == 2 ? '>' : ' ', 1000U / ui_image_period[ui_image_rate]);
        ui_text(160, "%c Pause when RUN: %s", ui_setting == 3 ? '>' : ' ', ui_auto_pause ? "ON" : "OFF");
        ui_text(208, "K3 select; K4 change");
        ui_text(232, "Freeze keeps control alive");
        ui_text(256, "Hold K4 always wakes screen");
    }
    ui_text(288, "K1 START/STOP  K2 PAGE");
    ui_text(304, ui_edit ? "K3 hold CANCEL; K2 hold APPLY" : "K4 hold: screen pause/wake");
}

// 先复制完整帧并校验序号，再用独立副本绘图；换帧时放弃而不阻塞等待。
static void ui_draw_image(void)
{
    uint16 row;
    uint32 before = VisionFrameSequence;
    if (!before || (before & 1U))
    {
        return;
    }
    __dsync();
    memcpy(ui_pixels, Pixle, sizeof(ui_pixels));
    ui_prospect = ControlProspectRow;
    ui_reference = (int16)ControlMidValue;
    ui_reference_valid = ControlMidValid;
    for (row = 0; row < LCDH; ++row)
    {
        ui_left[row] = ImageDeal[row].IsLeftFind == 'T' ? ImageDeal[row].LeftBorder : -1;
        ui_right[row] = ImageDeal[row].IsRightFind == 'T' ? ImageDeal[row].RightBorder : -1;
        ui_mid[row] = (int16)ControlMidLine[row];
    }
    __dsync();
    if (before != VisionFrameSequence || (VisionFrameSequence & 1U))
    {
        return;
    }
    ui_image_sequence = before;
    ips200_show_gray_image(UI_IMAGE_X, UI_IMAGE_Y, ui_pixels[0], LCDW, LCDH, UI_IMAGE_W, UI_IMAGE_H, 0);
    for (row = 0; row < LCDH; ++row)
    {
        uint16 y = UI_IMAGE_Y + row;
        if (ui_left[row] >= 0 && ui_left[row] < LCDW)
            ips200_draw_point(UI_IMAGE_X + ui_left[row], y, RGB565_BLUE);
        if (ui_right[row] >= 0 && ui_right[row] < LCDW)
            ips200_draw_point(UI_IMAGE_X + ui_right[row], y, RGB565_GREEN);
        if (ui_mid[row] >= 0 && ui_mid[row] < LCDW)
            ips200_draw_point(UI_IMAGE_X + ui_mid[row], y, RGB565_RED);
    }
    if (ui_prospect < LCDH)
    {
        uint16 y = UI_IMAGE_Y + ui_prospect;
        ips200_draw_line(UI_IMAGE_X, y, UI_IMAGE_X + UI_IMAGE_W - 1U, y, RGB565_YELLOW);
        if (ui_reference_valid && ui_reference >= 0 && ui_reference < LCDW)
        {
            uint16 x = UI_IMAGE_X + ui_reference;
            uint16 top = ui_prospect >= 3U ? y - 3U : UI_IMAGE_Y;
            uint16 bottom = ui_prospect + 3U < LCDH ? y + 3U : UI_IMAGE_Y + UI_IMAGE_H - 1U;
            // 参考标记也限制在裁剪图内，避免在图像外多画像素。
            ips200_draw_line(x, top, x, bottom, RGB565_MAGENTA);
        }
    }
}

void run_ui_while(void)
{
    run_monitor_t monitor;
    uint8 event, count = 0, force;
    uint32 now = ui_ms;
    run_get_monitor(&monitor);
    // 启动等待、运行或故障时丢弃未提交草稿，控制层只接收完整参数组。
    if (ui_edit && (monitor.status.state != RUN_STATE_IDLE || monitor.start_pending))
    {
        ui_edit = 0;
        ui_notice = "Edit cancelled by run state";
        ui_redraw = 1;
    }
    while (count < UI_QUEUE_SIZE && (event = ui_take_event()) != 0)
    {
        ui_handle(event, &monitor);
        ++count;
    }
    if (count && ui_page == UI_SPEED_TEST)
    {
        run_get_monitor(&monitor);
    }
    if (monitor.status.state != ui_last_state || monitor.status.fault != ui_last_fault
        || monitor.start_pending != ui_last_pending) ui_redraw = 1;
    ui_last_state = monitor.status.state;
    ui_last_fault = monitor.status.fault;
    ui_last_pending = monitor.start_pending;
    // 刷新关闭仅停止显示，按键事件和控制请求已在前面处理。
    if (!ui_screen_on)
    {
        return;
    }
    if (ui_auto_pause && monitor.status.state == RUN_STATE_RUNNING && !monitor.speed_test_active)
    {
        return;
    }
    force = ui_redraw;
    if (force)
    {
        // 切页或唤醒时清屏；普通数值更新覆盖原文字，减少闪烁和 SPI 传输。
        if (ui_layout_dirty)
        {
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);
            ips200_clear();
            ui_layout_dirty = 0;
        }
        ui_redraw = 0;
    }
    if (force || (uint32)(now - ui_last_text) >= UI_TEXT_MS)
    {
        ui_last_text = now;
        ui_draw_values(&monitor);
    }
    if (ui_page == UI_CAMERA && ui_image_on
        && (force || (uint32)(now - ui_last_image) >= ui_image_period[ui_image_rate]))
    {
        ui_last_image = now;
        ui_draw_image();
    }
}
