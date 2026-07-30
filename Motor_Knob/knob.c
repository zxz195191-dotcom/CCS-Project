#include "headfile.h"
#include "../race_log.h"
#include "../race_mode.h"

#define KNOB_BTN_CLICK       1U
#define KNOB_BTN_LONG        3U
#define OLED_UI_REFRESH_US   200000U
#define OLED_UI_LIVE_US      500000U
#define OLED_UI_SPEED_LIMIT  30000.0f

extern volatile float g_target_speed;
extern volatile float g_K_steer;
extern int32_t control_err;
extern float current_base_speed;
extern volatile uint32_t g_lap_time_ms;
extern volatile uint32_t g_lap_pulses;

static int32_t enc_cnt = 0;
static uint8_t last_button = 0;
static uint8_t led_on = 1;
static uint32_t buzzer_freq = 0;

typedef enum {
    UI_PAGE_MODE = 0,
    UI_PAGE_MOTOR,
    UI_PAGE_TRACE,
    UI_PAGE_GYRO,
    UI_PAGE_IO,
    UI_PAGE_RAW,
    UI_PAGE_RUN_LOG,
    UI_PAGE_MASK_LOG_1,
    UI_PAGE_MASK_LOG_2,
    UI_PAGE_COUNT
} UiPage;

typedef enum {
    UI_PAGE_SELECT = 0,
    UI_PARAM_SELECT,
    UI_EDIT
} UiState;

static UiPage  ui_page = UI_PAGE_MODE;
static UiState ui_state = UI_PAGE_SELECT;
static int8_t  ui_param = 0;
static uint8_t ui_dirty = 1;
static uint32_t ui_last_draw = 0;
static UiPage ui_drawn_page = UI_PAGE_COUNT;

static uint8_t ui_param_count(UiPage page)
{
    switch (page) {
    case UI_PAGE_MODE:  return 4U;
    case UI_PAGE_MOTOR: return 4U;
    case UI_PAGE_TRACE: return 2U;
    case UI_PAGE_GYRO:  return 2U;
    case UI_PAGE_IO:    return 2U;
    case UI_PAGE_RAW:   return 1U;
    case UI_PAGE_RUN_LOG:
    case UI_PAGE_MASK_LOG_1:
    case UI_PAGE_MASK_LOG_2:
        return 1U;
    default:            return 1U;
    }
}

static void ui_mark_dirty(void)
{
    ui_dirty = 1U;
}

void Knob_UI_Refresh(void)
{
    ui_mark_dirty();
}

void Knob_UI_OpenRaceLog(void)
{
    ui_page = UI_PAGE_RUN_LOG;
    ui_state = UI_PAGE_SELECT;
    ui_param = 0;
    ui_mark_dirty();
}

static void ui_wrap_param(int8_t delta)
{
    int8_t count = (int8_t)ui_param_count(ui_page);
    int8_t next = (int8_t)(ui_param + delta);

    while (next < 0) next += count;
    while (next >= count) next -= count;
    ui_param = next;
}

static void ui_adjust_frequency(int8_t delta)
{
    int32_t next = (int32_t)buzzer_freq;

    if (next == 0 && delta > 0) {
        next = 1221;
    } else {
        next += (int32_t)delta * 100;
        if (next <= 0) next = 0;
        else if (next < 1221) next = 1221;
    }
    if (next > 10000) next = 10000;

    buzzer_freq = (uint32_t)next;
    Buzzer_SetTone(buzzer_freq);
}

static void ui_select_mode(int8_t delta)
{
    int8_t direction = (delta < 0) ? -1 : 1;
    uint8_t attempts = 0U;
    uint8_t next = g_race_mode;

    while (attempts++ < 3U) {
        int8_t candidate = (int8_t)next + direction;
        if (candidate < (int8_t)RACE_MODE_1) candidate = (int8_t)RACE_MODE_3;
        if (candidate > (int8_t)RACE_MODE_3) candidate = (int8_t)RACE_MODE_1;
        next = (uint8_t)candidate;
        if (Race_Mode_IsConfigured(next)) {
            (void)Race_Mode_Select(next);
            return;
        }
    }
}

static void ui_adjust_value(int8_t delta)
{
    if (Race_Mode_ParametersLocked() &&
        (ui_page == UI_PAGE_MOTOR || ui_page == UI_PAGE_TRACE)) {
        return;
    }

    switch (ui_page) {
    case UI_PAGE_MODE:
        if (ui_param == 0) {
            ui_select_mode(delta);
        } else if (g_race_mode == RACE_MODE_2 && ui_param == 1) {
            int32_t next = (int32_t)Race_Mode_GetStopPulses() +
                (int32_t)delta * 1000;
            if (next < 10000) next = 10000;
            (void)Race_Mode_SetStopPulses((uint32_t)next);
        } else if (g_race_mode == RACE_MODE_2 && ui_param == 2) {
            int32_t next = (int32_t)Race_Mode_GetBrakeStartPulses() +
                (int32_t)delta * 1000;
            if (next < 10000) next = 10000;
            (void)Race_Mode_SetBrakeStartPulses((uint32_t)next);
        } else if (g_race_mode == RACE_MODE_2 && ui_param == 3) {
            int32_t next = (int32_t)Race_Mode_GetStopLeadPulses() +
                (int32_t)delta * 100;
            if (next < 0) next = 0;
            (void)Race_Mode_SetStopLeadPulses((uint32_t)next);
        }
        break;

    case UI_PAGE_MOTOR:
        if (ui_param == 0) {
            float next = Race_Mode_GetRunSpeed() + (float)delta * 4000.0f;
            (void)Race_Mode_SetRunSpeed(next);
        } else if (ui_param == 1) {
            float next = g_motor_Kp + (float)delta * 0.0001f;
            if (next < 0.0f) next = 0.0f;
            if (next > 5.0f) next = 5.0f;
            (void)Race_Mode_SetMotorKp(next);
        } else if (ui_param == 2) {
            float next = g_motor_Ki + (float)delta * 0.005f;
            if (next < 0.0f) next = 0.0f;
            if (next > 10.0f) next = 10.0f;
            (void)Race_Mode_SetMotorKi(next);
        }
        break;

    case UI_PAGE_TRACE:
        if (ui_param == 1) {
            float next = g_K_steer + (float)delta * 0.5f;
            if (next < 0.0f) next = 0.0f;
            if (next > 100.0f) next = 100.0f;
            (void)Race_Mode_SetSteerK(next);
        }
        break;

    case UI_PAGE_IO:
        if (ui_param == 0) ui_adjust_frequency(delta);
        break;

    default:
        break;
    }
}

static void ui_calibrate_gyro(void)
{
    Motor_Set_Speed(Left_Wheel, 0);
    Motor_Set_Speed(Right_Wheel, 0);
    OLED_Clear();
    OLED_ShowString(0, 2, (uint8_t*)"CALIBRATING", 8);
    Gyro_Calibrate_Bias(5000);
    IMU_Reset_Attitude();
    OLED_Clear();
    ui_mark_dirty();
}

static void ui_process(int8_t enc, uint8_t button)
{
    if (enc != 0) {
        if (ui_state == UI_PAGE_SELECT) {
            int8_t next = (int8_t)ui_page + enc;
            if (next < 0) next = (int8_t)UI_PAGE_COUNT - 1;
            if (next >= (int8_t)UI_PAGE_COUNT) next = 0;
            ui_page = (UiPage)next;
        } else if (ui_state == UI_PARAM_SELECT) {
            ui_wrap_param(enc);
        } else {
            ui_adjust_value(enc);
        }
        ui_mark_dirty();
    }

    if (button == KNOB_BTN_LONG) {
        ui_state = UI_PAGE_SELECT;
        ui_mark_dirty();
        return;
    }
    if (button != KNOB_BTN_CLICK) return;

    if (ui_state == UI_PAGE_SELECT) {
        ui_state = UI_PARAM_SELECT;
        ui_param = 0;
    } else if (ui_state == UI_PARAM_SELECT) {
        if (ui_page == UI_PAGE_MOTOR && ui_param == 3) {
            if (g_target_speed > 1.0f) Race_Mode_Stop();
            else Race_Mode_StartAt(OLED_Button_GetLastPressTimeUs());
        } else if (ui_page == UI_PAGE_GYRO && ui_param == 1) {
            ui_calibrate_gyro();
        } else if (ui_page == UI_PAGE_IO && ui_param == 1) {
            led_on = (uint8_t)!led_on;
            LED_Set(led_on != 0U);
        } else if (!(ui_page == UI_PAGE_TRACE && ui_param == 0) &&
                   !(ui_page == UI_PAGE_GYRO && ui_param == 0) &&
                   !(ui_page == UI_PAGE_MODE && ui_param > 0 &&
                     g_race_mode != RACE_MODE_2) &&
                   !(ui_page == UI_PAGE_RAW) &&
                   !(ui_page == UI_PAGE_RUN_LOG) &&
                   !(ui_page == UI_PAGE_MASK_LOG_1) &&
                   !(ui_page == UI_PAGE_MASK_LOG_2) &&
                   !(Race_Mode_ParametersLocked() &&
                     ((ui_page == UI_PAGE_MOTOR && ui_param < 3) ||
                      (ui_page == UI_PAGE_TRACE && ui_param == 1)))) {
            ui_state = UI_EDIT;
        }
    } else {
        ui_state = UI_PARAM_SELECT;
    }
    ui_mark_dirty();
}

void Knob_Init(void)
{
    DL_GPIO_disableInterrupt(GPIOB, Encoder_Knob__A_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB, Encoder_Knob__A_PIN);
    OLED_Encoder_Init();
    Buzzer_SetTone(0);
}

void Knob_Tick(void)
{
    int8_t delta = OLED_Encoder_Read();
    if (delta) enc_cnt += delta;
    uint8_t button = (uint8_t)OLED_Button_Read();
    if (button) last_button = button;
    ui_process(delta, button);
}

int8_t Knob_Read(void)
{
    int8_t delta = OLED_Encoder_Read();
    if (delta) enc_cnt += delta;
    return delta;
}

int32_t Knob_GetCount(void) { return enc_cnt; }
uint8_t Knob_Button(void) { return (uint8_t)OLED_Button_Read(); }
uint8_t Knob_GetLastButton(void) { return last_button; }

static void ui_line(uint8_t y, uint8_t selected, const char *text)
{
    char line[24];
    sprintf(line, "%c%-15s", selected ? '>' : ' ', text);
    OLED_ShowString(0, y, (uint8_t*)line, 8);
}

static void ui_plain_line(uint8_t y, const char *text)
{
    char line[24];
    sprintf(line, "%-16s", text);
    OLED_ShowString(0, y, (uint8_t*)line, 8);
}

void Knob_UI_Show(void)
{
    char line[24];
    uint32_t now = Micros();
    uint8_t selected;
    const char *state_name;

    uint32_t elapsed = (uint32_t)(now - ui_last_draw);

    /* Never let display traffic disturb the running control loop. */
    if (g_race_logging_active) return;
    if (fabsf(current_base_speed) > OLED_UI_SPEED_LIMIT) return;

    if (!ui_dirty) {
        if (fabsf(current_base_speed) > 1.0f || elapsed < OLED_UI_LIVE_US) return;

        ui_last_draw = now;
        if (ui_page == UI_PAGE_TRACE) {
            selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
            sprintf(line, "ERR:%6d", (int)control_err);
            ui_line(1, selected, line);
        } else if (ui_page == UI_PAGE_GYRO) {
            selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
            sprintf(line, "YAW:%+.1f", (double)yaw);
            ui_line(1, selected, line);
        } else if (ui_page == UI_PAGE_RAW) {
            for (uint8_t row = 0; row < 4U; row++) {
                uint8_t ch = (uint8_t)(row * 2U);
                sprintf(line, "%u:%4u %u:%4u", ch,
                        (unsigned int)sensors[ch].current_ADC,
                        (unsigned int)(ch + 1U),
                        (unsigned int)sensors[ch + 1U].current_ADC);
                ui_plain_line((uint8_t)(row + 1U), line);
            }
            sprintf(line, "ERR:%+3d V:%u", (int)control_err,
                    (unsigned int)trace_line_is_valid());
            ui_plain_line(5, line);
        }
        return;
    }

    if (elapsed < OLED_UI_REFRESH_US) return;
    ui_last_draw = now;
    ui_dirty = 0U;

    if (ui_drawn_page != ui_page) {
        OLED_Clear();
        ui_drawn_page = ui_page;
    }

    if (ui_state == UI_PAGE_SELECT) state_name = "PAGE";
    else if (ui_state == UI_PARAM_SELECT) state_name = "SELECT";
    else state_name = "EDIT";

    switch (ui_page) {
    case UI_PAGE_MODE:
        sprintf(line, "MODE %s", state_name);
        ui_plain_line(0, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
        sprintf(line, "MODE:M%u %s", (unsigned int)g_race_mode,
                Race_Mode_ParametersLocked() ? "SEALED" : "READY");
        ui_line(1, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 1);
        if (g_race_mode == RACE_MODE_2) {
            sprintf(line, "B:%7lu",
                    (unsigned long)Race_Mode_GetStopPulses());
        } else if (g_race_mode == RACE_MODE_3) {
            sprintf(line, "CURV:%6.0f",
                    (double)Race_Mode_GetCurveSpeed());
        } else {
            sprintf(line, "B:      -");
        }
        ui_line(2, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 2);
        if (g_race_mode == RACE_MODE_2) {
            sprintf(line, "BRK:%7lu",
                    (unsigned long)Race_Mode_GetBrakeStartPulses());
        } else if (g_race_mode == RACE_MODE_3) {
            sprintf(line, "A:PASS FIRST");
        } else {
            sprintf(line, "BRK:    -");
        }
        ui_line(3, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 3);
        if (g_race_mode == RACE_MODE_2) {
            sprintf(line, "LEAD:%5lu",
                    (unsigned long)Race_Mode_GetStopLeadPulses());
        } else if (g_race_mode == RACE_MODE_3) {
            sprintf(line, "STOP:A+140K");
        } else {
            sprintf(line, "LEAD:   -");
        }
        ui_line(4, selected, line);
        sprintf(line, "SPD:%6.0f", (double)Race_Mode_GetRunSpeed());
        ui_plain_line(5, line);
        ui_plain_line(6, "M1 M2 M3 READY");
        break;

    case UI_PAGE_MOTOR:
        if (Race_Mode_ParametersLocked()) {
            sprintf(line, "M%u MOTOR LOCK", (unsigned int)g_race_mode);
        } else {
            sprintf(line, "M%u MOTOR %s",
                    (unsigned int)g_race_mode, state_name);
        }
        ui_plain_line(0, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
        sprintf(line, "SPD:%6.0f", (double)Race_Mode_GetRunSpeed());
        ui_line(1, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 1);
        sprintf(line, "KP:%+.4f", (double)g_motor_Kp);
        ui_line(2, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 2);
        sprintf(line, "KI:%+.3f", (double)g_motor_Ki);
        ui_line(3, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 3);
        ui_line(4, selected,
                (g_target_speed > 1.0f) ? "RUN:STOP" : "RUN:START");
        sprintf(line, "T:%2lu.%1lu P:%7lu",
                (unsigned long)(g_lap_time_ms / 1000U),
                (unsigned long)((g_lap_time_ms / 100U) % 10U),
                (unsigned long)g_lap_pulses);
        ui_plain_line(5, line);
        break;

    case UI_PAGE_TRACE:
        if (Race_Mode_ParametersLocked()) {
            sprintf(line, "M%u TRACE LOCK", (unsigned int)g_race_mode);
        } else {
            sprintf(line, "M%u TRACE %s",
                    (unsigned int)g_race_mode, state_name);
        }
        ui_plain_line(0, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
        sprintf(line, "ERR:%6d", (int)control_err);
        ui_line(1, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 1);
        sprintf(line, "K:%+.2f", (double)g_K_steer);
        ui_line(2, selected, line);
        sprintf(line, "LINE:%u ADC:%u",
                (unsigned int)trace_line_is_valid(),
                (unsigned int)ADC_OK());
        ui_plain_line(3, line);
        break;

    case UI_PAGE_GYRO:
        sprintf(line, "GYRO %s", state_name);
        ui_plain_line(0, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
        sprintf(line, "YAW:%+.1f", (double)yaw);
        ui_line(1, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 1);
        ui_line(2, selected, "CAL:CLICK");
        sprintf(line, "CFG:0x%02X", (unsigned int)g_mpu_gyro_config);
        ui_plain_line(3, line);
        sprintf(line, "SCALE:%.5f", (double)g_mpu_gyro_scale);
        ui_plain_line(4, line);
        break;

    case UI_PAGE_RAW:
        sprintf(line, "RAW %s", state_name);
        ui_plain_line(0, line);
        for (uint8_t row = 0; row < 4U; row++) {
            uint8_t ch = (uint8_t)(row * 2U);
            sprintf(line, "%u:%4u %u:%4u", ch,
                    (unsigned int)sensors[ch].current_ADC,
                    (unsigned int)(ch + 1U),
                    (unsigned int)sensors[ch + 1U].current_ADC);
            ui_plain_line((uint8_t)(row + 1U), line);
        }
        sprintf(line, "ERR:%+3d V:%u", (int)control_err,
                (unsigned int)trace_line_is_valid());
        ui_plain_line(5, line);
        break;

    case UI_PAGE_RUN_LOG: {
        char reason = 'N';
        uint32_t total_tenths = g_race_log.final_stop_time_ms / 100U;
        if (g_race_log.stop_reason == RACE_STOP_CALIBRATION_PULSE) reason = 'P';
        else if (g_race_log.stop_reason == RACE_STOP_MANUAL) reason = 'M';
        else if (g_race_log.stop_reason == RACE_STOP_FINISH_LINE) reason = 'L';
        else if (g_race_log.stop_reason == RACE_STOP_TARGET_PULSE) reason = 'B';
        else if (g_race_log.stop_reason == RACE_STOP_POST_A) reason = 'A';

        if (g_race_log.race_mode == RACE_MODE_2) {
            int32_t stop_error = (int32_t)g_race_log.final_stop_pulse -
                (int32_t)g_race_log.target_stop_pulse;
            sprintf(line, "M2 LOG R%02lu %c",
                    (unsigned long)g_race_log.run_number, reason);
            ui_plain_line(0, line);
            sprintf(line, "BRK:%7lu",
                    (unsigned long)g_race_log.brake_start_pulse);
            ui_plain_line(1, line);
            sprintf(line, "BDET:%6lu", (unsigned long)g_race_log.pulse_b);
            ui_plain_line(2, line);
            sprintf(line, "CMD:%7lu",
                    (unsigned long)g_race_log.stop_command_pulse);
            ui_plain_line(3, line);
            sprintf(line, "STP:%7lu",
                    (unsigned long)g_race_log.final_stop_pulse);
            ui_plain_line(4, line);
            sprintf(line, "ERR:%+7ld", (long)stop_error);
            ui_plain_line(5, line);
            sprintf(line, "T:%lu.%lus TOTAL",
                    (unsigned long)(total_tenths / 10U),
                    (unsigned long)(total_tenths % 10U));
            ui_plain_line(6, line);
            sprintf(line, "G:%3u TARGET:B",
                    (unsigned int)(g_race_log.max_abs_gyro_dps_x10 / 10U));
            ui_plain_line(7, line);
            break;
        }

        if (g_race_log.race_mode == RACE_MODE_3) {
            sprintf(line, "M3 LOG R%02lu %c",
                    (unsigned long)g_race_log.run_number, reason);
            ui_plain_line(0, line);
            sprintf(line, "B:%7lu", (unsigned long)g_race_log.pulse_b);
            ui_plain_line(1, line);
            sprintf(line, "D:%7lu", (unsigned long)g_race_log.pulse_d);
            ui_plain_line(2, line);
            sprintf(line, "A:%7lu %02X",
                    (unsigned long)g_race_log.first_finish_candidate_pulse,
                    (unsigned int)g_race_log.first_finish_candidate_mask);
            ui_plain_line(3, line);
            sprintf(line, "BRK:%7lu",
                    (unsigned long)g_race_log.brake_start_pulse);
            ui_plain_line(4, line);
            sprintf(line, "CMD:%7lu",
                    (unsigned long)g_race_log.stop_command_pulse);
            ui_plain_line(5, line);
            sprintf(line, "STP:%7lu",
                    (unsigned long)g_race_log.final_stop_pulse);
            ui_plain_line(6, line);
            sprintf(line, "T:%lu.%lus G:%3u%c",
                    (unsigned long)(total_tenths / 10U),
                    (unsigned long)(total_tenths % 10U),
                    (unsigned int)(g_race_log.max_abs_gyro_dps_x10 / 10U),
                    reason);
            ui_plain_line(7, line);
            break;
        }

        sprintf(line, "LOG R%02lu S%u N%u",
                (unsigned long)g_race_log.run_number,
                (unsigned int)g_race_log.phase,
                (unsigned int)g_race_log.finish_event_count);
        ui_plain_line(0, line);
        sprintf(line, "B:%7lu", (unsigned long)g_race_log.pulse_b);
        ui_plain_line(1, line);
        sprintf(line, "C:%7lu", (unsigned long)g_race_log.pulse_c);
        ui_plain_line(2, line);
        sprintf(line, "D:%7lu", (unsigned long)g_race_log.pulse_d);
        ui_plain_line(3, line);
        sprintf(line, "F:%7lu %02X",
                (unsigned long)g_race_log.first_finish_candidate_pulse,
                (unsigned int)g_race_log.first_finish_candidate_mask);
        ui_plain_line(4, line);
        sprintf(line, "CMD:%7lu", (unsigned long)g_race_log.stop_command_pulse);
        ui_plain_line(5, line);
        sprintf(line, "STP:%7lu", (unsigned long)g_race_log.final_stop_pulse);
        ui_plain_line(6, line);
        sprintf(line, "T:%lu.%lus G:%3u%c",
                (unsigned long)(total_tenths / 10U),
                (unsigned long)(total_tenths % 10U),
                (unsigned int)(g_race_log.max_abs_gyro_dps_x10 / 10U),
                reason);
        ui_plain_line(7, line);
        break;
    }

    case UI_PAGE_MASK_LOG_1:
    case UI_PAGE_MASK_LOG_2: {
        uint8_t first = (ui_page == UI_PAGE_MASK_LOG_1) ? 0U : 7U;
        uint8_t page_number = (ui_page == UI_PAGE_MASK_LOG_1) ? 1U : 2U;
        sprintf(line, "MASK%u R%02lu N%u%s",
                (unsigned int)page_number,
                (unsigned long)g_race_log.run_number,
                (unsigned int)g_race_log.finish_event_count,
                g_race_log.finish_event_overflow ? "+" : "");
        ui_plain_line(0, line);
        for (uint8_t row = 0U; row < 7U; row++) {
            uint8_t index = (uint8_t)(first + row);
            if (index < g_race_log.finish_event_count) {
                sprintf(line, "%02u %7lu %02X",
                        (unsigned int)index,
                        (unsigned long)g_race_log.finish_events[index].pulse,
                        (unsigned int)g_race_log.finish_events[index].mask);
            } else {
                line[0] = '\0';
            }
            ui_plain_line((uint8_t)(row + 1U), line);
        }
        break;
    }

    default:
        sprintf(line, "IO %s", state_name);
        ui_plain_line(0, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
        sprintf(line, "BEEP:%5lu", (unsigned long)buzzer_freq);
        ui_line(1, selected, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 1);
        sprintf(line, "LED:%s", led_on ? "ON" : "OFF");
        ui_line(2, selected, line);
        break;
    }
}

void Knob_Show_OLED(int32_t cnt, uint8_t btn)
{
    (void)cnt;
    (void)btn;
    Knob_UI_Show();
}
