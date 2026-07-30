#include "headfile.h"

#define KNOB_BTN_CLICK       1U
#define KNOB_BTN_LONG        3U
#define OLED_UI_REFRESH_US   200000U
#define OLED_UI_LIVE_US      500000U
#define OLED_UI_SPEED_LIMIT  30000.0f
#define MOTOR_RUN_SPEED      100000.0f

extern volatile float g_target_speed;
extern volatile float g_K_steer;
extern int32_t control_err;
extern float current_base_speed;

static int32_t enc_cnt = 0;
static uint8_t last_button = 0;
static uint8_t led_on = 1;
static uint32_t buzzer_freq = 0;

typedef enum {
    UI_PAGE_MOTOR = 0,
    UI_PAGE_TRACE,
    UI_PAGE_GYRO,
    UI_PAGE_IO,
    UI_PAGE_RAW,
    UI_PAGE_COUNT
} UiPage;

typedef enum {
    UI_PAGE_SELECT = 0,
    UI_PARAM_SELECT,
    UI_EDIT
} UiState;

static UiPage  ui_page = UI_PAGE_MOTOR;
static UiState ui_state = UI_PAGE_SELECT;
static int8_t  ui_param = 0;
static uint8_t ui_dirty = 1;
static uint32_t ui_last_draw = 0;
static UiPage ui_drawn_page = UI_PAGE_COUNT;

static uint8_t ui_param_count(UiPage page)
{
    switch (page) {
    case UI_PAGE_MOTOR: return 4U;
    case UI_PAGE_TRACE: return 2U;
    case UI_PAGE_GYRO:  return 2U;
    case UI_PAGE_IO:    return 2U;
    case UI_PAGE_RAW:   return 1U;
    default:            return 1U;
    }
}

static void ui_mark_dirty(void)
{
    ui_dirty = 1U;
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

static void ui_adjust_value(int8_t delta)
{
    switch (ui_page) {
    case UI_PAGE_MOTOR:
        if (ui_param == 0) {
            g_target_speed += (float)delta * 4000.0f;
            if (g_target_speed < 0.0f) g_target_speed = 0.0f;
            if (g_target_speed > 200000.0f) g_target_speed = 200000.0f;
        } else if (ui_param == 1) {
            g_motor_Kp += (float)delta * 0.0001f;
            if (g_motor_Kp < 0.0f) g_motor_Kp = 0.0f;
            if (g_motor_Kp > 5.0f) g_motor_Kp = 5.0f;
        } else if (ui_param == 2) {
            g_motor_Ki += (float)delta * 0.005f;
            if (g_motor_Ki < 0.0f) g_motor_Ki = 0.0f;
            if (g_motor_Ki > 10.0f) g_motor_Ki = 10.0f;
        }
        break;

    case UI_PAGE_TRACE:
        if (ui_param == 1) {
            g_K_steer += (float)delta * 0.5f;
            if (g_K_steer < 0.0f) g_K_steer = 0.0f;
            if (g_K_steer > 100.0f) g_K_steer = 100.0f;
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
            g_target_speed = (g_target_speed > 1.0f) ?
                0.0f : MOTOR_RUN_SPEED;
        } else if (ui_page == UI_PAGE_GYRO && ui_param == 1) {
            ui_calibrate_gyro();
        } else if (ui_page == UI_PAGE_IO && ui_param == 1) {
            led_on = (uint8_t)!led_on;
            LED_Set(led_on != 0U);
        } else if (!(ui_page == UI_PAGE_TRACE && ui_param == 0) &&
                   !(ui_page == UI_PAGE_GYRO && ui_param == 0) &&
                   !(ui_page == UI_PAGE_RAW)) {
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
    case UI_PAGE_MOTOR:
        sprintf(line, "MOTOR %s", state_name);
        ui_plain_line(0, line);
        selected = (ui_state != UI_PAGE_SELECT && ui_param == 0);
        sprintf(line, "SPD:%6.0f", (double)g_target_speed);
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
        break;

    case UI_PAGE_TRACE:
        sprintf(line, "TRACE %s", state_name);
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
