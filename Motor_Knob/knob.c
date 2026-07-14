#include "headfile.h"

static int32_t enc_cnt = 0;          /* 旋钮累积脉冲 */

void Knob_Init(void)
{
    OLED_Encoder_Init();
}

/* 返回本次脉冲增量 (1=CW, -1=CCW, 0=无) */
int8_t Knob_Read(void)
{
    int8_t d = OLED_Encoder_Read();
    if (d) enc_cnt += d;
    return d;
}

/* 旋钮累积计数 */
int32_t Knob_GetCount(void) { return enc_cnt; }

/* 按钮: 0=NONE 1=CLICK 2=DOUBLE 3=LONG */
uint8_t Knob_Button(void) { return (uint8_t)OLED_Button_Read(); }

/* OLED 显示旋钮状态 */
void Knob_Show_OLED(int32_t cnt, uint8_t btn)
{
    static uint32_t last = 0;
    char buf[20];
    uint32_t now = Micros();

    if (now - last < 200000) return;
    last = now;

    sprintf(buf, "%d  ", (int)cnt);
    OLED_ShowString(0, 0, (uint8_t*)buf, 16);

    const char *evt[] = {"none","SINGLE","DOUBLE","LONG"};
    sprintf(buf, "%s       ", evt[btn]);
    OLED_ShowString(0, 4, (uint8_t*)buf, 16);
}
/*    // 1. 刷新旋钮+按键状态（必须高频调用）
    int8_t knob_delta = Knob_Read();    // 本次转了多少：+1正转，-1反转，0没动
    uint8_t btn_event = Knob_Button();  // 按键事件：0无，1单击，2双击，3长按

    // 2. 你的业务逻辑：比如用旋钮调Kp
    if(knob_delta != 0)
    {
        g_Kp += knob_delta * 0.1f; // 每格改0.1
    }
    
    // 3. 按键逻辑：比如单击切换参数、长按清零
    if(btn_event == BTN_CLICK)
    {
        // 切换到下一个参数
    }
    if(btn_event == BTN_LONG)
    {
        g_Kp = 0.0f;
    }

    // 4. OLED刷新（200ms一次就行，不用太快）
    static uint32_t last_oled = 0;
    uint32_t now = Micros();
    if(now - last_oled >= 200 * 1000U)
    {
        last_oled = now;
        char buf[20];
        sprintf(buf, "Kp=%.1f", g_Kp);
        OLED_ShowString(0, 0, (uint8_t*)buf, 16);
    }*/