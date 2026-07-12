#include "headfile.h"

char     oled_buf[20];

uint8_t  btn_last = 99;   

void Encoder(void){
    uint32_t last_uart_us = Micros();
    uint32_t last_us = 0;
        /* ── 编码器 (轮询，格雷码解码) ── */
    {
        static int32_t enc_cnt = 0;
        int8_t enc = OLED_Encoder_Read();
        if (enc) enc_cnt += enc;

        /* ── 按钮事件 ── */
        uint8_t btn = OLED_Button_Read();
        if (btn != 0 && btn != btn_last) {
            btn_last = btn;
            OLED_Clear();
        }

        MPU9250_Read_All_Axis_Plus_Pro(&mpu_data);

        uint32_t now_us = Micros();
        float dt = (now_us - last_us) * 0.000001f;
        last_us = now_us;

        IMU_Update_Attitude_6Axis(&mpu_data, dt);
        ComputeEulerAngles();

        if (now_us - last_uart_us >= 20000) {
            last_uart_us = now_us;
            uart_send_float4(pitch, roll, yaw, mpu_data.gyro_dps[z]);
        }

        /* OLED */
        {
            static uint32_t last_disp = 0;
            if (now_us - last_disp >= 200000) {
                last_disp = now_us;
                sprintf(oled_buf, "%d  ", (int)enc_cnt);
                OLED_ShowString(0, 0, (uint8_t*)oled_buf, 16);

                const char *evt[] = {"none","SINGLE","DOUBLE","LONG"};
                sprintf(oled_buf, "%s       ", evt[btn]);
                OLED_ShowString(0, 4, (uint8_t*)oled_buf, 16);
            }
        }
    }

}
