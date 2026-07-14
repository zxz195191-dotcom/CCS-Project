
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"

/* 全局变量 — 目标速度，cmd_parser 通过 extern 引用 */
volatile float g_target_speed = 0.0f;

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    __enable_irq();

    DL_TimerA_startCounter(WHEELS_INST);
    DL_TimerG_startCounter(Left_INST);
    trace_init();

    DL_UART_Main_transmitDataBlocking(UART1,'e');

    char tx_buffer[128];
    Scan_I2C_Devices();
    MPU9250_Init();
    SysTick_Init();
    uint32_t last_us = Micros();
    Scan_I2C_Devices();

    CMD_Init();  // 开启 UART0 RX 中断（接收 PID 调参命令）
    Knob_Init();
    /* OLED 初始化 */
    OLED_Init();
    OLED_Clear();
    // OLED_ShowString(0, 0, (uint8_t*)"OLED OK", 16);

    // Mag_Calibration();

    //OLED_Startup_Calib_Gyro();  /* 开机校准菜单 (OLED 驱动封装) */
    OLED_Encoder_Init();    /* 初始化旋钮编码器 */
    float p = 0.2f;
    float i = 1.3f;
    PID_Init(&PID_Left,  p, i, 0.0f, 400.0f, 300.0f);
    PID_Init(&PID_Right, p, i, 0.0f, 400.0f, 300.0f);

    int32_t dL,dR;

    /* ──── PID 速度闭环 ──── */
    uint32_t last_pid = Micros();
    while (1) {
        uint32_t now_us = Micros();           /* ① 先采样 */
        if (now_us - last_pid >= 20000) {
            float dt = (float)(now_us - last_pid) * 1e-6f;  /* ② dt≈0.02s */
            last_pid = now_us;

            /* 串口 kp/ki 实时生效 */
            PID_Left.Kp  = g_Kp;  PID_Left.Ki  = g_Ki;
            PID_Right.Kp = g_Kp;  PID_Right.Ki = g_Ki;

            Motor_Get_Delta(&dL, &dR);                       /* ③ 编码器 */

            float speedL = (float)dL;                        /* ④ 脉冲/20ms */
            float speedR = (float)dR;

            float pwm_left  = PID_Compute(&PID_Left,  g_target_speed, speedL, dt);
            float pwm_right = PID_Compute(&PID_Right, g_target_speed, speedR, dt);

            Motor_Set_Speed(Left_Wheel,  (int32_t)pwm_left);
            Motor_Set_Speed(Right_Wheel, (int32_t)pwm_right);

            uart_send_float4(g_target_speed, speedL, speedR, pwm_left);
        }
        CMD_RX();
    }



    //IMU_Update_Attitude_6Axis(&mpu_data, dt);
    //ComputeEulerAngles();
    //CMD_RX(); 调节kp ki


  /* while(1) */
 }


