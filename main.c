
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"

/* 全局变量 — 目标速度，cmd_parser 通过 extern 引用 */
volatile float g_target_speed = 300.0f;

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
    float p = 0.05f;
    float i = 0.01f;
    PID_Init(&PID_Left,  p, i, 0.0f, 400.0f, 300.0f);
    PID_Init(&PID_Right, p, i, 0.0f, 400.0f, 300.0f);

    int32_t dL,dR;

    /* ──── 纯开环测试：spd 命令直接控制电机，无 PID ──── */
    /* 先无条件执行一次，排除时序问题 */
    Motor_Set_Speed(Left_Wheel,  500);
    Motor_Set_Speed(Right_Wheel, 100);

    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_17); // 强行拉高 PA17
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_18); // 强行拉高 PA18

    uint32_t last = Micros();
    while (1) {
        uint32_t now = Micros();
        if (now - last >= 20000) {
            last = now;
            Motor_Get_Delta(&dL, &dR);
            Motor_Set_Speed(Left_Wheel,  (int32_t)g_target_speed);
            Motor_Set_Speed(Right_Wheel, (int32_t)g_target_speed);
            uart_send_float4(g_target_speed, (float)dL, (float)dR, 0.0f);
        }
        CMD_RX();
    }



    //IMU_Update_Attitude_6Axis(&mpu_data, dt);
    //ComputeEulerAngles();
    //CMD_RX(); 调节kp ki


  /* while(1) */
 }


