
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"


volatile uint32_t micros_counter = 0;
volatile uint32_t system_millis = 0;

void SysTick_Handler(void) {
    micros_counter += 1000;
    system_millis++;
}

void SysTick_Init(void) {
    SysTick_Config(CPUCLK_FREQ / 1000);
}

uint32_t Micros(void) {
    uint32_t load = SysTick->LOAD;
    uint32_t val = SysTick->VAL;
    uint32_t us_in_tick = (load - val) / (CPUCLK_FREQ / 1000000);
    return micros_counter + us_in_tick;
}

int main(void)

{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    __enable_irq();

    DL_TimerA_startCounter(WHEELS_INST);
    DL_TimerG_startCounter(Left_INST);
    trace_init();

    DL_UART_Main_transmitDataBlocking(UART1,'A');

    char tx_buffer[128];
    Scan_I2C_Devices();
    MPU9250_Init();
    SysTick_Init();
    uint32_t last_us = Micros();
    Scan_I2C_Devices();

    CMD_Init();  // 开启 UART0 RX 中断（接收 PID 调参命令）

    /* OLED 初始化 */
    OLED_Init();
    OLED_Clear();
    // OLED_ShowString(0, 0, (uint8_t*)"OLED OK", 16);

   // Mag_Calibration();

    Motor_Set_Speed(Left_Wheel, 0);
    Motor_Set_Speed(Right_Wheel, 0);

    uint32_t last_uart_us = Micros();
    uint32_t last_oled_us = Micros();

    /* Yaw 漂移追踪 */
    float yaw_min = 999.0f, yaw_max = -999.0f;

    uint8_t count = 0;
extern uint8_t Rx_flag;
float drift = 0;

    while (1) {

    CMD_Poll();  // 检查串口命令（调 PID 参数）

    MPU9250_Read_All_Axis_Plus_Pro(&mpu_data);

    // sprintf (tx_buffer, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
    // mpu_data.accel_g[x],mpu_data.accel_g[y],mpu_data.accel_g[z],
    // mpu_data.gyro_dps[x],mpu_data.gyro_dps[y],mpu_data.gyro_dps[z],
    // mpu_data.mag_uT[x],mpu_data.mag_uT[y],mpu_data.mag_uT[z] );
    // uart_transmit(tx_buffer);

    // 计算真实 dt（秒）
    uint32_t now_us = Micros();
    float dt = (now_us - last_us) * 0.000001f;
    last_us = now_us;

    IMU_Update_Attitude(&mpu_data, dt);

    ComputeEulerAngles();

    /* 更新 Yaw 极值 第一秒有抖动 不记录 */
    if (now_us - last_oled_us >= 1000000) {
        if (yaw < yaw_min) yaw_min = yaw;
        if (yaw > yaw_max) yaw_max = yaw;
        if(Rx_flag == 1){
            Rx_flag = 0;
            yaw_min = yaw;   /* 从当前值重新开始追踪 */
            yaw_max = yaw;
            drift = 0;
        }
    }

    /* UART: JustFloat 每 20ms 发一次 (50Hz)，不淹死串口 */
    if (now_us - last_uart_us >= 20000) {
        last_uart_us = now_us;
        uart_send_float4(pitch, roll, yaw, dt);
    }

    /* OLED: 每 1 秒刷新，显示漂移追踪 */
    if (now_us - last_oled_us >= 1000000) {
        last_oled_us = now_us;
        char oled_buf[20];
        uint32_t uptime = system_millis / 1000;
        drift = yaw_max - yaw_min;

        //sprintf(oled_buf, "T=%lus Kp=%.2f", uptime, (double)g_Kp);
        sprintf(oled_buf, "T %lus", uptime);
        OLED_ShowString(0, 0, (uint8_t*)oled_buf, 16);

        sprintf(oled_buf, "Y=%.1f D=%.1f", (double)yaw, (double)drift);
        OLED_ShowString(0, 2, (uint8_t*)oled_buf, 16);

        count++;
        do{//5s刷新一次
            count = 0;
            sprintf(oled_buf, "Ki%.3f Kp%.1f", (double)g_Ki, (double)g_Kp);
            OLED_ShowString(0, 4, (uint8_t*)oled_buf, 16);
            sprintf(oled_buf, "mx=%.0f mn=%.0f", (double)yaw_max, (double)yaw_min);
            OLED_ShowString(0, 6, (uint8_t*)oled_buf, 16);
        }while(count > 5);
    }

   // delay_cycles(3200000);

        // char buf[64];

        // uint16_t left_cnt = DL_TimerG_getTimerCount(Left_INST);
        // int32_t right_val = right_cnt;
        // uint32_t isr_cnt  = right_isr_cnt;

        // sprintf(buf, "L:%u R:%d ISR:%lu\r\n", left_cnt, right_val, isr_cnt);
        // uart_transmit(buf);

        // delay_cycles(320000); // 32MHz主频下，约10ms

        // for (uint8_t i = 0 ; i < TRACE_SENSOR_COUNT; i++) {
        //     trace_readByADC();
        //     delay_cycles(32000);
        // }

        // int32_t final_error = trace_get_error(sensors);

        // delay_cycles(3200000);

        

    }
}
