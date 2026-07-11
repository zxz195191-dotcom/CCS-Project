
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"


volatile uint32_t micros_counter = 0;

void SysTick_Handler(void) {
    micros_counter += 1000;
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

    //Maf_calibration();

    Motor_Set_Speed(Left_Wheel, 0);
    Motor_Set_Speed(Right_Wheel, 0);
    while (1) {

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

    sprintf(tx_buffer, "%.1f,%.1f,%.1f,%.4f\r\n", pitch, roll, yaw, dt);
    uart_transmit(tx_buffer);

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
