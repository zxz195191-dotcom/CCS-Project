#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"

#define CONTROL_PERIOD_US 20000U
#define CONTROL_DT_MAX_US 50000U

//345164 电机最快速速度

/* ── 全局变量（串口可调）── */
volatile float g_target_speed = 0.0f;       /* spd 命令 */
float current_base_speed = 0;
volatile float g_K_steer      = 15.0f;      /* steer 命令：转向 P */ //10-5
volatile float g_steer_Kd     = 15.55f;     /* steerd 命令：转向 D */
int32_t control_err = 0;
float filter_error = 0.0f;
float alpha = 0.60f;
volatile uint32_t g_control_dt_us = 0U;
volatile uint32_t g_control_dt_faults = 0U;
volatile uint32_t g_imu_read_failures = 0U;

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    __enable_irq();

    SysTick_Init();
//    Scan_I2C_Devices();
    MPU9250_6Axis_Init();
//    Scan_I2C_Devices();

    CMD_Init();
    Knob_Init();
    OLED_Init();
    trace_init();
    OLED_Encoder_Init();

    DL_TimerA_startCounter(WHEELS_INST);
    DL_TimerG_startCounter(Left_INST);
    Motor_Set_Speed(Left_Wheel, 0);
    Motor_Set_Speed(Right_Wheel, 0);

    
    /* 速度环 PID（脉冲/秒 单位） */
    float p = 0.0005f;//0.001 0.0005 
    float i = 0.08f;//0.55 0.3 0.2 0.1 0.09 0.08
    PID_Init(&PID_Left,  p, i, 0.0f, 400.0f, 15000.0f);
    PID_Init(&PID_Right, p, i, 0.0f, 400.0f, 15000.0f);

    int32_t  dL, dR;
    float    last_trace_err = 0.0f;
    bool previous_normal_line = false;
    bool current_normal_line = false;
    bool allow_d = false;

  //  Buzzer_SetTone(2000);
    LED_Set(true); 
    
    OLED_Startup_Calib_Gyro();
    IMU_Reset_Attitude();

//    uart_transmit(OLED_IsPresent() ? "OLED OK\r\n" : "OLED FAIL\r\n");

    DL_UART_Main_transmitDataBlocking(UART1, 'R');   /* Ready */

    uint32_t last_pid = Micros();

    while (1) {

        uint32_t now_us = Micros();              

        uint32_t control_elapsed_us = (uint32_t)(now_us - last_pid);
        g_control_dt_us = control_elapsed_us;

        if (control_elapsed_us > CONTROL_DT_MAX_US) {
            /* Drop stale encoder accumulation and resynchronize time. */
            g_control_dt_faults++;
            last_pid = now_us;
            Motor_Get_Delta(&dL, &dR);
        } else if (control_elapsed_us >= CONTROL_PERIOD_US) {
            float dt    = (float)control_elapsed_us * 1e-6f;
            float inv_dt = 1.0f / dt;
            last_pid    = now_us;

            if (MPU9250_Read_6Axis_Plus_Pro(&mpu_data)) {
                IMU_Update_Attitude_6Axis(&mpu_data, dt);
                ComputeEulerAngles();
            } else {
                g_imu_read_failures++;
            }

            /* 串口 kp/ki 实时生效 */
            PID_Left.Kp  = g_motor_Kp;   PID_Left.Ki  = g_motor_Ki;
            PID_Right.Kp = g_motor_Kp;   PID_Right.Ki = g_motor_Ki;

            /* 八路灰度 ADC：完成一整帧采样后再进行循迹计算。 */
            trace_readByADC();
            int32_t trace_err = trace_get_error(sensors);

            if (!ADC_OK()) {
                /* ADC 转换失败时停机，避免沿用上一帧数据继续行驶。 */
                current_base_speed = 0.0f;
                filter_error = 0.0f;
                last_trace_err = 0.0f;
            }else{
                current_base_speed = g_target_speed;
                control_err = trace_err;
                filter_error += alpha *
                    ((float)control_err - filter_error);
                if (fabsf(filter_error) < 0.1f) {
                    filter_error = 0.0f;
                }
            }

            current_normal_line = ADC_OK() && trace_line_is_valid();
            allow_d = current_normal_line && previous_normal_line;

            float steer_error = filter_error;
            if (steer_error > 5.0f) {
                steer_error -= 5.0f;
            } else if (steer_error < -5.0f) {
                steer_error += 5.0f;
            } else {
                steer_error = 0.0f;
            }

            float steer_P = g_K_steer * steer_error;
            float steer_D = 0.0f;
            if (allow_d) {
                steer_D = g_steer_Kd *
                    (steer_error - last_trace_err);
                if (steer_D > 150.0f) steer_D = 150.0f;
                if (steer_D < -150.0f) steer_D = -150.0f;
            }
            last_trace_err = steer_error;
            previous_normal_line = current_normal_line;
            float steering = (steer_P + steer_D) * (current_base_speed / 1000.0f);

            float target_L = (current_base_speed < 1.0f) ? 0.0f : current_base_speed + steering;
            float target_R = (current_base_speed < 1.0f) ? 0.0f : current_base_speed - steering;
            if (target_L < -200000) target_L = -200000;
            if (target_L >  200000) target_L =  200000;
            if (target_R < -200000) target_R = -200000;
            if (target_R >  200000) target_R =  200000;

            /* ── 速度环 ── */
            Motor_Get_Delta(&dL, &dR);//速度不要/dt
            float speedL = (float)dL * inv_dt;                   /* 脉冲 */
            float speedR = (float)dR * inv_dt;
 
            float pwm_left  = PID_Compute(&PID_Left,  target_L, speedL, dt);
            float pwm_right = PID_Compute(&PID_Right, target_R, speedR, dt);

            Motor_Set_Speed(Left_Wheel,  (int32_t)pwm_left);
            Motor_Set_Speed(Right_Wheel, (int32_t)pwm_right);
            
            /* Vofa ch1..5: base, speedL, speedR, trace_err, yaw */
            uart_send_float5(current_base_speed, speedL, speedR,
                filter_error * 2500.0f, yaw);
            
        }//白色的话 大于0 黑色小于0   黑色大于0
        Knob_Tick();
        Knob_UI_Show();
        CMD_RX();
        uart_tx_poll();
    }
}
