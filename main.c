#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"

#define CONTROL_PERIOD_US 20000U
#define CONTROL_DT_MAX_US 50000U
#define FINISH_ARM_TIME_US 16000000U
#define FINISH_ARM_PULSES 1600000U
#define FINISH_OUTER_RIGHT_MASK (1U << CH8)
#define FINISH_INNER_RIGHT_MASK \
    ((1U << CH5) | (1U << CH6) | (1U << CH7))

//345164 电机最快速速度

/* ── 全局变量（串口可调）── */
volatile float g_target_speed = 0.0f;       /* spd 命令 */
float current_base_speed = 0;
volatile float g_K_steer      = 13.5f;      /* steer 命令：转向 P */ //10-5
int32_t control_err = 0;
volatile uint32_t g_control_dt_us = 0U;
volatile uint32_t g_control_dt_faults = 0U;
volatile uint32_t g_imu_read_failures = 0U;
volatile uint32_t g_lap_time_ms = 0U;
volatile uint32_t g_lap_pulses = 0U;
volatile uint8_t g_lap_recording = 0U;
volatile uint8_t g_finish_armed_seen = 0U;
volatile uint8_t g_finish_max_count = 0U;
volatile uint8_t g_finish_best_mask = 0U;
volatile uint8_t g_finish_last_candidate_mask = 0U;

static uint32_t lap_start_us = 0U;
static uint32_t lap_pulse_sum = 0U;

static void lap_record_finish(uint32_t now_us)
{
    g_lap_pulses = lap_pulse_sum / 2U;
    g_lap_time_ms = (uint32_t)(now_us - lap_start_us) / 1000U;
    g_lap_recording = 0U;
}

static void lap_record_update(uint32_t now_us, int32_t dL, int32_t dR)
{
    bool run_requested = g_target_speed > 1.0f;

    if (run_requested && !g_lap_recording) {
        g_lap_recording = 1U;
        lap_start_us = now_us;
        lap_pulse_sum = 0U;
        g_lap_time_ms = 0U;
        g_lap_pulses = 0U;
        g_finish_armed_seen = 0U;
        g_finish_max_count = 0U;
        g_finish_best_mask = 0U;
        g_finish_last_candidate_mask = 0U;
    }

    if (g_lap_recording) {
        uint32_t left = (dL < 0) ? (uint32_t)(-dL) : (uint32_t)dL;
        uint32_t right = (dR < 0) ? (uint32_t)(-dR) : (uint32_t)dR;

        lap_pulse_sum += left + right;

        if (!run_requested) {
            lap_record_finish(now_us);
        }
    }
}

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
            lap_record_update(now_us, dL, dR);
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

            if (ADC_OK()) {
                control_err = trace_err;
                current_base_speed = g_target_speed;
            } else {
                control_err = 0;
                current_base_speed = 0.0f;
            }

            /* 最终循迹逻辑：当前重心误差直接做比例差速。 */
            float steering = g_K_steer * (float)control_err *
                (current_base_speed / 1000.0f);
            float target_L = current_base_speed + steering;
            float target_R = current_base_speed - steering;

            /* ── 速度环 ── */
            Motor_Get_Delta(&dL, &dR);//速度不要/dt
            lap_record_update(now_us, dL, dR);

            if (g_lap_recording &&
                ((uint32_t)(now_us - lap_start_us) >= FINISH_ARM_TIME_US) &&
                (lap_pulse_sum >= FINISH_ARM_PULSES * 2U)) {
                uint8_t mask = trace_get_active_mask();
                uint8_t active_count = 0U;

                g_finish_armed_seen = 1U;

                for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
                    if ((mask & (uint8_t)(1U << i)) != 0U) {
                        active_count++;
                    }
                }

                if (active_count > g_finish_max_count) {
                    g_finish_max_count = active_count;
                    g_finish_best_mask = mask;
                }
                if (active_count >= 2U) {
                    g_finish_last_candidate_mask = mask;
                }

                if ((active_count >= 4U) ||
                    (((mask & FINISH_OUTER_RIGHT_MASK) != 0U) &&
                     ((mask & FINISH_INNER_RIGHT_MASK) != 0U))) {
                    g_target_speed = 0.0f;
                    current_base_speed = 0.0f;
                    lap_record_finish(now_us);
                    Knob_UI_Refresh();
                }
            }

            float speedL = (float)dL * inv_dt;                   /* 脉冲 */
            float speedR = (float)dR * inv_dt;
 
            float pwm_left = 0.0f;
            float pwm_right = 0.0f;
            if (current_base_speed > 0.0f) {
                pwm_left = PID_Compute(&PID_Left, target_L, speedL, dt);
                pwm_right = PID_Compute(&PID_Right, target_R, speedR, dt);
            } else {
                PID_Left.integral = 0.0f;
                PID_Right.integral = 0.0f;
            }

            Motor_Set_Speed(Left_Wheel,  (int32_t)pwm_left);
            Motor_Set_Speed(Right_Wheel, (int32_t)pwm_right);
            
            /* Vofa ch1..5: base, speedL, speedR, trace_err, yaw */
            uart_send_float5(current_base_speed, speedL, speedR,
                (float)control_err, yaw);
            
        }//白色的话 大于0 黑色小于0   黑色大于0
        Knob_Tick();
        Knob_UI_Show();
        CMD_RX();
        uart_tx_poll();
    }
}
