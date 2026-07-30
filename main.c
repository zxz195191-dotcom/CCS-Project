#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"
#include "race_log.h"

#define CONTROL_PERIOD_US 20000U
#define CONTROL_DT_MAX_US 50000U
#define CALIBRATION_STOP_PULSES 1598000U
#define FINISH_LOG_START_PULSES 1500000U
#define FINISH_MASK_STABLE_CYCLES 2U
#define RACE_STOP_STABLE_CYCLES 10U
#define RACE_STOP_DELTA_LIMIT 2U
#define TURN_ENTRY_RAW_DPS 5.0f
#define TURN_ENTRY_CONFIRM_DPS 12.0f
#define TURN_ENTRY_CONFIRM_DEG 5.0f
#define TURN_EXIT_CANDIDATE_DPS 12.0f
#define TURN_EXIT_STABLE_DPS 6.0f
#define TURN_EXIT_CONFIRM_DEG 135.0f
#define TURN_EXIT_STABLE_CYCLES 8U
#define B_SEARCH_START_PULSES 250000U
#define C_SEARCH_START_PULSES 650000U
#define D_SEARCH_START_PULSES 1000000U
#define A_SEARCH_START_PULSES 1450000U

//345164 电机最快速速度

/* ── 全局变量（串口可调）── */
volatile float g_target_speed = 0.0f;       /* spd 命令 */
float current_base_speed = 0;
volatile float g_K_steer      = 8.0f;      /* steer 命令：转向 P */ //10-5
int32_t control_err = 0;
volatile uint32_t g_control_dt_us = 0U;
volatile uint32_t g_control_dt_faults = 0U;
volatile uint32_t g_imu_read_failures = 0U;
volatile uint32_t g_lap_time_ms = 0U;
volatile uint32_t g_lap_pulses = 0U;
volatile uint8_t g_lap_recording = 0U;
RaceRunLog g_race_log;
volatile uint8_t g_race_log_ready = 0U;
volatile uint8_t g_race_logging_active = 0U;

static uint32_t lap_start_us = 0U;
static uint32_t lap_pulse_sum = 0U;
static uint32_t race_run_counter = 0U;
static uint8_t race_stop_pending = 0U;
static uint8_t race_stop_stable_count = 0U;
static float race_relative_yaw_deg = 0.0f;
static float race_filtered_gyro_dps = 0.0f;
static float race_turn_start_yaw_deg = 0.0f;
static int8_t race_turn_direction = 0;
static uint8_t race_entry_candidate = 0U;
static uint8_t race_entry_release_count = 0U;
static int8_t race_entry_direction = 0;
static uint32_t race_entry_candidate_pulse = 0U;
static float race_entry_candidate_yaw_deg = 0.0f;
static uint8_t race_exit_candidate = 0U;
static uint8_t race_exit_stable_count = 0U;
static uint32_t race_exit_candidate_pulse = 0U;
static uint8_t race_mask_candidate = 0U;
static uint8_t race_mask_candidate_count = 0U;
static uint32_t race_mask_candidate_pulse = 0U;
static uint8_t race_last_logged_mask = 0U;
static uint8_t race_mask_log_started = 0U;

static uint32_t race_current_pulses(void)
{
    return lap_pulse_sum / 2U;
}

static float race_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t race_signf(float value)
{
    return (value < 0.0f) ? -1 : 1;
}

static void race_log_start(uint32_t now_us)
{
    race_run_counter++;
    g_race_log = (RaceRunLog){0};
    g_race_log.run_number = race_run_counter;
    g_race_log.phase = RACE_PHASE_AB;
    g_race_log_ready = 0U;
    g_race_logging_active = 1U;
    g_lap_recording = 1U;

    lap_start_us = now_us;
    lap_pulse_sum = 0U;
    g_lap_time_ms = 0U;
    g_lap_pulses = 0U;
    race_stop_pending = 0U;
    race_stop_stable_count = 0U;
    race_relative_yaw_deg = 0.0f;
    race_filtered_gyro_dps = 0.0f;
    race_turn_start_yaw_deg = 0.0f;
    race_turn_direction = 0;
    race_entry_candidate = 0U;
    race_entry_release_count = 0U;
    race_entry_direction = 0;
    race_entry_candidate_pulse = 0U;
    race_entry_candidate_yaw_deg = 0.0f;
    race_exit_candidate = 0U;
    race_exit_stable_count = 0U;
    race_exit_candidate_pulse = 0U;
    race_mask_candidate = 0U;
    race_mask_candidate_count = 0U;
    race_mask_candidate_pulse = 0U;
    race_last_logged_mask = 0U;
    race_mask_log_started = 0U;
}

static void race_request_stop(uint32_t now_us, RaceStopReason reason)
{
    if (!g_lap_recording || race_stop_pending) return;

    g_race_log.stop_command_pulse = race_current_pulses();
    g_race_log.stop_command_time_ms =
        (uint32_t)(now_us - lap_start_us) / 1000U;
    g_race_log.stop_reason = (uint8_t)reason;
    g_lap_pulses = g_race_log.stop_command_pulse;
    g_lap_time_ms = g_race_log.stop_command_time_ms;
    g_lap_recording = 0U;
    race_stop_pending = 1U;
    race_stop_stable_count = 0U;
}

static void race_log_complete(uint32_t now_us)
{
    g_race_log.final_stop_pulse = race_current_pulses();
    g_race_log.final_stop_time_ms =
        (uint32_t)(now_us - lap_start_us) / 1000U;
    g_race_log.final_relative_yaw_x10 =
        (int16_t)(race_relative_yaw_deg * 10.0f);
    g_race_logging_active = 0U;
    g_race_log_ready = 1U;
    race_stop_pending = 0U;
    Knob_UI_OpenRaceLog();
}

static void race_finish_mask_commit(uint32_t pulse, uint8_t mask)
{
    if (mask == race_last_logged_mask &&
        g_race_log.finish_event_count > 0U) return;

    race_last_logged_mask = mask;
    if (g_race_log.finish_event_count < RACE_FINISH_EVENT_CAPACITY) {
        uint8_t index = g_race_log.finish_event_count;
        g_race_log.finish_events[index].pulse = pulse;
        g_race_log.finish_events[index].mask = mask;
        g_race_log.finish_event_count++;
    } else {
        g_race_log.finish_event_overflow = 1U;
    }

    if (g_race_log.first_finish_candidate_pulse == 0U &&
        (mask & (uint8_t)(1U << CH8)) != 0U &&
        (mask & (uint8_t)~(1U << CH8)) != 0U) {
        g_race_log.first_finish_candidate_pulse = pulse;
        g_race_log.first_finish_candidate_mask = mask;
    }
}

static void race_log_finish_mask(uint8_t mask)
{
    uint32_t pulse = race_current_pulses();
    if (pulse < FINISH_LOG_START_PULSES) return;

    if (!race_mask_log_started || mask != race_mask_candidate) {
        race_mask_log_started = 1U;
        race_mask_candidate = mask;
        race_mask_candidate_count = 1U;
        race_mask_candidate_pulse = pulse;
        return;
    }

    if (race_mask_candidate_count < FINISH_MASK_STABLE_CYCLES) {
        race_mask_candidate_count++;
        if (race_mask_candidate_count == FINISH_MASK_STABLE_CYCLES) {
            race_finish_mask_commit(race_mask_candidate_pulse,
                                    race_mask_candidate);
        }
    }
}

static void race_reset_entry_candidate(void)
{
    race_entry_candidate = 0U;
    race_entry_release_count = 0U;
}

static void race_update_straight_entry(uint32_t pulse, float raw_rate_dps)
{
    uint32_t search_start = (g_race_log.phase == RACE_PHASE_AB) ?
        B_SEARCH_START_PULSES : D_SEARCH_START_PULSES;
    if (pulse < search_start) {
        race_reset_entry_candidate();
        return;
    }

    int8_t direction = race_signf(raw_rate_dps);
    uint8_t direction_allowed = (race_turn_direction == 0) ||
        (direction == race_turn_direction);

    if (race_absf(raw_rate_dps) >= TURN_ENTRY_RAW_DPS && direction_allowed) {
        if (!race_entry_candidate || direction != race_entry_direction) {
            race_entry_candidate = 1U;
            race_entry_release_count = 0U;
            race_entry_direction = direction;
            race_entry_candidate_pulse = pulse;
            race_entry_candidate_yaw_deg = race_relative_yaw_deg;
        } else {
            race_entry_release_count = 0U;
        }
    } else if (race_entry_candidate) {
        if (race_absf(raw_rate_dps) < 3.0f || direction != race_entry_direction) {
            if (++race_entry_release_count >= 3U) {
                race_reset_entry_candidate();
            }
        }
    }

    if (race_entry_candidate &&
        race_absf(race_filtered_gyro_dps) >= TURN_ENTRY_CONFIRM_DPS) {
        float confirmed_angle =
            (race_relative_yaw_deg - race_entry_candidate_yaw_deg) *
            (float)race_entry_direction;
        if (confirmed_angle >= TURN_ENTRY_CONFIRM_DEG) {
            race_turn_direction = race_entry_direction;
            race_turn_start_yaw_deg = race_entry_candidate_yaw_deg;
            race_exit_candidate = 0U;
            race_exit_stable_count = 0U;

            if (g_race_log.phase == RACE_PHASE_AB) {
                g_race_log.pulse_b = race_entry_candidate_pulse;
                g_race_log.phase = RACE_PHASE_BC;
            } else if (g_race_log.phase == RACE_PHASE_CD) {
                g_race_log.pulse_d = race_entry_candidate_pulse;
                g_race_log.phase = RACE_PHASE_DA;
            }
            race_reset_entry_candidate();
        }
    }
}

static void race_update_curve_exit(uint32_t pulse)
{
    uint32_t search_start = (g_race_log.phase == RACE_PHASE_BC) ?
        C_SEARCH_START_PULSES : A_SEARCH_START_PULSES;
    float curve_angle = race_absf(race_relative_yaw_deg - race_turn_start_yaw_deg);

    if (pulse < search_start || curve_angle < TURN_EXIT_CONFIRM_DEG) {
        race_exit_candidate = 0U;
        race_exit_stable_count = 0U;
        return;
    }

    if (race_absf(race_filtered_gyro_dps) <= TURN_EXIT_CANDIDATE_DPS) {
        if (!race_exit_candidate) {
            race_exit_candidate = 1U;
            race_exit_candidate_pulse = pulse;
            race_exit_stable_count = 0U;
        }

        if (race_absf(race_filtered_gyro_dps) <= TURN_EXIT_STABLE_DPS) {
            if (race_exit_stable_count < TURN_EXIT_STABLE_CYCLES) {
                race_exit_stable_count++;
            }
        } else {
            race_exit_stable_count = 0U;
        }
    } else {
        race_exit_candidate = 0U;
        race_exit_stable_count = 0U;
    }

    if (race_exit_candidate &&
        race_exit_stable_count >= TURN_EXIT_STABLE_CYCLES) {
        if (g_race_log.phase == RACE_PHASE_BC) {
            g_race_log.pulse_c = race_exit_candidate_pulse;
            g_race_log.phase = RACE_PHASE_CD;
        } else if (g_race_log.phase == RACE_PHASE_DA) {
            g_race_log.pulse_a_curve_exit = race_exit_candidate_pulse;
            g_race_log.phase = RACE_PHASE_AFTER_A;
        }
        race_exit_candidate = 0U;
        race_exit_stable_count = 0U;
    }
}

static void race_log_navigation(float corrected_gyro_dps, float dt)
{
    if (!g_race_logging_active) return;

    uint32_t pulse = race_current_pulses();
    race_relative_yaw_deg += corrected_gyro_dps * dt;
    race_filtered_gyro_dps +=
        0.2f * (corrected_gyro_dps - race_filtered_gyro_dps);

    float abs_gyro = race_absf(corrected_gyro_dps);
    uint32_t abs_gyro_x10 = (uint32_t)(abs_gyro * 10.0f);
    if (abs_gyro_x10 > 65535U) abs_gyro_x10 = 65535U;
    if (abs_gyro_x10 > g_race_log.max_abs_gyro_dps_x10) {
        g_race_log.max_abs_gyro_dps_x10 = (uint16_t)abs_gyro_x10;
    }

    if (g_race_log.phase == RACE_PHASE_AB ||
        g_race_log.phase == RACE_PHASE_CD) {
        race_update_straight_entry(pulse, corrected_gyro_dps);
    } else if (g_race_log.phase == RACE_PHASE_BC ||
               g_race_log.phase == RACE_PHASE_DA) {
        race_update_curve_exit(pulse);
    }
}

static void lap_record_update(uint32_t now_us, int32_t dL, int32_t dR)
{
    bool run_requested = g_target_speed > 1.0f;

    if (run_requested && !g_race_logging_active) {
        race_log_start(now_us);
    }

    if (g_race_logging_active) {
        uint32_t left = (dL < 0) ? (uint32_t)(-dL) : (uint32_t)dL;
        uint32_t right = (dR < 0) ? (uint32_t)(-dR) : (uint32_t)dR;

        lap_pulse_sum += left + right;

        if (g_lap_recording && !run_requested) {
            race_request_stop(now_us, RACE_STOP_MANUAL);
        }

        if (race_stop_pending) {
            if ((left + right) <= RACE_STOP_DELTA_LIMIT) {
                if (race_stop_stable_count < RACE_STOP_STABLE_CYCLES) {
                    race_stop_stable_count++;
                }
            } else {
                race_stop_stable_count = 0U;
            }

            if (race_stop_stable_count >= RACE_STOP_STABLE_CYCLES) {
                race_log_complete(now_us);
            }
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

            uint8_t imu_ok = MPU9250_Read_6Axis_Plus_Pro(&mpu_data);
            if (imu_ok) {
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

            if (imu_ok) {
                race_log_navigation(mpu_data.gyro_dps[z] - gyro_bias[z], dt);
            }
            if (g_race_logging_active) {
                race_log_finish_mask(trace_get_active_mask());
            }

            if (g_lap_recording &&
                (race_current_pulses() >= CALIBRATION_STOP_PULSES)) {
                g_target_speed = 0.0f;
                current_base_speed = 0.0f;
                race_request_stop(now_us, RACE_STOP_CALIBRATION_PULSE);
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
