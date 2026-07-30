#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"
#include "race_log.h"
#include "race_mode.h"

#define CONTROL_PERIOD_US 20000U
#define CONTROL_DT_MAX_US 50000U
#define FINISH_LOG_START_PULSES 1500000U
#define FINISH_MASK_STABLE_CYCLES 2U
#define FINISH_PATTERN_STABLE_CYCLES 1U
#define FINISH_RIGHT_CH4_CH7_MASK \
    ((1U << CH4) | (1U << CH5) | (1U << CH6) | (1U << CH7))
#define FINISH_RIGHT_CH5_CH8_MASK \
    ((1U << CH5) | (1U << CH6) | (1U << CH7) | (1U << CH8))
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
#define MODE2_LAUNCH_SPEED 1000.0f
#define MODE2_ACCEL_RATE 50000.0f
#define MODE2_DECEL_RATE 90000.0f
#define MODE2_APPROACH_SPEED 12000.0f
#define MODE2_DEFAULT_BRAKE_START_PULSES 350000U
#define MODE2_DEFAULT_STOP_LEAD_PULSES 500U

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
volatile uint8_t g_race_mode = RACE_MODE_1;

static RaceStopReason race_planned_stop_reason = RACE_STOP_NONE;
static uint8_t mode2_plan_active = 0U;
static uint8_t mode2_brake_logged = 0U;
static uint32_t mode2_brake_start_pulses =
    MODE2_DEFAULT_BRAKE_START_PULSES;
static uint32_t mode2_stop_lead_pulses =
    MODE2_DEFAULT_STOP_LEAD_PULSES;

static RaceModeConfig race_mode_profiles[3] = {
    /* Mode 1: sealed 20 s lap / A-point precision-stop baseline. */
    {100000.0f, 0.0005f, 0.08f, 8.0f, 2.0f, 0.0f, 1603000U, 1U},
    /* Mode 2: independent smooth A-to-B profile; values remain field-tunable. */
    {80000.0f, 0.0005f, 0.08f, 8.0f, 2.0f, 0.0f, 426000U, 1U},
    /* Mode 3 remains reserved until its own calibration is complete. */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U, 0U}
};

static RaceModeConfig *race_mode_profile(uint8_t mode)
{
    if (mode < RACE_MODE_1 || mode > RACE_MODE_3) return NULL;
    return &race_mode_profiles[mode - RACE_MODE_1];
}

static const RaceModeConfig *race_mode_current_config(void)
{
    uint8_t index = g_race_mode;
    if (index < RACE_MODE_1 || index > RACE_MODE_3) {
        return &race_mode_profiles[0];
    }

    const RaceModeConfig *config = &race_mode_profiles[index - RACE_MODE_1];
    return config->configured ? config : &race_mode_profiles[0];
}

void Race_Mode_Apply(void)
{
    const RaceModeConfig *config = race_mode_current_config();
    g_motor_Kp = config->motor_kp;
    g_motor_Ki = config->motor_ki;
    g_K_steer = config->steer_k;
    g_imu_Kp = config->imu_kp;
    g_imu_Ki = config->imu_ki;
    trace_set_bg(true);
}

void Race_Mode_Start(void)
{
    if (g_race_logging_active || !Race_Mode_IsConfigured(g_race_mode)) return;

    Race_Mode_Apply();
    race_planned_stop_reason = RACE_STOP_NONE;
    mode2_brake_logged = 0U;

    if (g_race_mode == RACE_MODE_2) {
        mode2_plan_active = 1U;
        g_target_speed = MODE2_LAUNCH_SPEED;
    } else {
        mode2_plan_active = 0U;
        g_target_speed = race_mode_current_config()->run_speed;
    }
}

void Race_Mode_Stop(void)
{
    mode2_plan_active = 0U;
    race_planned_stop_reason = RACE_STOP_NONE;
    g_target_speed = 0.0f;
}

float Race_Mode_GetRunSpeed(void)
{
    return race_mode_current_config()->run_speed;
}

uint32_t Race_Mode_GetStopPulses(void)
{
    return race_mode_current_config()->stop_pulses;
}

uint8_t Race_Mode_ParametersLocked(void)
{
    return (g_race_mode == RACE_MODE_1) ? 1U : 0U;
}

uint8_t Race_Mode_IsConfigured(uint8_t mode)
{
    RaceModeConfig *config = race_mode_profile(mode);
    return (config != NULL && config->configured) ? 1U : 0U;
}

uint8_t Race_Mode_Select(uint8_t mode)
{
    if (!Race_Mode_IsConfigured(mode) || g_race_logging_active ||
        g_target_speed > 1.0f) {
        return 0U;
    }

    g_race_mode = mode;
    mode2_plan_active = 0U;
    race_planned_stop_reason = RACE_STOP_NONE;
    g_target_speed = 0.0f;
    Race_Mode_Apply();
    return 1U;
}

static RaceModeConfig *race_mode_editable_config(void)
{
    RaceModeConfig *config = race_mode_profile(g_race_mode);
    if (Race_Mode_ParametersLocked() || config == NULL || !config->configured) {
        return NULL;
    }
    return config;
}

uint8_t Race_Mode_SetRunSpeed(float value)
{
    RaceModeConfig *config = race_mode_editable_config();
    if (config == NULL) return 0U;
    if (value < 1000.0f) value = 1000.0f;
    if (value > 200000.0f) value = 200000.0f;
    config->run_speed = value;
    return 1U;
}

uint8_t Race_Mode_SetMotorKp(float value)
{
    RaceModeConfig *config = race_mode_editable_config();
    if (config == NULL) return 0U;
    config->motor_kp = value;
    Race_Mode_Apply();
    return 1U;
}

uint8_t Race_Mode_SetMotorKi(float value)
{
    RaceModeConfig *config = race_mode_editable_config();
    if (config == NULL) return 0U;
    config->motor_ki = value;
    Race_Mode_Apply();
    return 1U;
}

uint8_t Race_Mode_SetSteerK(float value)
{
    RaceModeConfig *config = race_mode_editable_config();
    if (config == NULL) return 0U;
    config->steer_k = value;
    Race_Mode_Apply();
    return 1U;
}

uint8_t Race_Mode_SetImuKp(float value)
{
    RaceModeConfig *config = race_mode_editable_config();
    if (config == NULL) return 0U;
    config->imu_kp = value;
    Race_Mode_Apply();
    return 1U;
}

uint8_t Race_Mode_SetImuKi(float value)
{
    RaceModeConfig *config = race_mode_editable_config();
    if (config == NULL) return 0U;
    config->imu_ki = value;
    Race_Mode_Apply();
    return 1U;
}

uint8_t Race_Mode_SetStopPulses(uint32_t value)
{
    RaceModeConfig *config = race_mode_editable_config();
    if (config == NULL || g_race_logging_active) return 0U;
    if (value < 10000U) value = 10000U;
    if (value > 3000000U) value = 3000000U;
    config->stop_pulses = value;
    if (g_race_mode == RACE_MODE_2) {
        if (mode2_brake_start_pulses >= value) {
            mode2_brake_start_pulses = value - 1000U;
        }
        if (mode2_stop_lead_pulses >= value) {
            mode2_stop_lead_pulses = value / 10U;
        }
    }
    return 1U;
}

uint32_t Race_Mode_GetBrakeStartPulses(void)
{
    return (g_race_mode == RACE_MODE_2) ? mode2_brake_start_pulses : 0U;
}

uint32_t Race_Mode_GetStopLeadPulses(void)
{
    return (g_race_mode == RACE_MODE_2) ? mode2_stop_lead_pulses : 0U;
}

uint8_t Race_Mode_SetBrakeStartPulses(uint32_t value)
{
    uint32_t target = Race_Mode_GetStopPulses();
    if (g_race_mode != RACE_MODE_2 || Race_Mode_ParametersLocked() ||
        g_race_logging_active) return 0U;
    if (value < 10000U) value = 10000U;
    if (value >= target) value = target - 1000U;
    mode2_brake_start_pulses = value;
    return 1U;
}

uint8_t Race_Mode_SetStopLeadPulses(uint32_t value)
{
    uint32_t target = Race_Mode_GetStopPulses();
    if (g_race_mode != RACE_MODE_2 || Race_Mode_ParametersLocked() ||
        g_race_logging_active) return 0U;
    if (value > 20000U) value = 20000U;
    if (value >= target) value = target / 10U;
    mode2_stop_lead_pulses = value;
    return 1U;
}

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
static uint8_t race_finish_pattern_count = 0U;
static uint32_t race_finish_pattern_pulse = 0U;
static uint8_t race_finish_pattern_mask = 0U;

static uint32_t race_current_pulses(void)
{
    return lap_pulse_sum / 2U;
}

static void race_mode_update_speed_plan(float dt)
{
    if (g_race_mode != RACE_MODE_2 || !mode2_plan_active) return;

    const RaceModeConfig *config = race_mode_current_config();
    uint32_t pulse = g_race_logging_active ? race_current_pulses() : 0U;
    uint32_t target_pulse = config->stop_pulses;
    uint32_t command_pulse = (target_pulse > mode2_stop_lead_pulses) ?
        (target_pulse - mode2_stop_lead_pulses) : target_pulse;
    uint32_t brake_pulse = mode2_brake_start_pulses;
    float approach_speed = MODE2_APPROACH_SPEED;
    float desired_speed = config->run_speed;

    if (brake_pulse >= command_pulse) {
        brake_pulse = (command_pulse > 1000U) ? command_pulse - 1000U : 0U;
    }
    if (approach_speed >= config->run_speed) {
        approach_speed = config->run_speed * 0.5f;
    }

    if (pulse >= command_pulse && g_race_logging_active) {
        mode2_plan_active = 0U;
        race_planned_stop_reason = RACE_STOP_TARGET_PULSE;
        g_target_speed = 0.0f;
        return;
    }

    if (pulse >= brake_pulse && command_pulse > brake_pulse) {
        uint32_t remaining = command_pulse - pulse;
        uint32_t span = command_pulse - brake_pulse;
        float ratio = (float)remaining / (float)span;
        desired_speed = approach_speed +
            (config->run_speed - approach_speed) * ratio;

        if (!mode2_brake_logged && g_race_logging_active) {
            mode2_brake_logged = 1U;
            g_race_log.brake_start_pulse = pulse;
        }
    }

    float delta = desired_speed - g_target_speed;
    float limit = ((delta >= 0.0f) ? MODE2_ACCEL_RATE : MODE2_DECEL_RATE) * dt;
    if (delta > limit) delta = limit;
    if (delta < -limit) delta = -limit;
    g_target_speed += delta;
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
    Race_Mode_Apply();
    race_run_counter++;
    g_race_log = (RaceRunLog){0};
    g_race_log.run_number = race_run_counter;
    g_race_log.target_stop_pulse = Race_Mode_GetStopPulses();
    g_race_log.race_mode = g_race_mode;
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
    race_finish_pattern_count = 0U;
    race_finish_pattern_pulse = 0U;
    race_finish_pattern_mask = 0U;
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

static uint8_t race_finish_pattern_matches(uint8_t mask)
{
    uint8_t ch4_ch7 = (uint8_t)FINISH_RIGHT_CH4_CH7_MASK;
    uint8_t ch5_ch8 = (uint8_t)FINISH_RIGHT_CH5_CH8_MASK;
    return (((mask & ch4_ch7) == ch4_ch7) ||
            ((mask & ch5_ch8) == ch5_ch8)) ? 1U : 0U;
}

static uint8_t race_finish_line_confirmed(uint8_t mask)
{
    uint32_t pulse = race_current_pulses();
    uint8_t in_final_curve =
        (g_race_log.phase == RACE_PHASE_DA ||
         g_race_log.phase == RACE_PHASE_AFTER_A) ? 1U : 0U;

    if (pulse < FINISH_LOG_START_PULSES ||
        !in_final_curve ||
        !race_finish_pattern_matches(mask)) {
        race_finish_pattern_count = 0U;
        return 0U;
    }

    if (race_finish_pattern_count == 0U) {
        race_finish_pattern_pulse = pulse;
        race_finish_pattern_mask = mask;
    }
    if (race_finish_pattern_count < FINISH_PATTERN_STABLE_CYCLES) {
        race_finish_pattern_count++;
    }

    if (race_finish_pattern_count >= FINISH_PATTERN_STABLE_CYCLES) {
        g_race_log.first_finish_candidate_pulse = race_finish_pattern_pulse;
        g_race_log.first_finish_candidate_mask = race_finish_pattern_mask;
        return 1U;
    }
    return 0U;
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
            RaceStopReason reason = (race_planned_stop_reason != RACE_STOP_NONE) ?
                race_planned_stop_reason : RACE_STOP_MANUAL;
            race_planned_stop_reason = RACE_STOP_NONE;
            race_request_stop(now_us, reason);
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
    Race_Mode_Apply();
    OLED_Encoder_Init();

    DL_TimerA_startCounter(WHEELS_INST);
    DL_TimerG_startCounter(Left_INST);
    Motor_Set_Speed(Left_Wheel, 0);
    Motor_Set_Speed(Right_Wheel, 0);

    
    /* 速度环 PID（脉冲/秒 单位） */
    PID_Init(&PID_Left,  g_motor_Kp, g_motor_Ki, 0.0f, 400.0f, 15000.0f);
    PID_Init(&PID_Right, g_motor_Kp, g_motor_Ki, 0.0f, 400.0f, 15000.0f);

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

            race_mode_update_speed_plan(dt);

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
            if (g_race_logging_active && g_race_mode == RACE_MODE_1) {
                race_log_finish_mask(trace_get_active_mask());
            }

            if (g_race_mode == RACE_MODE_1 && g_lap_recording &&
                race_finish_line_confirmed(trace_get_active_mask())) {
                Race_Mode_Stop();
                current_base_speed = 0.0f;
                race_request_stop(now_us, RACE_STOP_FINISH_LINE);
            }

            if (g_race_mode == RACE_MODE_1 && g_lap_recording &&
                (race_current_pulses() >= Race_Mode_GetStopPulses())) {
                Race_Mode_Stop();
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
