#pragma once

#include <stdint.h>

#define RACE_FINISH_EVENT_CAPACITY 14U

typedef enum {
    RACE_STOP_NONE = 0,
    RACE_STOP_CALIBRATION_PULSE,
    RACE_STOP_MANUAL
} RaceStopReason;

typedef enum {
    RACE_PHASE_AB = 0,
    RACE_PHASE_BC,
    RACE_PHASE_CD,
    RACE_PHASE_DA,
    RACE_PHASE_AFTER_A
} RacePhase;

typedef struct {
    uint32_t pulse;
    uint8_t mask;
} RaceMaskEvent;

typedef struct {
    uint32_t run_number;
    uint32_t pulse_b;
    uint32_t pulse_c;
    uint32_t pulse_d;
    uint32_t pulse_a_curve_exit;
    uint32_t first_finish_candidate_pulse;
    uint32_t stop_command_pulse;
    uint32_t final_stop_pulse;
    uint32_t stop_command_time_ms;
    uint32_t final_stop_time_ms;
    uint16_t max_abs_gyro_dps_x10;
    int16_t final_relative_yaw_x10;
    uint8_t first_finish_candidate_mask;
    uint8_t stop_reason;
    uint8_t phase;
    uint8_t finish_event_count;
    uint8_t finish_event_overflow;
    RaceMaskEvent finish_events[RACE_FINISH_EVENT_CAPACITY];
} RaceRunLog;

extern RaceRunLog g_race_log;
extern volatile uint8_t g_race_log_ready;
extern volatile uint8_t g_race_logging_active;

