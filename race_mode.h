#pragma once

#include <stdint.h>

typedef enum {
    RACE_MODE_1 = 1,
    RACE_MODE_2,
    RACE_MODE_3
} RaceMode;

typedef struct {
    float run_speed;
    float motor_kp;
    float motor_ki;
    float steer_k;
    float imu_kp;
    float imu_ki;
    uint32_t stop_pulses;
    uint8_t configured;
} RaceModeConfig;

extern volatile uint8_t g_race_mode;

void Race_Mode_Apply(void);
void Race_Mode_Start(void);
void Race_Mode_Stop(void);
float Race_Mode_GetRunSpeed(void);
uint32_t Race_Mode_GetStopPulses(void);
uint8_t Race_Mode_ParametersLocked(void);
