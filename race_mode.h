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
float Race_Mode_GetCurveSpeed(void);
uint32_t Race_Mode_GetStopPulses(void);
uint8_t Race_Mode_ParametersLocked(void);
uint8_t Race_Mode_IsConfigured(uint8_t mode);
uint8_t Race_Mode_Select(uint8_t mode);
uint8_t Race_Mode_SetRunSpeed(float value);
uint8_t Race_Mode_SetMotorKp(float value);
uint8_t Race_Mode_SetMotorKi(float value);
uint8_t Race_Mode_SetSteerK(float value);
uint8_t Race_Mode_SetImuKp(float value);
uint8_t Race_Mode_SetImuKi(float value);
uint8_t Race_Mode_SetStopPulses(uint32_t value);
uint32_t Race_Mode_GetBrakeStartPulses(void);
uint32_t Race_Mode_GetStopLeadPulses(void);
uint8_t Race_Mode_SetBrakeStartPulses(uint32_t value);
uint8_t Race_Mode_SetStopLeadPulses(uint32_t value);
