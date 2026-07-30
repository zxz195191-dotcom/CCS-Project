#pragma once
#include "headfile.h"

void Mag_Calibration(void);
void Gyro_Calibrate_Bias(uint16_t samples);                        /* 静止校准陀螺仪零偏 */
void IMU_Update_Attitude(MPU9250_Data_t *dat, float dt);          /* 9 轴 (保留) */
void IMU_Update_Attitude_6Axis(MPU9250_Data_t *dat, float dt);    /* 6 轴 (当前使用) */
void ComputeEulerAngles(void);
void IMU_Reset_Attitude(void);

extern float pitch;   // 俯仰角
extern float roll ;   // 横滚角
extern float yaw  ;   // 航向角

extern volatile float g_imu_Kp;
extern volatile float g_imu_Ki;
extern float gyro_bias[3];    // 陀螺仪零偏 dps (开机校准后更新)
