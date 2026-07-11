#pragma once
#include "headfile.h"

void Mag_Calibration(void);
void IMU_Update_Attitude(MPU9250_Data_t *dat, float dt);
void ComputeEulerAngles(void);

extern float pitch;   // 俯仰角
extern float roll ;   // 横滚角
extern float yaw  ;   // 航向角

extern volatile float g_Kp;   // PID 比例增益（可串口在线修改）
extern volatile float g_Ki;   // PID 积分增益（可串口在线修改）