#pragma once
#include "headfile.h"

void Mag_Calibration(void);
void IMU_Update_Attitude(MPU9250_Data_t *dat, float dt);
void ComputeEulerAngles(void);

extern float pitch;   // 俯仰角
extern float roll ;   // 横滚角
extern float yaw  ;   // 航向角