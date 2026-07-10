#pragma once
#include "headfile.h"

void Mag_Calibration(void);
void IMU_Update_Attitude(MPU9250_Data_t *dat, float dt);