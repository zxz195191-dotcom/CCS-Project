#pragma once
#include "headfile.h"

void Scan_I2C_Devices(void);

uint8_t MPU9250_Read_Reg(uint8_t dev_addr, uint8_t reg_addr);

uint8_t MPU9250_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

typedef enum{
    x=0,y,z,num
}MPU9250_axis_e;

typedef struct MPU9250_Data_T{
//Gyro
    int16_t gyro_raw[num];//原始数据
    float gyro_dps[num];//换算后的数据

//Accel
    int16_t accel_raw[num];
    float accel_g[num];//重力加速度
}MPU9250_Data_t;

extern MPU9250_Data_t mpu_data;

void MPU9250_Read_All_Axis(MPU9250_Data_t *dat);

void MPU9250_Init(void);