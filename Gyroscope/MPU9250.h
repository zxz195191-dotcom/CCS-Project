#pragma once
#include "headfile.h"

#define MPU_ADDR 0x68
#define MPU_REG 0x3B
#define LEN 14

const float MAG_OFFSET_X = 76.09f;
const float MAG_OFFSET_Y = 27.86f;
const float MAG_OFFSET_Z = 88.34f;

#define ACC_SCALE = 1.0/16384.0f;//用乘法代替除法 提升运算速度
#define GYRO_SCALE = 1.0/131.0f;//编译器会在这里预处理
#define MAG_SCALE       0.15f
#define DEG_TO_RAD      0.0174532925f // 角度转弧度 (PI / 180)

void Scan_I2C_Devices(void);

uint8_t MPU9250_Read_Reg(uint8_t dev_addr, uint8_t reg_addr);

uint8_t MPU9250_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

typedef enum{
    x=0,y,z,num
}MPU9250_axis_e;

typedef struct MPU9250_Data_T{
//Gyro
    //int16_t gyro_raw[num];//原始数据
    float gyro_dps[num];//换算后的数据

//Accel
    //int16_t accel_raw[num];
    float accel_g[num];//重力加速度

//Magn 
    //int16_t mag_raw[num];
    float mag_uT[num];
}MPU9250_Data_t;

extern MPU9250_Data_t mpu_data;

void MPU9250_Read_All_Axis(MPU9250_Data_t *dat);

void MPU9250_Init(void);

uint8_t MPU9250_Read_Len( uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *buf );

void MPU9250_Read_All_Axis_Plus(MPU9250_Data_t *dat);

void Maf_calibration(void);