#pragma once
#include "headfile.h"

// ==========================================
// 1. MPU9250 常量与地址定义
// ==========================================
#define MPU_ADDR 0x68
#define MPU_REG  0x3B
#define MAG_ADDR 0x0C

// 编译期算好倒数，用乘法代替除法
#define ACC_SCALE       (1.0f / 16384.0f)
#define GYRO_SCALE      (1.0f / 16.4f)
#define MAG_SCALE       0.15f
#define DEG_TO_RAD      0.0174532925f // 角度转弧度 (PI / 180)

// 【硬磁校准参数】：填入 Maf_calibration 测出的圆心
#define MAG_OFFSET_X    76.09f
#define MAG_OFFSET_Y    27.86f
#define MAG_OFFSET_Z    88.34f

typedef enum {
    x = 0, y, z, num
} MPU9250_axis_e;

typedef struct {
    // 陀螺仪
    int16_t gyro_raw[num];     
    float   gyro_dps[num];    // 度/秒
    // 加速度计
    int16_t accel_raw[num];
    float   accel_g[num];     // g (重力加速度)
    // 磁力计
    int16_t mag_raw[num];
    float   mag_uT[num];      // 微特斯拉 (未校准的原始磁场)
} MPU9250_Data_t;

extern MPU9250_Data_t mpu_data;
extern float mag_offset[3]; // 给校准函数打印用

// ==========================================
// 3. 外部函数声明
// ==========================================
void MPU9250_Init(void);
void Scan_I2C_Devices(void);

// 核心读取函数
uint8_t MPU9250_Read_Len(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *buf);
void MPU9250_Read_All_Axis(MPU9250_Data_t *dat);
void MPU9250_Read_All_Axis_Plus(MPU9250_Data_t *dat);
void MPU9250_Read_All_Axis_Plus_Pro(MPU9250_Data_t *dat);