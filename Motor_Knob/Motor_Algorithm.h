#pragma once
#include "stdint.h"

/* 脉冲 → 圈数 */
float Motor_Revolution(int32_t pulses);

/* 脉冲 → 米/秒 (dt_s=采样间隔秒, wheel_circumference_m=轮周长米) */
float Motor_SpeedMs(int32_t pulses, float dt_s, float wheel_circumference_m);


typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target;       // 目标速度 (单位：脉冲/20ms)
    float actual;       // 实际测得速度 (单位：脉冲/20ms)

    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分项

    float out_max;      // 输出限幅 (最大 PWM，比如 1000)
    float integral_max; // 积分限幅 (防止积分风饱和)

    float output;       // 最终输出的控制量 (可以直接喂给电机)
} PID_TypeDef;

// 暴露给外部使用
extern PID_TypeDef PID_Left;
extern PID_TypeDef PID_Right;

// Kp=5.0, Ki=0.5, Kd=0.0, PWM限幅=out_max, 积分限幅=int_max
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float out_max, float int_max);
float PID_Compute(PID_TypeDef *pid, float target, float actual,float dt) ;