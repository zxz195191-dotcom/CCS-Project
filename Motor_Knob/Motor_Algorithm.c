#include "headfile.h"

#define PULSE_PER_REV  364   /* 13磁环 × 28减速比 */

float Motor_Revolution(int32_t pulses) {
    return (float)pulses / (float)PULSE_PER_REV;
}

float Motor_SpeedMs(int32_t pulses, float dt_s, float wheel_circumference_m) {
    return Motor_Revolution(pulses) * wheel_circumference_m / dt_s;
}


PID_TypeDef PID_Left;
PID_TypeDef PID_Right;

volatile float g_motor_Kp = 0.0005f;
volatile float g_motor_Ki = 0.08f;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float out_max, float int_max){
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;

    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;

    pid->out_max = out_max;
    pid->integral_max = int_max;
}



float PID_Compute(PID_TypeDef *pid, float target, float actual,float dt) {
    // 1. 记录当前目标和真实值
    pid->target = target;
    pid->actual = actual;
    
    // 2. 计算当前偏差 (Error)
    pid->error = pid->target - pid->actual;

    // 3. 积分项累加
    pid->integral += pid->error * dt;

    // 积分限幅
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    // 4. 计算 PID 最终输出
    pid->output = (pid->Kp * pid->error) +
                  (pid->Ki * pid->integral) +
                  (pid->Kd * (pid->error - pid->last_error) / dt);

    // 5. 更新上次偏差，供下一次微分(D)计算使用
    pid->last_error = pid->error;

    // 6. 【关键】：输出限幅
    // 你的 Motor_Set_Speed 最大接受 1000 的 PWM，所以算出来的输出不能超过物理极限
    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    } else if (pid->output < -pid->out_max) {
        pid->output = -pid->out_max;
    }

    return pid->output;
}

float smooth_speed(float current_spd,float wanted_spd){

    float spd_step = wanted_spd * 0.03f;

    if(current_spd < wanted_spd){
        current_spd += spd_step;
        if(current_spd > wanted_spd){
            current_spd = wanted_spd;
        }
    }    
    if(current_spd > wanted_spd){
        current_spd -= spd_step;
        if(current_spd < wanted_spd){
            current_spd = wanted_spd;
        }
    }
    return current_spd;
}
