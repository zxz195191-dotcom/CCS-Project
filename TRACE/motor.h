#pragma once
#include "headfile.h"

typedef enum{
    Left_Wheel = 0,
    Right_Wheel,
    Wheel_count
}Motor_ID_e;

typedef struct{
    GPIO_Regs* dir_port;
    uint32_t in1_pin;
    uint32_t in2_pin;
    GPTIMER_Regs* pmw_tiemr;
    uint32_t cc_index;
}Motor_Config_t;

extern const Motor_Config_t Motor_Cfg[Wheel_count] ;

void Motor_Set_Speed(Motor_ID_e motor_id,int32_t speed);