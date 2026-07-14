#pragma once
#include "headfile.h"

typedef enum {
    Right_Wheel = 0,
    Left_Wheel,
    Wheel_count
} Motor_ID_e;

typedef struct {
    GPIO_Regs    *dir_port;
    uint32_t      in1_pin;
    uint32_t      in2_pin;
    GPTIMER_Regs *pmw_tiemr;
    uint32_t      cc_index;
} Motor_Config_t;

extern const Motor_Config_t Motor_Cfg[Wheel_count];

void Motor_Set_Speed(Motor_ID_e motor_id, int32_t speed);

/* 右轮编码器 (ISR 更新) */
extern volatile int32_t  right_cnt;
extern volatile uint32_t right_isr_cnt;

/* 获取左右轮机械脉冲增量 (一圈=1456, 两轮统一) */
void Motor_Get_Delta(int32_t *dL, int32_t *dR);