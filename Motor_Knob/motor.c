#include "headfile.h"

#define Max_Speed 1000        /* PID 输出上限 */

const Motor_Config_t Motor_Cfg[Wheel_count] = {
    [Left_Wheel] = {
        .dir_port = Motor_Direction_PORT,
        .in1_pin = Motor_Direction_IN1_PIN,   // PB0
        .in2_pin = Motor_Direction_IN2_PIN,   // PB1
        .pmw_tiemr = WHEELS_INST,
        .cc_index = DL_TIMER_CC_0_INDEX,      // PA17
    },
    [Right_Wheel] = {
        .dir_port = Motor_Direction_PORT,
        .in1_pin = Motor_Direction_IN3_PIN,   // PB8
        .in2_pin = Motor_Direction_IN4_PIN,   // PB9
        .pmw_tiemr = WHEELS_INST,
        .cc_index = DL_TIMER_CC_1_INDEX,      // PA18
    }
};

void Motor_Set_Speed(Motor_ID_e motor_id, int32_t speed)
{
    uint32_t abs_speed = 0;
    const Motor_Config_t *m = &Motor_Cfg[motor_id];

    if (speed >  Max_Speed) speed =  Max_Speed;
    if (speed < -Max_Speed) speed = -Max_Speed;

    if (speed >= 0) {
        DL_GPIO_setPins(m->dir_port, m->in1_pin);
        DL_GPIO_clearPins(m->dir_port, m->in2_pin);
        abs_speed = speed;
    } else {
        DL_GPIO_setPins(m->dir_port, m->in2_pin);
        DL_GPIO_clearPins(m->dir_port, m->in1_pin);
        abs_speed = -speed;
    }
    DL_TimerA_setCaptureCompareValue(m->pmw_tiemr, (Max_Speed - abs_speed), m->cc_index);
}

/* ── 右轮编码器 ISR ── */
volatile int32_t  right_cnt     = 0;
volatile uint32_t right_isr_cnt = 0;

void GROUP1_IRQHandler(void)
{
    if (DL_GPIO_getEnabledInterruptStatus(GPIOB, Right_Encoder_A_PIN)) {
        right_isr_cnt++;
        uint8_t a = DL_GPIO_readPins(GPIOB, Right_Encoder_A_PIN) ? 1 : 0;
        uint8_t b = DL_GPIO_readPins(GPIOB, Right_Encoder_B_PIN) ? 1 : 0;
        if (a != b) right_cnt++;
        else        right_cnt--;
        DL_GPIO_clearInterruptStatus(GPIOB, Right_Encoder_A_PIN);
    }
    DL_Interrupt_clearGroup(DL_INTERRUPT_GROUP_1, DL_INTERRUPT_GROUP1_IIDX_GPIOB);
}

/* 获取左右轮机械脉冲增量 (两轮统一: 一圈=364) */
void Motor_Get_Delta(int32_t *dL, int32_t *dR)
{
    static int32_t prev_L = 0, prev_R = 0;

    /* 左轮 QEI (4倍频) — 16bit 溢出矫正 → /4 得机械脉冲 */
    int32_t L = (int32_t)(int16_t)DL_TimerG_getTimerCount(Left_INST);
    *dL = L - prev_L;
    if (*dL >  32767) *dL -= 65536;
    if (*dL < -32768) *dL += 65536;
    prev_L = L;

    /* 右轮 GPIO 中断 (2倍频) */
    int32_t R = right_cnt;
    *dR = (R - prev_R) * 2;
    prev_R = R;
}

