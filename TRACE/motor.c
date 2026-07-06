#include "headfile.h"
#define Max_Speed 1000


const Motor_Config_t Motor_Cfg[Wheel_count] = {
    [Left_Wheel] = {
        .dir_port = Motor_Direction_PORT,
        .in1_pin = Motor_Direction_IN1_PIN,//PB0
        .in2_pin = Motor_Direction_IN2_PIN,//PB1
        .pmw_tiemr = WHEELS_INST,//TimerA
        .cc_index = DL_TIMER_CC_0_INDEX,//PA17
    },

    [Right_Wheel] = {
        .dir_port = Motor_Direction_PORT,
        .in1_pin = Motor_Direction_IN3_PIN,//PB8
        .in2_pin = Motor_Direction_IN4_PIN,//PB9
        .pmw_tiemr = WHEELS_INST,//TimerA
        .cc_index = DL_TIMER_CC_1_INDEX,//PA18
    }
};

void Motor_Set_Speed(Motor_ID_e motor_id,int32_t speed){
    uint32_t abs_speed = 0;
    //获取对象的信息
    const Motor_Config_t* m = &Motor_Cfg[motor_id];
    if(speed > Max_Speed) speed = Max_Speed;
    if(speed < -Max_Speed) speed = -Max_Speed;
    
    if(speed >= 0){
        DL_GPIO_setPins(m->dir_port, m->in1_pin);    
        DL_GPIO_clearPins(m->dir_port, m->in2_pin);    
        abs_speed = speed;
    }else{
        DL_GPIO_setPins(m->dir_port, m->in2_pin);    
        DL_GPIO_clearPins(m->dir_port, m->in1_pin);    
        abs_speed = -speed;        
    }
    DL_TimerA_setCaptureCompareValue(m->pmw_tiemr, (Max_Speed - abs_speed),m->cc_index);
    //debug_once('Y');
}

volatile int32_t right_cnt = 0;
volatile uint32_t right_isr_cnt = 0;

void GROUP1_IRQHandler(void)
{
    uint32_t mis = DL_GPIO_getEnabledInterruptStatus(GPIOB, Right_Encoder_A_PIN);

    if (mis) {
        right_isr_cnt++;
        if (DL_GPIO_readPins(GPIOB, Right_Encoder_B_PIN)) {
            right_cnt--;
        } else {
            right_cnt++;
        }
        DL_GPIO_clearInterruptStatus(GPIOB, Right_Encoder_A_PIN);
    }

    DL_Interrupt_clearGroup(DL_INTERRUPT_GROUP_1, DL_INTERRUPT_GROUP1_IIDX_GPIOB);
}