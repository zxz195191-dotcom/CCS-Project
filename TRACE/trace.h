#pragma once
#include "headfile.h"



#define AD0_H   DL_GPIO_setPins(TRACE_PORT, TRACE_AD0_PIN)
#define AD0_L   DL_GPIO_clearPins(TRACE_PORT, TRACE_AD0_PIN)
#define AD1_H   DL_GPIO_setPins(TRACE_PORT, TRACE_AD1_PIN)
#define AD1_L   DL_GPIO_clearPins(TRACE_PORT, TRACE_AD1_PIN)
#define AD2_H   DL_GPIO_setPins(TRACE_PORT, TRACE_AD2_PIN)
#define AD2_L   DL_GPIO_clearPins(TRACE_PORT, TRACE_AD2_PIN)

#define TRACE_WAIT_CONVERSION_US 10//分频了24 实际是1.33mhz

typedef enum{
    CH1 = 0,
    CH2,CH3,CH4,CH5,CH6,CH7,CH8,TRACE_SENSOR_COUNT
}trace_channel_e;


typedef struct{
    uint16_t current_ADC;
    int32_t weight;
}Trace_OUT_t;

extern Trace_OUT_t sensors[TRACE_SENSOR_COUNT];

void trace_init();//自动触发修改成软件触发
void trace_readByADC();
void CHx(uint8_t i); 
int32_t trace_get_error(Trace_OUT_t *t);
void debug_once(char character);