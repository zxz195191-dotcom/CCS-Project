#pragma once
#include "headfile.h"

#define S 1000000
#define MS 1000

void SysTick_Handler(void);
void SysTick_Init(void);
uint32_t Micros(void);

/* 蜂鸣器：TIMG12/PB20，频率单位 Hz；frequency_hz 为 0 等价于停止。 */
void Buzzer_SetTone(uint32_t frequency_hz);
void Buzzer_Stop(void);

/* 指示灯：PA12。当前按 SysConfig 的高电平点亮处理。 */
void LED_Set(bool on);
void LED_Toggle(void);
