#pragma once
#include "headfile.h"

#define S 1000000
#define MS 1000

void SysTick_Handler(void);
void SysTick_Init(void);
uint32_t Micros(void);
