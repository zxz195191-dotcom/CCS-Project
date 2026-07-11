#pragma once
#include "ti_msp_dl_config.h"

/* 32MHz 主频下，32000 cycle ≈ 1ms */
static inline void mspm0_delay_ms(uint32_t ms)
{
    delay_cycles(ms * 32000);
}

/* 简易 ms 获取 — 用 SysTick 的 system_millis */
extern volatile uint32_t system_millis;
static inline void mspm0_get_clock_ms(unsigned long *ms)
{
    *ms = (unsigned long)system_millis;
}
