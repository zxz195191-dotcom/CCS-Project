#include "headfile.h"


volatile uint32_t micros_counter = 0;
volatile uint32_t system_millis = 0;

void SysTick_Handler(void) {
    micros_counter += 1000;
    system_millis++;
}

void SysTick_Init(void) {
    SysTick_Config(CPUCLK_FREQ / 1000);
}

uint32_t Micros(void) {
    uint32_t load = SysTick->LOAD;
    uint32_t val = SysTick->VAL;
    uint32_t us_in_tick = (load - val) / (CPUCLK_FREQ / 1000000);
    return micros_counter + us_in_tick;
}