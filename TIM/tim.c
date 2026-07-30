#include "headfile.h"


volatile uint32_t micros_counter = 0;
volatile uint32_t system_millis = 0;

void SysTick_Handler(void) {
    micros_counter += 1000;
    system_millis++;
}

void SysTick_Init(void) {
    SysTick_Config(CPUCLK_FREQ / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0U);
}

uint32_t Micros(void) {
    uint32_t primask = __get_PRIMASK();
    uint32_t base;
    uint32_t load;
    uint32_t val;

    /*
     * Keep the software millisecond counter and the hardware down-counter
     * from straddling a SysTick rollover. A delayed SysTick ISR otherwise
     * makes time appear to move backwards for nearly 1 ms.
     */
    __disable_irq();
    base = micros_counter;
    load = SysTick->LOAD;
    val = SysTick->VAL;
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U) {
        base += 1000U;
        val = SysTick->VAL;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    uint32_t us_in_tick = (load - val) / (CPUCLK_FREQ / 1000000);
    return base + us_in_tick;
}

/*
 * 蜂鸣器使用已经由 SysConfig 配好的 TIMG12 PWM。
 * 当前定时器时钟为 80 MHz，边沿对齐 PWM 的周期计数为 N 时：
 *     f_tone = 80 MHz / N
 * TIMG12 为 16 位计数器，因此这里支持约 1.2 kHz 以上的音调。
 */
void Buzzer_SetTone(uint32_t frequency_hz)
{
    uint32_t period;

    if (frequency_hz == 0U) {
        Buzzer_Stop();
        return;
    }

    /* 防止 16 位定时器装载值溢出；常用蜂鸣器音调一般在 2~4 kHz。 */
    if (frequency_hz < 1221U) {
        frequency_hz = 1221U;
    }

    period = Buzzer_INST_CLK_FREQ / frequency_hz;
    if (period < 2U) {
        period = 2U;
    }

    /* 先停表，再安全更新周期和 50% 占空比。 */
    DL_TimerG_stopCounter(Buzzer_INST);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_Buzzer_C0_IOMUX, GPIO_Buzzer_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Buzzer_C0_PORT, GPIO_Buzzer_C0_PIN);
    DL_TimerG_setLoadValue(Buzzer_INST, period - 1U);
    DL_TimerG_setCaptureCompareValue(
        Buzzer_INST, period / 2U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_startCounter(Buzzer_INST);
}

void Buzzer_Stop(void)
{
    DL_TimerG_stopCounter(Buzzer_INST);

    /* PWM 停止后切回 GPIO，明确拉低，避免停在上一个高电平。 */
    DL_GPIO_initDigitalOutput(GPIO_Buzzer_C0_IOMUX);
    DL_GPIO_enableOutput(GPIO_Buzzer_C0_PORT, GPIO_Buzzer_C0_PIN);
    DL_GPIO_clearPins(GPIO_Buzzer_C0_PORT, GPIO_Buzzer_C0_PIN);
}

void LED_Set(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIOA, LED_PA12_PIN);
    } else {
        DL_GPIO_clearPins(GPIOA, LED_PA12_PIN);
    }
}

void LED_Toggle(void)
{
    DL_GPIO_togglePins(GPIOA, LED_PA12_PIN);
}
