
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"



uint8_t chx = 0;
int main(void)
{
    SYSCFG_DL_init();
    
    trace_init();

    DL_UART_Main_transmitDataBlocking(UART1,'A');

    while (1) {
        
        for (uint8_t i = 0 ; i < TRACE_SENSOR_COUNT; i++) {
            trace_readByADC();
            delay_cycles(32000);
        }        

        int32_t final_error = trace_get_error(sensors);

printf("CH1: %d, CH8: %d | ERROR: %d \r\n", 
               sensors[0].current_ADC, 
               sensors[7].current_ADC, 
               final_error);
        
        delay_cycles(3200000);
    }
}
