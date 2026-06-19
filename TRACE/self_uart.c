#include "headfile.h"

void uart_transmit(char *str){
    while(*str != '\0'){
        DL_UART_Main_transmitDataBlocking(UART1,*str);
        str++;
    }
} 
