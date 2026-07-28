#pragma once
#include "headfile.h"

void uart_transmit(char *str);
void uart_send_nb(const uint8_t *data, uint16_t len);       /* 非阻塞发送 */
void uart_tx_poll(void);
void uart_send_float5(float a, float b, float c, float d, float e);  /* Vofa JustFloat */
