#pragma once
#include "headfile.h"

void uart_transmit(char *str);
void uart_send_nb(const uint8_t *data, uint16_t len);       /* 非阻塞发送 */
void uart_send_float4(float a, float b, float c, float d);  /* Vofa JustFloat */