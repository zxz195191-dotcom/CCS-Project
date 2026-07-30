#pragma once

#include <stdint.h>

void Trace_UART_Init(void);
int Trace_UART_ReadByte(void);

typedef struct {
    uint16_t x[8];
    uint8_t valid;
} Trace_UART_Frame_t;

void Trace_UART_ResetParser(void);
int Trace_UART_FeedByte(uint8_t ch, Trace_UART_Frame_t *frame);

/* 第一阶段调试变量：在 Expressions/Watch 中直接观察这些变量。 */
extern volatile uint32_t g_trace_uart_irq_count;
extern volatile uint32_t g_trace_uart_rx_count;
extern volatile uint32_t g_trace_uart_last_iidx;
extern volatile uint8_t  g_trace_uart_last_byte;
extern volatile uint8_t  g_trace_uart_rx_head;
extern volatile uint8_t  g_trace_uart_rx_buffer[256];
extern volatile uint16_t g_trace_uart_latest_x[8];
