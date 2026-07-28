#include "headfile.h"

#define UART_TX_BUF_SIZE 256U

static uint8_t tx_buf[UART_TX_BUF_SIZE];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;

void uart_transmit(char *str){
    while(*str != '\0'){
        DL_UART_Main_transmitDataBlocking(UART1,*str);
        str++;
    }
}

/* 非阻塞发送：FIFO 有空就往里塞，满了就停 */
void uart_send_nb(const uint8_t *data, uint16_t len)
{
    uint16_t used = (uint8_t)(tx_head - tx_tail);
    uint16_t free_bytes = (UART_TX_BUF_SIZE - 1U) - used;

    /* 空间不足时整包丢弃，不能只发半个VOFA帧。 */
    if ((data == NULL) || (len == 0U) || (len > free_bytes)) {
        return;
    }

    for (uint16_t i = 0; i < len; i++) {
        tx_buf[tx_head++] = data[i];
    }
}

void uart_tx_poll(void)
{
    while ((tx_tail != tx_head) &&
           !DL_UART_Main_isTXFIFOFull(UART1)) {
        DL_UART_Main_transmitData(UART1, tx_buf[tx_tail++]);
    }
}

/* Vofa JustFloat: 5 通道 float + 4 字节尾标识 */
void uart_send_float5(float a, float b, float c, float d, float e)
{
    uint8_t buf[24];
    uint8_t *p = buf;

    /* 拼 5 个 float（小端），不用 memcpy 省 flash */
    union { float f; uint32_t u; } conv;
    conv.f = a; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);
    conv.f = b; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);
    conv.f = c; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);
    conv.f = d; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);
    conv.f = e; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);

    /* Vofa JustFloat 帧尾 */
    buf[20] = 0x00; buf[21] = 0x00; buf[22] = 0x80; buf[23] = 0x7F;

    uart_send_nb(buf, sizeof(buf));
}
