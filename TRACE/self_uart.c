#include "headfile.h"

void uart_transmit(char *str){
    while(*str != '\0'){
        DL_UART_Main_transmitDataBlocking(UART1,*str);
        str++;
    }
}

/* 非阻塞发送：FIFO 有空就往里塞，满了就停 */
void uart_send_nb(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (DL_UART_Main_isTXFIFOFull(UART1)) {
            break;  /* FIFO 满了，不等——剩下的数据下一轮再发 */
        }
        DL_UART_Main_transmitData(UART1, data[i]);
    }
}

/* Vofa JustFloat: 4 通道 float + 4 字节尾标识 */
void uart_send_float4(float a, float b, float c, float d)
{
    uint8_t buf[20];
    uint8_t *p = buf;

    /* 拼 4 个 float（小端），不用 memcpy 省 flash */
    union { float f; uint32_t u; } conv;
    conv.f = a; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);
    conv.f = b; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);
    conv.f = c; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);
    conv.f = d; *p++ = (uint8_t)conv.u; *p++ = (uint8_t)(conv.u >> 8);
                *p++ = (uint8_t)(conv.u >> 16); *p++ = (uint8_t)(conv.u >> 24);

    /* Vofa JustFloat 帧尾 */
    buf[16] = 0x00; buf[17] = 0x00; buf[18] = 0x80; buf[19] = 0x7F;

    /* 整帧原子发送 — 20 字节 @115200bps ≈ 1.7ms，等得起 */
    for (uint8_t i = 0; i < 20; i++) {
        DL_UART_Main_transmitDataBlocking(UART1, buf[i]);
    }
}