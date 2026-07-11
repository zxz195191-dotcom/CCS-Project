#pragma once
#include "headfile.h"

void CMD_Init(void);              // 使能 UART0 RX 中断 + NVIC
void CMD_Poll(void);              // 主循环调用，检查是否有完整命令
