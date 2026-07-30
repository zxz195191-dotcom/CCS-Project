#include "trace_uart.h"
#include "ti_msp_dl_config.h"

/*
 * 第一阶段只验证 UART0 是否真的收到数据。
 * 暂时不做帧解析，也不把数据交给 trace.c。
 */
volatile uint32_t g_trace_uart_irq_count = 0;
volatile uint32_t g_trace_uart_rx_count = 0;
volatile uint32_t g_trace_uart_last_iidx = 0;
volatile uint8_t  g_trace_uart_last_byte = 0;
volatile uint8_t  g_trace_uart_rx_head = 0;
volatile uint8_t  g_trace_uart_rx_buffer[256];
static volatile uint8_t g_trace_uart_rx_tail = 0;

typedef enum {
    TRACE_UART_PARSE_WAIT_DOLLAR = 0,
    TRACE_UART_PARSE_WAIT_COLON,
    TRACE_UART_PARSE_VALUE
} Trace_UART_ParseState_t;

static Trace_UART_ParseState_t g_trace_uart_parse_state =
    TRACE_UART_PARSE_WAIT_DOLLAR;
static uint8_t g_trace_uart_parse_field = 0;
static uint32_t g_trace_uart_parse_value = 0;
static uint8_t g_trace_uart_parse_value_valid = 0;
volatile uint16_t g_trace_uart_latest_x[8] = {0};

static void Trace_UART_SendString(const char *str)
{
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(
            Master_Controller_INST,
            (uint8_t)*str);
        str++;
    }
}

void Trace_UART_ResetParser(void)
{
    g_trace_uart_parse_state = TRACE_UART_PARSE_WAIT_DOLLAR;
    g_trace_uart_parse_field = 0;
    g_trace_uart_parse_value = 0;
    g_trace_uart_parse_value_valid = 0;
}

static void Trace_UART_CopyFrame(Trace_UART_Frame_t *frame)
{
    uint8_t i;

    if (frame == 0) {
        return;
    }

    for (i = 0; i < 8U; i++) {
        frame->x[i] = g_trace_uart_latest_x[i];
    }
    frame->valid = 1U;
}

int Trace_UART_FeedByte(uint8_t ch, Trace_UART_Frame_t *frame)
{
    switch (g_trace_uart_parse_state) {
    case TRACE_UART_PARSE_WAIT_DOLLAR:
        if (ch == '$') {
            g_trace_uart_parse_state = TRACE_UART_PARSE_WAIT_COLON;
            g_trace_uart_parse_field = 0;
            g_trace_uart_parse_value = 0;
            g_trace_uart_parse_value_valid = 0;
        }
        break;

    case TRACE_UART_PARSE_WAIT_COLON:
        if (ch == ':') {
            g_trace_uart_parse_state = TRACE_UART_PARSE_VALUE;
            g_trace_uart_parse_value = 0;
            g_trace_uart_parse_value_valid = 0;
        } else if (ch == '$') {
            g_trace_uart_parse_field = 0;
        } else if (ch == '#') {
            Trace_UART_ResetParser();
        }
        break;

    case TRACE_UART_PARSE_VALUE:
        if ((ch >= '0') && (ch <= '9')) {
            /* External modules sometimes report full scale as 4096. */
            if (g_trace_uart_parse_value > 409U ||
                ((g_trace_uart_parse_value == 409U) && (ch > '6'))) {
                Trace_UART_ResetParser();
                return 0;
            }
            g_trace_uart_parse_value =
                g_trace_uart_parse_value * 10U + (uint32_t)(ch - '0');
            g_trace_uart_parse_value_valid = 1U;
        } else if ((ch == ',') || (ch == '#')) {
            if (g_trace_uart_parse_value_valid &&
                (g_trace_uart_parse_field < 8U)) {
                if (g_trace_uart_parse_value > 4095U) {
                    g_trace_uart_parse_value = 4095U;
                }
                g_trace_uart_latest_x[g_trace_uart_parse_field] =
                    (uint16_t)g_trace_uart_parse_value;
                g_trace_uart_parse_field++;
            }

            if ((ch == '#') && (g_trace_uart_parse_field == 8U)) {
                Trace_UART_CopyFrame(frame);
                Trace_UART_ResetParser();
                return 1;
            }

            if (ch == '#') {
                Trace_UART_ResetParser();
            } else {
                g_trace_uart_parse_state = TRACE_UART_PARSE_WAIT_COLON;
                g_trace_uart_parse_value = 0;
                g_trace_uart_parse_value_valid = 0;
            }
        } else {
            Trace_UART_ResetParser();
        }
        break;
    }

    return 0;
}

void Trace_UART_Init(void)
{
    g_trace_uart_irq_count = 0;
    g_trace_uart_rx_count = 0;
    g_trace_uart_last_iidx = 0;
    g_trace_uart_last_byte = 0;
    g_trace_uart_rx_head = 0;
    g_trace_uart_rx_tail = 0;
    Trace_UART_ResetParser();

    /* 主控通常比红外模块启动快，先等待约 200 ms。 */
    delay_cycles(8000000U);

    /* 丢弃初始化之前可能残留在接收寄存器里的字节。 */
    while (!DL_UART_Main_isRXFIFOEmpty(Master_Controller_INST)) {
        (void)DL_UART_Main_receiveData(Master_Controller_INST);
    }

    /* 开启 UART0 接收中断。全局中断已在 main 中开启。 */
    DL_UART_Main_clearInterruptStatus(
        Master_Controller_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(
        Master_Controller_INST,
        DL_UART_MAIN_INTERRUPT_RX);

    NVIC_ClearPendingIRQ(Master_Controller_INST_INT_IRQN);
    NVIC_SetPriority(Master_Controller_INST_INT_IRQN, 3);
    NVIC_EnableIRQ(Master_Controller_INST_INT_IRQN);

    /* 每次上电明确要求模块只发送模拟型八路数据。 */
    Trace_UART_SendString("$0,1,0#");
}

int Trace_UART_ReadByte(void)
{
    uint8_t ch;

    if (g_trace_uart_rx_tail == g_trace_uart_rx_head) {
        return -1;
    }

    ch = g_trace_uart_rx_buffer[g_trace_uart_rx_tail];
    g_trace_uart_rx_tail++;
    return (int)ch;
}

void Master_Controller_INST_IRQHandler(void)
{
    uint32_t iidx;

    g_trace_uart_irq_count++;
    iidx = DL_UART_Main_getPendingInterrupt(Master_Controller_INST);
    g_trace_uart_last_iidx = iidx;

    if (iidx == DL_UART_MAIN_IIDX_RX ||
        iidx == DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR) {

        /* 判断是否为空要用 isRXFIFOEmpty；receiveData 本身会取走字节。 */
        while (!DL_UART_Main_isRXFIFOEmpty(Master_Controller_INST)) {
            uint8_t ch =
                (uint8_t)DL_UART_Main_receiveData(Master_Controller_INST);
            uint8_t next = (uint8_t)(g_trace_uart_rx_head + 1U);

            g_trace_uart_last_byte = ch;
            g_trace_uart_rx_count++;

            if (next != g_trace_uart_rx_tail) {
                g_trace_uart_rx_buffer[g_trace_uart_rx_head] = ch;
                g_trace_uart_rx_head = next;
            }
        }
    }
}
