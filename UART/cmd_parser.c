#include "headfile.h"
#include "cmd_parser.h"
#include "race_mode.h"

/* ── 环形缓冲区（ISR 写，主循环读）── */
#define RX_BUF_SIZE 64
static volatile char rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

/* ── 命令行拼装 ── */
static char cmd_line[32];
static uint8_t cmd_len = 0;

/* ── 外部 PID 变量（定义在 MPU_Algorithm.c）── */
extern volatile float g_motor_Kp;
extern volatile float g_motor_Ki;
extern volatile float g_imu_Kp;
extern volatile float g_imu_Ki;
extern float gyro_bias[3];
extern void Gyro_Calibrate_Bias(uint16_t samples);
extern volatile float g_target_speed;
extern volatile float g_K_steer;

/* ── 前向声明 ── */
static void cmd_push_char(char c);
static void cmd_reply(const char *str);
static void cmd_respond_float(const char *label, float val);

/* ================================================================
 *  CMD_Init — 使能 UART1 RX 中断
 * ================================================================ */
void CMD_Init(void)
{
    /* 外设级中断：RX 收到数据 → 触发 */
    DL_UART_Main_enableInterrupt(TRACE_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX);

    /* NVIC 优先级 3（最低），不干扰编码器（GPIOB 优先级 2）和 SysTick（优先级 0） */
    NVIC_SetPriority(TRACE_UART_INST_INT_IRQN, 3);
    NVIC_EnableIRQ(TRACE_UART_INST_INT_IRQN);
}

/* ================================================================
 *  ISR — UART1 中断服务函数
 * ================================================================ */
void UART1_IRQHandler(void)
{
    /* UART1 配了 FIFO (3/4满): 单字节靠 RX_TIMEOUT 触发，连发靠 IIDX_RX */
    uint32_t iidx = DL_UART_Main_getPendingInterrupt(TRACE_UART_INST);
    if (iidx == DL_UART_MAIN_IIDX_RX || iidx == DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR) {
        char c = (char)DL_UART_Main_receiveData(TRACE_UART_INST);
        cmd_push_char(c);
    }
}

/* ================================================================
 *  环形缓冲区操作（ISR 侧 push，主循环侧 pop）
 * ================================================================ */
static void cmd_push_char(char c)
{
    uint8_t next = (uint8_t)((rx_head + 1) % RX_BUF_SIZE);
    if (next != rx_tail) {          /* 没满才存 */
        rx_buf[rx_head] = c;
        rx_head = next;
    }
    /* 满则丢弃 — ISR 里禁止阻塞等待 */
}

static int cmd_pop_char(void)
{
    if (rx_tail == rx_head) return -1;   /* 空 */
    char c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) % RX_BUF_SIZE);
    return (int)c;
}

/* ================================================================
 *  回复函数 — 通过 UART1 TX 发回上位机
 * ================================================================ */
static void cmd_reply(const char *str)
{
    uart_send_nb((const uint8_t *)str, (uint16_t)strlen(str));
}

static void cmd_respond_float(const char *label, float val)
{
    char buf[48];
    sprintf(buf, "%s = %.4f\r\n", label, val);
    cmd_reply(buf);
}

/* ================================================================
 *  命令解析与执行
 * ================================================================ */
static int cmd_strcmp_nocase(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;
        if (ca != cb) return 1;
        a++; b++;
    }
    return (*a == *b) ? 0 : 1;
}

static void cmd_execute(char *line)
{
    /* 跳过行首空白 */
    while (*line == ' ' || *line == '\t') line++;

    /* 空行，不管 */
    if (*line == '\0' || *line == '\r' || *line == '\n') return;

    /* ── SHOW ── */
    if (cmd_strcmp_nocase(line, "show") == 0) {
        char buf[96];
        sprintf(buf, "Mode=%u Run=%.0f Curve=%.0f Target=%lu Brake=%lu Lead=%lu\r\n",
                (unsigned int)g_race_mode,
                (double)Race_Mode_GetRunSpeed(),
                (double)Race_Mode_GetCurveSpeed(),
                (unsigned long)Race_Mode_GetStopPulses(),
                (unsigned long)Race_Mode_GetBrakeStartPulses(),
                (unsigned long)Race_Mode_GetStopLeadPulses());
        cmd_reply(buf);
        sprintf(buf, "MotorKp=%.4f MotorKi=%.4f Steer=%.2f ImuKp=%.4f ImuKi=%.4f\r\n",
                (double)g_motor_Kp, (double)g_motor_Ki,
                (double)g_K_steer,
                (double)g_imu_Kp, (double)g_imu_Ki);
        cmd_reply(buf);
        return;
    }

    /* ── KP / KI ── */
    /* 取命令 token */
    char cmd[8] = {0};
    int n = 0;
    while (*line && *line != ' ' && *line != '\t' && n < 7) {
        cmd[n++] = *line++;
    }
    cmd[n] = '\0';

    /* 取数值 */
    while (*line == ' ' || *line == '\t') line++;
    float val = 0.0f;
    if (*line) {
        /* 简易字符串转浮点 — 裸机环境无 stdlib */
        int s = 1;
        float v = 0.0f, frac = 0.1f;
        if (*line == '-') { s = -1; line++; }
        while (*line >= '0' && *line <= '9') { v = v * 10.0f + (float)(*line - '0'); line++; }
        if (*line == '.') { line++; while (*line >= '0' && *line <= '9') { v += (float)(*line - '0') * frac; frac *= 0.1f; line++; } }
        val = (float)s * v;
    }

    if (cmd_strcmp_nocase(cmd, "kp") == 0) {
        if (Race_Mode_ParametersLocked()) cmd_reply("MODE1 LOCKED\r\n");
        else {
            (void)Race_Mode_SetMotorKp(val);
            cmd_respond_float("MotorKp", g_motor_Kp);
        }
    } else if (cmd_strcmp_nocase(cmd, "ki") == 0) {
        if (Race_Mode_ParametersLocked()) cmd_reply("MODE1 LOCKED\r\n");
        else {
            (void)Race_Mode_SetMotorKi(val);
            cmd_respond_float("MotorKi", g_motor_Ki);
        }
    } else if (cmd_strcmp_nocase(cmd, "imukp") == 0) {
        if (Race_Mode_ParametersLocked()) cmd_reply("MODE1 LOCKED\r\n");
        else {
            (void)Race_Mode_SetImuKp(val);
            cmd_respond_float("ImuKp", g_imu_Kp);
        }
    } else if (cmd_strcmp_nocase(cmd, "imuki") == 0) {
        if (Race_Mode_ParametersLocked()) cmd_reply("MODE1 LOCKED\r\n");
        else {
            (void)Race_Mode_SetImuKi(val);
            cmd_respond_float("ImuKi", g_imu_Ki);
        }
    } else if (cmd_strcmp_nocase(cmd, "cal") == 0) {
        cmd_reply("Calibrating gyro 5s, keep STILL!\r\n");
        Gyro_Calibrate_Bias(5000);
        char buf[64];
        sprintf(buf, "Bias X=%.2f Y=%.2f Z=%.2f dps\r\n",
                (double)gyro_bias[0], (double)gyro_bias[1], (double)gyro_bias[2]);
        cmd_reply(buf);
    } else if (cmd_strcmp_nocase(cmd, "bias") == 0) {
        char buf[64];
        sprintf(buf, "Bias X=%.2f Y=%.2f Z=%.2f dps\r\n",
                (double)gyro_bias[0], (double)gyro_bias[1], (double)gyro_bias[2]);
        cmd_reply(buf);
    } else if (cmd_strcmp_nocase(cmd, "spd") == 0) {
        if (val > 1.0f) {
            if (!Race_Mode_ParametersLocked()) {
                (void)Race_Mode_SetRunSpeed(val);
            }
            Race_Mode_Start();
        } else {
            Race_Mode_Stop();
        }
        cmd_respond_float("RunSpeed", Race_Mode_GetRunSpeed());
    } else if (cmd_strcmp_nocase(cmd, "steer") == 0) {
        if (Race_Mode_ParametersLocked()) cmd_reply("MODE1 LOCKED\r\n");
        else {
            (void)Race_Mode_SetSteerK(val);
            cmd_respond_float("SteerP", g_K_steer);
        }
    } else if (cmd_strcmp_nocase(cmd, "mode") == 0) {
        uint8_t mode = (uint8_t)val;
        if (Race_Mode_Select(mode)) {
            char buf[32];
            sprintf(buf, "MODE %u READY\r\n", (unsigned int)g_race_mode);
            cmd_reply(buf);
        } else if (!Race_Mode_IsConfigured(mode)) {
            cmd_reply("MODE NOT CONFIGURED\r\n");
        } else {
            cmd_reply("STOP BEFORE MODE CHANGE\r\n");
        }
    } else if (cmd_strcmp_nocase(cmd, "target") == 0) {
        if (g_race_mode != RACE_MODE_2) cmd_reply("MODE2 ONLY\r\n");
        else {
            (void)Race_Mode_SetStopPulses((uint32_t)val);
            cmd_respond_float("TargetPulse", (float)Race_Mode_GetStopPulses());
        }
    } else if (cmd_strcmp_nocase(cmd, "brake") == 0) {
        if (g_race_mode != RACE_MODE_2) cmd_reply("MODE2 ONLY\r\n");
        else {
            (void)Race_Mode_SetBrakeStartPulses((uint32_t)val);
            cmd_respond_float("BrakePulse", (float)Race_Mode_GetBrakeStartPulses());
        }
    } else if (cmd_strcmp_nocase(cmd, "lead") == 0) {
        if (g_race_mode != RACE_MODE_2) cmd_reply("MODE2 ONLY\r\n");
        else {
            (void)Race_Mode_SetStopLeadPulses((uint32_t)val);
            cmd_respond_float("StopLead", (float)Race_Mode_GetStopLeadPulses());
        }
    } else if (cmd_strcmp_nocase(cmd, "reset") == 0) {
        Race_Mode_Apply();
        cmd_reply("Current mode profile restored\r\n");
    } else if(cmd_strcmp_nocase(cmd, "bg") == 0){
        if (Race_Mode_ParametersLocked()) {
            cmd_reply("MODE1 LOCKED\r\n");
        } else if(val == 0.0f ){
            trace_set_bg(false);
            cmd_reply("Black\r\n");
            cmd_reply("Black\r\n");
        }else if(val == 1.0f){
            trace_set_bg(true);
            cmd_reply("White\r\n");
            cmd_reply("White\r\n");
        }else{
            cmd_reply("Wdf?\r\n");
        }
    }else{
        cmd_reply("? [mode N] [spd N] [target N] [brake N] [lead N] [kp N] [ki N] [steer N] [show]\r\n");
    }
}

/* ================================================================
 *  CMD_Poll — 主循环调用，收字符 → 拼行 → 执行
 * ================================================================ */
uint8_t Rx_flag = 0;
void CMD_RX(void)
{
    /* 直接轮询 UART1 RX FIFO — 绕过中断，简单可靠 */
    while (!DL_UART_Main_isRXFIFOEmpty(TRACE_UART_INST)) {
        Rx_flag = 1;
        char c = (char)DL_UART_Main_receiveData(TRACE_UART_INST);
        cmd_push_char(c);
    }

    int c;
    while ((c = cmd_pop_char()) >= 0) {
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_line[cmd_len] = '\0';
                cmd_execute(cmd_line);
                cmd_len = 0;
            }
        } else if (cmd_len < (uint8_t)(sizeof(cmd_line) - 1)) {
            cmd_line[cmd_len++] = (char)c;
        }
    }
}
