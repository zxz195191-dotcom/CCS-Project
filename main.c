#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "TRACE/trace.h"
#include "headfile.h"
#include "cmd_parser.h"

#define TRACE_UART_BRIDGE_TEST 0
#define TRACE_UART_FRAME_TIMEOUT_US 100000U
#define CONTROL_PERIOD_US 20000U
#define CONTROL_DT_MAX_US 50000U

//345164 电机最快速速度

/* ── 全局变量（串口可调）── */
volatile float g_target_speed = 0.0f;       /* spd 命令 */
float current_base_speed = 0;
volatile float g_K_steer      = 15.0f;      /* steer 命令：转向 P */ //10-5
volatile float g_steer_Kd     = 0.8f;       /* steerd 命令：转向 D */ // 1
int32_t control_err = 0;
float filter_error = 0.0f;
float alpha = 0.25f;
volatile uint32_t g_control_dt_us = 0U;
volatile uint32_t g_control_dt_faults = 0U;
volatile uint32_t g_imu_read_failures = 0U;
volatile uint8_t g_trace_uart_fresh = 0U;

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    __enable_irq();

#if TRACE_UART_BRIDGE_TEST
    /*
     * 临时 UART0 接收测试：
     * 只用 UART1 定时打印 UART0 中断/接收计数，先判断红外模块数据是否进来。
     * 此模式不会继续初始化电机、IMU、OLED，也不会进入循迹控制。
     */
    SysTick_Init();
    Trace_UART_Init();

    {
        const char *banner =
            "\r\nTRACE UART0 IRQ COUNT TEST\r\n"
            "UART0 TX sent: $0,1,0#\r\n";
        while (*banner != '\0') {
            DL_UART_Main_transmitDataBlocking(
                TRACE_UART_INST,
                (uint8_t)*banner);
            banner++;
        }
    }

    uint32_t last_report_us = Micros();
    uint32_t last_irq_count = 0;
    uint32_t last_rx_count = 0;
    uint32_t parsed_frame_count = 0;
    uint32_t last_parsed_frame_count = 0;
    char debug_line[128];
    Trace_UART_Frame_t latest_frame = {0};

    while (1) {
        int ch;
        uint32_t now_us = Micros();

        while ((ch = Trace_UART_ReadByte()) >= 0) {
            Trace_UART_Frame_t frame;
            if (Trace_UART_FeedByte((uint8_t)ch, &frame)) {
                latest_frame = frame;
                parsed_frame_count++;
            }
        }

        if (now_us - last_report_us >= 500000U) {
            uint8_t last = g_trace_uart_last_byte;
            char printable = ((last >= 32U) && (last <= 126U)) ? (char)last : '.';

            sprintf(debug_line,
                "[trace] irq=%lu(+%lu) rx=%lu(+%lu) frame=%lu(+%lu) last=0x%02X '%c' iidx=%lu head=%u\r\n",
                (unsigned long)g_trace_uart_irq_count,
                (unsigned long)(g_trace_uart_irq_count - last_irq_count),
                (unsigned long)g_trace_uart_rx_count,
                (unsigned long)(g_trace_uart_rx_count - last_rx_count),
                (unsigned long)parsed_frame_count,
                (unsigned long)(parsed_frame_count - last_parsed_frame_count),
                (unsigned int)last,
                printable,
                (unsigned long)g_trace_uart_last_iidx,
                (unsigned int)g_trace_uart_rx_head);

            uart_transmit(debug_line);
            if (latest_frame.valid) {
                sprintf(debug_line,
                    "[frame] %u,%u,%u,%u,%u,%u,%u,%u\r\n",
                    (unsigned int)latest_frame.x[0],
                    (unsigned int)latest_frame.x[1],
                    (unsigned int)latest_frame.x[2],
                    (unsigned int)latest_frame.x[3],
                    (unsigned int)latest_frame.x[4],
                    (unsigned int)latest_frame.x[5],
                    (unsigned int)latest_frame.x[6],
                    (unsigned int)latest_frame.x[7]);
                uart_transmit(debug_line);
            }
            last_irq_count = g_trace_uart_irq_count;
            last_rx_count = g_trace_uart_rx_count;
            last_parsed_frame_count = parsed_frame_count;
            last_report_us = now_us;
        }
    }
#endif

    SysTick_Init();
    uint32_t last_us = Micros();

    char tx_buffer[128];
//    Scan_I2C_Devices();
    MPU9250_6Axis_Init();
//    Scan_I2C_Devices();

    CMD_Init();
    Knob_Init();
    OLED_Init();
    trace_init();
    OLED_Encoder_Init();

    DL_TimerA_startCounter(WHEELS_INST);
    DL_TimerG_startCounter(Left_INST);
    Motor_Set_Speed(Left_Wheel, 0);
    Motor_Set_Speed(Right_Wheel, 0);

    
    /* 速度环 PID（脉冲/秒 单位） */
    float p = 0.0005f;//0.001 0.0005 
    float i = 0.08f;//0.55 0.3 0.2 0.1 0.09 0.08
    PID_Init(&PID_Left,  p, i, 0.0f, 400.0f, 15000.0f);
    PID_Init(&PID_Right, p, i, 0.0f, 400.0f, 15000.0f);

    int32_t  dL, dR;
    float    last_trace_err = 0.0f;
    bool previous_normal_line = false, current_normal_line = false;
    bool allow_d = false;

  //  Buzzer_SetTone(2000);
    LED_Set(true); 
    
    OLED_Startup_Calib_Gyro();
    IMU_Reset_Attitude();

//    uart_transmit(OLED_IsPresent() ? "OLED OK\r\n" : "OLED FAIL\r\n");

    DL_UART_Main_transmitDataBlocking(UART1, 'R');   /* Ready */

    Trace_UART_Init();
    Trace_UART_Frame_t trace_uart_frame = {0};
    uint32_t trace_uart_last_frame_us = 0;
    uint32_t last_pid = Micros();

    while (1) {

        uint32_t now_us = Micros();              

        /* 中断只负责收字节；解帧与循迹运算均在主循环中完成。 */
        {
            int ch;
            while ((ch = Trace_UART_ReadByte()) >= 0) {
                Trace_UART_Frame_t frame;
                if (Trace_UART_FeedByte((uint8_t)ch, &frame)) {
                    trace_uart_frame = frame;
                    trace_uart_last_frame_us = now_us;
                }
            }
        }

        uint32_t control_elapsed_us = (uint32_t)(now_us - last_pid);
        g_control_dt_us = control_elapsed_us;

        if (control_elapsed_us > CONTROL_DT_MAX_US) {
            /* Drop stale encoder accumulation and resynchronize time. */
            g_control_dt_faults++;
            last_pid = now_us;
            Motor_Get_Delta(&dL, &dR);
        } else if (control_elapsed_us >= CONTROL_PERIOD_US) {
            float dt    = (float)control_elapsed_us * 1e-6f;
            float inv_dt = 1.0f / dt;
            last_pid    = now_us;

            if (MPU9250_Read_6Axis_Plus_Pro(&mpu_data)) {
                IMU_Update_Attitude_6Axis(&mpu_data, dt);
                ComputeEulerAngles();
            } else {
                g_imu_read_failures++;
            }

            /* 串口 kp/ki 实时生效 */
            PID_Left.Kp  = g_motor_Kp;   PID_Left.Ki  = g_motor_Ki;
            PID_Right.Kp = g_motor_Kp;   PID_Right.Ki = g_motor_Ki;

            /* ── 循迹 PD：只使用最新完整 UART 帧 ── */
            bool trace_uart_frame_fresh = trace_uart_frame.valid &&
                ((uint32_t)(now_us - trace_uart_last_frame_us) <=
                    TRACE_UART_FRAME_TIMEOUT_US);
            g_trace_uart_fresh = trace_uart_frame_fresh ? 1U : 0U;
            int32_t trace_err = 0;
            if (trace_uart_frame_fresh) {
                trace_readFromUART(trace_uart_frame.x);
                trace_err = trace_get_error(sensors);
            }
            control_err = trace_err;
            if(abs(control_err) <= 8){
                control_err = 0;
            }
            if(abs(control_err) <= 18){
                filter_error += alpha * ((float)control_err - filter_error);
                if (fabsf(filter_error) < 0.1f) filter_error = 0.0f;
            }else{
                filter_error = (float)control_err;
            }


            current_normal_line = trace_uart_frame_fresh && trace_line_is_valid();
            allow_d = current_normal_line && previous_normal_line;
                    

// 如果偏差超过 20（说明遇到了急弯），基础速度强制打 6 折！
            // 取绝对值，正负弯道都减速
            if (!trace_uart_frame_fresh) {
                /* 丢失视觉数据 100 ms 后禁止继续按旧指令行驶。 */
                current_base_speed = 0.0f;
                filter_error = 0.0f;
                last_trace_err = 0.0f;
            } else if (!current_normal_line || fabsf(filter_error) > 20.0f) {
                current_base_speed = g_target_speed; 
               
            }else{
                // current_base_speed =  smooth_speed(current_base_speed,
                //                         g_target_speed);
                current_base_speed = g_target_speed;
            }


            float steer_P = g_K_steer * filter_error;
            float steer_D = 0;

            if(allow_d){
                steer_D = g_steer_Kd * (filter_error - last_trace_err) ;//取消使用变化率 防止“阶跃噪声”导致尖峰毛刺
            }
            last_trace_err = filter_error;
            previous_normal_line = current_normal_line;
            float steering = (steer_P + steer_D) * (current_base_speed / 1000.0f);

            float target_L = (current_base_speed < 1.0f) ? 0.0f : current_base_speed + steering;
            float target_R = (current_base_speed < 1.0f) ? 0.0f : current_base_speed - steering;
            if (target_L < -200000) target_L = -200000;
            if (target_L >  200000) target_L =  200000;
            if (target_R < -200000) target_R = -200000;
            if (target_R >  200000) target_R =  200000;

            /* ── 速度环 ── */
            Motor_Get_Delta(&dL, &dR);//速度不要/dt
            float speedL = (float)dL * inv_dt;                   /* 脉冲 */
            float speedR = (float)dR * inv_dt;
 
            float pwm_left  = PID_Compute(&PID_Left,  target_L, speedL, dt);
            float pwm_right = PID_Compute(&PID_Right, target_R, speedR, dt);

            Motor_Set_Speed(Left_Wheel,  (int32_t)pwm_left);
            Motor_Set_Speed(Right_Wheel, (int32_t)pwm_right);
            
            /* Vofa ch1..5: base, speedL, speedR, trace_err, yaw */
            uart_send_float5(current_base_speed, speedL, speedR,
                filter_error, yaw);
            
        }//白色的话 大于0 黑色小于0   黑色大于0
        Knob_Tick();
        Knob_UI_Show();
        CMD_RX();
        uart_tx_poll();
    }
}
