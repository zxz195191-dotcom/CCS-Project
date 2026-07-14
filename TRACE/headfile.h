#pragma once

/* 标准库 */
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

/* SDK 生成 */
#include "ti_msp_dl_config.h"

/* TRACE (本地) */
#include "trace.h"

/* UART */
#include "../UART/self_uart.h"
#include "../UART/cmd_parser.h"

/* Motor_Knob */
#include "../Motor_Knob/motor.h"
#include "../Motor_Knob/knob.h"
#include "../Motor_Knob/Motor_Algorithm.h"

/* Gyroscope */
#include "../Gyroscope/MPU9250.h"
#include "../Gyroscope/MPU_Algorithm.h"

/* OLED */
#include "../OLED/oled_hardware_i2c.h"
#include "../OLED/oledfont.h"

/* TIM */
#include "../TIM/tim.h"
