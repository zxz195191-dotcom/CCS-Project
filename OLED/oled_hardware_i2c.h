/*
 * SysConfig Configuration Steps:
 *   I2C:
 *     1. Add an I2C module.
 *     2. Name it as "I2C_OLED".
 *     3. Check the box "Enable Controller Mode".
 *     4. Set "Standard Bus Speed" to "Fast Mode (400kHz)". (optional)
 *     5. Set the pins according to your needs.
 */

#ifndef __OLED_HARDWARE_I2C_H
#define __OLED_HARDWARE_I2C_H

#include "ti_msp_dl_config.h"

/* OLED 共用 I2C_0 (PA3=SDA, PA4=SCL) — 别名映射 */
#define I2C_OLED_INST                I2C_0_INST
#define GPIO_I2C_OLED_SDA_PORT       GPIO_I2C_0_SDA_PORT
#define GPIO_I2C_OLED_SDA_PIN        GPIO_I2C_0_SDA_PIN
#define GPIO_I2C_OLED_IOMUX_SDA      GPIO_I2C_0_IOMUX_SDA
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC GPIO_I2C_0_IOMUX_SDA_FUNC
#define GPIO_I2C_OLED_SCL_PORT       GPIO_I2C_0_SCL_PORT
#define GPIO_I2C_OLED_SCL_PIN        GPIO_I2C_0_SCL_PIN
#define GPIO_I2C_OLED_IOMUX_SCL      GPIO_I2C_0_IOMUX_SCL
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC GPIO_I2C_0_IOMUX_SCL_FUNC

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

//OLED控制用函数
void delay_ms(uint32_t ms);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey);
uint32_t oled_pow(uint8_t m,uint8_t n);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t sizey);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t sizey);
void OLED_ShowFloat(uint8_t x, uint8_t y, int32_t v_q24);  /* Q24 定点, 显示 ±xxx.x */
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t no,uint8_t sizey);
void OLED_DrawBMP(uint8_t x,uint8_t y,uint8_t sizex, uint8_t sizey,uint8_t BMP[]);
void OLED_Init(void);
void oled_i2c_sda_unlock(void);

/* Gyro开机校准：3s*/
void OLED_Startup_Calib_Gyro(void);

/* 旋转编码器 + 按钮 + 菜单框架 */
void        OLED_Encoder_Init(void);
int8_t      OLED_Encoder_Read(void);              /* 1=CW, -1=CCW, 0=无 */
uint8_t     OLED_Button_Read(void);               /* 0=NONE,1=CLICK,2=DOUBLE,3=LONG */

void        OLED_Menu_Tick(void);                 /* 主循环每轮调用 */
void        OLED_Menu_Register(uint8_t page, uint8_t param_cnt);
void        OLED_Menu_SetEdit(float *val_ptr, float step);

uint8_t     OLED_Menu_GetState(void);             /* 0=PAGE, 1=PARAM, 2=EDIT */
int8_t      OLED_Menu_GetPage(void);
int8_t      OLED_Menu_GetParam(void);

#endif /* #ifndef __OLED_HARDWARE_I2C_H */
