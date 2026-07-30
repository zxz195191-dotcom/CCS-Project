#pragma once
#include "headfile.h"

void Encoder(void);
void    Knob_Init(void);
void    Knob_Tick(void);
int8_t  Knob_Read(void);                    /* 1=CW, -1=CCW, 0=无 */
int32_t Knob_GetCount(void);                /* 累积脉冲数 */
uint8_t Knob_Button(void);                  /* 0=NONE 1=CLICK 2=DOUBLE 3=LONG */
uint8_t Knob_GetLastButton(void);
void    Knob_UI_Show(void);
void    Knob_UI_Refresh(void);
void    Knob_UI_OpenRaceLog(void);
void    Knob_Show_OLED(int32_t cnt, uint8_t btn);
