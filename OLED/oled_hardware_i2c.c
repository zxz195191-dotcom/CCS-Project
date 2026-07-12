#include "oled_hardware_i2c.h"
#include "oledfont.h"
#include "clock.h"
#include <ti/iqmath/include/IQmathLib.h>
#include <string.h>

#define I2C_TIMEOUT_MS  (10)

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127

void delay_ms(uint32_t ms)
{
    mspm0_delay_ms(ms);
}

static int mspm0_i2c_disable(void)
{
    DL_I2C_reset(I2C_OLED_INST);
    DL_GPIO_initDigitalOutput(GPIO_I2C_OLED_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_OLED_IOMUX_SDA,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
    return 0;
}

static int mspm0_i2c_enable(void)
{
    DL_I2C_reset(I2C_OLED_INST);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SDA,
        GPIO_I2C_OLED_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SCL,
        GPIO_I2C_OLED_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SCL);
    DL_I2C_enablePower(I2C_OLED_INST);
    SYSCFG_DL_I2C_0_init();
    return 0;
}

void oled_i2c_sda_unlock(void)
{
    uint8_t cycleCnt = 0;
    mspm0_i2c_disable();
    do
    {
        DL_GPIO_clearPins(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
        mspm0_delay_ms(1);
        DL_GPIO_setPins(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
        mspm0_delay_ms(1);

        if(DL_GPIO_readPins(GPIO_I2C_OLED_SDA_PORT, GPIO_I2C_OLED_SDA_PIN))
            break;
    }while(++cycleCnt < 100);
    mspm0_i2c_enable();
}

//反显函数
void OLED_ColorTurn(uint8_t i)
{
    if(i==0)
    {
        OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
    }
    if(i==1)
    {
        OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
    }
}

//屏幕旋转180度
void OLED_DisplayTurn(uint8_t i)
{
if(i==0)
    {
        OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
        OLED_WR_Byte(0xA1,OLED_CMD);
    }
    if(i==1)
    {
        OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
        OLED_WR_Byte(0xA0,OLED_CMD);
    }
}

//发送一个字节
//向SSD1306写入一个字节。
//mode:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(uint8_t dat,uint8_t mode)
{
    unsigned char ptr[2];
    unsigned long start, cur;

    if(mode)
    {
        ptr[0] = 0x40;
        ptr[1] = dat;
    }
    else
    {
        ptr[0] = 0x00;
        ptr[1] = dat;
    }

    mspm0_get_clock_ms(&start);

    DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, ptr, 2);
    DL_I2C_clearInterruptStatus(I2C_OLED_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_OLED_INST, 0x3C, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    while (!DL_I2C_getRawInterruptStatus(I2C_OLED_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        mspm0_get_clock_ms(&cur);
        if(cur >= (start + I2C_TIMEOUT_MS))
        {
            oled_i2c_sda_unlock();
            break;
        }
    }
}

//坐标设置
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f),OLED_CMD);
}

//开启OLED显示    
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

//关闭OLED显示     
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
	 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
    uint8_t i,n;		    
    for(i=0;i<8;i++)  
    {  
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
        for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
    } //更新显示
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63				 
//sizey:选择字体 6x8  8x16
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey)
{      	
    uint8_t c=0,sizex=sizey/2;
    uint16_t i=0,size1;
    if(sizey==8)size1=6;
    else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
    c=chr-' ';//得到偏移后的值
    OLED_Set_Pos(x,y);
    for(i=0;i<size1;i++)
    {
        if(i%sizex==0&&sizey!=8) OLED_Set_Pos(x,y++);
        if(sizey==8) OLED_WR_Byte(asc2_0806[c][i],OLED_DATA);//6X8字号
        else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i],OLED_DATA);//8x16字号
        //		else if(sizey==xx) OLED_WR_Byte(asc2_xxxx[c][i],OLED_DATA);//用户添加字号
        else return;
    }
}

//m^n函数
uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;	 
    while(n--)result*=m;    
    return result;
}

//显示数字
//x,y :起点坐标
//num:要显示的数字
//len :数字的位数
//sizey:字体大小		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t sizey)
{         	
    uint8_t t,temp,m=0;
    uint8_t enshow=0;
    if(sizey==8)m=2;
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
                continue;
            }else enshow=1;
        }
        OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
    }
}

//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t sizey)
{
    uint8_t j=0;
    while (chr[j]!='\0')
    {		
        OLED_ShowChar(x,y,chr[j++],sizey);
        if(sizey==8)x+=6;
        else x+=sizey/2;
    }
}

/* ── 显示 Q24 定点角度值 (±xxx.x) ── */
void OLED_ShowFloat(uint8_t x, uint8_t y, int32_t v_q24)
{
    char    buf[9];
    int32_t raw, abs_v;
    _iq     v = (_iq)v_q24;

    memset(buf, ' ', 8); buf[8] = 0;

    /* 接近零 */
    if (v > _IQ(-0.05) && v < _IQ(0.05)) {
        buf[0] = '+'; buf[4] = '0'; buf[5] = '.'; buf[6] = '0';
        OLED_ShowString(x, y, (uint8_t*)buf, 8);
        return;
    }

    /* v×10 四舍五入 → tenths of degrees */
    {
        _iq v10 = _IQmpy(v, _IQ(10.0));
        if (v10 >= 0)
            raw = _IQint(v10 + _IQ(0.5));
        else
            raw = _IQint(v10 - _IQ(0.5));
    }

    buf[0] = (raw < 0) ? '-' : '+';
    abs_v  = (raw < 0) ? -raw : raw;

    buf[6] = '0' + (abs_v % 10);  abs_v /= 10;   /* 小数位 */
    buf[5] = '.';
    buf[4] = '0' + (abs_v % 10);  abs_v /= 10;   /* 个位   */
    if (abs_v) { buf[3] = '0' + (abs_v % 10); abs_v /= 10; }  /* 十位 */
    if (abs_v) { buf[2] = '0' + (abs_v % 10); }                /* 百位 */

    OLED_ShowString(x, y, (uint8_t*)buf, 8);
}

//显示汉字
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t no,uint8_t sizey)
{
    uint16_t i,size1=(sizey/8+((sizey%8)?1:0))*sizey;
    for(i=0;i<size1;i++)
    {
        if(i%sizey==0) OLED_Set_Pos(x,y++);
        if(sizey==16) OLED_WR_Byte(Hzk[no][i],OLED_DATA);//16x16字号
        //		else if(sizey==xx) OLED_WR_Byte(xxx[c][i],OLED_DATA);//用户添加字号
        else return;
    }				
}

//显示图片
//x,y显示坐标
//sizex,sizey,图片长宽
//BMP：要显示的图片
void OLED_DrawBMP(uint8_t x,uint8_t y,uint8_t sizex, uint8_t sizey,uint8_t BMP[])
{ 	
    uint16_t j=0;
    uint8_t i,m;
    sizey=sizey/8+((sizey%8)?1:0);
    for(i=0;i<sizey;i++)
    {
        OLED_Set_Pos(x,i+y);
        for(m=0;m<sizex;m++)
        {      
            OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
        }
    }
}

//初始化SSD1306					    
void OLED_Init(void)
{
    if(DL_I2C_getSDAStatus(I2C_OLED_INST) == DL_I2C_CONTROLLER_SDA_LOW)
        oled_i2c_sda_unlock();

    delay_ms(200);

    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
    OLED_Clear();
    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
}

/* ================================================================
 *  开机校准菜单 — 封装进 OLED 驱动，供 main.c 一行调用
 *  返回: 1=做了精校准, 0=只用快速校准
 * ================================================================ */
#include "MPU_Algorithm.h"  /* Gyro_Calibrate_Bias, gyro_bias */
#include <stdio.h>

void OLED_Startup_Calib_Gyro(void)
{
    const char *dots[] = {"Calibrating.  ", "Calibrating.. ", "Calibrating..."};
    float sx = 0, sy = 0, sz = 0;
    uint16_t total = 0;

    OLED_Clear();
    for (uint8_t s = 0; s < 6; s++) {
        OLED_ShowString(10, 3, (uint8_t*)dots[s % 3], 16);
        for (uint16_t i = 0; i < 500; i++) {
            MPU9250_Read_All_Axis_Plus_Pro(&mpu_data);
            sx += mpu_data.gyro_dps[x];
            sy += mpu_data.gyro_dps[y];
            sz += mpu_data.gyro_dps[z];
            delay_cycles(32000);
        }
        total += 500;
    }
    gyro_bias[x] = sx / (float)total;
    gyro_bias[y] = sy / (float)total;
    gyro_bias[z] = sz / (float)total;
    OLED_Clear();
}

/* ================================================================
 *  旋转编码器 + 按钮状态机 + 菜单框架
 *  (PB2=A相, PB3=B相, PB13=按键)
 * ================================================================ */

/* ── 按钮事件 ── */
typedef enum {
    BTN_NONE = 0,
    BTN_CLICK,
    BTN_DOUBLE,
    BTN_LONG,
} ButtonEvent;

/* ── 编码器：正交格雷码解码 ── */
static uint8_t enc_last_ab = 0;

void OLED_Encoder_Init(void)
{
    uint8_t a = DL_GPIO_readPins(GPIOB, Encoder_Knob__A_PIN) ? 1 : 0;
    uint8_t b = DL_GPIO_readPins(GPIOB, Encoder_Knob__B_PIN) ? 1 : 0;
    enc_last_ab = (a << 1) | b;
}

int8_t OLED_Encoder_Read(void)
{
    uint8_t a  = DL_GPIO_readPins(GPIOB, Encoder_Knob__A_PIN) ? 1 : 0;
    uint8_t b  = DL_GPIO_readPins(GPIOB, Encoder_Knob__B_PIN) ? 1 : 0;
    uint8_t ab = (a << 1) | b;
    int8_t  ret = 0;

    if (ab != enc_last_ab) {
        uint8_t old = enc_last_ab;
        enc_last_ab = ab;
        if ((old == 0 && ab == 1) || (old == 1 && ab == 3) ||
            (old == 3 && ab == 2) || (old == 2 && ab == 0)) ret = 1;
        else if ((old == 0 && ab == 2) || (old == 2 && ab == 3) ||
                 (old == 3 && ab == 1) || (old == 1 && ab == 0)) ret = -1;
    }
    return ret;
}

/* ── 按钮状态机 ── */
#define BTN_TICK_MS  4
#define BTN_LONG_MS  800
#define BTN_DBL_MS   350

ButtonEvent OLED_Button_Read(void)
{
    static uint16_t press_ticks = 0;
    static uint16_t wait_ticks  = 0;
    static uint8_t  state       = 0;
    static uint8_t  long_fired  = 0;
    uint8_t now = DL_GPIO_readPins(GPIOB, Encoder_Knob__Button_PIN) ? 1 : 0;

    switch (state) {
    case 0: /* IDLE */
        if (now == 0) { press_ticks = 1; long_fired = 0; state = 1; }
        break;
    case 1: /* PRESS */
        if (now == 0) {
            press_ticks++;
            if (press_ticks >= BTN_LONG_MS / BTN_TICK_MS && !long_fired)
                { long_fired = 1; state = 0; return BTN_LONG; }
        } else {
            if (press_ticks >= 2) { wait_ticks = 1; state = 2; }
            else state = 0;
        }
        break;
    case 2: /* WAIT_CLICK */
        if (now == 0) { press_ticks = 1; state = 3; }
        else { wait_ticks++; if (wait_ticks >= BTN_DBL_MS / BTN_TICK_MS)
               { state = 0; return BTN_CLICK; } }
        break;
    case 3: /* WAIT_DBL */
        if (now == 1) { state = 0; return BTN_DOUBLE; }
        break;
    }
    return BTN_NONE;
}

/* ── 菜单状态 ── */
typedef enum {
    MENU_PAGE = 0,
    MENU_PARAM,
    MENU_EDIT,
} MenuState;

static MenuState menu_state = MENU_PAGE;
static int8_t    menu_page  = 0;
static int8_t    menu_param = 0;
static int8_t    menu_param_cnt[4];
static float    *menu_val_ptr = NULL;
static float     menu_val_step = 0.1f;

void OLED_Menu_Tick(void)
{
    int8_t      enc = OLED_Encoder_Read();
    ButtonEvent btn = OLED_Button_Read();

    switch (menu_state) {
    case MENU_PAGE:
        if (enc) { menu_page += enc; if (menu_page < 0) menu_page = 3; menu_page %= 4; }
        if (btn == BTN_CLICK) { menu_state = MENU_PARAM; menu_param = 0; }
        break;
    case MENU_PARAM:
        if (enc) { menu_param += enc; if (menu_param < 0) menu_param = menu_param_cnt[menu_page]-1;
                   menu_param %= menu_param_cnt[menu_page]; }
        if (btn == BTN_CLICK) { menu_state = MENU_EDIT; }
        if (btn == BTN_DOUBLE) { menu_state = MENU_PAGE; }
        break;
    case MENU_EDIT:
        if (enc && menu_val_ptr) { *menu_val_ptr += (float)enc * menu_val_step; }
        if (btn == BTN_CLICK) { menu_state = MENU_PARAM; }
        if (btn == BTN_DOUBLE) { menu_state = MENU_PARAM; }
        if (btn == BTN_LONG)  { if (menu_val_ptr) *menu_val_ptr = 0.0f; }
        break;
    }
}

void OLED_Menu_Register(uint8_t page, uint8_t param_cnt)
    { if (page < 4) menu_param_cnt[page] = param_cnt; }
void OLED_Menu_SetEdit(float *val_ptr, float step)
    { menu_val_ptr = val_ptr; menu_val_step = step; }
MenuState OLED_Menu_GetState(void)  { return menu_state; }
int8_t    OLED_Menu_GetPage(void)   { return menu_page; }
int8_t    OLED_Menu_GetParam(void)  { return menu_param; }
