#include "headfile.h"

/*  sprintf(buf, "L:%u R:%d ISR:%lu\r\n", left_cnt, right_val, isr_cnt);
    uart_transmit(buf); */

char buffer[64];
void Scan_I2C_Devices(void){
    
    sprintf(buffer,"Start Scan...");
    uart_transmit(buffer);
    delay_cycles(120000);
// I2C 7位地址范围通常是 0x01 到 0x7F
    for(uint8_t addr = 0x01 ; addr < 0x80 ; addr++){

        //发起一次通讯(通讯对象再syscfg的命名 通讯的数据 
        //direction: 方向，写填 DL_I2C_CONTROLLER_DIRECTION_TX，读填 DL_I2C_CONTROLLER_DIRECTION_RX。
        //扫描不需要真的传输字节 0)
        DL_I2C_startControllerTransfer(I2C_0_INST, addr , DL_I2C_CONTROLLER_DIRECTION_TX , 0);

        /*DL_I2C_getControllerStatus(I2C_Regs *i2c)
        作用： 获取当前 I2C 控制器的状态寄存器。
        返回值： 一个由各种状态掩码组成的 32 位整数。
        关键掩码（Mask）：
        DL_I2C_CONTROLLER_STATUS_BUSY_BUS: 总线忙。
        DL_I2C_CONTROLLER_STATUS_ERROR_NACK: 收到 NACK*/
 
        while(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS){}

        uint32_t final_addr = DL_I2C_getControllerStatus(I2C_0_INST);

        bool is_nack = ((final_addr & DL_I2C_CONTROLLER_STATUS_ERROR) != 0);

        if(!is_nack){
            sprintf(buffer, "Device 0x%02X\r\n", addr);
            uart_transmit(buffer);        
        }

        delay_cycles(120000);
    }
}    