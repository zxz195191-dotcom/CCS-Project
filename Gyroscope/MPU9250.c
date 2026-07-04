#include "headfile.h"

/*  sprintf(buf, "L:%u R:%d ISR:%lu\r\n", left_cnt, right_val, isr_cnt);
    uart_transmit(buf); */

char buffer[64];
// void Scan_I2C_Devices(void){

//     sprintf(buffer,"Start Scan...");
//     uart_transmit(buffer);
//    // delay_cycles(120000);
// // I2C 7位地址范围通常是 0x01 到 0x7F
//     for(uint8_t addr = 0x01 ; addr < 0x80 ; addr++){

// // 1. 准备一封“空传单”（试探数据）
//         uint8_t dummy_data = 0x00;

// /*DL_I2C_fillControllerTXFIFO(I2C_Regs *i2c, uint8_t *buffer, uint16_t count)
// 作用： 把数据塞进 I2C 的发送缓存区（就像把传单塞给服务员）。
// 参数： 模块名，数据的指针，数据长度。*/
//         DL_I2C_fillControllerTXFIFO(I2C_0_INST,&dummy_data,1);

// //发起一次通讯(通讯对象再syscfg的命名 通讯的数据 
// //direction: 方向，写填 DL_I2C_CONTROLLER_DIRECTION_TX，读填 DL_I2C_CONTROLLER_DIRECTION_RX。
// //扫描不需要真的传输字节 0)
//         DL_I2C_startControllerTransfer(I2C_0_INST, addr , DL_I2C_CONTROLLER_DIRECTION_TX , 1);

// /*DL_I2C_getControllerStatus(I2C_Regs *i2c)
// 作用： 获取当前 I2C 控制器的状态寄存器。
// 返回值： 一个由各种状态掩码组成的 32 位整数。
// 关键掩码（Mask）：
// DL_I2C_CONTROLLER_STATUS_BUSY_BUS: 总线忙。
// DL_I2C_CONTROLLER_STATUS_ERROR_NACK: 收到 NACK*/



// // 3. 等待总线空闲 (带超时防卡死)
//         uint32_t timeout = 100000;
//         while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
//             timeout--;
//             if (timeout == 0) {
//                 sprintf(buffer, "Bus Stuck at 0x%02X!\r\n", addr);
//                 uart_transmit(buffer);
//                 return;
//             }
//         }

//         uint32_t final_addr = DL_I2C_getControllerStatus(I2C_0_INST);
//         bool is_nack = ((final_addr & DL_I2C_CONTROLLER_STATUS_ERROR_NACK) != 0);

//         if(!is_nack){
//             sprintf(buffer, "Device 0x%02X\r\n", addr);
//             uart_transmit(buffer);        
//         }else{
//             DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//         }

//         delay_cycles(120000);
//     }
//     sprintf(buffer, "Scan Complete.\r\n");
//     uart_transmit(buffer);
// }    
void Scan_I2C_Devices(void) {
    sprintf(buffer, "Start Scan...\r\n");
    uart_transmit(buffer); 

    for(uint8_t addr = 0x00; addr < 0x80; addr++) {
        
        uint8_t dummy_data = 0x00;
        
        // 1. 塞数据并启动
        DL_I2C_fillControllerTXFIFO(I2C_0_INST, &dummy_data, 1);
        DL_I2C_startControllerTransfer(I2C_0_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

        // 2. 超时防卡死
        uint32_t timeout = 100000;
        while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
            timeout--;
            if (timeout == 0) {
                sprintf(buffer, "Bus Stuck at 0x%02X!\r\n", addr);
                uart_transmit(buffer);
                return;
            }
        }

        // 3. 查验结果
        //uint32_t final_status = DL_I2C_getControllerStatus(I2C_0_INST);
        uint32_t final_status = DL_I2C_getRawInterruptStatus(I2C_0_INST,DL_I2C_INTERRUPT_CONTROLLER_NACK);
        bool is_nack = ((final_status & DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0);

        if (!is_nack) {
            // 抓到设备！
            sprintf(buffer, "Found Device at 0x%02X\r\n", addr);
            uart_transmit(buffer);        
        } else {
            // 收到 NACK，清理战场
            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
            
            // 【核心修复】：一键重置内部状态机，清除 ERROR 标志，防止下次传输死锁！
            DL_I2C_resetControllerTransfer(I2C_0_INST);

            DL_I2C_resetControllerTransfer(I2C_0_INST);

            DL_I2C_clearInterruptStatus(I2C_0_INST,DL_I2C_INTERRUPT_CONTROLLER_NACK);
        }

        // 【核心修复】：给物理总线产生 STOP 电平留出充足的时间 (约10ms)
        delay_cycles(320000); 
    }
    
    sprintf(buffer, "Scan Complete.\r\n");
    uart_transmit(buffer); 
}