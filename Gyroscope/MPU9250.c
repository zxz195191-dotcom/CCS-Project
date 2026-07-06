#include "headfile.h"


char buffer[64];


/*
▎MSPM0 I2C 控制器 = 状态机 + FIFO + 中断标志，三个独立模块。 
▎没有 "resetAll" 函数，必须 resetControllerTransfer + flush + clearInterruptStatus 
▎三个一起上，才能确保下一次传输从零开始。 
*/
/*
扫描函数流程

START（分支 无ACK ->无设备）

地址（从机地址 & ACK?）                 0x68 + W（9250地址）

WRITE

ACK ?

STOP
*/
void Scan_I2C_Devices(void) {
    sprintf(buffer, "Start Clean Scan...\r\n");
    uart_transmit(buffer); 

    // I2C 7位地址都是从 0x01 开始的
    for(uint8_t addr = 0x01; addr < 0x80; addr++) {
        
        uint8_t dummy_data = 0x00;
        
        DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);
        // 防止总线因为各种原因导致的中断挂起没清理干净 而死机（比如说上次传输如果NACK）
        //效果： 直接在总线上把sda从低拉高（在时钟线高电平时）->直接把通讯逻辑复位了
        //否则：卡在出错状态 状态机不是IDLE 所有请求会被忽视
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        /*防止上次传输数据还有残留 扫描设备需要发送地址和0x00 
        To write the internal MPU-9250 registers, the master transmits the start condition (S), followed by the I2C address and the write bit (0).*/
        //如果有残留数据 就会通信失败


        DL_I2C_fillControllerTXFIFO(I2C_0_INST, &dummy_data, 1);
        // 塞入探路数据并启动
        DL_I2C_startControllerTransfer(I2C_0_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
        //实际实现的 START → 地址 → R/W → ACK 检测 → 数据发送 → STOP（或等待后续操作）

        // 要么成功(TX_DONE)，要么失败(NACK)
        uint32_t timeout = 100000;
        bool is_nack = true; // 默认没回应
        
        while (--timeout > 0) {
            // 检查情况 1：收到 NACK（没人理）
            if (DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
                is_nack = true;
                break;
            }
            // 检查情况 2：收到 TX_DONE（对方不仅回了 ACK，单片机还把数据发完了）
            if (DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
                is_nack = false; // 抓到了！
                break;
            }
        }

        // 查验最终结论
        if (!is_nack) {
            // 只有真正的物理 ACK 才会走到这里
            sprintf(buffer, ">>> REAL Device Found at 0x%02X <<<\r\n", addr);
            uart_transmit(buffer);        
        }

        // 不管成功还是失败，只要这轮结束了，立刻拍碎状态机，清空所有灯
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);

        // 给物理总线留 2~3 毫秒彻底释放电平
        delay_cycles(96000); 
    }
    
    sprintf(buffer, "Scan Complete.\r\n");
    uart_transmit(buffer); 
}




/*
寄存器地址本身需要通过“写”操作发送

START

地址 + Write

ACK

寄存器地址

ACK
但现在总线的数据方向还是“主机 → 从机”（写模式），而真正的数据应该由 MPU 发给主机。

所以不能直接继续。（如果scl=HIGH的时候 SDA=LOW->HIGH（挂电话））
Repeated START（保持通讯）

地址 + Read

ACK
*/
uint8_t MPU9250_Read_Reg(uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t data = 0;

    // 0. 清理战场，防止上次的旧便利贴干扰
    DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);

    // ==========================================
    // 1. TX 阶段：发寄存器地址，强行扣留 STOP
    // ==========================================
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg_addr, 1);
    

/*
@brief Sets up a transfer from I2C controller with control of START,
STOP and ACK
*/  DL_I2C_startControllerTransferAdvanced(
        I2C_0_INST, 
        dev_addr, 
        DL_I2C_CONTROLLER_DIRECTION_TX, 
        1,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,   // <--- 不发 STOP，霸占总线！
        DL_I2C_CONTROLLER_ACK_ENABLE         // <--- 期待从机给出 ACK
    );

    // 【核心修复】：绝对不能等 BUSY_BUS！只能等 TX_DONE 中断标志！
    uint32_t timeout = 100000;
    while (!(DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))) {
        if (DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
            DL_I2C_resetControllerTransfer(I2C_0_INST);
            return 0xFF; // TX 阶段被拒收
        }
        if (--timeout == 0) return 0xAA; // TX 卡死
    }

    DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);

    // ==========================================
    // 2. RX 阶段：带着 Repeated Start 开始读数据
    // ==========================================
    DL_I2C_startControllerTransferAdvanced(
        I2C_0_INST, 
        dev_addr, 
        DL_I2C_CONTROLLER_DIRECTION_RX, 
        1,
        DL_I2C_CONTROLLER_START_ENABLE,   // <--- 因为没发 STOP，硬件自动将其变为 Sr (重复起始)
        DL_I2C_CONTROLLER_STOP_ENABLE,    // <--- 这次读完，可以发 STOP 释放总线了
        DL_I2C_CONTROLLER_ACK_DISABLE        // <--- I2C规范：主控读最后一个字节必须给 NACK
    );

    // 等待 RX 接收完成标志
    timeout = 100000;
    while (!(DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE))) {
        if (DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
            DL_I2C_resetControllerTransfer(I2C_0_INST);
            return 0xDD; // RX 阶段被拒收
        }
        if (--timeout == 0) return 0xBB; // RX 卡死
    }

    // ==========================================
    // 3. 收货：查 FIFO
    // ==========================================
    if (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
        data = DL_I2C_receiveControllerData(I2C_0_INST);
    } else {
        return 0xEE; // 最极端的硬件 Bug
    }

    delay_cycles(32000); 
    return data;
}



uint8_t MPU9250_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buf[2] = {reg_addr, data};
    
    DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);

    // 把寄存器地址和数据一起塞进 FIFO，长度为 2
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, tx_buf, 2);
    
    // 启动普通写传输（写完会自动发 STOP 释放总线）
    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    // 等待发送完成 (TX_DONE)
    uint32_t timeout = 100000;
    while (!(DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))) {
        if (DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_NACK)) {
            DL_I2C_resetControllerTransfer(I2C_0_INST);
            return 1; // 写入被拒收，失败
        }
        if (--timeout == 0) return 2; // 卡死
    }

    return 0; // 成功！
}

MPU9250_Data_t mpu_data = {0};

void MPU9250_Read_All_Axis(MPU9250_Data_t *dat){    

    uint8_t accel_x_h = MPU9250_Read_Reg(0x68, 0x3B);
    uint8_t accel_x_l = MPU9250_Read_Reg(0x68, 0x3C);
    uint8_t accel_y_h = MPU9250_Read_Reg(0x68, 0x3D);
    uint8_t accel_y_l = MPU9250_Read_Reg(0x68, 0x3E);
    uint8_t accel_z_h = MPU9250_Read_Reg(0x68, 0x3F);
    uint8_t accel_z_l = MPU9250_Read_Reg(0x68, 0x40);

    dat->accel_raw[x] = (int16_t)((accel_x_h << 8) | accel_x_l);
    dat->accel_raw[y] = (int16_t)((accel_y_h << 8) | accel_y_l);
    dat->accel_raw[z] = (int16_t)((accel_z_h << 8) | accel_z_l);

    for(uint8_t i = 0; i < num ; i++){
        dat->accel_g[i] = (float)(dat->accel_raw[i] / 16384.0f);
    }

    uint8_t gyro_x_h = MPU9250_Read_Reg(0x68, 0x43);
    uint8_t gyro_x_l = MPU9250_Read_Reg(0x68, 0x44);
    uint8_t gyro_y_h = MPU9250_Read_Reg(0x68, 0x45);
    uint8_t gyro_y_l = MPU9250_Read_Reg(0x68, 0x46);
    uint8_t gyro_z_h = MPU9250_Read_Reg(0x68, 0x47);
    uint8_t gyro_z_l = MPU9250_Read_Reg(0x68, 0x48);

    dat->gyro_raw[x] = (int16_t)((gyro_x_h << 8) | gyro_x_l);
    dat->gyro_raw[y] = (int16_t)((gyro_y_h << 8) | gyro_y_l);
    dat->gyro_raw[z] = (int16_t)((gyro_z_h << 8) | gyro_z_l);

    for(uint8_t i = 0; i < num ; i++){
        dat->gyro_dps[i] = (float)(dat->gyro_raw[i] / 131.0f);
    }
}

void MPU9250_Init(void){
    MPU9250_Write_Reg(0x68, 0x6B, 0x00);
    delay_cycles(320000);
}


/*整个函数就是为了完成这一串动作。
START -> 0x68 + Write -> ACK -> 0x75(寄存器地址) -> ACK -> Repeated START -> 0x68 + Read

 -> ACK -> Data1 -> Data2 -> ACK -> Data3 -> ACK -> Data4 -> ACK -> Data5 -> ACK -> Data6

 -> NACK -> STOP
*/

void MPU9250_Read_Len(  ){
// 1. 发送首寄存器地址 (强行扣留 STOP)
    DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);//->处理9250内部的寄存器
    //XXXX错误//9250里面会有很多用于判断状态的标志位 R/TX_DONE；NACK；ARBITRATION LOST；STOP ；START
    //--->清除 MSPM0 I2C 控制器内部的中断状态寄存器。这些标志属于 MSPM0，不属于 MPU9250。

    //如果第一次发送完成 第二次想要再次接收9250发送的数据 就必须要把发送完成标志位清理干净（上一轮结束之后tx_done = 1）
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg_addr, 1);
    
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_ENABLE);
    //普通的发送函数会在结尾附带stop （但是这才是 0x68 + Read -> ACK -> Data1 -> Data2 -> ACK需要的流程）
   //或者说 普通的发送函数就是默认内部 STOP = ENABLE
   /*普通发送接口内部默认：
    START_ENABLE
    STOP_ENABLE
    ACK_ENABLE
    因此一次传输结束后会自动发送 STOP。
    而读取寄存器需要 Repeated START，因此这里必须关闭 STOP。*/
  //这里就可以理解成Repeated START的实现

    uint32_t timeout = 100000;
    while (!(DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))) {
        if (--timeout == 0) return 1; // 卡死或 NACK
    }
    //必须要确定是否tx_done "cpu -> xMHz" "I2C -> 400KHz"cpu跑了几百轮 i2c可能连地址都还没发完
    
    // 2. 发起连续读取任务，长度设置为请求的 len
    DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);
    //上面已经使用了tx_done 下面需要判断cpu是否rx_done 但是两个公用一个标志位 必须先清除
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, len,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_NACK);
    //设置总长度len 然后 DL_I2C_CONTROLLER_ACK_NACK的意思就是 在到达len-1之前 一直ack 
    //直到len-1 发送nack 此时数据传输结束


    // 3. 边收边拿：守在流水线旁边，来一个拿一个，防止 FIFO 溢出卡死
    uint8_t got_bytes = 0;
    timeout = 500000;
    
    while (got_bytes < len) {
       
        if ( !DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST) ) {
           
            buf[got_bytes] = DL_I2C_receiveControllerData(I2C_0_INST);
            got_bytes++;
        }
        
        if (--timeout == 0) return 2; // 严重超时
    }
    
    delay_cycles(32000); 
    return 0; // 完美收工！
}