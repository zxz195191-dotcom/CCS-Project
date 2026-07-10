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

    DL_I2C_startControllerTransferAdvanced(
        I2C_0_INST,
        dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,   // 不发 STOP，霸占总线
        DL_I2C_CONTROLLER_ACK_ENABLE
    );

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
        DL_I2C_CONTROLLER_START_ENABLE,   // 硬件自动将其变为 Sr (重复起始)
        DL_I2C_CONTROLLER_STOP_ENABLE,    // 读完发 STOP 释放总线
        DL_I2C_CONTROLLER_ACK_DISABLE     // 读最后一个字节给 NACK
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
            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
            DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);
            return 1;  // 收到NACK
        }
        if (--timeout == 0) return 2; //什么都没收到 
    }//收到ACK

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
    // 唤醒 MPU6500
    MPU9250_Write_Reg(0x68, 0x6B, 0x00);
    delay_cycles(320000);

    // 关掉内部 I2C 主机，防止抢控制权
    MPU9250_Write_Reg(0x68, 0x6A, 0x00);
    delay_cycles(320000);

    // 开启 Bypass 旁路模式，让 0x0C 暴露出来！
    MPU9250_Write_Reg(0x68, 0x37, 0x02);
    delay_cycles(320000);

    // 重置 AK8963 (必须先掉电，才能配新模式)
    MPU9250_Write_Reg(0x0C, 0x0A, 0x00);
    delay_cycles(320000); 

    // 开启 AK8963 16位高精度, 100Hz 连测
    MPU9250_Write_Reg(0x0C, 0x0A, 0x16);
    delay_cycles(320000); 
}
// void MPU9250_Init(void){
//     // ==========================================
//     // 0. 强行软复位 (DEVICE_RESET = 1)
//     // 强制芯片恢复出厂状态，消除上一次调试残留的 Bypass 状态
//     // ==========================================
//     MPU9250_Write_Reg(0x68, 0x6B, 0x80);
//     delay_cycles(3200000); // 必须给它留足重启时间 (大概 100ms)

//     // ==========================================
//     // 1. 唤醒并选择最佳时钟源 (Auto PLL)
//     // 0x01 比 0x00 更好，0x00 是内部20MHz晶振，0x01是自动选择最优陀螺仪时钟
//     // ==========================================
//     MPU9250_Write_Reg(0x68, 0x6B, 0x01);
//     delay_cycles(320000);

//     // 2. 关掉内部 I2C 主机，防止抢控制权
//     MPU9250_Write_Reg(0x68, 0x6A, 0x00);
//     delay_cycles(320000);

//     // 3. 开启 Bypass 旁路模式，让 0x0C 暴露出来！
//     MPU9250_Write_Reg(0x68, 0x37, 0x02);
//     delay_cycles(320000);

//     // 4. 重置 AK8963 (必须先掉电，才能配新模式)
//     MPU9250_Write_Reg(0x0C, 0x0A, 0x00);
//     delay_cycles(320000); 

//     // 5. 开启 AK8963 16位高精度, 100Hz 连测
//     MPU9250_Write_Reg(0x0C, 0x0A, 0x16);
//     delay_cycles(320000); 
// }

/*整个函数就是为了完成这一串动作。
START -> 0x68 + Write -> ACK -> 0x75(寄存器地址) -> ACK -> Repeated START -> 0x68 + Read

 -> ACK -> Data1 -> Data2 -> ACK -> Data3 -> ACK -> Data4 -> ACK -> Data5 -> ACK -> Data6

 -> NACK -> STOP
*/

uint8_t MPU9250_Read_Len(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *buf) {
    // 阶段 1：发首地址，不发 STOP
    DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg_addr, 1);
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    
    uint32_t timeout = 100000;
    while (!(DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))) {
        if (DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_NACK) || --timeout == 0) {
            DL_I2C_resetControllerTransfer(I2C_0_INST);
            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
            DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);
            return 1; // 发送地址失败
        }
    }
    
    // 阶段 2：带有 Repeated Start 的连读
    DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, len,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
        
    uint8_t got_bytes = 0;
    timeout = 500000;
    while (got_bytes < len) {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
            buf[got_bytes] = DL_I2C_receiveControllerData(I2C_0_INST);
            got_bytes++;
        }
        
        if (DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_NACK) || --timeout == 0) {
            DL_I2C_resetControllerTransfer(I2C_0_INST);
            DL_I2C_flushControllerRXFIFO(I2C_0_INST);
            DL_I2C_clearInterruptStatus(I2C_0_INST, 0xFFFFFFFF);
            return 2; // 接收数据失败
        }
    }
    delay_cycles(32000); 
    return 0; // 成功！
}



void MPU9250_Read_All_Axis_Plus(MPU9250_Data_t *dat){
    //Acc6 + temp2 + Gyr6 = 14
    uint8_t buffer[14];

    //consist reading
    if(MPU9250_Read_Len(MPU_ADDR, MPU_REG, LEN, buffer) != 0) {
        return;//如果读取失败 还能保留上一次的数据（防止车子抽搐）
    }

    dat->accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    dat->accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    dat->accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

    dat->gyro_raw[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    dat->gyro_raw[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    dat->gyro_raw[2] = (int16_t)((buffer[12] << 8) | buffer[13]);

    for (uint8_t i = 0; i < 3; i++) {//0 x   1 y   2 z
        dat->accel_g[i]   = (float)dat->accel_raw[i] / 16384.0f;
        dat->gyro_dps[i]  = (float)dat->gyro_raw[i] / 131.0f;
    }


    // ==========================================
    // 3. 读取磁力计 (AK8963)
    // ==========================================
    // ---------- 读 AK8963 (三轴磁力) ----------
    uint8_t mag_buffer[8]; 
    // 必须从 ST1 (0x02) 开始读 8 个字节
    if (MPU9250_Read_Len(0x0C, 0x02, 8, mag_buffer) == 0) {
        // 检查 ST1 状态寄存器的最低位 (DRDY: Data Ready)
        if (mag_buffer[0] & 0x01) { 
            // 小端模式拼接：低字节在前，高字节在后
            dat->mag_raw[x] = (int16_t)((mag_buffer[2] << 8) | mag_buffer[1]);
            dat->mag_raw[y] = (int16_t)((mag_buffer[4] << 8) | mag_buffer[3]);
            dat->mag_raw[z] = (int16_t)((mag_buffer[6] << 8) | mag_buffer[5]);

            dat->mag_uT[x] = (float)dat->mag_raw[x] * 0.15f;
            dat->mag_uT[y] = (float)dat->mag_raw[y] * 0.15f;
            dat->mag_uT[z] = (float)dat->mag_raw[z] * 0.15f;
        }
    }
}



void MPU9250_Read_All_Axis_Plus_Pro(MPU9250_Data_t *dat){
    uint8_t buf[14];
    uint8_t mag_buf[8];

    if(MPU9250_Read_Len(MPU_ADDR, MPU_REG, LEN, buffer) != 0) {
        return;//如果读取失败 还能保留上一次的数据（防止车子抽搐）
    }

    dat->accel_g[x] = (int16_t)((buf[0] << 8) | buf[1]) * ACC_SCALE;
    dat->accel_g[y] = (int16_t)((buf[2] << 8) | buf[3]) * ACC_SCALE;
    dat->accel_g[z] = (int16_t)((buf[4] << 8) | buf[5]) * ACC_SCALE;

    dat->gyro_dps[x] = (int16_t)((buf[8] << 8) | buf[9]) * GYR_SCALE_RAD;
    dat->gyro_dps[y] = (int16_t)((buf[10] << 8) | buf[11]) * GYR_SCALE_RAD;
    dat->gyro_dps[z] = (int16_t)((buf[12] << 8) | buf[13]) * GYR_SCALE_RAD;

// 2. 读磁力计 (连读8字节)
    if (MPU9250_Read_Len(0x0C, 0x02, 8, mag_buf) == 0) {
        if (mag_buf[0] & 0x01) { 
            // 磁力计是小端模式
            dat->mag_uT[x] = (int16_t)((mag_buf[2] << 8) | mag_buf[1]) * MAG_SCALE;
            dat->mag_uT[y] = (int16_t)((mag_buf[4] << 8) | mag_buf[3]) * MAG_SCALE;
            dat->mag_uT[z] = (int16_t)((mag_buf[6] << 8) | mag_buf[5]) * MAG_SCALE;
        }
    }
    

}



float mag_offset[3] = {0,0,0};//硬磁导致侧错位
float mag_scale[3] = {0,0,0};//软磁铁导致的变形

void Maf_calibration(void){
    float max_mag[3] = {-9999.0f,-9999.0f,-9999.0f};
    float min_mag[3] = {9999.0f,9999.0f,9999.0f};
    char tx_buf[64];

    sprintf(tx_buf, "\r\n[CAL] Start Mag Calibration!\r\n");
    uart_transmit(tx_buf);
    sprintf(tx_buf, "[CAL] Please spin the board in 8-shape...\r\n");
    uart_transmit(tx_buf);

    for(int i = 0; i < 1000;i++){

        MPU9250_Read_All_Axis_Plus(&mpu_data);

        for(int j = 0; j < 3;j++){
            if(mpu_data.mag_uT[j] > max_mag[j]) max_mag[j] = mpu_data.mag_uT[j];
            if(mpu_data.mag_uT[j] < min_mag[j]) min_mag[j] = mpu_data.mag_uT[j];
        }        

        if(i % 100 == 0){
           
            sprintf(tx_buf, "%d%%/r/n",i/10);
            uart_transmit(tx_buf);
        }
        delay_cycles(320000);
    }

    for (int j = 0; j < 3; j++) {
        //DL_UART_Main_transmitDataBlocking(UART1,'C');

        mag_offset[j] = (max_mag[j] + min_mag[j]) / 2.0f;
    }

    sprintf(tx_buf, "\r\n[CAL] Done!\r\n");
    uart_transmit(tx_buf);
    sprintf(tx_buf, "Offset X: %.2f  Y: %.2f  Z: %.2f\r\n", mag_offset[0], mag_offset[1], mag_offset[2]);
    uart_transmit(tx_buf);

}

/*const float MAG_OFFSET_X = 76.09f;
const float MAG_OFFSET_Y = 27.86f;
const float MAG_OFFSET_Z = 88.34f;*/