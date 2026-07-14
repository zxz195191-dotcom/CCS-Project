#include "headfile.h"

float mag_offset[3] = {0,0,0};//硬磁导致侧错位
float mag_scale[3] = {0,0,0};//软磁铁导致的变形

// void Mag_calibration(void){
//     float max_mag[3] = {-9999.0f,-9999.0f,-9999.0f};
//     float min_mag[3] = {9999.0f,9999.0f,9999.0f};
//     char tx_buf[64];

//     sprintf(tx_buf, "\r\n[CAL] Start Mag Calibration!\r\n");
//     uart_transmit(tx_buf);
//     sprintf(tx_buf, "[CAL] Please spin the board in 8-shape...\r\n");
//     uart_transmit(tx_buf);

//     for(int i = 0; i < 1000;i++){

//         MPU9250_Read_All_Axis_Plus(&mpu_data);

//         for(int j = 0; j < 3;j++){
//             if(mpu_data.mag_uT[j] > max_mag[j]) max_mag[j] = mpu_data.mag_uT[j];
//             if(mpu_data.mag_uT[j] < min_mag[j]) min_mag[j] = mpu_data.mag_uT[j];
//         }        

//         if(i % 100 == 0){
           
//             sprintf(tx_buf, "%d%%/r/n",i/10);
//             uart_transmit(tx_buf);
//         }
//         delay_cycles(320000);
//     }

//     for (int j = 0; j < 3; j++) {
//         //DL_UART_Main_transmitDataBlocking(UART1,'C');

//         mag_offset[j] = (max_mag[j] + min_mag[j]) / 2.0f;
//     }

//     sprintf(tx_buf, "\r\n[CAL] Done!\r\n");
//     uart_transmit(tx_buf);
//     sprintf(tx_buf, "Offset X: %.2f  Y: %.2f  Z: %.2f\r\n", mag_offset[0], mag_offset[1], mag_offset[2]);
//     uart_transmit(tx_buf);

// }
void Mag_Calibration(void) {
    float max_mag[3] = {-9999.0f, -9999.0f, -9999.0f};
    float min_mag[3] = {9999.0f,  9999.0f,  9999.0f};
    char tx_buf[256];

    sprintf(tx_buf, "\r\n[CAL] Start Mag Calibration!\r\n");
    uart_transmit(tx_buf);
    sprintf(tx_buf, "[CAL] Please spin the board in 8-shape...\r\n");
    uart_transmit(tx_buf);

    for(int i = 0; i < 1000; i++) {
        // 读取最新数据
        MPU9250_Read_All_Axis_Plus_Pro(&mpu_data);

        // 寻找最大最小值
        for(int j = 0; j < 3; j++) {
            if(mpu_data.mag_uT[j] > max_mag[j]) max_mag[j] = mpu_data.mag_uT[j];
            if(mpu_data.mag_uT[j] < min_mag[j]) min_mag[j] = mpu_data.mag_uT[j];
        }        

        if(i % 100 == 0) {
            sprintf(tx_buf, "Calibrating... %d%%\r\n", i/10);
            uart_transmit(tx_buf);
        }
        // 延时约 10ms，总耗时 10 秒
        delay_cycles(320000); 
    }

    // 计算圆心坐标
    for (int j = 0; j < 3; j++) {
        mag_offset[j] = (max_mag[j] + min_mag[j]) / 2.0f;
    }

    sprintf(tx_buf, "\r\n[CAL] Done!\r\n");
    uart_transmit(tx_buf);
    sprintf(tx_buf, "Copy to Header:\r\n#define MAG_OFFSET_X %.2ff\r\n#define MAG_OFFSET_Y %.2ff\r\n#define MAG_OFFSET_Z %.2ff\r\n", 
            mag_offset[0], mag_offset[1], mag_offset[2]);
    uart_transmit(tx_buf);
}

/*const float MAG_OFFSET_X = 76.09f;
const float MAG_OFFSET_Y = 27.86f;
const float MAG_OFFSET_Z = 88.34f;*/




/* ========== 1. 算法参数 ========== */
volatile float g_Kp = 2.0f;      // 比例增益：控制收敛速度
volatile float g_Ki = 0.0f;      // 积分增益：6轴模式下必须为0！

/* ========== 2. 内部状态与输出 ========== */
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // 四元数
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;      // 积分误差累积

float pitch = 0.0f;   // 俯仰角
float roll  = 0.0f;   // 横滚角
float yaw   = 0.0f;   // 航向角

/* ========== 3. 快速反平方根 (已修正) ========== */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    // 原写法 (long)&y 是错误的，应使用指针转换
    long i = *(long*)&y;               // 把 float 的二进制位当作 long 来处理
    i = 0x5f3759df - (i >> 1);         // 神奇的初始猜测
    y = *(float*)&i;                   // 再把 long 转回 float
    y = y * (1.5f - (halfx * y * y));  // 一次牛顿迭代
    return y;
}

/* ========== 4. Mahony 核心融合算法 ========== */
void MahonyAHRSupdate(float gx, float gy, float gz,   // 陀螺仪 rad/s
                      float ax, float ay, float az,   // 加速度计 g
                      float mx, float my, float mz,   // 磁力计 已校准
                      float dt)                       // 采样周期 秒
{
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // 预计算四元数乘积
    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 1. 归一化加速度计
    if ((ax*ax + ay*ay + az*az) == 0.0f) return;
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    // 2. 归一化磁力计
    if ((mx*mx + my*my + mz*mz) == 0.0f) return;
    norm = invSqrt(mx*mx + my*my + mz*mz);
    mx *= norm; my *= norm; mz *= norm;

    // 3. 参考磁场方向（提取水平分量）
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    bx = sqrtf(hx*hx + hy*hy);
    bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

    // 4. 用当前姿态估算重力和磁场方向
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

    // 5. 叉积计算误差
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    // 6. PI 补偿（消除陀螺仪漂移）
    if (g_Ki > 0.0f) {
        exInt += ex * g_Ki * dt;
        eyInt += ey * g_Ki * dt;
        ezInt += ez * g_Ki * dt;
    }
    gx += g_Kp * ex + exInt;
    gy += g_Kp * ey + eyInt;
    gz += g_Kp * ez + ezInt;

    // 7. 用校正后的角速度更新四元数
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += (qa*gx + qc*gz - q3*gy);
    q2 += (qa*gy - qb*gz + q3*gx);
    q3 += (qa*gz + qb*gy - qc*gx);

    // 8. 归一化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}

/* ========== 4b. Mahony 6 轴融合 (无磁力计，yaw 相对角度) ========== */
void MahonyAHRSupdateIMU(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    /* 归一化加速度计 */
    if ((ax*ax + ay*ay + az*az) == 0.0f) return;
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    /* 用当前姿态估算重力方向 */
    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* 叉积求误差（只有加速度计） */
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    /* PI 补偿 */
    if (g_Ki > 0.0f) {
        exInt += ex * g_Ki * dt;
        eyInt += ey * g_Ki * dt;
        ezInt += ez * g_Ki * dt;
    }
    gx += g_Kp * ex + exInt;
    gy += g_Kp * ey + eyInt;
    gz += g_Kp * ez + ezInt;

    /* 四元数更新 */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += (qa*gx + qc*gz - q3*gy);
    q2 += (qa*gy - qb*gz + q3*gx);
    q3 += (qa*gz + qb*gy - qc*gx);

    /* 归一化 */
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}


/* ========== 5. 四元数 -> 欧拉角 (含 Yaw 解缠) ========== */
void ComputeEulerAngles(void) {
    static float prev_raw = 0.0f;     /* 上一帧 atan2f 原始值 */
    static float yaw_full = 0.0f;     /* 解缠后的全量程 yaw */
    float raw_yaw, delta;

    /* 四元数数值保护：norm 严重偏离 1.0 → 数据已崩，重置 */
    float qnorm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (qnorm < 0.5f || qnorm > 2.0f) {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
        exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;
        return;
    }

    pitch = asinf(2.0f * (q0 * q2 - q1 * q3)) * 57.29578f;
    roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 57.29578f;

    /* Yaw 解缠：atan2f 只能输出 [-180,180]，跨边界时自动补 360° */
    raw_yaw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                     q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.29578f;

    delta = raw_yaw - prev_raw;
    if      (delta >  180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;

    yaw_full += delta;
    prev_raw  = raw_yaw;
    yaw       = yaw_full;
}


void IMU_Update_Attitude(MPU9250_Data_t *dat, float dt) {
    // a. 提取加速度数据 (直接使用)
    float ax = dat->accel_g[x];
    float ay = dat->accel_g[y];
    float az = dat->accel_g[z];

    // b. 提取陀螺仪数据，并在此处转为算法必须的弧度制 (rad/s)
    float gx_rad = dat->gyro_dps[x] * DEG_TO_RAD;
    float gy_rad = dat->gyro_dps[y] * DEG_TO_RAD;
    float gz_rad = dat->gyro_dps[z] * DEG_TO_RAD;

    // c. 提取磁力计数据，并在此处扣除硬磁干扰偏移量 (Offset)
    //依据 MPU9250 数据手册的物理布局：Mag_X 对齐 Gyro_Y, Mag_Y 对齐 Gyro_X, Mag_Z 对齐 -Gyro_Z
    float my = dat->mag_uT[x] - MAG_OFFSET_X;
    float mx = dat->mag_uT[y] - MAG_OFFSET_Y;
    float mz = -(dat->mag_uT[z] - MAG_OFFSET_Z);

    // d. 9 轴融合 — 原始版本，保留
    MahonyAHRSupdate(gx_rad, gy_rad, gz_rad,
                     ax, ay, az,
                     mx, my, mz,
                     dt);
}

/* ========== Gyro 零偏 (全局，可跳过校准用默认值) ========== */
float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };  /* dps, 开机校准后更新 */

void Gyro_Calibrate_Bias(uint16_t samples) {
    float sx = 0, sy = 0, sz = 0;
    for (uint16_t i = 0; i < samples; i++) {
        MPU9250_Read_All_Axis_Plus_Pro(&mpu_data);
        sx += mpu_data.gyro_dps[x];
        sy += mpu_data.gyro_dps[y];
        sz += mpu_data.gyro_dps[z];
        delay_cycles(32000);  /* ~1ms */
    }
    gyro_bias[x] = sx / (float)samples;
    gyro_bias[y] = sy / (float)samples;
    gyro_bias[z] = sz / (float)samples;
}

/* 6 轴融合 wrapper：减去零偏 */
void IMU_Update_Attitude_6Axis(MPU9250_Data_t *dat, float dt) {
    float ax = dat->accel_g[x];
    float ay = dat->accel_g[y];
    float az = dat->accel_g[z];

    float gx_rad = (dat->gyro_dps[x] - gyro_bias[x]) * DEG_TO_RAD;
    float gy_rad = (dat->gyro_dps[y] - gyro_bias[y]) * DEG_TO_RAD;
    float gz_rad = (dat->gyro_dps[z] - gyro_bias[z]) * DEG_TO_RAD;

    MahonyAHRSupdateIMU(gx_rad, gy_rad, gz_rad, ax, ay, az, dt);
}