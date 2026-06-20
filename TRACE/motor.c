#include "headfile.h"
#define Max_Speed 1000


const Motor_Config_t Motor_Cfg[Wheel_count] = {
    [Left_Wheel] = {
        .dir_port = Motor_Direction_PORT,
        .in1_pin = Motor_Direction_IN1_PIN,//PB0
        .in2_pin = Motor_Direction_IN2_PIN,//PB1
        .pmw_tiemr = WHEELS_INST,//TimerA
        .cc_index = DL_TIMER_CC_0_INDEX,//PA17
    },

    [Right_Wheel] = {
        .dir_port = Motor_Direction_PORT,
        .in1_pin = Motor_Direction_IN3_PIN,//PB8
        .in2_pin = Motor_Direction_IN4_PIN,//PB9
        .pmw_tiemr = WHEELS_INST,//TimerA
        .cc_index = DL_TIMER_CC_1_INDEX,//PA18
    }
};

void Motor_Set_Speed(Motor_ID_e motor_id,int32_t speed){
    uint32_t abs_speed = 0;
    //获取对象的信息
    const Motor_Config_t* m = &Motor_Cfg[motor_id];//有点惊艳到 当然更多的是懵逼 之前接触过这个 蓝桥杯扫描按键 就是结构体数组赋值
    //实现类似c++的对象编程 但是这句话没有见到过 以及当时也没用上const 还有初始化对象也没有使用上指针 包括这里
    //之前写掐头去尾的滤波(自己设置长度 冒泡排序 最大最小去掉 最后求平均值)为了方便调用 形参也是指针 但当时也不是很理解
    //指针是存放变量地址的变量 形参可以直接操作结构体实例化对象的数据 这么理解吧 但是总觉得还差了点什么
    if(speed > Max_Speed) speed = Max_Speed;
    if(speed < -Max_Speed) speed = -Max_Speed;
    
    if(speed >= 0){
        DL_GPIO_setPins(m->dir_port, m->in1_pin);    
        DL_GPIO_clearPins(m->dir_port, m->in2_pin);    
        abs_speed = speed;
    }else{
        DL_GPIO_setPins(m->dir_port, m->in2_pin);    
        DL_GPIO_clearPins(m->dir_port, m->in1_pin);    
        abs_speed = -speed;        
    }
    DL_TimerA_setCaptureCompareValue(m->pmw_tiemr, (Max_Speed - abs_speed),m->cc_index);
    debug_once('Y');
}