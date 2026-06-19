#include "headfile.h"
#define Max_Speed 1000

typedef enum{
    Left_Wheel = 0,
    Right_Wheel,
    Wheel_count
}Motor_ID_e;


# L_IN1_HIGH DL_GPIO_setPins();
# L_IN2_HIGH DL_GPIO_setPins();
# R_IN3_HIGH DL_GPIO_setPins();
# R_IN4_HIGH DL_GPIO_setPins();

# L_IN1_LOW DL_GPIO_clearPins();
# L_IN2_LOW DL_GPIO_clearPins();
# R_IN3_LOW DL_GPIO_clearPins();
# r_IN4_LOW DL_GPIO_clearPins();

#或者之前你教我的 寄存器写入01 配置好同一组gpio 然后就可以原子态同时修改in1 2(虽然6612不需要死区 不用担心同时导通) 具体写法忘记了 ccs的函数也不熟悉 然后我还想学点新的
#就是更"高级"的写法

void Motor_Forward(Motor_ID_e m){
    switch m{
        case Left_Wheel:
//      TODO
        break;
        case Right_Wheel:
//      TODO
        break;
    }
}

void Motor_Backward(Motor_ID_e m){

}

void Motor_Set_Speed(Motor_ID_e m,int32_t speed){
    if(speed > Max_Speed) speed = Max_Speed;
    if(speed < -Max_Speed) speed = -Max_Speed;
    
    if(speed >= 0){
        Motor_Forward(m);
    }else{
        Motor_Backward(m);
    }

    // DL_TimerA_setCaptureCompareValue()这里面的参数不理解 我想要的是 这里单纯设置速度 上面if确定对象 不过这个函数不确定支不支持
    // #然后这是突发奇想 更加优质 高级的做法?
}