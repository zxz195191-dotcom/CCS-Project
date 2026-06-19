#include "trace.h"

Trace_OUT_t sensors[TRACE_SENSOR_COUNT];


/*#define TRACE_SENSOR_COUNT 8
#define TRACE_WAIT_CONVERSION_US 10//分频了24 实际是1.33mhz

typedef struct{
    uint16_t white_value;
    uint16_t black_value;
}Trace_OUT_t;
*/



void trace_init() {
    // 1. 先停用 ADC（拔插头，方便我们改接线）
    DL_ADC12_disableConversions(OUT_INST);

    // 2. 强制单次、纯软件触发、无重复模式（全套官方标准答案）
    DL_ADC12_initSingleSample(OUT_INST, 
        DL_ADC12_REPEAT_MODE_DISABLED, 
        DL_ADC12_SAMPLING_SOURCE_AUTO, 
        DL_ADC12_TRIG_SRC_SOFTWARE, 
        DL_ADC12_SAMP_CONV_RES_12_BIT, 
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

    sensors[CH1].weight = -40;
    sensors[CH2].weight = -30;
    sensors[CH3].weight = -20;
    sensors[CH4].weight = -10;
    sensors[CH5].weight =  10;
    sensors[CH6].weight =  20;
    sensors[CH7].weight =  30;
    sensors[CH8].weight =  40;

    // 3. 重新插上插头（使能 ADC），准备接客
    DL_ADC12_enableConversions(OUT_INST);
}


void debug_once(char character){//->判断程序卡死在哪里了
     DL_UART_Main_transmitDataBlocking(UART1,character);
     DL_UART_Main_transmitDataBlocking(UART1,'\r');
     DL_UART_Main_transmitDataBlocking(UART1,'\n');
}


void CHx(uint8_t channel){
    switch(channel){
        case 0://000
            AD0_L; AD1_L; AD2_L; break;

        case 1://001
            AD0_H; AD1_L; AD2_L; break;

        case 2://010
            AD0_L; AD1_H; AD2_L; break;

        case 3://011
            AD0_H; AD1_H; AD2_L; break;

        case 4://100
            AD0_L; AD1_L; AD2_H; break;

        case 5://101
            AD0_H; AD1_L; AD2_H; break;

        case 6://110
            AD0_L; AD1_H; AD2_H; break;

        case 7://111
            AD0_H; AD1_H; AD2_H; break;
        default: break;
    }
}


uint16_t raw_out_value[TRACE_SENSOR_COUNT] = {0};

/*
    先换通道 去处理其他逻辑 然后读取（给电容充分的时间充电）
    第一次直接获取数据 存入数组 换通道 函数结束
    第二次执行函数时间会排到其他逻辑后面
*/
void trace_readByADC(){
        static uint8_t cur_ch = 0;     

        //重置adc 防止访问到未定义内存
        DL_ADC12_disableConversions(OUT_INST);
        DL_ADC12_enableConversions(OUT_INST);

        DL_ADC12_clearInterruptStatus(OUT_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        DL_ADC12_startConversion(OUT_INST);

        //等待转换
        while ((DL_ADC12_getRawInterruptStatus(OUT_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)) == 0) { }
      
        //下来的就是转换完成的 但是为什么这里还要clear
        DL_ADC12_clearInterruptStatus(OUT_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        sensors[cur_ch].current_ADC = DL_ADC12_getMemResult(OUT_INST,DL_ADC12_MEM_IDX_0);
       
        //通道0获取到准确数据之后 就轮到下一个channel了
        cur_ch = (cur_ch+1) % TRACE_SENSOR_COUNT;
        CHx(cur_ch);
}

/*
加权平均（重心法）
Error =  \frac{\sum (\text{传感器电压值} \times \text{对应权重})}{\sum \text{传感器电压值}}$$
(分子是所有通道的加权和，分母是所有通道的电压总和)
*/
int32_t trace_get_error(Trace_OUT_t *t){
    int32_t numerator = 0;   // 分子（加权和）
    int32_t denominator = 0; // 分母（电压总和）

    //设定死区 防止小数据干扰计算（0-4095 小于500的就不要了）
    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        int32_t tem_val = t[i].current_ADC;
        if(tem_val < 500) tem_val = 0;

        numerator += tem_val * t[i].weight;
        denominator += tem_val;
    }
        //如果车子飞出赛道 分母会->0 value->无穷

        if(denominator == 0){return 0;}

        int32_t Error = numerator / denominator ;
        return Error;
    
}