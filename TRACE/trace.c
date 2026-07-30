#include "trace.h"

Trace_OUT_t sensors[TRACE_SENSOR_COUNT];
static int32_t last_valid_error = 0;
static bool is_white_bg = true ;
static bool adc_ok = true ;
static bool line_valid = false;
static uint8_t active_mask = 0U;

static const int32_t trace_weights_normal[TRACE_SENSOR_COUNT] = {
    -38, -26, -18, -5, 5, 18, 26, 38
};
static const int32_t trace_weights_mode1[TRACE_SENSOR_COUNT] = {
    -50, -40, -30, -5, 5, 30,40, 50
};

bool ADC_OK(void){ return adc_ok; }
bool trace_line_is_valid(void){ return line_valid; }
uint8_t trace_get_active_mask(void){ return active_mask; }

bool trace_is_white_bg(void) { return is_white_bg; }
void trace_set_bg(bool white_bg){ is_white_bg = white_bg; }

void trace_set_weight_profile(bool mode1_boost)
{
    const int32_t *weights = mode1_boost ?
        trace_weights_mode1 : trace_weights_normal;
    for (uint8_t i = 0U; i < TRACE_SENSOR_COUNT; i++) {
        sensors[i].weight = weights[i];
    }
}

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

    trace_set_weight_profile(false);

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

void trace_readByADC(){
    adc_ok = true;

    for (uint8_t ch = 0; ch < TRACE_SENSOR_COUNT; ch++) {
        CHx(ch);
        delay_cycles(800); // 延时约 10us 等待模拟开关建立时间

        DL_ADC12_enableConversions(OUT_INST);

        DL_ADC12_clearInterruptStatus(OUT_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        DL_ADC12_startConversion(OUT_INST);

        uint32_t timeout = 100000;
        //等待转换
        while ((DL_ADC12_getRawInterruptStatus(OUT_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)) == 0) {
        if (--timeout == 0) {
            adc_ok = false;
            // 发生硬件崩溃！ 丢掉数据
            DL_ADC12_disableConversions(OUT_INST);
            DL_ADC12_enableConversions(OUT_INST);
            DL_ADC12_clearInterruptStatus(OUT_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);            
            return; 
        }
    }
        DL_ADC12_disableConversions(OUT_INST);
    if(timeout > 0){
        DL_ADC12_clearInterruptStatus(OUT_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        sensors[ch].current_ADC = DL_ADC12_getMemResult(OUT_INST,DL_ADC12_MEM_IDX_0);        
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t trace_get_error(Trace_OUT_t *t)
{
    int32_t numerator = 0;
    int32_t denominator = 0;
    int32_t max_val = 0;
    int32_t min_val = 4095;
    int32_t processed_val[TRACE_SENSOR_COUNT];

    line_valid = false;
    active_mask = 0U;

    if (!adc_ok) {
        return 0;
    }

    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        int32_t raw = (int32_t)t[i].current_ADC;
        if (raw > 4095) raw = 4095;

        processed_val[i] = is_white_bg ? raw : (4095 - raw);
        if (processed_val[i] > max_val) max_val = processed_val[i];
        if (processed_val[i] < min_val) min_val = processed_val[i];
    }

    if ((max_val - min_val) < 800) {
        return last_valid_error;
    }

    int32_t active_threshold = min_val + (max_val - min_val) / 3;

    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        int32_t value = processed_val[i] - min_val;
        if (processed_val[i] > active_threshold) {
            active_mask |= (uint8_t)(1U << i);
        }
        numerator += value * t[i].weight;
        denominator += value;
    }

    if (denominator == 0) {
        return last_valid_error;
    }

    line_valid = true;
    last_valid_error = numerator / denominator;
    return last_valid_error;
}
