#include "trace.h"

Trace_OUT_t sensors[TRACE_SENSOR_COUNT];
static int32_t current_error = 0 ; 
static int32_t last_strong_error = 0 ;
static bool is_white_bg = true ;
static bool adc_ok = true ;
static bool normal_line = false;
static bool wide_line = false;
static uint8_t center_frame = 0;//窄线的次数
uint16_t raw_out_value[TRACE_SENSOR_COUNT] = {0} ;

bool ADC_OK(void){ return adc_ok; }
bool trace_line_is_valid(void){ return normal_line; }

bool trace_is_white_bg(void) { return is_white_bg; }
void trace_set_bg(bool white_bg){ is_white_bg = white_bg; }

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
    sensors[CH4].weight = -5;
    sensors[CH5].weight =  5;
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

void trace_readFromUART(const uint16_t values[TRACE_SENSOR_COUNT])
{
    uint8_t ch;

    for (ch = 0; ch < TRACE_SENSOR_COUNT; ch++) {
        sensors[ch].current_ADC = values[ch];
    }
    adc_ok = true;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
加权平均（重心法）
Error =  \frac{\sum (\text{传感器电压值} \times \text{对应权重})}{\sum \text{传感器电压值}}$$
(分子是所有通道的加权和，分母是所有通道的ADC数值总和)
带阈值自适应
滞回阈值
*/
int32_t trace_get_error(Trace_OUT_t *t){
    bool has_gap = false;
    wide_line = false;
    normal_line = false;

    if(!adc_ok){
        center_frame = 0;
        normal_line = false;
        return last_strong_error;
    }

    int32_t numerator = 0;   // 分子（加权和）
    int32_t denominator = 0; // 分母（电压总和）

    int32_t max_val = 0;
    int32_t min_val = 4095;
    int32_t processed_val[TRACE_SENSOR_COUNT];

    for(uint8_t i = 0; i < TRACE_SENSOR_COUNT ; i++){  
        int32_t raw = (int32_t)t[i].current_ADC;
        if (raw > 4095) raw = 4095;

        if(!is_white_bg){
            // 黑底白线：白线反射强、ADC低，需要反相
            processed_val[i] = 4095 - raw;
        }else{
            // 白底黑线：黑线反射弱、ADC高，直接使用
            processed_val[i] = raw;
        }
        if(processed_val[i] > max_val) max_val = processed_val[i];
        if(processed_val[i] < min_val) min_val = processed_val[i];
}    

        //uint8_t temp = 0;
        int8_t first_active = -1,last_active = -1;//记录两通道的差值
        int32_t span;// 当前帧有效通道跨度
        uint8_t active_count = 0;

    if((max_val - min_val) < 800){ //说明没有扫到线
        denominator = 0;//若是丢线 则触发救车flag
    }else{
        ///取极差0.333作为背景光噪音
        //min就是所有传感器中最小的adc数值 可以直接看作环境的背景底噪（黑色背景）
        //max则是最强反光（白色循迹线）
        //极差则是"线"和"背景"的 对比度 对比度取1/3获得相对阈值
        //将底噪和相对阈值结合  只有超出背景的信号才定义为有效
        int32_t noise_threshold = min_val + (max_val - min_val) / 3;
        
        for(uint8_t i = 0; i < TRACE_SENSOR_COUNT ; i++){
            int32_t tem_val = processed_val[i];
            if(tem_val > noise_threshold){//大于底部噪音 那就是真的“特征”
                tem_val -= noise_threshold;
//不希望再重复定义一堆变量了 但是这种临时参与运算的用结构以固定会占据固定位置 “浪费 ”？
//更觉得不应该在主要逻辑没有弄清楚的时候纠结这个
//我发现别说记录清楚多少个通道了 我可以简单在这里把active++ 但是在哪里归零？ 获取这个数据之后就可以 可这个数据用在什么地方呢  在判断宽窄线之后就能丢 那大概能理解归零 但是又应该怎么记录 0110和0101呢 按位与和移位吗 试试看
                if(first_active < 0) first_active = i;
                last_active = i;
//                temp |= (1u << i);
                active_count++;
            }else{
                tem_val = 0;
            }
            numerator += tem_val * t[i].weight;
            denominator += tem_val;
        }

    }

            //超出赛道
    if(denominator == 0){//回忆 上一刻循迹线的位置（越左边越小（小于零））
        center_frame = 0;
        normal_line = false;
        return last_strong_error;    
    }
        //重心计算
        current_error = numerator / denominator ;
        uint8_t abs_error = abs(current_error);

        span = last_active - first_active;
        //如果中间有空洞 0101 active就是2 跨度是三 而0111的active是三 跨度也是三
        // if(span > (active_count - 1)) has_gap = true;
        // else if (span = (active_count - 1)) has_gap = false;
        //01110 ac=3 s=2 = ac -1 ; 01011 ac=3 s=3 > ac-1 (有空洞)
        has_gap = (active_count > 1) && (span > (active_count - 1));
/*0110：count=2 span=1 → 窄线
  0101：count=2 span=2 → 有空洞，模糊
  0111：count=3 span=2 → 宽线
  1111：count=4 span=3 → 宽线*/
        wide_line = (active_count >= 3) || has_gap;
     
        normal_line = !wide_line;

    /*
     * A centered line must override the previous escape direction. Without
     * this, a wide center pattern keeps returning the last +/-14 error and
     * only changes sign after the line has already crossed the center.
     */
    if (abs_error <= 8U) {
        if (center_frame < 3U) center_frame++;
        if (center_frame >= 3U) last_strong_error = 0;
        normal_line = true;
        return current_error;
    }

//普通 有效的窄线 返回current
//宽线 当前方向不可靠 返回strong
//丢线 返回strong
//不要把刷新数值和条件返回一起写 容易乱
//刷新
    if(abs_error >= 14){//如果强方向

        last_strong_error = current_error;
        center_frame = 0;

    }else{
        center_frame = 0;
    }

//返回
/*   当前情况          返回值
  ━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━
   ADC失败           last_strong_error
  ────────────────  ───────────────────
   丢线              last_strong_error
  ────────────────  ───────────────────
   宽线、小误差      last_strong_error
  ────────────────  ───────────────────
   宽线、强方向      current_error
  ────────────────  ───────────────────
   窄线、强方向      current_error
  ────────────────  ───────────────────
   窄线、中等误差    current_error
  ────────────────  ───────────────────
   窄线、中心误差    current_error
*/
    if(wide_line && abs_error < 14 && last_strong_error != 0){
        return last_strong_error;
    }else{
        return current_error;
    }


//不能无脑返回strong 也不能直接让current 直接返回 那这不是一定会有警告吗
}
