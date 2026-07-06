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

/*### 一、整体实现原理
这套代码是**模拟多路开关 + 单ADC轮询采样 + 重心法（加权平均）**的模拟量循迹方案，核心是通过反射光强的连续分布计算赛道位置，而非简单的黑白二值判断。

#### 1. 硬件与通道切换
- 8路红外反射传感器通过**3位地址线控制的模拟多路开关（典型如CD4051）**共用1路ADC输入，节省硬件ADC通道。
- `CHx(channel)` 是模拟开关的地址译码函数：通过 `AD0/AD1/AD2` 三个IO的高低电平组合，选通8路中的某一路接入ADC。

#### 2. 采样逻辑：流水线式轮询
`trace_readByADC()` 采用“读当前通道 → 预切下一通道”的流水线设计，把模拟开关切换后的RC充电稳定时间“隐藏”在其他业务逻辑中，不浪费CPU。

单次调用的执行流程：
1. 复位ADC、清除转换完成中断标志，软件触发一次ADC转换。
2. 阻塞等待转换完成，读取结果存入 `sensors[cur_ch].current_ADC`。
3. 通道号自增并取模，调用 `CHx()` 切换到下一路传感器。
4. 下一次调用该函数时，下一路传感器已经经过了充足的稳定时间，可直接采样到稳定值。

#### 3. 偏差计算：重心法
`trace_get_error()` 是循迹的核心算法，通过加权平均计算黑线的位置偏差，是模拟循迹的标准方案。

对应公式与代码逻辑：
- **分子**：每个传感器的ADC值 × 对应权重（位置系数），全部求和
- **分母**：所有有效传感器的ADC值求和
- **偏差 Error = 分子 / 分母**

权重设计为左右对称：`-40, -30, -20, -10, 10, 20, 30, 40`。黑线在中间时左右加权抵消，Error≈0；黑线偏左Error为负，偏右为正，数值大小对应偏离程度。

额外的工程处理：
- **死区过滤**：ADC值小于500直接置0，滤除传感器底噪和微弱杂散光。
- **分母保护**：所有通道值均为0时（完全脱离赛道）直接返回0，避免除0异常。

---

### 二、环境光抗干扰能力分析
#### 1. 相比二值循迹，抗光性确实更强
你的判断方向是对的：它依赖的是**反射光的相对分布（对比度）**，而非绝对的黑白阈值，因此对环境光的整体变化有天然鲁棒性。

环境光整体变强/变弱时，所有传感器的接收光强会同步抬升或同步下降，但“白色反射值与黑色反射值的比例关系”基本不变。重心法计算的是相对分布，最终偏差结果受整体光强变化的影响很小；而传统二值循迹用固定阈值判黑白，环境光一变就容易大面积误判。

#### 2. 并非完全不受灯光影响，存在明显局限
当前代码的抗光能力是有限的，主要短板有三点：
1. **未做黑白标定与归一化**
   结构体中预留了 `white_value` 和 `black_value` 字段，但代码完全没有使用。理想做法是先标定纯白、纯黑下的ADC基准值，将原始值归一化到0~1区间：
   ```
   归一化值 = (current_ADC - black_value) / (white_value - black_value)
   ```
   归一化后的值只反映“相对黑白程度”，与环境光绝对强度无关，抗光能力会大幅提升。当前直接用原始ADC值计算，环境光大幅漂移时仍会影响死区和权重比例。

2. **死区为固定阈值**
   500的固定阈值在暗光环境下，可能把白色路面的反射值也过滤掉，导致分母为0、误判脱线；在强光环境下，黑色赛道反射值也会偏高，死区失效，底噪会干扰计算。

3. **无法抵消单侧强光**
   如果场地有定向侧光，会导致某一侧传感器基线整体偏高，相当于引入固定偏移，重心计算结果会出现系统性偏差。

---

### 三、进一步提升抗光性的优化方向
如果要适配不同灯光环境，可在现有代码基础上改进：
1. 增加标定流程：上电或按键触发时，记录纯白路面和纯黑赛道的ADC值，计算偏差前先做归一化。
2. 将固定死区改为相对死区（如按最大值的百分比过滤），适配不同光强下的底噪水平。
3. 硬件层面可增加红外发射管的载波调制（如38KHz），从物理层面滤除环境可见光，这是最彻底的抗光方案。

需要我帮你补充黑白标定与归一化的完整代码吗？*/