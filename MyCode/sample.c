#include "sample.h"
#include "main.h"
#include "math.h"
#include "string.h"

extern uint16_t ADC1_Value[];

// 初始化RMS计算器
void RMS_Init(RMS_Calculator *calc) 
{
    memset(calc, 0, sizeof(RMS_Calculator));
}

// 更新采样值并计算有效值 (20 kHz control callback)
float RMS_Update(RMS_Calculator *calc, float newSample , uint16_t f) 
{
	uint16_t a = (ADC_SAMPLE_RATE + f / 2) / f;//四舍五入 
	
	if(a>=RMS_SAMPLE_COUNT)
	{
		a = RMS_SAMPLE_COUNT;
	}
    // 频率变化 → 强制重置缓冲区
    if(f != calc->lastFreq)
    {
        calc->lastFreq = f;       // 记录新频率
        calc->sumSquares = 0.0f;  // 清空平方和
        calc->sampleIndex = 0;    // 清空索引
    }
    
    // 累加平方和
    calc->sumSquares += newSample * newSample;
    calc->sampleBuffer[calc->sampleIndex] = newSample;
    calc->sampleIndex++;
    
    // 当累积到一周期的样本时，计算RMS并重置
    if(calc->sampleIndex >= a)
    {
        calc->lastRMS = sqrtf(calc->sumSquares / a);
        calc->sumSquares = 0.0f;
        calc->sampleIndex = 0;
        return calc->lastRMS;
    }
    
    // 周期未满时返回上一次的RMS值
    return calc->lastRMS;
}

float F=0;
//计算A相交流电流有效值(线电流)
float Calculate_ACCurrent_RMS_A(ST_ELEC_OBS *pstM,uint16_t f) 
{
    static RMS_Calculator rmsCalc3;
    static uint8_t isInitialized = 0;
    
    // 首次调用时初始化
    if (!isInitialized) 
	{
        RMS_Init(&rmsCalc3);
        isInitialized = 1;
    }
    
    //瞬时值已由控制环在本周期开头统一采样,这里不能再调一次:
    //Cal_*内含滤波器/零点跟踪状态,重复调用会让滤波器每周期被推进两次
    F= pstM->fpPha1CrtFB;
    // 更新RMS计算并返回有效值
    return RMS_Update(&rmsCalc3,pstM->fpPha1CrtFB,f);
}
//交流侧电流采样A(线电流)
void Cal_ACCurrent_A(ST_ELEC_OBS *pstM)
{
	float raw = (float)((ADC1_Value[1])*3.3f/4096.f-1.646f)/0.2f;
	//慢速跟踪并扣除零点:硬编码的1.646V与实际零点有偏差,直接变成环路的直流误差
	pstM->fpPha1CrtDC += DC_TRACK_K * (raw - pstM->fpPha1CrtDC);
	pstM->fpPha1CrtFB = raw - pstM->fpPha1CrtDC;
	//一阶低通,供内环使用
	pstM->fpPha1CrtFilt += CRT_FILT_K * (pstM->fpPha1CrtFB - pstM->fpPha1CrtFilt);
}

//交流侧电流采样C(线电流)
void Cal_ACCurrent_C(ST_ELEC_OBS *pstM)
{
	float raw = (float)((ADC1_Value[3])*3.3f/4096.f-1.646f)/0.2f;
	pstM->fpPha3CrtDC += DC_TRACK_K * (raw - pstM->fpPha3CrtDC);
	pstM->fpPha3CrtFB = raw - pstM->fpPha3CrtDC;
	pstM->fpPha3CrtFilt += CRT_FILT_K * (pstM->fpPha3CrtFB - pstM->fpPha3CrtFilt);
}

//计算交流电压有效值 Uab
float Calculate_ACVoltage_RMS_AB(ST_ELEC_OBS *pstM,uint16_t f) 
{
    static RMS_Calculator rmsCalc1;
    static uint8_t isInitialized = 0;
    
    // 首次调用时初始化
    if (!isInitialized) 
	{
        RMS_Init(&rmsCalc1);
        isInitialized = 1;
    }
    
    //瞬时值已由控制环在本周期开头统一采样,不能重复调用(见Cal_ACVolt_AB内的滤波状态)
    return RMS_Update(&rmsCalc1,pstM->fpABVolt,f);
}

//计算交流电压有效值 Ubc
float Calculate_ACVoltage_RMS_BC(ST_ELEC_OBS *pstM,uint16_t f) 
{
    static RMS_Calculator rmsCalc2;
    static uint8_t isInitialized = 0;
    
    // 首次调用时初始化
    if (!isInitialized) 
	{
        RMS_Init(&rmsCalc2);
        isInitialized = 1;
    }
    
    //瞬时值已由控制环在本周期开头统一采样,不能重复调用
    return RMS_Update(&rmsCalc2,pstM->fpBCVolt,f);
}

//AB线电压
void Cal_ACVolt_AB(ST_ELEC_OBS *pstM)
{
	float raw = (float)((ADC1_Value[0])*3.3f/4096.f-1.644f)*30.f;
	//1.慢速跟踪并扣除零点:零点误差经前馈会变成输出直流分量(实测Ua有-1.96%直流)
	pstM->fpABVoltDC += DC_TRACK_K * (raw - pstM->fpABVoltDC);
	raw -= pstM->fpABVoltDC;
	//2.一阶低通:固定载波谷值采样后仅保留测量噪声抑制。
	pstM->fpABVolt += VOLT_FILT_K * (raw - pstM->fpABVolt);
}

//BC线电压
void Cal_ACVolt_BC(ST_ELEC_OBS *pstM)
{
	float raw = (float)((ADC1_Value[2])*3.3f/4096.f-1.656f)*29.94f;
	pstM->fpBCVoltDC += DC_TRACK_K * (raw - pstM->fpBCVoltDC);
	raw -= pstM->fpBCVoltDC;
	pstM->fpBCVolt += VOLT_FILT_K * (raw - pstM->fpBCVolt);
}

 //根据Uab Ubc 计算相电压Ua Uc
//二阶IIR陷波器(直接II型转置),消除1600Hz极限环
//H(z)=(1+B1*z^-1+z^-2)/(1+A1*z^-1+A2*z^-2)
fp32 Notch_Update(ST_NOTCH *s, fp32 x)
{
	fp32 w0 = x - NOTCH_A1 * s->w1 - NOTCH_A2 * s->w2;
	fp32 y  = w0 + NOTCH_B1 * s->w1 + s->w2;
	s->w2 = s->w1;
	s->w1 = w0;
	return y;
}

//Three voltage-feedback notch states are only needed when the optional notch
//is compiled in.
#if NOTCH_ENABLE
static ST_NOTCH notch_A = {0}, notch_B = {0}, notch_C = {0};
#endif

void Calculate_PhaseVoltage(ST_ELEC_OBS *pstM)
{
	fp32 uA = (2.f * pstM->fpABVolt + pstM->fpBCVolt)/3.f;
	fp32 uB = (pstM->fpBCVolt - pstM->fpABVolt)/3.f;
	fp32 uC = -(pstM->fpABVolt + 2*pstM->fpBCVolt)/3.f;

#if NOTCH_ENABLE
	//送入外环前陷掉1600Hz,切断极限环的能量来源。基波(20~100Hz)几乎不受影响。
	uA = Notch_Update(&notch_A, uA);
	uB = Notch_Update(&notch_B, uB);
	uC = Notch_Update(&notch_C, uC);
#endif

	pstM->fpPhaAVoltFB = uA;
	pstM->fpPhaBVoltFB = uB;
	pstM->fpPhaCVoltFB = uC;
}

