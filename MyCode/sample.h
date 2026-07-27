#ifndef __SAMPLE_H	
#define __SAMPLE_H

#include "main.h"
#include "suanfa.h"

#define ADC_SAMPLE_RATE  CTRL_FREQUENCY
#define RMS_SAMPLE_COUNT 1000U  // 20 Hz at 20 kHz requires 1000 samples per period.

//零点慢速跟踪系数:fc约0.5Hz,扣除采样零点漂移(输出直流分量的来源)
//50Hz处衰减可忽略,相移仅0.5度
#define DC_TRACK_K   0.0001500f
//交流电压反馈一阶低通系数
//!! 必须保持1.0(旁路) !! 这条反馈直接决定外环在LC谐振点的相位裕度,
//   fc=640Hz时在1.6kHz谐振处引入-68度滞后,空载(负载阻尼消失)时会振荡
#define VOLT_FILT_K  1.00f
//The current path is the active-damping feedback path. With a carrier-locked,
//settled ADC sample it must not add a low-pass phase lag: 0.1815 produces
//about 54 degrees at 1.6 kHz, which combines with the PWM update delay and
//removes the damping around the observed 32nd harmonic.
#define CRT_FILT_K   1.00f

//===== 谐振陷波器(消除1600Hz极限环自激) =====
//谐波表实测:能量集中在32次(1600Hz),两分钟内从0.13%涨到0.55%,是控制环负阻尼极限环
//在电压反馈进外环前插二阶IIR陷波,把环路在1600Hz的增益压到1以下,极限环失去能量来源
//中心频率f0=1600Hz,采样20kHz,w0=2*pi*1600/20000=0.50265rad, cos(w0)=0.87631
//带阻Q=3(-3dB带宽约±267Hz,覆盖29~35次边带),r=1-w0/(2Q)=0.91622
//对50Hz基波相移<0.4度,对20~100Hz变频全程无影响(陷波中心是物理频率,不随基波走)
//!! 若实测主峰不在1600Hz,只改NOTCH_COS=cos(2*pi*f_peak/20000),其余不动 !!
#define NOTCH_ENABLE   0   //已回退:反馈通道陷波器DC增益1.16导致输出掉到ref/1.16=20V,且在零裕度环里加动态环节使空载发散
#define NOTCH_COS      0.87631f
#define NOTCH_R        0.91622f
#define NOTCH_A1       (-2.0f*NOTCH_R*NOTCH_COS)   //分母z^-1系数
#define NOTCH_A2       (NOTCH_R*NOTCH_R)           //分母z^-2系数
#define NOTCH_B1       (-2.0f*NOTCH_COS)           //分子z^-1系数(分子b0=b2=1)

//单路二阶陷波器状态(直接II型)
typedef struct { fp32 w1, w2; } ST_NOTCH;
fp32 Notch_Update(ST_NOTCH *s, fp32 x);


typedef struct
{
	fp32 fpABVolt;			 
	fp32 fpBCVolt;			 
	fp32 fpACCVolt;			 
	
	fp32 fpPhaAVoltFB;    //A相电压反馈
	fp32 fpPhaBVoltFB;    //B相电压反馈
	fp32 fpPhaCVoltFB;    //C相电压反馈

	fp32 fpPhaAVoltRMS;    //A相电压反馈有效值
	fp32 fpPhaBVoltRMS;    //B相电压反馈有效值
	fp32 fpPhaCVoltRMS;    //C相电压反馈有效值
	
	fp32 fpPha1CrtFB;          //A相电流反馈
	fp32 fpPha2CrtFB;          //B相电流反馈
	fp32 fpPha3CrtFB;          //C相电流反馈

	fp32 fpPha1CrtFilt;        //A相电流反馈(低通滤波后,用于内环)
	fp32 fpPha3CrtFilt;        //C相电流反馈(低通滤波后,用于内环)

	fp32 fpABVoltDC;           //AB线电压采样零点(慢速跟踪值)
	fp32 fpBCVoltDC;           //BC线电压采样零点(慢速跟踪值)
	fp32 fpPha1CrtDC;          //A相电流采样零点(慢速跟踪值)
	fp32 fpPha3CrtDC;          //C相电流采样零点(慢速跟踪值)
	
	int32_t ssVPower;			     //电源电压
	int16_t ssZeroVPowerValue; //电源电压零点对应
	fp32 fpCPUTemp;
} ST_ELEC_OBS;


//有效值计算结构体
typedef struct {
    float sampleBuffer[RMS_SAMPLE_COUNT];  // 存储一个周期的采样值
    uint16_t sampleIndex;                  // 当前采样位置
    float sumSquares;                      // 平方累加和
    uint8_t isBufferFull;                  // 缓冲区已满标志
	uint16_t lastFreq;  // 保存上一次频率，用于检测变化
	float lastRMS;     // 保存上一次的RMS值
} RMS_Calculator;


void RMS_Init(RMS_Calculator *calc);// 初始化RMS计算器
float RMS_Update(RMS_Calculator *calc, float newSample , uint16_t f) ;// 更新采样值并计算有效值


float Calculate_ACCurrent_RMS_A(ST_ELEC_OBS *pstM,uint16_t f);//计算A相线电流有效值

float Calculate_ACVoltage_RMS_AB(ST_ELEC_OBS *pstM,uint16_t f); //计算交流电压有效值 Uab
float Calculate_ACVoltage_RMS_BC(ST_ELEC_OBS *pstM,uint16_t f); //计算交流电压有效值 Ubc


void Cal_ACCurrent_A(ST_ELEC_OBS *pstM);//A相交流电流瞬时值
void Cal_ACCurrent_C(ST_ELEC_OBS *pstM);//C相交流电流瞬时值

void Cal_ACVolt_AB(ST_ELEC_OBS *pstM);//AB线电压瞬时值采样
void Cal_ACVolt_BC(ST_ELEC_OBS *pstM);//BC线电压瞬时值采样


void Calculate_PhaseVoltage(ST_ELEC_OBS *pstM);  //根据Uab Ubc 计算相电压Ua Uc
	
#endif


