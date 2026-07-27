#ifndef __SAMPLE_H	
#define __SAMPLE_H

#include "main.h"
typedef float fp32;
typedef double fp64;

#define ADC_SAMPLE_RATE  10000    // 采样率 10kHz
#define RMS_SAMPLE_COUNT 500  // 20ms周期/0.1ms采样间隔 = 200个采样点

//零点慢速跟踪系数:fc约0.5Hz,扣除采样零点漂移(输出直流分量的来源)
//50Hz处衰减可忽略,相移仅0.5度
#define DC_TRACK_K   0.0003f
//交流电压反馈一阶低通系数
//!! 必须保持1.0(旁路) !! 这条反馈直接决定外环在LC谐振点的相位裕度,
//   fc=640Hz时在1.6kHz谐振处引入-68度滞后,空载(负载阻尼消失)时会振荡
#define VOLT_FILT_K  1.00f
//电流反馈一阶低通系数
//内环靠这条反馈提供LC有源阻尼:系数越小(滤得越狠)在1600Hz谐振点的相位滞后越大,
//有效阻尼分量cos(滞后角)越小,阻尼越弱->1600Hz(32次)极限环失去抑制,自激成THD主峰。
// 0.33: fc约640Hz, 1600Hz处滞后约68度, 阻尼分量cos68≈0.37 (ζ≈0.007,近零阻尼,32次谐波2.4%)
// 0.70: fc约1900Hz,1600Hz处滞后约40度, 阻尼分量cos40≈0.77 (阻尼约翻倍,压32次极限环)
//实测:改此系数(0.33->0.70)对32次(1600Hz)极限环无影响,证明该极限环不经过电流反馈这条路,
//已定位其唯一维持通路为电压前馈(见control.c FF_FILT_K),故此处退回原值,不引入无关变量。
#define CRT_FILT_K   0.33f

//===== 谐振陷波器(消除1600Hz极限环自激) =====
//谐波表实测:能量集中在32次(1600Hz),两分钟内从0.13%涨到0.55%,是控制环负阻尼极限环
//在电压反馈进外环前插二阶IIR陷波,把环路在1600Hz的增益压到1以下,极限环失去能量来源
//中心频率f0=1600Hz,采样10kHz,w0=2*pi*1600/10000=1.00531rad, cos(w0)=0.53203
//带阻Q=3(-3dB带宽约±267Hz,覆盖29~35次边带),r=1-w0/(2Q)=0.83245
//对50Hz基波相移<0.4度,对20~100Hz变频全程无影响(陷波中心是物理频率,不随基波走)
//!! 若实测主峰不在1600Hz,只改NOTCH_COS=cos(2*pi*f_peak/10000),其余不动 !!
#define NOTCH_ENABLE   0   //已回退:反馈通道陷波器DC增益1.16导致输出掉到ref/1.16=20V,且在零裕度环里加动态环节使空载发散
#define NOTCH_COS      0.53203f
#define NOTCH_R        0.83245f
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


