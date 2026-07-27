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
//!! 必须保持1.0(旁路) !! 这条反馈决定外环在LC谐振点(L=2mH,C=12uF -> 1027Hz)的相位裕度,
//   任何额外低通都会在谐振处叠加滞后,空载(负载阻尼消失)时导致振荡
#define VOLT_FILT_K  1.00f
//电流通道是有源阻尼通路,内环Kp=8.0靠它提供等效阻尼电阻(见inital.c)。
//!! 必须保持1.0(旁路) !! 阻尼要在1027Hz谐振处有效,该通路的相位滞后会直接
//   抵消阻尼(cos(相移)项),与PWM更新延时叠加后会把有效阻尼推向零。
#define CRT_FILT_K   1.00f

//===== 谐振陷波器(历史遗留,保持关闭) =====
//背景:曾用于压制观测到的极限环。极限环的真实成因是
//  (1)电压前馈取自实测电容电压,把LC振铃正反馈回占空比(已改参考式前馈修复)
//  (2)内环Kp=1.0对应等效阻尼电阻仅1欧,而Z0=sqrt(L/C)=12.91欧,阻尼严重不足(已改Kp=8.0)
//两处修复后20~100Hz全工况(含空载)残差为0,不需要陷波器。
//!! 下面的系数是按 C=2uF 假设的1600Hz算的,而实际 C=12uF -> LC谐振在1027Hz。
//   若确有需要重新启用,NOTCH_COS 必须按实测主峰重算,不能直接用现值 !!
#define NOTCH_ENABLE   0   //保持0:该陷波器DC增益1.16会让输出掉到ref/1.16,且在低裕度环里加动态环节有害
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


