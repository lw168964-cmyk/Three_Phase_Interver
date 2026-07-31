#ifndef __SAMPLE_H	
#define __SAMPLE_H

#include "main.h"
#include "suanfa.h"

#define ADC_SAMPLE_RATE  CTRL_FREQUENCY
/* 20 Hz at 10 kHz requires 500 samples per period.
   保留 1000 是为了给 RMS_Update 的窗口 a 留裕度(a 被钳在此值以内),
   下限是 500 —— 面板频率下限 20Hz 时 a=500, 低于此值窗口会被截断,
   RMS 就不再是整周期有效值。 */
#define RMS_SAMPLE_COUNT 1000U

//零点慢速跟踪系数:fc = K*fs/(2*pi) 约0.48Hz,扣除采样零点漂移(输出直流分量的来源)
//50Hz处衰减可忽略,相移仅0.5度
//!! 该系数与采样率耦合 !! fs 20k->10k 时必须翻倍(1.5e-4 -> 3.0e-4)才能保持同一fc,
//   否则跟踪带宽减半、零点收敛慢一倍。这是一次等效改动, 不改变闭环行为。
#define DC_TRACK_K   0.0003000f
//交流电压反馈一阶低通系数
//!! 必须保持1.0(旁路) !! 这条反馈决定外环在LC谐振点(L=2mH,C=9.9uF -> 1131Hz)的相位裕度,
//   任何额外低通都会在谐振处叠加滞后,空载(负载阻尼消失)时导致振荡
#define VOLT_FILT_K  1.00f
//电流通道是有源阻尼通路(见inital.c的内环Kp)。
//!! 保持1.0(旁路) !! 低通对空载不稳没有帮助:开关级实测在Kr=1.0下扫K,
//   K=1.0残差0.49V, K=0.6升到9.2V, K=0.4升到27.5V, K=0.2升到47.1V——
//   它在谐振点(现1131Hz)引入的相位滞后(K=0.4时约-26度)造成的损失大于抑制纹波的收益。
//   纹波问题的正解是把采样点移到纹波平均点,不是滤波。
#define CRT_FILT_K   1.00f

//===== 谐振陷波器(历史遗留,保持关闭) =====
//背景:曾用于压制观测到的极限环。极限环的真实成因是
//  (1)电压前馈取自实测电容电压,把LC振铃正反馈回占空比(已改参考式前馈修复)
//  (2)内环Kp=1.0对应等效阻尼电阻仅1欧,而Z0=sqrt(L/C)量级十几欧,阻尼严重不足
//     (现C=9.9uF -> Z0=14.21欧, 内环已改Kp=2.2, 见inital.c)
//两处修复后20~100Hz全工况(含空载)残差为0,不需要陷波器。
//!! 下面的系数是按 C=2uF 假设的1600Hz算的,而实际 C=9.9uF -> LC谐振在1131Hz。
//   若确有需要重新启用,NOTCH_COS 必须按实测主峰重算,不能直接用现值 !!
#define NOTCH_ENABLE   0   //保持0:该陷波器DC增益1.16会让输出掉到ref/1.16,且在低裕度环里加动态环节有害
#define NOTCH_COS      0.87631f
#define NOTCH_R        0.91622f
#define NOTCH_A1       (-2.0f*NOTCH_R*NOTCH_COS)   //分母z^-1系数
#define NOTCH_A2       (NOTCH_R*NOTCH_R)           //分母z^-2系数
#define NOTCH_B1       (-2.0f*NOTCH_COS)           //分子z^-1系数(分子b0=b2=1)

//基波同步解调状态
typedef struct {
    fp32 sumSin;      //与sin(theta)的相关累加
    fp32 sumCos;      //与cos(theta)的相关累加
    uint16_t n;        //本窗口已累加样本数
    uint16_t lastFreq; //频率变化时重置窗口
    fp32 lastRMS;     //上一窗口的结果(窗口未满时返回它)
} FUND_RMS;

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

/* 基波有效值提取(同步解调)。
   背景: 采样点取的是电感电流, 空载时开关纹波峰峰约0.31A(∝Udc, 60V档), 而基波
   峰值只有 2*pi*50*9.9uF*26.1V = 0.081A —— 纹波比基波大数倍。
   (升到60V/32V后纹波与基波同向增长, 比值反而从4.1略降到3.8)
   直接对采样序列求RMS
   得到的是"纹波+基波+噪声"的总有效值, 会把空载电流显示成几百mA甚至上A,
   那不是流过负载的电流。
   本函数用 sin/cos 与基波相关积分, 只留下与基波同频同相的分量, 纹波和
   量化噪声因与基波不相关而被积分平均掉。theta 直接用控制环的电角度, 天然同步。 */
float Fundamental_RMS_Update(FUND_RMS *s, float sample, float theta, uint16_t f);

float Calculate_ACVoltage_RMS_AB(ST_ELEC_OBS *pstM,uint16_t f); //计算交流电压有效值 Uab
float Calculate_ACVoltage_RMS_BC(ST_ELEC_OBS *pstM,uint16_t f); //计算交流电压有效值 Ubc


void Cal_ACCurrent_A(ST_ELEC_OBS *pstM);//A相交流电流瞬时值
void Cal_ACCurrent_C(ST_ELEC_OBS *pstM);//C相交流电流瞬时值

void Cal_ACVolt_AB(ST_ELEC_OBS *pstM);//AB线电压瞬时值采样
void Cal_ACVolt_BC(ST_ELEC_OBS *pstM);//BC线电压瞬时值采样


void Calculate_PhaseVoltage(ST_ELEC_OBS *pstM);  //根据Uab Ubc 计算相电压Ua Uc
	
#endif


