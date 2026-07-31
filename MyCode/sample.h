#ifndef __SAMPLE_H	
#define __SAMPLE_H

#include "main.h"
#include "suanfa.h"

#define ADC_SAMPLE_RATE  CTRL_FREQUENCY

/* ===== 线电压采样标定修正 =====
 * 两路线电压的硬件增益(35.1617 / 35.3857)是按各自通道标定出来的, 但整条链路
 * (分压电阻 + 运放 + VDDA 基准)仍有零点几个百分点的残余增益误差。
 * 这个误差是致命的: Uab_rms 同时是慢环反馈"和" OLED 显示源, 所以环路会把
 * 一个偏高的读数拉到 32.00, OLED 也跟着显示 32.00, 而真实输出停在 32.00/c 以下
 * —— 表面看闭环收敛得很好, 实际就是差了这个比例。
 *
 * 实测(万用表 vs OLED, 二者都显示/读 32.00):
 *   60Hz: 真实 31.91~31.92  -> c = 31.915/32.00 = 0.99734
 *   30Hz: 真实 31.89        -> c = 31.890/32.00 = 0.99656
 * 两点差 0.06%, 其中 ±0.05% 是下面 RMS 窗口截断抖动(60Hz 最严重), 不是真实增益差,
 * 所以取中值 0.996875 一个常数覆盖全频段:
 *   修正后 60Hz 真实 32.015V, 30Hz 真实 31.990V, 都在 ±0.1V 内。
 *
 * !! 必须两路同乘同一个常数 !! sample.c:215 用 Uab/Ubc 重构三相相电压,
 * 只改一路等于给反馈注入 0.09% 负序, PR 环会照着这个假不平衡去调, 反而恶化其余两个线电压。
 * 换硬件/换采样板后重新按上面的方法测一次即可, 不要动 35.1617/35.3857。 */
#define VOLT_SENSE_TRIM  0.996875f

/* RMS 窗口累计的基波周期数(滑动平均, 不改变慢环每周期更新一次的节奏)。
 * 为什么需要 >1: 窗口只能是整数个采样点, 而一个基波周期是 fs/f 个点, 一般不是整数。
 * 截断的那零点几个点让 RMS 结果带上 ±(1/2N)*|sum cos| 的偏差, 且因为窗口起始相位
 * 逐周期缓慢游走, 这个偏差表现为几秒周期的慢漂 —— 慢环时间常数 1s, 会跟着漂,
 * 直接吃掉 ±0.1V 指标的一部分, 也让台面上标定读数抓不准。
 * 误差严格按 1/K 缩小(数值扫 20~100Hz 验证):
 *   K=1 最差 0.040V(75Hz) | K=2 0.020V | K=4 0.010V | K=8 0.005V
 * 取 K=4: 最差 0.010V(占指标 10%), 滞后 K/2=2 个基波周期(60Hz 33ms),
 * 相对慢环 50 周期的时间常数可忽略。
 * 注: 20/50/100Hz 处 fs/f 本就是整数, 任何 K 都精确, 抖动为 0。 */
#define RMS_AVG_PERIODS  4U

/* 单个基波周期的采样点数上限(仅作保护, 防止 theta 因故不回绕时无限累加)。
   面板频率下限 20Hz @20kHz = 1000 点, 留 10% 余量取 1100:
   窗口一旦撞上这个上限就不是整周期了(RMS 变成部分周期值), 所以它必须大于
   最低频率对应的周期点数, 不能刚好卡满。
   现在只存 (和, 点数), 不再存采样数组, 所以放大这个值几乎不花 RAM。 */
#define RMS_SAMPLE_COUNT 1100U

//零点慢速跟踪系数:fc = K*fs/(2*pi) 约0.48Hz,扣除采样零点漂移(输出直流分量的来源)
//50Hz处衰减可忽略,相移仅0.5度
//!! 该系数与采样率耦合 !! fs 改变时必须反比缩放才能保持同一 fc:
//   20kHz -> 1.5e-4, 10kHz -> 3.0e-4。忘了改则跟踪带宽跟着 fs 变, 零点收敛快慢不一。
//   这是一次等效改动, 本身不改变闭环行为。
#define DC_TRACK_K   0.0001500f
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


/* 有效值计算结构体。
   窗口边界由控制环电角度 theta 的回绕给出, 不再用 a=fs/f 估算:
   theta 就是生成输出电压的那个累加器, 它的回绕点即真实输出周期边界, 天然同步,
   且与面板频率整数值无关(f 只用于检测频率切换时清状态)。
   原先的 sampleBuffer[1000] 只写不读, 已删除 —— 省 12kB RAM(三个实例)和每次中断
   一次无用的数组写。 */
typedef struct {
    float periodSum[RMS_AVG_PERIODS];      // 近K个基波周期各自的平方和
    uint16_t periodN[RMS_AVG_PERIODS];     // 对应各周期的采样点数(可能相差1个点)
    uint8_t  ringIndex;                    // 环形写指针
    uint8_t  ringFill;                     // 已填充的周期数(<=K, 用于启动阶段)
    uint16_t sampleIndex;                  // 本周期已累加点数
    float sumSquares;                      // 本周期平方累加和
	uint16_t lastFreq;  // 保存上一次频率，用于检测变化
	float lastRMS;     // 保存上一次的RMS值
	float prevTheta;   // 上一次的电角度,用于检测周期回绕
} RMS_Calculator;


void RMS_Init(RMS_Calculator *calc);// 初始化RMS计算器
void RMS_ResetAll(void);// 复位全部RMS实例(Uab/Ubc/Ia),每次启动由Control_Reset调用
//theta: 控制环电角度, 其回绕即窗口边界
float RMS_Update(RMS_Calculator *calc, float newSample, float theta, uint16_t f);


float Calculate_ACCurrent_RMS_A(ST_ELEC_OBS *pstM,float theta,uint16_t f);//计算A相线电流有效值

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

float Calculate_ACVoltage_RMS_AB(ST_ELEC_OBS *pstM,float theta,uint16_t f); //计算交流电压有效值 Uab
float Calculate_ACVoltage_RMS_BC(ST_ELEC_OBS *pstM,float theta,uint16_t f); //计算交流电压有效值 Ubc


void Cal_ACCurrent_A(ST_ELEC_OBS *pstM);//A相交流电流瞬时值
void Cal_ACCurrent_C(ST_ELEC_OBS *pstM);//C相交流电流瞬时值

void Cal_ACVolt_AB(ST_ELEC_OBS *pstM);//AB线电压瞬时值采样
void Cal_ACVolt_BC(ST_ELEC_OBS *pstM);//BC线电压瞬时值采样


void Calculate_PhaseVoltage(ST_ELEC_OBS *pstM);  //根据Uab Ubc 计算相电压Ua Uc
	
#endif


