#include "inital.h"
#include "main.h"


#define idmax 1000
#define iqmax 1000
#define uqmax 1500
#define udmax 1500

FixedangleGenerator dianjiaodu;//电角度结构体
CLARK_STRUCT clark;            //clark变换结构体

SVPWM_STRUCT SVPWM;            //SVPWM调制结构体
ST_ELEC_OBS  input_volt1;      //电压数据结构体

//A相电压PR控制器
ST_PR PR_Volt_PhaseA={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpE=0, .fpPreE=0,.fpPre_PreE= 0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .pre_fpU=0,.pre_pre_fpU= 0,
						  //注:PR输出是内环的电流指令(单位A),不是电压。实际iL峰值仅约0.2A,
						  //40这个限幅等于允许40A,形同虚设(实测0.5~40全区间结果一致),暂留不动
						  .fpUMax=40,.fpEMin= 0,
						  //omiga_0由Control_SetFundamentalFreq按Line_f1重算,这里的50只是上电初值
						  .omiga_c=1*3.1415926f*5,.omiga_0=2*3.1415926f*50,
						  //!! 量纲: PR输出是内环的电流指令(A), 故Kp/Kr量纲为 A/V;
						  //   内环Kp_i量纲为欧姆; 无量纲环路增益 = Kr[A/V] * Kp_i[ohm]。
						  //   原 Kr=30 配 Kp_i=8 -> 环路增益240, 空载必发散。
						  //空载物理需求: 19.6V相电压只需电容电流2*pi*50*12u*19.6=0.074A,
						  //PR稳态输出应在0.1A量级; Kr=30 意味着1V误差索求30A, 量级错了。
						  //环路增益不能太小: Kr*Kp_i=1 时PR几乎没有修正能力, 输出退化成
						  //纯前馈开环, 死区/母线纹波/传感器误差全部直通到输出 -> 波形变乱。
						  //修正ADC采样窗口(见hrtim.h)后重扫二维稳定区, 比之前宽得多:
						  //  Kr\Kp_i    1      2      3      4
						  //     8     0.92   0.82   0.89  振荡
						  //    16     0.98   0.74  振荡   振荡
						  //    30     1.27  振荡   振荡   振荡     (单元=空载THD%)
						  //取 Kr=8 配 Kp_i=2 -> 环路增益16, 距发散边界约2倍余量
						  .Kp=0.2f,.Kr=8.0f,
						  .fpDt=CTRL_DT};
//A相电流P控制器（内环有源阻尼，输出为电压指令V）
//滤波参数: L=2mH, C=12uF(星接) -> f0=1027Hz, 特征阻抗Z0=sqrt(L/C)=12.91欧
//fpKp量纲是欧姆, 物理含义是串进LC回路的等效阻尼电阻: zeta=Kp/(2*Z0)
//
//!! 上限受电流采样质量制约, 不是越大越好 !!
//空载时电感电流纹波(峰峰约0.25A)是基波(0.074A峰值)的数倍, ADC定点采样采到的
//不是周期平均值, 误差直接被Kp放大后进占空比。
//修正ADC采样窗口(起点3.60->0.80us, SMP 24.5->12.5, 见hrtim.h)后重扫:
//  空载 Kr=8 时 Kp_i=1/2/3 的THD为 0.92/0.82/0.89%, Kp_i=4 振荡。
//取 Kp_i=2.0: 等效阻尼电阻2欧(Z0=12.91欧, zeta=0.077), 配Kr=8得环路增益16。
//全工况(20~100Hz x 100R/500R/空载)THD 0.23~1.02%, 且Rs=0仍稳定。
//若要再往上开, 需先改中心对齐PWM让谷值采样等于纹波平均点(见hrtim.h)。
ST_PID P_Crt_PhaseA={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=2.0f,.fpKi=0,
					.fpDt=CTRL_DT};
//C相电压PR控制器
ST_PR PR_Volt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpE=0, .fpPreE=0,.fpPre_PreE= 0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .pre_fpU=0,.pre_pre_fpU= 0,
						  .fpUMax=40,.fpEMin= 0,
						  .omiga_c=1*3.1415926f*5,.omiga_0=2*3.1415926f*50,
						  .Kp=0.2f,.Kr=8.0f,  // 同A相
						  .fpDt=CTRL_DT};
//C相电流P控制器（内环有源阻尼，输出为电压指令V）
ST_PID P_Crt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=2.0f,.fpKi=0,  // 同A相
					.fpDt=CTRL_DT};


