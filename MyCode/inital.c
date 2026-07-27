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
						  //开关级模型(12bit量化+真实纹波)二维扫描: Kr<=1.5可稳, Kr=1.0残差最小
						  .Kp=0.2f,.Kr=1.0f,
						  .fpDt=CTRL_DT};
//A相电流P控制器（内环有源阻尼，输出为电压指令V）
//滤波参数: L=2mH, C=12uF(星接) -> f0=1027Hz, 特征阻抗Z0=sqrt(L/C)=12.91欧
//fpKp量纲是欧姆, 物理含义是串进LC回路的等效阻尼电阻: zeta=Kp/(2*Z0)
//
//!! 不要靠加大这个Kp来压空载振荡 !! 曾按平均值模型把它改到8.0, 实机与开关级
//   模型都证明更差。原因: 空载时电感电流纹波(峰峰约0.25A)是基波(0.074A峰值)的
//   数倍, 而ADC在3.60us定点采样, 采到的是纹波边缘而非周期平均, 采样误差
//   RMS约0.15A。Kp越大, 这个误差被放大得越多直接进占空比。
//   开关级实测(空载, Rs=0.3欧): Kp=1残差0.49V; Kp=2以上全部振荡(>18V)。
//   真正的瓶颈是采样点不在纹波平均点(见hrtim.h的HRTIM_ADC_SAMPLE_DELAY_TICKS),
//   左对齐PWM没有"载波谷值"可用。改中心对齐后才能提高Kp获得真正的有源阻尼。
ST_PID P_Crt_PhaseA={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=1.0f,.fpKi=0,
					.fpDt=CTRL_DT};
//C相电压PR控制器
ST_PR PR_Volt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpE=0, .fpPreE=0,.fpPre_PreE= 0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .pre_fpU=0,.pre_pre_fpU= 0,
						  .fpUMax=40,.fpEMin= 0,
						  .omiga_c=1*3.1415926f*5,.omiga_0=2*3.1415926f*50,
						  .Kp=0.2f,.Kr=1.0f,  // 同A相
						  .fpDt=CTRL_DT};
//C相电流P控制器（内环有源阻尼，输出为电压指令V）
ST_PID P_Crt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=1.0f,.fpKi=0,  // 同A相
					.fpDt=CTRL_DT};


