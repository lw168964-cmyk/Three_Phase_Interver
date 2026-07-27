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
						  //Kr决定谐振峰高度(|PR|约=Kr)。稳态误差已由参考式前馈直通解决,
						  //实测Kr=30时幅值误差-0.03%,不需要再往上加;Kr>=100会破坏稳定(实测振荡)
						  .Kp=0.2f,.Kr=30,
						  .fpDt=CTRL_DT};
//A相电流P控制器（内环有源阻尼，输出为电压指令V）
//滤波参数: L=2mH, C=12uF(星接) -> f0=1027Hz, 特征阻抗Z0=sqrt(L/C)=12.91欧
//Kp的量纲是欧姆,物理含义是串进LC回路的等效阻尼电阻: ζ=Kp/(2*Z0)
//  Kp=1.0 -> 内环带宽仅80Hz, ζ=0.039, 远低于f0=1027Hz, 根本来不及提供阻尼
//           带载时靠负载电阻凑阻尼还能稳,空载阻尼消失->1100Hz极限环,
//           基波被振荡吃掉只剩11.3V(-42%),这同时是"空载不稳"和"电压达不到目标"的共同原因
//  Kp=8.0 -> 带宽637Hz, ζ=0.31; 实测稳定边界在3~4之间, 取8留约2倍余量
//空载最苛刻工况扫描: Kp=3.0残差83.8V(振荡), Kp>=4.0残差0.000V
//20~100Hz x 100R/500R/空载 全工况: 幅值误差<=0.03%, 无限幅, iL峰值<0.25A
ST_PID P_Crt_PhaseA={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=8.0f,.fpKi=0,
					.fpDt=CTRL_DT};
//C相电压PR控制器
ST_PR PR_Volt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpE=0, .fpPreE=0,.fpPre_PreE= 0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .pre_fpU=0,.pre_pre_fpU= 0,
						  .fpUMax=40,.fpEMin= 0,
						  .omiga_c=1*3.1415926f*5,.omiga_0=2*3.1415926f*50,
						  .Kp=0.2f,.Kr=30,  // 同A相
						  .fpDt=CTRL_DT};
//C相电流P控制器（内环有源阻尼，输出为电压指令V）
ST_PID P_Crt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=8.0f,.fpKi=0,  // 同A相
					.fpDt=CTRL_DT};


