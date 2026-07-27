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
						  .fpUMax=40,.fpEMin= 0,  // 48V母线，相电压峰值约28V，限幅40V留余量
						  .omiga_c=1*3.1415926f*5,.omiga_0=2*3.1415926f*50,  // 带宽收窄到5Hz,谐振峰变尖,50Hz处增益大增,消除稳态误差冲24V
						  //Kp是宽带增益,同时决定外环对LC谐振峰的抑制量,不要往下调
						  .Kp=0.2f,.Kr=30,
						  .fpDt=CTRL_DT};
//A相电流P控制器（内环有源阻尼，输出为电压指令V）
ST_PID P_Crt_PhaseA={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					//Kp是LC有源阻尼系数: ζ=(Kp/2)*sqrt(C/L)*Re[F(jw_res)]
					//当前ζ≈0.007(严重欠阻尼),只能往上调不能往下调
					.fpKp=1.0f,.fpKi=0,
					.fpDt=CTRL_DT};
//C相电压PR控制器
ST_PR PR_Volt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpE=0, .fpPreE=0,.fpPre_PreE= 0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .pre_fpU=0,.pre_pre_fpU= 0,
						  .fpUMax=40,.fpEMin= 0,  // 48V母线，相电压峰值约28V，限幅40V留余量
						  .omiga_c=1*3.1415926f*5,.omiga_0=2*3.1415926f*50,  // 带宽收窄到5Hz,谐振峰变尖,50Hz处增益大增,消除稳态误差冲24V
						  .Kp=0.2f,.Kr=30,  // 同A相
						  .fpDt=CTRL_DT};
//C相电流P控制器（内环有源阻尼，输出为电压指令V）
ST_PID P_Crt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=1.0f,.fpKi=0,  // 同A相
					.fpDt=CTRL_DT};


