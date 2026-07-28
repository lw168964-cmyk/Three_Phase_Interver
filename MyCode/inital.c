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
						  //omiga_c 15.7->6.0: 谐振峰宽度, 同时决定PR在高频的残留增益
						  //(远离谐振时 |R| ≈ Kr*2*omiga_c/omega)。见下方稳定判据。
						  //代价是谐振峰变窄(时间常数1/omiga_c=0.17s), 但基波频率是由开环角度
						  //发生器精确给出的、且改频率时系数会重算, 峰窄不影响跟踪。
						  .omiga_c=6.0f,.omiga_0=2*3.1415926f*50,
						  //!! 量纲: PR输出是内环的电流指令(A), 故Kp/Kr量纲为 A/V;
						  //   内环Kp_i量纲为欧姆; 无量纲环路增益 = Kr[A/V] * Kp_i[ohm]。
						  //   原 Kr=30 配 Kp_i=8 -> 环路增益240, 空载必发散。
						  //空载物理需求: 19.6V相电压只需电容电流2*pi*50*12u*19.6=0.074A,
						  //PR稳态输出应在0.1A量级; Kr=30 意味着1V误差索求30A, 量级错了。
						  //环路增益不能太小: Kr*Kp_i=1 时PR几乎没有修正能力, 输出退化成
						  //纯前馈开环, 死区/母线纹波/传感器误差全部直通到输出 -> 波形变乱。
						  //===== 空载稳定判据(这是Kp从0.2降到0.01的唯一原因) =====
						  //闭合内环后, 外环环路增益在LC谐振点(1-w^2*L*C=0)处为:
						  //  |L(w_LC)| = |PR| * Kp_i / (w_LC*C*Kp_i) = |PR| * Z0
						  //Kp_i被精确约掉 -> 内环阻尼增益对谐振点的|L|贡献恒为零。
						  //(这就是为什么Kp_i从2加到4对空载不稳完全无效; 且谐振以上
						  // |L|~|PR|*Kp_i/(w^2*L*C), Kp_i越大穿越点越往高频跑, 延时相位更差)
						  //故稳定条件只能靠PR的高频增益:
						  //      |PR(j*w_LC)| < 1/Z0 = 1/12.91 = 0.0775 A/V
						  //旧值 Kp=0.2, omiga_c=15.7 -> |PR|=0.2+0.039=0.239 -> |L|=3.09,
						  //穿越点被推到1250~1400Hz, 对象相位-154度叠加75us延时-38度
						  //= -192度 -> 相位裕度为负, 空载必振(带载时负载电阻压低谐振峰才稳)。
						  //现值 Kp=0.01, omiga_c=6 -> |PR|=0.01+0.020=0.030 -> |L|max约0.32,
						  //即约10dB增益裕度, LC区不再穿越, 穿越只发生在基波附近(相位良好)。
						  //Kr保持8: PR在基波处增益=Kp+Kr, 与高频增益解耦,
						  //所以降Kp不牺牲基波跟踪能力。
						  .Kp=0.01f,.Kr=10.0f,
						  .fpDt=CTRL_DT};
//A相电流P控制器（内环有源阻尼，输出为电压指令V）
//滤波参数: L=2mH, C=12uF(星接) -> f0=1027Hz, 特征阻抗Z0=sqrt(L/C)=12.91欧
//fpKp量纲是欧姆, 物理含义是串进LC回路的等效阻尼电阻: zeta=Kp/(2*Z0)
//
//本项是"虚拟阻尼/有源阻尼": 用实测电感电流构造等效串联电阻, 给LC加阻尼。
//取 Kp_i=4.0 -> 等效阻尼电阻4欧, zeta=Kp/(2*Z0)=0.155。
//
//!! 必须清楚它能做什么、不能做什么 !!
//能: 压低LC谐振峰的物理Q值, 抑制谐振点的能量积累。
//不能: 单靠它保证空载稳定。原因见PR控制器处的推导——外环在谐振点的环路增益
//      |L(w_LC)| = |PR|*Z0, Kp_i被精确约掉。实测已验证: Kp_i从2改到4,
//      空载依旧振荡(仅振荡频率从约1250Hz升到约1400Hz)。
//      且谐振以上|L|随Kp_i增大, 盲目加大反而恶化相位裕度。
//空载稳定由PR的高频增益负责(那才是穿越点所在), 二者分工不同, 不能互相替代。
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
						  .omiga_c=6.0f,.omiga_0=2*3.1415926f*50,
						  .Kp=0.01f,.Kr=10.0f,  // 同A相
						  .fpDt=CTRL_DT};
//C相电流P控制器（内环有源阻尼，输出为电压指令V）
ST_PID P_Crt_PhaseC={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
					.fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,
					.fpUMax=30,.fpEpMax=30,.fpEMin= 0,  // 电压指令限幅30V
					.fpKp=2.0f,.fpKi=0,  // 同A相
					.fpDt=CTRL_DT};


