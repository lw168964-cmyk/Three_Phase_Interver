#include "control.h"
#include "tim.h"
#include "suanfa.h"
#include "inital.h"
#include "main.h"
#include "arm_math.h"
#include "svpwm.h"
#include "oledui.h"
#include "adc.h"
#include "hrtim.h"
#include <math.h>
#include <string.h>

#if CTRL_FREQUENCY != HRTIM_PWM_FREQUENCY_HZ
#error "The control and PWM frequencies must remain identical."
#endif

#define CONTROL_SOFTSTART_SECONDS  0.20f
/* 改居中脉冲后, 采样正确性不再依赖零矢量窗口宽度(见hrtim.h), 该限幅回归其
 * 本来的作用: 只做过调制保护。六边形内接圆对应 m=1.0, 留15%余量给瞬态。
 * m=0.85 对应线电压有效值上限 0.85*60/sqrt2 = 36.1V, 32V目标余量12.7%。
 * 注: 余量比48V/24V那一档(20%)小, 负载突变时更容易碰饱和, 碰到就靠
 * modulation_saturated 走抗积分饱和, 不会失控但会短暂掉幅。 */
#define CONTROL_MODULATION_LIMIT   0.85f

/* 直流母线电压。工程无母线采样(ADC四通道全部用于Uab/Ia/Ubc/Ic), 故这是个
   开环常数, 必须与实际母线一致。它是第7步归一化的分母:
   写小了 -> 实际输出按 Udc_real/Udc_code 倍数超调, 且环路增益同比例放大
   (48写在60上 = 增益×1.25, 8.89dB裕度掉到6.95dB);
   写大了 -> 输出不足, 慢环要靠 rms_trim 往上补。
   带载母线压降同样不被感知, 只能由慢环慢慢补。 */
float Udc = 60.0f;//直流母线电压(实测值,改硬件必须同步改这里)
float Uab_rms=0;//AB线电压有效值
float Ia_rms=0;//A相线电流基波有效值(显示用)
float Ia_rms_raw=0;//A相线电流原始RMS(含纹波,排查用)
static FUND_RMS fund_Ia;//A相电流基波解调状态
//目标值统一由oledui.c的Line_U1_Set(线电压有效值)给出,经调用处折算成相电压峰值传入des_d

float D=0;
uint16_t cnt=0;
static float reference_ramp = 0.0f;
static volatile uint8_t control_enabled = 0U;

/* ===== 有效值慢环 =====
   作用: PR只保证"瞬时波形跟踪参考", 但参考幅值到实际线电压有效值之间还串着
   死区压降、桥臂导通损耗、采样标定误差等一堆不进模型的因素。慢环直接以
   Uab_rms 为反馈, 乘性修正幅值指令, 使有效值必定收敛到 Line_U1_Set。
   为什么用乘性而非加性: 修正量应随幅值等比缩放, 换频率/换给定时无需重调增益。
   带宽必须远低于LC谐振(1131Hz)和PR谐振(基波), 否则会与它们互相作用:
   每基波周期更新一次, Ki=0.02 -> 时间常数约50个基波周期(50Hz时1.0s),
   与1131Hz差4个数量级, 完全解耦, 不影响前面算的相位裕度。 */
#define RMS_LOOP_KI        0.02f
#define RMS_TRIM_MIN       0.80f
#define RMS_TRIM_MAX       1.30f
float rms_trim = 1.0f;                 //幅值乘性修正(watch窗口可观察收敛过程)
static float prev_theta = 0.0f;        //用于检测基波周期翻转
static uint8_t modulation_saturated = 0U;  //本基波周期内调制是否饱和(抗积分饱和)

/* ===== 控制环时序实测 =====
   用DWT周期计数器直接量,不靠估算。全部volatile,调试器watch窗口可直接读。
   SystemCoreClock=170MHz -> 一个20kHz周期 = 8500 cycle。
   !! 从 10kHz 退回 20kHz 后预算减半, 这是必须复核的一项 !!
   判读:
     ctrl_cycles_max  < 8500      -> 中断跑得完,不超时
     ctrl_entry_delta_max ~= 8500 -> 触发节奏准,输出频率由晶振决定
     ctrl_missed_trig > 0         -> 有周期没进中断,输出频率会偏低(这才会造成频率不稳) */
#define CTRL_CYCLES_PER_PERIOD   (170000000UL / CTRL_FREQUENCY)

volatile uint32_t ctrl_cycles = 0;            //本次中断耗时(cycle)
volatile uint32_t ctrl_cycles_max = 0;        //历史最大耗时(cycle)
volatile uint32_t ctrl_entry_delta = 0;       //相邻两次进中断的间隔(cycle)
volatile uint32_t ctrl_entry_delta_max = 0;   //间隔最大值
volatile uint32_t ctrl_missed_trig = 0;       //间隔超过1.5个周期的次数(丢触发)
volatile uint32_t ctrl_isr_count = 0;         //进中断累计次数
volatile uint32_t adc_error_count = 0;        //ADC错误(主要是OVR)累计次数
static uint32_t ctrl_last_entry = 0;          //上次进中断的CYCCNT快照

static void Ctrl_Profile_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

//ADC溢出会让循环DMA的通道对应关系错位,必须能看见
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
	{
		adc_error_count++;
	}
}

static void Reset_PR_State(ST_PR *controller)
{
	controller->fpDes = 0.0f;
	controller->fpFB = 0.0f;
	controller->fpE = 0.0f;
	controller->fpPreE = 0.0f;
	controller->fpPre_PreE = 0.0f;
	controller->fpSumE = 0.0f;
	controller->fpU = 0.0f;
	controller->pre_fpU = 0.0f;
	controller->pre_pre_fpU = 0.0f;
}

static void Reset_PI_State(ST_PID *controller)
{
	controller->fpDes = 0.0f;
	controller->fpFB = 0.0f;
	controller->fpE = 0.0f;
	controller->fpPreE = 0.0f;
	controller->fpSumE = 0.0f;
	controller->fpU = 0.0f;
	controller->fpUp = 0.0f;
	controller->fpUi = 0.0f;
	controller->fpUd = 0.0f;
}

//PR的谐振峰必须跟着基波频率走。原先omiga_0在inital.c里被写死成2*pi*50,
//而PR_Init只在main()里调一次,面板改频率时只更新了角度发生器,PR系数从未重算。
//实测系数下|PR(z)|: 50Hz=30.2, 40Hz=6.55, 30Hz=2.83, 80Hz=3.09, 100Hz=2.02
//->偏离50Hz就退化成Kp=0.2的纯比例,30Hz轻载幅值误差-14.0%。
//(注:这条不影响输出频率稳定性,PR极点|z|=0.999215衰减时间常数63.7ms,稳态已衰完)
/* 谐波谐振器的频率上限。
   LC谐振在1131Hz, 谐振器的峰不能靠得太近: 那里对象相位已接近-180度,
   在峰上叠一个±90度相位跳变的环节会直接吃掉相位裕度。
   取800Hz(约0.71*f_LC): 5次全程可用(100Hz基波时5次=500Hz),
   7次在基波>114Hz时才关掉, 而面板上限是100Hz(7次=700Hz), 故实际全程可用。
   超限时把Kr置0再PR_Init: Kp本就是0, 于是n0/n1/n2全为0, 该谐振器输出恒为0。 */
#define HARMONIC_MAX_HZ   800.0f
/* Kr=4.8: 由差分方程精确算出的裕度定标(host端复算PR_Init所得, 非解析近似)。
   |L(w_LC)| 基波单独 0.2774(11.14dB) -> 加两个谐振器 0.3592(8.89dB)。
   实测各Kr对应: 2.4->9.94dB, 4.8->8.89dB, 6.0->8.41dB, 7.2->7.96dB(破8dB底线)。
   取4.8而非6.0: 留一档给上机后可能需要的微调, 且谐波抑制已达 |H5|=4.45。 */
#define HARMONIC_KR       4.8f

/* ===== Tustin 频率预畸变 =====
 * PR_Init 用双线性变换离散化, 它把模拟频率 w 映射到数字频率 wd:
 *     w = (2/T)*tan(wd*T/2)
 * 所以直接把 omiga_0 写成 2*pi*f, 数字谐振峰实际落在比 f 略低的地方。
 * 峰位误差 ≈ f*( (w0*T/2)^2 /3 ), 对基波无所谓, 对谐振器是致命的:
 * omiga_c=2.0 使半功率带宽只有 ±0.32Hz, 而峰位偏移 ∝ Ts^2:
 *   Ts=50us (20kHz):  7次偏 0.35Hz -> |H7|=3.21   5次偏 0.18Hz -> |H5|=4.45
 *   Ts=100us(10kHz):  7次偏 1.40Hz -> |H7|=1.05   5次偏 0.51Hz -> |H5|=2.52
 * 上面四个数与代码历史记录的实测/精算值一致, 可作模型自检。
 * 预畸变后数字峰精确落在目标频率, 两个采样率下都是 |H5|=|H7|=Kp+Kr=4.8。
 * !! 20kHz 下预畸变依然有价值 !! 它把 |H7| 从 3.21 提到 4.8, 即 7 次谐波的
 *    抑制能力提升 50%。这不是只为 10kHz 加的补丁, 退回 20kHz 后应保留。
 *
 * 对 LC 稳定裕度无副作用: 谐振器在远离峰处的残留增益 ≈ 2*Kr*wc/w, 与 omiga_0
 * 几乎无关。用同一差分方程模型复算 |L(w_LC)| (基波+5次+7次的最坏情况同相叠加):
 *     20kHz 有预畸变  0.3591 (8.89dB)   <- 当前配置
 *     10kHz 有预畸变  0.3495 (9.13dB)
 * 10kHz 略好的原因不是预畸变, 而是双线性变换把 1131Hz 映到的等效模拟频率
 * w_a 从 7182 升到 7422 rad/s, 而 PR 残留 ∝ 1/w_a。这 0.24dB 是退回 20kHz
 * 的唯一代价(仍在 8dB 底线之上), 换来控制延时减半、zeta_eff 0.0374->0.0667。
 *
 * 基波 PR 也走同一函数: 50Hz 处畸变量仅 0.001Hz(带宽 ±0.95Hz), 可忽略,
 * 统一处理只是为了不留第二套公式。 */
static float PR_PrewarpOmega(float hz, float dt)
{
	//tanf 参数 = pi*f*T; 最高用到 800Hz@20kHz -> 0.126 rad, 远离 pi/2, 安全
	//(10kHz 时 0.251 rad, 同样安全 —— 该函数对两个采样率都成立)
	return (2.0f / dt) * tanf(3.1415926f * hz * dt);
}

//按基波频率重算一个谐波谐振器。超出安全频率则令其失效并清状态。
static void Harmonic_Retune(ST_PR *pr, float hz, uint8_t order)
{
	const float fh = hz * (float)order;

	pr->omiga_0 = PR_PrewarpOmega(fh, pr->fpDt);
	if (fh > HARMONIC_MAX_HZ)
	{
		pr->Kr = 0.0f;
		//残留状态若不清, 失效瞬间递推项还会衰减着往外吐一段输出
		Reset_PR_State(pr);
	}
	else
	{
		pr->Kr = HARMONIC_KR;
	}
	PR_Init(pr);
}

void Control_SetFundamentalFreq(float hz)
{
	const float w0 = PR_PrewarpOmega(hz, PR_Volt_PhaseA.fpDt);
	uint32_t primask = __get_PRIMASK();

	//系数是5个变量的一组,ISR读到"新旧混搭"会瞬时改变差分方程的极点,
	//在ζ极小的环里足以踢出一次振荡。整组更新期间屏蔽中断,耗时约1us。
	//谐波谐振器同理, 且必须与基波在同一临界区内完成, 否则会出现
	//基波已按新频率、谐波仍按旧频率的中间态。
	__disable_irq();
	PR_Volt_PhaseA.omiga_0 = w0;
	PR_Volt_PhaseC.omiga_0 = w0;
	PR_Init(&PR_Volt_PhaseA);
	PR_Init(&PR_Volt_PhaseC);
	Harmonic_Retune(&PR_H5_PhaseA, hz, 5U);
	Harmonic_Retune(&PR_H7_PhaseA, hz, 7U);
	Harmonic_Retune(&PR_H5_PhaseC, hz, 5U);
	Harmonic_Retune(&PR_H7_PhaseC, hz, 7U);
	if (primask == 0U)
	{
		__enable_irq();
	}
}

void Control_Reset(void)
{
	control_enabled = 0U;
	D = 0.0f;
	cnt = 0U;
	reference_ramp = 0.0f;

	//时序统计随每次启动清零,这样读到的数就只属于本次运行
	Ctrl_Profile_Init();
	ctrl_cycles = 0U;
	ctrl_cycles_max = 0U;
	ctrl_entry_delta = 0U;
	ctrl_entry_delta_max = 0U;
	ctrl_missed_trig = 0U;
	ctrl_isr_count = 0U;   //置0使下一次中断跳过delta计算,避免首拍算出巨大间隔
	adc_error_count = 0U;
	dianjiaodu.theta = 0.0f;
	Uab_rms = 0.0f;
	Ia_rms = 0.0f;
	Ia_rms_raw = 0.0f;
	memset(&fund_Ia, 0, sizeof(fund_Ia));

	//慢环状态随每次启动复位,否则上次运行(可能是别的给定/负载)的修正量会
	//在本次启动瞬间直接作用到幅值上,形成一次阶跃
	rms_trim = 1.0f;
	prev_theta = 0.0f;
	modulation_saturated = 0U;

	Reset_PR_State(&PR_Volt_PhaseA);
	Reset_PR_State(&PR_Volt_PhaseC);
	//谐波谐振器状态同样要清:时间常数0.5s,上次运行(可能是别的负载/给定)
	//积累的谐波修正量会在本次启动后持续半秒作用到输出上
	Reset_PR_State(&PR_H5_PhaseA);
	Reset_PR_State(&PR_H7_PhaseA);
	Reset_PR_State(&PR_H5_PhaseC);
	Reset_PR_State(&PR_H7_PhaseC);
	Reset_PI_State(&P_Crt_PhaseA);
	Reset_PI_State(&P_Crt_PhaseC);
	memset(&input_volt1, 0, sizeof(input_volt1));
	Clark_Init(&clark);
	my_svpwm_Init(&SVPWM);
}

void Control_Enable(void)
{
	control_enabled = 1U;
}

void Control_Disable(void)
{
	control_enabled = 0U;
}

//控制环路入口:四路ADC扫描居中于对称点counter=0,居中脉冲下该点即电感电流纹波的
//周期平均点(推导见hrtim.h),扫描结束后进入本回调。
//CMP preload在下一个counter=0统一装载,控制始终具有一个完整PWM周期的确定延时。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	 if((hadc == &hadc1) && (control_enabled != 0U))
	 {
		 const uint32_t t_entry = DWT->CYCCNT;

		 //触发节奏:相邻两次进中断的间隔。这一项直接决定输出频率是否准。
		 if (ctrl_isr_count != 0U)
		 {
			 //CYCCNT为32位自由回绕,无符号相减在回绕时依然正确
			 ctrl_entry_delta = t_entry - ctrl_last_entry;
			 if (ctrl_entry_delta > ctrl_entry_delta_max)
			 {
				 ctrl_entry_delta_max = ctrl_entry_delta;
			 }
			 if (ctrl_entry_delta > (CTRL_CYCLES_PER_PERIOD + CTRL_CYCLES_PER_PERIOD / 2U))
			 {
				 ctrl_missed_trig++;
			 }
		 }
		 ctrl_last_entry = t_entry;
		 ctrl_isr_count++;

		 D=fixed_angle_update(&dianjiaodu);

		 Volt_Loop_Control(Line_U1_Set*1.414f/1.732f,0,D,dianjiaodu.Fo);

		 //本次中断耗时。放在最后,包含整条控制链但不含HAL的DMA分发开销。
		 ctrl_cycles = DWT->CYCCNT - t_entry;
		 if (ctrl_cycles > ctrl_cycles_max)
		 {
			 ctrl_cycles_max = ctrl_cycles;
		 }
	 }
}

void Volt_Loop_Control(float des_d,float des_q,float sita,uint16_t f) //单电压环控制
{
	const float ramp_step = CTRL_DT / CONTROL_SOFTSTART_SECONDS;
	const float modulation_limit_sq = CONTROL_MODULATION_LIMIT * CONTROL_MODULATION_LIMIT;
	(void)des_q;

	if (reference_ramp < 1.0f)
	{
		reference_ramp += ramp_step;
		if (reference_ramp > 1.0f)
		{
			reference_ramp = 1.0f;
		}
	}
	des_d *= reference_ramp;
	//慢环修正: 软启动完成前不施加(此时有效值窗口还没稳定,反馈无意义)
	if (reference_ramp >= 1.0f)
	{
		des_d *= rms_trim;
	}

	//1.采样线电压AB，BC，线电流A，B
	Cal_ACVolt_AB(&input_volt1);//AB线电压瞬时值采样
	Cal_ACVolt_BC(&input_volt1);//BC线电压瞬时值采样
	Cal_ACCurrent_A(&input_volt1);//A相线电流瞬时值采样
	Cal_ACCurrent_C(&input_volt1);//C相线电流瞬时值采样

	//2.还原三路相电压
	Calculate_PhaseVoltage(&input_volt1);//计算A相B相C相电压

	//3.生成三路相电压参考（A-B-C相序）
	//des_d由调用处传入,已是相电压峰值: Line_U1_Set(线电压有效值) * 1.414 / 1.732
	//直接用形参,面板上改Line_U1_Set才能真正生效
	float Phase_A_ref = des_d * arm_sin_f32(sita);
	// float Phase_B_ref = des_d * arm_sin_f32(sita-2.0f*3.1415926f/3);
	float Phase_C_ref = des_d * arm_sin_f32(sita + 2.0f*3.1415926f/3);

	//4.A相环路控制（PR电压外环+P电流内环）
//	printf_DMA("%f ,%f\n",input_volt1.fpABVolt,input_volt1.fpPha1CrtFB);调试接口
	PR_Volt_PhaseA.fpDes = Phase_A_ref;
	PR_Volt_PhaseA.fpFB = input_volt1.fpPhaAVoltFB;
	PR_Controller(&PR_Volt_PhaseA);
	//5/7次谐振器吃同一个电压误差(参考只含基波,故误差里天然带着谐波),
	//各自只对自己那一根谱线有增益,输出并入同一个电流指令节点,
	//这样谐波修正量同样经过内环的有源阻尼。
	PR_H5_PhaseA.fpDes = Phase_A_ref;
	PR_H5_PhaseA.fpFB = input_volt1.fpPhaAVoltFB;
	PR_Controller(&PR_H5_PhaseA);
	PR_H7_PhaseA.fpDes = Phase_A_ref;
	PR_H7_PhaseA.fpFB = input_volt1.fpPhaAVoltFB;
	PR_Controller(&PR_H7_PhaseA);
	P_Crt_PhaseA.fpDes = PR_Volt_PhaseA.fpU + PR_H5_PhaseA.fpU + PR_H7_PhaseA.fpU;
	P_Crt_PhaseA.fpFB = input_volt1.fpPha1CrtFilt;//用滤波后电流,内环Kp可开大而不放大谐波
	PI_Controller(&P_Crt_PhaseA);
	//A相电压前馈:必须用参考值,不能用实测反馈值。
	//实测fpPhaAVoltFB本身带着LC的谐振振铃,反馈式前馈把这个振铃原封不动加回占空比,
	//等于在谐振点上套了一条正反馈通路,唯一的阻尼来源只剩负载电阻->轻载即起极限环。
	//参考值不含谐振状态,通路直接断开。
	//注:切断这条正反馈是必要条件但不是充分条件——环路总增益还必须落在稳定范围内。
	//无量纲环路增益 = PR的Kr[A/V] * 内环Kp[ohm], 原 30*8=240 空载必发散,
	//现取 10*2.2=22(基波处)。空载稳定不由这个数决定, 而由PR在LC谐振点的
	//高频残留增益决定(|PR(jw_LC)|*Z0 < 1), 详见inital.c的量纲与判据说明。
	P_Crt_PhaseA.fpU += Phase_A_ref;

	//5.C相环路控制
	PR_Volt_PhaseC.fpDes = Phase_C_ref;
	PR_Volt_PhaseC.fpFB = input_volt1.fpPhaCVoltFB;
	PR_Controller(&PR_Volt_PhaseC);
	PR_H5_PhaseC.fpDes = Phase_C_ref;   //同A相
	PR_H5_PhaseC.fpFB = input_volt1.fpPhaCVoltFB;
	PR_Controller(&PR_H5_PhaseC);
	PR_H7_PhaseC.fpDes = Phase_C_ref;
	PR_H7_PhaseC.fpFB = input_volt1.fpPhaCVoltFB;
	PR_Controller(&PR_H7_PhaseC);
	P_Crt_PhaseC.fpDes = PR_Volt_PhaseC.fpU + PR_H5_PhaseC.fpU + PR_H7_PhaseC.fpU;
	P_Crt_PhaseC.fpFB = input_volt1.fpPha3CrtFilt;//用滤波后电流,内环Kp可开大而不放大谐波
	PI_Controller(&P_Crt_PhaseC);
	P_Crt_PhaseC.fpU += Phase_C_ref; //C相电压前馈(参考式,同A相)

	//6.AC得到B相环路输出
	float Phase_B_out = -P_Crt_PhaseA.fpU - P_Crt_PhaseC.fpU;

	//7.归一化（相电压到调制比m）
	//本SVPWM的alpha/beta即调制比m = sqrt3 * 相电压峰值 / Udc
	//(alpha幅值=1时对应相电压峰值Udc/sqrt3=34.6V,线电压有效值Udc/sqrt2=42.4V)
	//所以必须乘sqrt3,只除Udc会让实际输出只有指令的1/sqrt3
	float Phase_A_out = P_Crt_PhaseA.fpU * sqrt3 / Udc;
	Phase_B_out = Phase_B_out * sqrt3 / Udc;
	float Phase_C_out = P_Crt_PhaseC.fpU * sqrt3 / Udc;

	//调试接口:vsnprintf处理3个float在M4上要几十us,而中断周期只有100us
	//会造成控制时序抖动->直接恶化THD,调波形时再临时打开
	//printf_DMA("%f,%f,%f\n",Phase_A_out,Phase_B_out,Phase_C_out);

	//8.参数计算显示
	Uab_rms = Calculate_ACVoltage_RMS_AB(&input_volt1,f);//计算AB线电压有效值
	//电流显示必须用基波提取, 不能用原始RMS:
	//采样的是电感电流, 空载纹波峰峰约0.31A >> 基波峰值0.081A(C=9.9uF,32V线电压),
	//原始RMS把纹波也算进去 -> 空载会显示成几百mA以上, 那不是负载电流。
	Ia_rms = Fundamental_RMS_Update(&fund_Ia, input_volt1.fpPha1CrtFB, sita, f);
	//原始RMS保留为排查用(调试器watch), 与Ia_rms的差值即纹波+噪声含量
	Ia_rms_raw = Calculate_ACCurrent_RMS_A(&input_volt1,f);

	//9.闭环调制
	Clark_Func(&clark, Phase_A_out, Phase_B_out, Phase_C_out, 1);
	float modulation_sq = clark.alpha * clark.alpha + clark.beta * clark.beta;
	if (modulation_sq > modulation_limit_sq)
	{
		float scale = CONTROL_MODULATION_LIMIT / sqrtf(modulation_sq);
		clark.alpha *= scale;
		clark.beta *= scale;
	}
	my_svpwm_calc(&SVPWM,clark.alpha,clark.beta);//调制同时改变占空比

	//10.有效值慢环:每个基波周期更新一次
	//每周期只更新一次的理由: Uab_rms本身就是整周期RMS, 一个周期内它是常量,
	//在周期中间反复用同一个值去积分等于把增益放大了(采样率/f)倍, 会失稳。
	if (modulation_sq > modulation_limit_sq)
	{
		//调制已饱和, 再往上修正没有物理意义, 记录下来供本周期末抗饱和用
		modulation_saturated = 1U;
	}

	//检测基波周期翻转(theta被fixed_angle_update归一化时会回绕)
	if (sita < prev_theta)
	{
		if ((reference_ramp >= 1.0f) && (Line_U1_Set > 0.1f))
		{
			//归一化误差:用相对量,使增益与给定值无关
			const float err = (Line_U1_Set - Uab_rms) / Line_U1_Set;
			//抗积分饱和:调制已到上限时禁止继续往上积分(向下修正仍允许)
			if ((modulation_saturated == 0U) || (err < 0.0f))
			{
				rms_trim += RMS_LOOP_KI * err;
				rms_trim = my_clip(rms_trim, RMS_TRIM_MIN, RMS_TRIM_MAX);
			}
		}
		modulation_saturated = 0U;
	}
	prev_theta = sita;
}


