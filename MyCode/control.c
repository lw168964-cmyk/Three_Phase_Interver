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
/* Keep margin for the carrier-synchronous ADC scan in the initial
 * zero-vector interval. Raising this limit requires a verified new ADC
 * sampling schedule, especially under no-load conditions. */
#define CONTROL_MODULATION_LIMIT   0.85f

float Udc = 48.0f;//直流母线电压
float Uab_rms=0;//AB线电压有效值
float Ia_rms=0;//A相线电流有效值
//目标值统一由oledui.c的Line_U1_Set(线电压有效值)给出,经调用处折算成相电压峰值传入des_d

float D=0;
uint16_t cnt=0;
static float reference_ramp = 0.0f;
static volatile uint8_t control_enabled = 0U;

/* ===== 控制环时序实测 =====
   用DWT周期计数器直接量,不靠估算。全部volatile,调试器watch窗口可直接读。
   SystemCoreClock=170MHz -> 一个20kHz周期 = 8500 cycle。
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
void Control_SetFundamentalFreq(float hz)
{
	const float w0 = 2.0f * 3.1415926f * hz;
	uint32_t primask = __get_PRIMASK();

	//系数是5个变量的一组,ISR读到"新旧混搭"会瞬时改变差分方程的极点,
	//在ζ极小的环里足以踢出一次振荡。整组更新期间屏蔽中断,耗时约1us。
	__disable_irq();
	PR_Volt_PhaseA.omiga_0 = w0;
	PR_Volt_PhaseC.omiga_0 = w0;
	PR_Init(&PR_Volt_PhaseA);
	PR_Init(&PR_Volt_PhaseC);
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

	Reset_PR_State(&PR_Volt_PhaseA);
	Reset_PR_State(&PR_Volt_PhaseC);
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

//控制环路入口:Master PERIOD与三相载波谷值重合，ADC完成四路转换后进入本回调。
//CMP preload在下一个谷值统一装载，控制始终具有一个完整PWM周期的确定延时。
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
	P_Crt_PhaseA.fpDes = PR_Volt_PhaseA.fpU;
	P_Crt_PhaseA.fpFB = input_volt1.fpPha1CrtFilt;//用滤波后电流,内环Kp可开大而不放大谐波
	PI_Controller(&P_Crt_PhaseA);
	//A相电压前馈:必须用参考值,不能用实测反馈值。
	//实测fpPhaAVoltFB本身带着LC的谐振振铃,反馈式前馈把这个振铃原封不动加回占空比,
	//等于在谐振点上套了一条正反馈通路,唯一的阻尼来源只剩负载电阻->轻载即起极限环。
	//仿真(单相理想LC,20kHz一拍延时):R=500欧时残差23.7V@1640Hz,R=1k时35.4V@1875Hz,
	//空载基波塌掉99.7%;改参考式前馈后同工况残差0.000V。与sample.h记录的实测1600Hz吻合。
	//参考值不含谐振状态,通路直接断开,与内环Kp的取值无关。
	P_Crt_PhaseA.fpU += Phase_A_ref;

	//5.C相环路控制
	PR_Volt_PhaseC.fpDes = Phase_C_ref;
	PR_Volt_PhaseC.fpFB = input_volt1.fpPhaCVoltFB;
	PR_Controller(&PR_Volt_PhaseC);
	P_Crt_PhaseC.fpDes = PR_Volt_PhaseC.fpU;
	P_Crt_PhaseC.fpFB = input_volt1.fpPha3CrtFilt;//用滤波后电流,内环Kp可开大而不放大谐波
	PI_Controller(&P_Crt_PhaseC);
	P_Crt_PhaseC.fpU += Phase_C_ref; //C相电压前馈(参考式,同A相)

	//6.AC得到B相环路输出
	float Phase_B_out = -P_Crt_PhaseA.fpU - P_Crt_PhaseC.fpU;

	//7.归一化（相电压到调制比m）
	//本SVPWM的alpha/beta即调制比m = sqrt3 * 相电压峰值 / Udc
	//(alpha幅值=1时对应相电压峰值Udc/sqrt3=27.7V,线电压有效值Udc/sqrt2=33.9V)
	//所以必须乘sqrt3,只除Udc会让实际输出只有指令的1/sqrt3
	float Phase_A_out = P_Crt_PhaseA.fpU * sqrt3 / Udc;
	Phase_B_out = Phase_B_out * sqrt3 / Udc;
	float Phase_C_out = P_Crt_PhaseC.fpU * sqrt3 / Udc;

	//调试接口:vsnprintf处理3个float在M4上要几十us,而中断周期只有100us
	//会造成控制时序抖动->直接恶化THD,调波形时再临时打开
	//printf_DMA("%f,%f,%f\n",Phase_A_out,Phase_B_out,Phase_C_out);

	//8.参数计算显示
	Uab_rms = Calculate_ACVoltage_RMS_AB(&input_volt1,f);//计算AB线电压有效值
	Ia_rms = Calculate_ACCurrent_RMS_A(&input_volt1,f);//计算A相线电流有效值

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
	
}


