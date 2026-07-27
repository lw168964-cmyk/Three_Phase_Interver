#include "control.h"
#include "tim.h"
#include "suanfa.h"
#include "inital.h"
#include "main.h"
#include "arm_math.h"
#include "svpwm.h"
#include "oledui.h"
#include "adc.h"

float Udc = 48.0f;//直流母线电压
float Uab_rms=0;//AB线电压有效值
float Ia_rms=0;//A相线电流有效值
//目标值统一由oledui.c的Line_U1_Set(线电压有效值)给出,经调用处折算成相电压峰值传入des_d

float D=0;
uint16_t cnt=0;
//控制环路入口:由ADC-DMA转换完成触发(ADC采样点已锁在载波波峰,见hrtim.c/adc.c)。
//HRTIM波峰每2个载波周期(100us)触发一次ADC->4路转换->DMA完成->本回调,即10kHz控制节拍。
//原TIM6中断链路已停用:采样与载波异步会把开关纹波混叠进反馈,现改为采完即算,采控同步。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	 if(hadc==&hadc1)
	 {
		 D=fixed_angle_update(&dianjiaodu);

		 Volt_Loop_Control(Line_U1_Set*1.414f/1.732f,0,D,dianjiaodu.Fo);
	 }
}

void Volt_Loop_Control(float des_d,float des_q,float sita,uint16_t f) //单电压环控制
{
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
	//A相电压前馈
	//注:换成参考值前馈(Phase_A_ref)会把闭环谐振点从0.71*w0抬到1.22*w0
	//(特征方程 s^2LC+K=0 变成 s^2LC+1+K=0),谐振频率越高控制延时的相位滞后越大,
	//在当前ζ≈0.007的欠阻尼条件下不利,所以维持反馈式前馈。
	//零点漂移由Cal_ACVolt_AB里的慢速零点跟踪扣除,不再经这条通道进入占空比。
	P_Crt_PhaseA.fpU += input_volt1.fpPhaAVoltFB;

	//5.C相环路控制
	PR_Volt_PhaseC.fpDes = Phase_C_ref;
	PR_Volt_PhaseC.fpFB = input_volt1.fpPhaCVoltFB;
	PR_Controller(&PR_Volt_PhaseC);
	P_Crt_PhaseC.fpDes = PR_Volt_PhaseC.fpU;
	P_Crt_PhaseC.fpFB = input_volt1.fpPha3CrtFilt;//用滤波后电流,内环Kp可开大而不放大谐波
	PI_Controller(&P_Crt_PhaseC);
	P_Crt_PhaseC.fpU += input_volt1.fpPhaCVoltFB; //C相电压前馈(同A相)

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

	//9.调制
	//开环接口(调试用,幅值恒为满调制m=1,即线电压33.9V)
	Clark_Func(&clark , 0.8*arm_sin_f32(sita) , 0.8*arm_sin_f32(sita-2.0f*3.1415926f/3), 0.8*arm_sin_f32(sita+2.0f*3.1415926f/3) ,1);

	//闭环调制
	// Clark_Func(&clark, Phase_A_out, Phase_B_out, Phase_C_out, 1);
	my_svpwm_calc(&SVPWM,clark.alpha,clark.beta);//调制同时改变占空比
	
}


