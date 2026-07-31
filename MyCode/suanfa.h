#ifndef __SUANFA_H
#define __SUANFA_H

#define SQRT3_OVER_2 0.86602540378f // v3/2

#include "arm_math.h"

typedef float fp32;
typedef double fp64;

/* One control update per 10 kHz PWM carrier.
   必须等于 HRTIM_PWM_FREQUENCY_HZ (control.c 顶部 #error 守卫)。
   CTRL_DT 由此推出, PR/PI 系数在 PR_Init 里按 fpDt 自动重算。 */
#define CTRL_FREQUENCY 10000U
#define CTRL_DT (fp32)(1.0f / CTRL_FREQUENCY)//控制周期

/*PID控制器结构体*/
typedef struct
{
	fp32 fpDes; //控制变量目标值
	fp32 fpFB;	//控制变量反馈值

	fp32 fpKp; //比例系数Kp
	fp32 fpKi; //积分系数Ki
	fp32 fpKd; //微分系数Kd

	fp32 fpUp; //比例输出
	fp32 fpUi; //积分输出
	fp32 fpUd; //微分输出

	fp32 fpE;	 //本次偏差
	fp32 fpPreE; //上次偏差
	fp32 fpSumE; //总偏差
	fp32 fpU;	 //本次PID运算结果

	fp32 fpUMax;	//PID运算后输出最大值及做遇限削弱时的上限值
	fp32 fpEpMax;	//比例项输出最大值
	fp32 fpEiMax_p; //积分项输出最大值//positive > 0
	fp32 fpEiMax_n; //积分项输出最大值//negative < 0
	fp32 fpEdMax;	//微分项输出最大值
	fp32 fpEMin;	//积分上限

	fp32 fpUiMax;

	fp32 fpDt; //控制周期
}ST_PID;

//电角度结构体
typedef struct   		
{
	
	float theta;   		//电角度
	float w;       		//角频率
	float Ts;      		//采样时间
	uint16_t Fo;			//目标频率

} FixedangleGenerator ;


//克拉克变换结构体
typedef struct
{
    float ua;
    float ub;
    float uc;
    float alpha;
    float beta;
    float u0;
} CLARK_STRUCT;

//park变换结构体
typedef struct
{
    float alpha;
    float beta;
    float ud;
    float uq;
    float theta;
} PARK_STRUCT;

//abc to dq结构体
typedef struct
{
    float ua;
    float ub;
    float uc;
    float ud;
    float uq;
    float u0;
    float theta;
} ABC_DQ0_STRUCT;

/*PR控制器结构体*/
typedef struct
{
	float fpDes; //控制变量目标值
	float fpFB;	//控制变量反馈值
  

	float fpE;	 //本次偏差
	float fpPreE; //上次偏差
	float fpPre_PreE;//上上次偏差
	
	float fpSumE; //总偏差
	
	float fpU;	 //本次PID运算结果
    float pre_fpU;//上次的输出
	float pre_pre_fpU;//上上次的输出
	
	float fpUMax;	//PID运算后输出最大值及做遇限削弱时的上限值
	
	float fpEMin;	//积分上限

    float omiga_c;
	float omiga_0;
	float Kp;
	float Kr;
	float fpDt; //控制周期
	
    float n0;
	float n1;
	float n2;
	float d1;
	float d2;
	
}ST_PR;


/*结构体初始化：1.结构体类型 2.基波频率 3.采样频率*/
void fixed_angle_init(FixedangleGenerator *angle_gen,float Hz,float sample_rate);
/*更新结构体，并返回该结构体*/
float fixed_angle_update(FixedangleGenerator *angle_gen);
/*更新结构体*/
void fixed_angle_update_1(FixedangleGenerator *angle_gen);
//输出限幅
float my_clip(float fpValue, float fpMin, float fpMax);
//PI控制器
void PI_Controller(ST_PID *pstPid);
//克拉克结构体初始化
void Clark_Init(CLARK_STRUCT *p);
//克拉克变换
void Clark_Func(CLARK_STRUCT *p, float ua, float ub, float uc, int sele);
//反克拉克变换
void iClark_Func(CLARK_STRUCT *p, float alpha, float beta, float u0, int sele);
//park变换结构体初始化
void Park_Init(PARK_STRUCT *p);
//park变换
/* alpha beta 0 to dq0 */
void Park_Func(PARK_STRUCT *p, float alpha, float beta, float theta, int park_sele);
//反park变换
/* dq0 to alpha beta 0 */
void iPark_Func(PARK_STRUCT *p, float ud, float uq, float theta, int park_sele);
//abc to dq 结构体初始化
void abc_dq0_Init(ABC_DQ0_STRUCT *p);
/* abc to dq0 */
void abcTodq0_Func(ABC_DQ0_STRUCT *p, float ua, float ub, float uc, float theta, int sele);
/* dq0 to abc */
void dq0Toabc_Func(ABC_DQ0_STRUCT *p, float ud, float uq, float u0, float theta, int sele);	
/*PR参数初始化*/
void PR_Init(ST_PR *pstPid);
/*PR控制器*/
void PR_Controller(ST_PR *pstPid);
#endif
