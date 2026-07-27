#include "suanfa.h"
#include "arm_math.h"
float b=0;
/*结构体初始化：1.结构体类型 2.基波频率 3.采样频率*/
void fixed_angle_init(FixedangleGenerator *angle_gen,float Hz,float sample_rate)   
{
	 angle_gen->theta=0;
	 angle_gen->Ts=1.0f/sample_rate;
	 angle_gen->w=2*3.1415926f*Hz;
	 angle_gen->Fo=Hz;
}

/*更新结构体，并返回该结构体*/
float fixed_angle_update(FixedangleGenerator *angle_gen)
{
	//更新电角度
	angle_gen->theta+= angle_gen->w*angle_gen->Ts;
	
	//角度归一化
	if(angle_gen->theta>=2*3.1415926f)
	{
		angle_gen->theta-=2*3.1415926f ;
	}
	return angle_gen->theta ;
}
/*更新结构体*/
void fixed_angle_update_1(FixedangleGenerator *angle_gen)
{
	//更新电角度
	angle_gen->theta+= angle_gen->w*angle_gen->Ts;
	
	//角度归一化
	if(angle_gen->theta>=2*PI)
	{
		angle_gen->theta-=2*PI ;
	}
}

float my_clip(float fpValue, float fpMin, float fpMax)    //输出钳位
{
	if (fpValue <= fpMin)
	{
		return fpMin;
	}
	else if (fpValue >= fpMax)
	{
		return fpMax;
	}
	else
	{
		return fpValue;
	}
}

//PI控制器
void PI_Controller(ST_PID *pstPid)
{
	pstPid->fpE = pstPid->fpDes - pstPid->fpFB; //计算当前偏差	
	pstPid->fpSumE += pstPid->fpE; //计算偏差累积
	pstPid->fpUi = pstPid->fpKi * pstPid->fpDt * pstPid->fpSumE;
	pstPid->fpUp = my_clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);//Kp限幅
	pstPid->fpU = pstPid->fpUp + pstPid->fpUi;
	pstPid->fpPreE = pstPid->fpE; //保存本次偏差

	/*输出限幅*/
	pstPid->fpU = my_clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

//克拉克变换结构体初始化
void Clark_Init(CLARK_STRUCT *p)
{
    p->ua = 0;
    p->ub = 0;
    p->uc = 0;
    p->alpha = 0;
    p->beta = 0;
    p->u0 = 0;
}
//克拉克变换
void Clark_Func(CLARK_STRUCT *p, float ua, float ub, float uc, int sele)
{
    //计算对应坐标值
    const float alpha_base = 0.6666667f*(ua - 0.5f * ub - 0.5f * uc);
    const float beta_base = 0.6666667f*(ub - uc) * SQRT3_OVER_2;

		//A和alpha重合
    if (sele == 1)
    {
        p->alpha = alpha_base;
        p->beta = beta_base;
    }
    else
    {
        p->alpha = -beta_base; 
        p->beta = alpha_base;
    }
}
//反克拉克变换
/* alpha beta 0 to abc */
void iClark_Func(CLARK_STRUCT *p, float alpha, float beta, float u0, int sele)
{
    if (sele == 1)
    {
        /* A & alpha */
        p->ua = (alpha + u0);
        p->ub = (-0.5f * alpha + 0.866f * beta + u0);
        p->uc = (-0.5f * alpha - 0.866f * beta + u0);
    }
    else
    {
        /* A & alpha - 90 */
        p->ua = (beta + u0);
        p->ub = (-0.866f * alpha - 0.5f * beta + u0);
        p->uc = (0.866f * alpha - 0.5f * beta + u0);
    }
}
//park变换结构体初始化
void Park_Init(PARK_STRUCT *p)
{
    p->alpha = 0;
    p->beta = 0;
    p->theta = 0;
    p->ud = 0;
    p->uq = 0;
}

//park变换
/* alpha beta 0 to dq0 */
void Park_Func(PARK_STRUCT *p, float alpha, float beta, float theta, int park_sele)
{

    float p_cos = arm_cos_f32(theta);
    float p_sin = arm_sin_f32(theta);
    if (park_sele == 1)
    {
        /* cos */
        p->ud = p_cos * alpha + p_sin * beta;
        p->uq = p_cos * beta - p_sin * alpha;
    }
    else
    {
        /* sin */
        p->ud = p_sin * alpha - p_cos * beta;
        p->uq = p_cos * alpha + p_sin * beta;
    }
}

//反park变换
/* dq0 to alpha beta 0 */
void iPark_Func(PARK_STRUCT *p, float ud, float uq, float theta, int park_sele)
{
    float p_cos = arm_cos_f32(theta);
    float p_sin = arm_sin_f32(theta);
    if (park_sele == 1)
    {
        /* cos */
        p->alpha = p_cos * ud - p_sin * uq;
        p->beta = p_sin * ud + p_cos * uq;
    }
    else
    {
        /* sin */
        p->alpha = p_sin * ud + p_cos * uq;
        p->beta = -p_cos * ud + p_sin * uq;
    }
}

//abc to dq 结构体初始化
void abc_dq0_Init(ABC_DQ0_STRUCT *p)
{
    p->ua = 0;
    p->ub = 0;
    p->uc = 0;
    p->ud = 0;
    p->uq = 0;
    p->u0 = 0;
    p->theta = 0;
}

/* abc to dq0 */
void abcTodq0_Func(ABC_DQ0_STRUCT *p, float ua, float ub, float uc, float theta, int sele)
{
    float p_sin0 = arm_sin_f32(theta);
    float p_sin1 = arm_sin_f32(theta - 2.0944f);
    float p_sin2 = arm_sin_f32(theta + 2.0944f);
    float p_cos0 = arm_cos_f32(theta);
    float p_cos1 = arm_cos_f32(theta - 2.0944f);
    float p_cos2 = arm_cos_f32(theta + 2.0944f);

    if (sele == 1)
    {
        /* sin */
        p->ud = 0.6667f * (p_sin0 * ua + p_sin1 * ub + p_sin2 * uc);
        p->uq = 0.6667f * (p_cos0 * ua + p_cos1 * ub + p_cos2 * uc);
        p->u0 = 0.3333f * (ua + ub + uc);
    }
    else
    {
        /* cos */
        p->ud = 0.6667f * (p_cos0 * ua + p_cos1 * ub + p_cos2 * uc);
        p->uq = -0.6667f * (p_sin0 * ua + p_sin1 * ub + p_sin2 * uc);
        p->u0 = 0.3333f * (ua + ub + uc);
    }
}

/* dq0 to abc */
void dq0Toabc_Func(ABC_DQ0_STRUCT *p, float ud, float uq, float u0, float theta, int sele)
{
    float p_sin0 = arm_sin_f32(theta);
    float p_sin1 = arm_sin_f32(theta - 2.0944f);
    float p_sin2 = arm_sin_f32(theta + 2.0944f);
    float p_cos0 = arm_cos_f32(theta);
    float p_cos1 = arm_cos_f32(theta - 2.0944f);
    float p_cos2 = arm_cos_f32(theta + 2.0944f);

    if (sele == 1)
    {
        /* sin */
        p->ua = (p_cos0 * ud - p_sin0 * uq + u0);
        p->ub = (p_cos1 * ud - p_sin1 * uq + u0);
        p->uc = (p_cos2 * ud - p_sin2 * uq + u0);
    }
    else
    {  
			  /* cos */
			  p->ua = (p_sin0 * ud + p_cos0 * uq + u0);
        p->ub = (p_sin1 * ud + p_cos1 * uq + u0);
        p->uc = (p_sin2 * ud + p_cos2 * uq + u0);
        
    }
}

/**
 * @brief PR控制器初始化
* @param pstPid: PR控制器结构体指针
 */
void PR_Init(ST_PR *pstPid)
{
	pstPid->n0=((4+4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt)*pstPid->Kp+4*pstPid->Kr*pstPid->fpDt*pstPid->omiga_c)/(4+4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt);
	pstPid->n1=pstPid->Kp*(2*pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt-8)/(4+4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt);
	pstPid->n2=((4-4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt)*pstPid->Kp-4*pstPid->Kr*pstPid->fpDt*pstPid->omiga_c)/(4+4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt);
	pstPid->d1=(2*pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt-8)/(4+4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt);
	pstPid->d2=(4-4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt)/(4+4*pstPid->fpDt*pstPid->omiga_c+pstPid->omiga_0*pstPid->omiga_0*pstPid->fpDt*pstPid->fpDt);
}

/**
 * @brief PR控制器
* @param pstPid: PR控制器结构体指针
 */
void PR_Controller(ST_PR *pstPid)
{
	pstPid->fpE = pstPid->fpDes - pstPid->fpFB;
	pstPid->fpU=pstPid->n0*pstPid->fpE+pstPid->n1*pstPid->fpPreE+pstPid->n2*pstPid->fpPre_PreE-pstPid->d1*pstPid->pre_fpU-pstPid->d2*pstPid->pre_pre_fpU;

	// 输出限幅，防止控制器输出过大
	pstPid->fpU = my_clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);

	pstPid->fpPre_PreE=pstPid->fpPreE;//迭代结果
	pstPid->fpPreE = pstPid->fpE; //保存本次偏差

	pstPid->pre_pre_fpU=pstPid->pre_fpU;
	pstPid->pre_fpU=pstPid->fpU;
}