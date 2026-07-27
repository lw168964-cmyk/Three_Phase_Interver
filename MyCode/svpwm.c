#include "svpwm.h"
#include "hrtim.h"
#include <math.h>

//占空比保护限幅(仅防极限,不做幅值压缩)
#define DUTY_MIN 0.02f
#define DUTY_MAX 0.98f

static float duty_clip(float d)
{
    if (!isfinite(d)) return 0.5f;
    if (d < DUTY_MIN) return DUTY_MIN;
    if (d > DUTY_MAX) return DUTY_MAX;
    return d;
}

void my_svpwm_Init(SVPWM_STRUCT *p)
{
    p->N = 0;
    p->na = 0;
    p->nb = 0;
    p->nc = 0;
    p->alpha = 0;
    p->beta = 0;
    p->ts = 0;
    p->t1 = 0;
    p->t2 = 0;
    p->t0 = 0;
    p->vta = 0;
    p->vtb = 0;
    p->vtc = 0;
}

void my_svpwm_calc(SVPWM_STRUCT *p, float alpha, float beta)
{
    p->alpha = alpha;
    p->beta = beta;

    if (p->beta > 0)
        p->na = 0;
    else
        p->na = 4;
    if (p->beta - 1.7321f * p->alpha > 0)
        p->nb = 0;
    else
        p->nb = 2;
    if (p->beta + 1.7321f * p->alpha > 0)
        p->nc = 0;
    else
        p->nc = 1;

    switch (p->na + p->nb + p->nc)
    {
    case 0:
        p->N = 2;
        break;
    case 1:
        p->N = 3;
        break;
    case 2:
        p->N = 1;
        break;
    case 3:
        p->N = 0;
        break;
    case 4:
        p->N = 0;
        break;
    case 5:
        p->N = 4;
        break;
    case 6:
        p->N = 6;
        break;
    case 7:
        p->N = 5;
        break;
    }

    float tem_alp = p->alpha * 0.866f;
    float tem_bet = p->beta * 0.5f;

    switch (p->N)
    {
    case 1:
        p->t1 = tem_alp - tem_bet;
        p->t2 = p->beta;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 2:
        p->t1 = tem_alp + tem_bet;
        p->t2 = -tem_alp + tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 3:
        p->t1 = p->beta;
        p->t2 = -tem_alp - tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 4:
        p->t1 = -tem_alp + tem_bet;
        p->t2 = -p->beta;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 5:
        p->t1 = -tem_alp - tem_bet;
        p->t2 = tem_alp - tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 6:
        p->t1 = -p->beta;
        p->t2 = tem_alp + tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    }

    //过调制处理:t1+t2>1时按比例缩到六边形边界(保幅角不变),保证t0>=0
    //否则t0为负会让占空比越界,update_hrtim_duty里转uint32时会回绕成满占空
    float t_sum = p->t1 + p->t2;
    if (t_sum > 1.0f)
    {
        p->t1 /= t_sum;
        p->t2 /= t_sum;
    }
    p->t0 = 1.0f - p->t1 - p->t2;

    switch (p->N)
    {
    case 1:
        p->vta = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtb = p->t0 * 0.5f + p->t2;
        p->vtc = p->t0 * 0.5f;
        break;
    case 2:
        p->vta = p->t0 * 0.5f + p->t1;
        p->vtb = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtc = p->t0 * 0.5f;
        break;
    case 3:
        p->vta = p->t0 * 0.5f;
        p->vtb = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtc = p->t0 * 0.5f + p->t2;
        break;
    case 4:
        p->vta = p->t0 * 0.5f;
        p->vtb = p->t0 * 0.5f + p->t1;
        p->vtc = p->t0 * 0.5f + p->t1 + p->t2;
        break;
    case 5:
        p->vta = p->t0 * 0.5f + p->t2;
        p->vtb = p->t0 * 0.5f;
        p->vtc = p->t0 * 0.5f + p->t1 + p->t2;
        break;
    case 6:
        p->vta = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtb = p->t0 * 0.5f;
        p->vtc = p->t0 * 0.5f + p->t1;
        break;
    }

    //占空比只做极限保护,不能做 d*0.8+0.1 这类压缩:
    //那是绕0.5的线性压缩,共模不变而差模幅值直接×0.8,输出线电压会等比例少20%
    //死区由HRTIM硬件插入(hrtim.c DeadTimeConfig),此处无需再留余量
    p->vta = duty_clip(p->vta);
    p->vtb = duty_clip(p->vtb);
    p->vtc = duty_clip(p->vtc);

    update_hrtim_duty(p->vta, p->vtb, p->vtc);
}
//更新占空比
void update_hrtim_duty(float dutyA, float dutyB, float dutyC)
{

  const uint32_t period = HRTIM_PWM_PERIOD_TICKS;

	//防御性限幅:占空比为负时float转uint32会回绕成极大值,导致占空比跳到满量程
	dutyA = duty_clip(dutyA);
	dutyB = duty_clip(dutyB);
	dutyC = duty_clip(dutyC);

	hhrtim1.Instance->sTimerxRegs[0].CMP1xR =(uint32_t)(period * dutyA);
	hhrtim1.Instance->sTimerxRegs[1].CMP1xR =(uint32_t)(period * dutyB);
	hhrtim1.Instance->sTimerxRegs[2].CMP1xR =(uint32_t)(period * dutyC);


}
