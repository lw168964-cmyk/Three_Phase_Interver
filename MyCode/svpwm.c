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

#if DPWM_MODE == 1
    /* DPWMMIN: 三相同减最小值, 等价于把 t0 全部搬到 V0。
       共模平移不改变任何线电压(已验算 vta-vtb, vtb-vtc 与 SVPWM 逐位相同),
       但最小相占空比变成 0, 那一相在整个 60 度窗口内不再换流。
       !! 这里不能再走 duty_clip !! DUTY_MIN=0.02 会把 0 抬回 0.02(680 tick = 500ns),
       远宽于死区 206ns, 于是钳位相照旧换流 —— 纹波代价照付而收益全丢, 反而不如 SVPWM。
       钳位相的零占空比由 set_centered_pulse 单独处理, 非钳位相在那里照常限幅。 */
    {
        float vt_min = p->vta;
        if (p->vtb < vt_min) { vt_min = p->vtb; }
        if (p->vtc < vt_min) { vt_min = p->vtc; }
        p->vta -= vt_min;
        p->vtb -= vt_min;
        p->vtc -= vt_min;
    }
    update_hrtim_duty(p->vta, p->vtb, p->vtc);
#else
    //占空比只做极限保护,不能做 d*0.8+0.1 这类压缩:
    //那是绕0.5的线性压缩,共模不变而差模幅值直接×0.8,输出线电压会等比例少20%
    //死区由HRTIM硬件插入(hrtim.c DeadTimeConfig),此处无需再留余量
    p->vta = duty_clip(p->vta);
    p->vtb = duty_clip(p->vtb);
    p->vtc = duty_clip(p->vtc);

    update_hrtim_duty(p->vta, p->vtb, p->vtc);
#endif
}
/* 把占空比写成关于 PER/2 对称的居中脉冲。
   CMP1 = (PER - pulse)/2 置位, CMP2 = (PER + pulse)/2 复位。
   duty 的语义不变(仍是高电平占整周期的比例), 故上层 SVPWM 无需改动。
   限幅后 pulse in [0.02,0.98]*34000 -> CMP1 in [340,16660], CMP2 in [17340,33660],
   均满足 RM0440 对比较值 >=3 且 <PER 的要求。 */
static void set_centered_pulse(uint32_t timer_index, float duty)
{
	const uint32_t period = HRTIM_PWM_PERIOD_TICKS;

#if DPWM_MODE == 1
	/* 钳位相: 高边整周期不导通, 低边由死区发生器保持常通 -> 真正的 V0 钳位。
	   做法是把置位比较值放到 period+1: 计数器只走 0..period-1(周期边界即回零),
	   永不匹配, 故 CMP1 的置位事件不发生, 输出停在非活动态。CMP2 仍给合法值,
	   复位事件照常发生但对已是非活动态的输出无效果。
	   为什么不用 duty=0 走正常路径: pulse=0 会让 CMP1=CMP2=period/2, 同一 tick
	   同时置位与复位, 优先级依赖实现 —— 若置位赢就是高边直通, 不能赌。
	   为什么不靠"窄脉冲被死区吞掉": 那样死区发生器会产生约 559ns 的低边缺口,
	   电流转由高边体二极管续流, 损耗只是从沟道转到二极管, 并没有省掉。
	   退出钳位时写回正常比较值, 预装载保证在下一个 counter=0 边界原子生效。
	   阈值取半个 tick(1/68000), 只有上面同减最小值产生的精确 0 会落进来。 */
	if (duty < (0.5f / (float)HRTIM_PWM_PERIOD_TICKS))
	{
		hhrtim1.Instance->sTimerxRegs[timer_index].CMP1xR = period + 1U;
		hhrtim1.Instance->sTimerxRegs[timer_index].CMP2xR = period / 2U;
		return;
	}
#endif

	const uint32_t pulse = (uint32_t)(period * duty_clip(duty));
	const uint32_t half_edge = (period - pulse) / 2U;

	hhrtim1.Instance->sTimerxRegs[timer_index].CMP1xR = half_edge;
	hhrtim1.Instance->sTimerxRegs[timer_index].CMP2xR = half_edge + pulse;
}

//更新占空比
void update_hrtim_duty(float dutyA, float dutyB, float dutyC)
{
	//防御性限幅在 set_centered_pulse 内统一做:
	//占空比为负时float转uint32会回绕成极大值,导致占空比跳到满量程
	set_centered_pulse(0U, dutyA);
	set_centered_pulse(1U, dutyB);
	set_centered_pulse(2U, dutyC);
}
