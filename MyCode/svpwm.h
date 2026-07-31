#ifndef __SVPWM_H
#define __SVPWM_H

#include "main.h"

/* ===== 调制方式 =====
 *   0 -> SVPWM   零矢量对称均分 (t0/2 在 V0, t0/2 在 V7)
 *   1 -> DPWMMIN 零矢量全给 V0, 每周期最小占空比相钳位到 0 (不再开关)
 *
 * 两者的线电压完全相同 —— 零序在三相三线里被消去, 改的只是共模。
 * 已验算: 扇区法与 min-max 注入在 m=0.7542, theta=15 度处三位小数一致,
 * DPWMMIN 只是把 min-max 的 v_zs 从 -(max+min)/2 换成 -0.5-min。
 *
 * 为什么选 V0-only 而不是标准 DPWM1(交替 V0/V7):
 *   (1) ADC 采样。V7 钳位时最大相占空比到 1.0, counter=0 处的 000 窗口宽度归零,
 *       四通道扫描落在另两相的换流边沿上 —— 正是居中脉冲改造要消除的问题。
 *       V0 钳位相反: 全部 t0 集中到 counter=0, 窗口从 t0/2 翻倍到 t0, 采样更稳。
 *   (2) 栅极驱动。V7 钳位要求高边连续导通 60 度 = 3.33ms = 67 个 PWM 周期,
 *       自举供电会把 bootstrap 电容耗光。V0 钳位是低边常通, 正是自举充电的时刻,
 *       两种驱动拓扑下都安全, 无需先确认驱动板类型。
 * 代价: 钳位窗口对准负峰而非正负峰, 损耗加权的开关次数削减
 *       从 DPWM1 的 50.0% 降到 43.3% (∫|cos| 2.0/4.0 -> 1.732/4.0), 即 87% 的收益。
 *
 * 全周期扫角验证结果(m=0.7542, 逐度):
 *   线电压与 SVPWM 的最大偏差 1.11e-16 (双精度机器精度) —— 共模平移完全消去
 *   钳位分布 A/B/C 各约 120 度连续窗口, 对准各自负峰, 合计 360 度无空缺
 *   theta=0/120/240 度处两相同时为零(该角参考矢量正落在某个有效矢量方向上,
 *     另两相本就不该导通), 属边界正确行为
 *   最大占空比 0.7542 —— 任何一相的高边都不会被连续保持, 自举供电无风险
 *   counter=0 处 000 窗口: SVPWM 2089 tick -> DPWMMIN 4177 tick, ADC 余量翻倍
 *
 * !! 净效率是赚是亏取决于磁损, 必须上机实测 !!
 *   省: 开关+反向恢复+栅极 约 0.846W 的 43.3% = 0.366W
 *   亏: 纹波涨约 1.2~1.4 倍, 磁损 ∝ dI^2.5 涨 1.58~2.32 倍
 *   净正收益要求 P_core(20kHz) < 约 0.28~0.63W
 * 测法: 同一负载点(32V/2A线), 改这个宏重编译, 比较输入功率。
 *       THD 预计 0.5% -> 0.55~0.65% (LC 在 20kHz 衰减 313 倍, 边带基本被滤掉)。 */
#define DPWM_MODE   1

//SVPWM结构体
typedef struct
{
    int N;
    int na;
    int nb;
    int nc;

    float alpha;
    float beta;

    float ts;
    float t1;
    float t2;
    float t0;

    float vta;
    float vtb;
    float vtc;
} SVPWM_STRUCT;

//SVPWM结构体初始化
void my_svpwm_Init(SVPWM_STRUCT *p);
//SVPWM调制
void my_svpwm_calc(SVPWM_STRUCT *p, float alpha, float beta);
//更新占空比
void update_hrtim_duty(float dutyA, float dutyB, float dutyC);

#endif
