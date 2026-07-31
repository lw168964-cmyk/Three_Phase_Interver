/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim.h
  * @brief   This file contains all the function prototypes for
  *          the hrtim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HRTIM_H__
#define __HRTIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern HRTIM_HandleTypeDef hhrtim1;

/* USER CODE BEGIN Private defines */

/* HRTIM runs from 170 MHz x4 = 680 MHz (CKPSC = MUL4, 见 hrtim.c)。
 *
 * ===== 为什么从 10kHz 退回 20kHz =====
 * 10kHz 曾用于降开关损耗, 但按损耗模型复算, 那笔账基本是平的:
 *   只有两项随 f 变 —— 开关损耗 ∝ f, 磁芯损耗 ∝ f^(-1.1)
 *   (Steinmetz P ∝ f^1.4 * B^2.5, 而 B ∝ 纹波 ∝ 1/f, 故 f^1.4 * f^-2.5)
 *   10k->20k: 开关多约 0.43W, 磁损少 0.533*P_core(10k)
 *   盈亏平衡在 P_core(10k)=0.81W; 13~20kHz 区间总损耗变化 <0.04W(极值很平)
 * 即效率上两者相差约 0.1W(0.09个效率点), 仪表测不出来。真正的理由是控制:
 *   控制延时 1.5Ts    150us -> 75us
 *   f_LC 处延时相移   61.1deg -> 30.5deg
 *   zeta_eff          0.0374 -> 0.0667  (Kp_i=2.2 不必改, 见 inital.c)
 *   反阻尼边界1/(6Ts) 1667Hz -> 3333Hz, 距 f_LC=1131Hz 从 1.47x 拉到 2.95x
 * 唯一代价: |L(w_LC)| 从 0.3495(9.13dB) 回到 0.3591(8.89dB) —— 双线性变换把
 * 1131Hz 映到的等效模拟频率 w_a 从 7422 降到 7182 rad/s, 而 PR 残留 ∝ 1/w_a。
 * 仍在 8dB 自设底线之上。
 * 40kHz 已排除: 总损耗约 1.90W(vs 20k 的 1.31W), ISR 预算只剩 4250 cycle,
 * 且 RMS_SAMPLE_COUNT 需翻到 2000(RAM 16.5->28.5kB)。
 *
 * !! 预分频必须与频率同改 !! 10kHz @ 680MHz 需要 PER=68000, 超过 PER 上限
 *    0xFFDF(65503) —— 这是当初降频时必须同时降预分频的原因。20kHz 无此约束。
 * 巧合但省事: MUL2/MUL4 下 PER 都是 34000, 所有按 tick 写的数值(CMP限幅范围等)
 * 不变, 只是每 tick 从 2.94ns 回到 1.47ns。 */
#define HRTIM_PWM_FREQUENCY_HZ       20000U
#define HRTIM_PWM_TIMER_CLOCK_HZ     680000000UL
#define HRTIM_PWM_PERIOD_TICKS       \
    (HRTIM_PWM_TIMER_CLOCK_HZ / HRTIM_PWM_FREQUENCY_HZ)

/* PER/CMP 寄存器是16位, 且 RM0440 规定 PER 最大 0x0000FFDF。
   这个守卫直接拦住"只改频率、忘了改预分频"的组合。 */
#if HRTIM_PWM_PERIOD_TICKS > 0xFFDFUL
#error "PWM period exceeds the HRTIM PER limit (0xFFDF): lower PrescalerRatio."
#endif

/* ===== 中心对齐(居中脉冲)调制 =====
 * 计数器仍是单向上数(UpDownMode=UP), 但脉冲用两个比较单元做成关于 PER/2 对称:
 *     CMP1 = (PER - pulse)/2   -> 上升沿 (SetSource = TIMCMP1)
 *     CMP2 = (PER + pulse)/2   -> 下降沿 (ResetSource = TIMCMP2)
 * 于是 000 零矢量落在周期两端, 111 落在中间, 波形对 counter=0 与 counter=PER/2
 * 两点严格时间对称。
 *
 * 为什么这能解决采样问题(这是本次改动的全部理由):
 *   设 di/dt = f(t) 关于对称点 t0 为偶函数, 则 i(t0+tau) + i(t0-tau) = 2*i(t0),
 *   两侧积分即得   (1/T) * integral(i dt) = i(t0)
 *   -> 在对称点采样, 采样值精确等于电感电流的周期平均值,
 *      且与占空比、零矢量窗口宽度无关。
 * 原左对齐配置(MASTERPER置位/CMP1复位)没有这个对称性, 采样点落在前沿111窗口
 * 起点附近, 采到的是纹波边缘: 实测与真实周期平均误差 RMS 约 0.15A, 而空载基波
 * 电流峰值只有 2*pi*50*9.9uF*26.1V = 0.081A —— 误差比信号还大。该误差被内环 Kp
 * 成比例放大进占空比, 逼得 Kp 只能取小值 (现 2.2 欧, 而 Z0=sqrt(L/C)=14.21欧,
 * zeta=0.077), 空载失去负载阻尼后即振荡。
 * 注: C 由 12uF 改 9.9uF 使基波电流变小(0.074->0.061A@24V), 之后母线48->60V、
 *     输出24->32V 又把它拉回 0.081A。两次改动叠加后纹波/信号比 3.8, 略优于
 *     C=12uF/24V 时的 4.1 —— 但绝对误差随 di/dt 上升(见下), 对称点采样依旧关键。
 *
 * 附带收益: 旧配置那张"四通道扫描装不进零矢量窗口"的表格(m=0.85 时窗口仅
 * 3.75us)不再适用 —— 对称点采样的正确性不依赖采样点是否处于零矢量内。
 *
 * 选择本方案而非 UPDOWN 中心对齐的原因: 二者数学上等价, 但本方案只用到
 * up-count + TIMCMP 置位/复位 + 现有死区发生器, 全是已在运行的机制;
 * UPDOWN 需要重推 GTCMP1 的 set/reset 方向语义, 配错的后果是桥臂直通。 */
#define HRTIM_PWM_HALF_PERIOD_TICKS  (HRTIM_PWM_PERIOD_TICKS / 2U)

/* ===== ADC 触发点 =====
 * 目标: 让四通道扫描"居中"于对称点 counter=0 (周期边界, 两端000窗口的中心)。
 * 四通道扫描耗时 = 4*(12.5+12.5)/42.5MHz = 2.353us = 1600 tick @680MHz,
 * 故提前半个扫描长度触发: PER - 800。
 *   rank1(Uab) 采样于 -0.80us, rank4(Ic) 采样于 +0.88us, 平均偏移约 0
 * 单通道相对对称点的最大偏移 ~1.18us, 残余纹波误差 = (di/dt)*offset,
 * 空载 di/dt=v_c/L=26.1/2mH=13060A/s -> 约 15mA, 远小于原方案的 0.15A。
 * (24V档时 9800A/s -> 11mA; 误差与信号同比例增长, 占基波峰值恒为约14%)
 * !! 该误差只取决于扫描的绝对时长, 与开关频率无关 -> 10kHz 退回 20kHz 后不变,
 *    尽管电感纹波本身减半(0.62->0.31A峰峰)。这是对称点采样的核心好处:
 *    改频率不需要重新评估采样误差, 只有 ADC 采样时间和通道数会影响它。
 *
 * 选 counter=0 而非 counter=PER/2 的原因: CMP 预装载也在 counter=0 生效,
 * 于是 ISR 从扫描结束到下一个装载边界仍有接近一整个 PWM 周期(20kHz下约48.8us)
 * 的预算 —— 不缩减 ctrl_cycles 的时间余量。
 *
 * 采样时间保持 12.5 周期(不用 24.5): 对称点采样的误差 ∝ 扫描时长, 24.5 周期会把
 * 单通道最大偏移从 1.18us 推到 1.74us, 误差 15mA->23mA。12.5 周期对应源阻抗上限
 * 约 6.5k欧, 现有分压网络/传感器满足即可。(10kHz 时零矢量窗口宽到能装 24.5 周期,
 * 但窗口够用不等于该用更长的扫描 —— 20kHz 下窗口本就不够, 结论不变。)
 *
 * 触发点 = PER - 800 = 33200 tick (= 48.82us, 即对称点前 1.18us)。 */
#define HRTIM_ADC_SCAN_TICKS         1600U
#define HRTIM_ADC_SAMPLE_DELAY_TICKS \
    (HRTIM_PWM_PERIOD_TICKS - (HRTIM_ADC_SCAN_TICKS / 2U))

#if HRTIM_ADC_SAMPLE_DELAY_TICKS >= HRTIM_PWM_PERIOD_TICKS
#error "The ADC sample delay must stay inside one PWM period."
#endif

/* ===== 互补输出(死区发生器)配置 =====
 * TA1/TB1/TC1 由交叉开关驱动(CMP1置位/CMP2复位), TA2/TB2/TC2 的交叉开关置空,
 * 由死区发生器以"输出1的发生器信号"为参考自动生成反相波 + 双边死区。
 * 这是 RM0440 28.3.4 的标准接法, DTEN=1 时输出2的交叉开关本就被忽略。
 *
 * !! POL2 必须按驱动板的低边输入极性刻意选择 !!
 * 原先 TA2/TB2/TC2 是复用 TA1 的 pOutputCfg 结构体配出来的, POL2 跟着 POL1
 * 一起等于 HIGH —— 那是"继承"而不是"选择"。若低边驱动是低有效(输入低电平
 * 导通), POL2=HIGH 会让低边管在整个周期里保持关断: 现象正是"CHx2 没有波形"
 * 或"CHx2 与 CHx1 同相"。
 *   0 -> POL2 = HIGH (低边驱动高有效, 与高边一致) —— 绝大多数半桥驱动如此
 *   1 -> POL2 = LOW  (低边驱动低有效)
 * 改这个宏只影响 CHx2 的电平语义, 不影响死区时长, 也不影响 CHx1。 */
#define HRTIM_COMP_OUTPUT_ACTIVE_LOW   0

/* 上电/重启时把 OUTxR、DTxR、OENR 抄一份到全局, 供调试器 watch 或串口打印。
   目的: 把"配置到底进没进寄存器"从推理变成读数。期望值见 hrtim.c 的实现处。 */
typedef struct
{
    uint32_t outar, outbr, outcr;   /* 期望 0x00000100 (仅DTEN=1, POL全高) */
    uint32_t dtar,  dtbr,  dtcr;    /* 期望 0x01180118 (DTR=DTF=280, DTPRSC=0) */
    uint32_t oenr;                  /* WaveformOutputStart 之后期望 0x0000003F */
    uint32_t mcr;                   /* 计数器启动后 MCEN|TACEN|TBCEN|TCCEN 应置位 */
} HRTIM_RegSnapshot;

extern volatile HRTIM_RegSnapshot hrtim_regs;

void HRTIM_CaptureRegs(void);

/* USER CODE END Private defines */

void MX_HRTIM1_Init(void);

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __HRTIM_H__ */

