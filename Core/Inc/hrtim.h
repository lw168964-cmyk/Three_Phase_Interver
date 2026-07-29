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

/* HRTIM runs from 170 MHz x4 = 680 MHz. */
#define HRTIM_PWM_FREQUENCY_HZ       20000U
#define HRTIM_PWM_TIMER_CLOCK_HZ     680000000UL
#define HRTIM_PWM_PERIOD_TICKS       \
    (HRTIM_PWM_TIMER_CLOCK_HZ / HRTIM_PWM_FREQUENCY_HZ)

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
 * 空载 di/dt=v_c/L=26.1/2mH=13060A/s -> 约 11.5mA, 远小于原方案的 0.15A。
 * (24V档时 9800A/s -> 8.6mA; 误差与信号同比例增长, 占基波峰值恒为约14%)
 *
 * 选 counter=0 而非 counter=PER/2 的原因: CMP 预装载也在 counter=0 生效,
 * 于是 ISR 从扫描结束到下一个装载边界仍有接近一整个 PWM 周期(约48.8us)的预算,
 * 与改动前一致 —— 不缩减 ctrl_cycles 的时间余量。
 *
 * 触发点 = PER - 800 = 33200 tick (= 48.82us, 即对称点前 1.18us)。 */
#define HRTIM_ADC_SCAN_TICKS         1600U
#define HRTIM_ADC_SAMPLE_DELAY_TICKS \
    (HRTIM_PWM_PERIOD_TICKS - (HRTIM_ADC_SCAN_TICKS / 2U))

#if HRTIM_ADC_SAMPLE_DELAY_TICKS >= HRTIM_PWM_PERIOD_TICKS
#error "The ADC sample delay must stay inside one PWM period."
#endif

/* USER CODE END Private defines */

void MX_HRTIM1_Init(void);

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __HRTIM_H__ */

