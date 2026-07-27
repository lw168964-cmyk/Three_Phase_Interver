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

/* ADC 采样时刻 = 主定时器计数到此值。2450 tick 约 3.60 us。
 *
 * !! 已知缺陷: 这里不是电感电流纹波的平均点 !!
 * 本配置是左对齐(前沿)PWM: 输出在 MASTERPER 置位、各相 CMP1 复位, 计数器只
 * 单向上数, 因此并不存在"载波谷值"。3.60us 落在周期开头的 111 零矢量内, 采到的
 * 是纹波的边缘而非周期平均值。开关级实测(空载 m=0.707): 采样值与周期真实平均值
 * 的误差 RMS 约 0.15 A, 而空载基波电流峰值只有 0.074 A —— 误差比信号还大。
 * 后果: 内环 Kp 一旦加大, 这个误差被成比例放大进占空比, 空载即振荡。
 * 实测不同采样点在空载下能稳定的内环 Kp 上限(Rs=0.3欧):
 *     3.60us  -> Kp<=1   (且 Rs<=0.1 欧时连 Kp=1 也不稳, 靠寄生阻尼吊着)
 *     25.0us  -> Kp 可到 4, 且 Rs=0 仍稳
 *
 * !! 更严重的问题: 四通道扫描根本装不进零矢量窗口 !!
 * 单通道 24.5+12.5=37 个 ADC 周期 @42.5MHz = 0.871us, 四通道合计 3.482us。
 * 从 3.60us 起扫, 需要前沿 111 窗口 >= 3.60+3.48 = 7.08us。而 111 窗口宽度
 * = t0/2 = (1-m)/2*T, 实测:
 *     m=0.500 -> 12.50us  (够)
 *     m=0.707 ->  7.33us  (刚够, 余量0.25us)
 *     m=0.800 ->  5.00us  (不够, 尾部2.08us在换流区)
 *     m=0.850 ->  3.75us  (不够, 尾部3.33us在换流区)
 * 默认 Line_U1_Set=24V 对应 m=0.707, 正好压在临界线上; 电压再往上调或
 * 任何瞬态抬高 m, rank2~rank4 就在桥臂换流过程中采样 -> 波形失真、THD 恶化。
 *
 * 正解: 改中心对齐。计数器上下往复(UpDownMode=UPDOWN), Period 取单向值 17000,
 * 脉冲居中, 零矢量被谷值一分为二且在谷值两侧连续, 因此:
 *   - 谷值处可用连续窗口 = 完整的 t0/2 (与左对齐同宽), 但采样点在窗口正中,
 *     而不是像现在从窗口起点开始往后扫
 *   - 谷值天然是纹波平均点, 与占空比无关
 * 开关级仿真(中心对齐+谷值采样, 死区206ns): 采样与真实周期平均的误差 RMS
 * 从 0.15A 降到 0.003A; 空载 Kp=4 时 THD 0.66%, 且 Rs=0 仍稳定
 * (左对齐在 Rs<=0.1 时已经不稳) —— 即真正脱离了对寄生阻尼的依赖。
 *
 * 改动要点(需示波器验证, 配错会导致桥臂直通):
 *   1. pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN (A/B/C)
 *   2. Period = 17000 (= 时钟/频率/2)
 *   3. RM0440 27.3.13: "In the fixed frequency configuration, the period event
 *      must trigger the output set and the 'greater than' compare triggers the
 *      output reset (or vice versa)" -> 需配合 GreaterCMP1 = HRTIM_TIMERGTCMP1_GREATER,
 *      不能沿用现在的 MASTERPER置位/CMP1复位(那是左对齐的配法)
 *   4. ADC 触发改用从定时器的谷值事件 HRTIM_ADCTRIGGEREVENT13_TIMERA_RESET
 *      (主定时器不支持 UPDOWN, 若主周期设 17000 会一个开关周期触发两次)
 *      注: UPDOWN 下"计数器复位事件"的定义受 ROM[1:0] 控制, 需查 RM0440 27.5.49
 *   5. update_hrtim_duty 的 CMP1 映射要跟着改: GREATER 模式下占空比与 CMP1
 *      的关系不再是 d = CMP1/PERIOD, 需按实际 set/reset 语义重推
 *   6. 已知风险: 社区有"UPDOWN 模式 PWM 随机翻转极性"的报告, 与 set/reset
 *      方向语义有关, 上电先用小占空比+假负载验证 */
/* 545 tick = 0.80us。取值依据(与 adc.c 的 SMP=12.5 是一组改动):
 *   可行区间下限 = 死区 206ns (须晚于死区, 避开换流瞬态)
 *   可行区间上限 = m=0.85 窗口3.75us - 四通道扫描2.353us = 1.40us
 *   取区间中点 0.80us
 * 各调制比下扫描结束时刻均为 3.15us, 余量:
 *   m=0.50 窗口12.50us 余9.35us | m=0.707 窗口7.33us 余4.17us
 *   m=0.80 窗口 5.00us 余1.85us | m=0.850 窗口3.75us 余0.60us  */
#define HRTIM_ADC_SAMPLE_DELAY_TICKS 545U

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

