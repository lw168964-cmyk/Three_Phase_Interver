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
 * 正解是改成中心对齐(UpDownMode=UPDOWN, Period 减半到 17000, ADC 触发放在谷值),
 * 谷值天然等于纹波平均点, 与占空比无关。改完才能提高内环 Kp 拿到真正的有源阻尼。
 * 注: 单纯把本值挪到周期中点(17000)在理想模型里有效, 但那一刻桥臂正在换流,
 *     实机会引入开关噪声; 中心对齐则同时满足"纹波平均"与"无噪声窗口"。 */
#define HRTIM_ADC_SAMPLE_DELAY_TICKS 2450U

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

