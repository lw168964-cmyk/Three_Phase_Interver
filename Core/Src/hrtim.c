/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim.c
  * @brief   This file provides code for the configuration
  *          of the HRTIM instances.
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
/* Includes ------------------------------------------------------------------*/
#include "hrtim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

HRTIM_HandleTypeDef hhrtim1;

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  /* The master period is one complete 10 kHz PWM cycle (100 us). */
  /* MUL2 -> fHRCK = 170MHz*2 = 340MHz, PER = 34000。
     !! 必须是 MUL2, 不能是 MUL4 !!: MUL4(680MHz) 下 10kHz 需要 PER=68000,
     超出 PER 寄存器上限 0xFFDF。降开关频率与降预分频是一组不可分的改动,
     hrtim.h 里有 #error 守卫。pTimeBaseCfg 被 MASTER/A/B/C 复用, 改这一处即全生效。 */
  pTimeBaseCfg.Period = HRTIM_PWM_PERIOD_TICKS;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL2;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }

  /* A/B/C use one up-counting 10 kHz period. CMP1 is the direct duty value. */
  pTimeBaseCfg.Period = HRTIM_PWM_PERIOD_TICKS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  /* CMP preload values written by the ADC-DMA ISR are committed at the next
     10 kHz period boundary, never during the active PWM period.
     !! 一个周期只提交一次 !! 这就是"10kHz开关 + 20kHz双点采样"方案不成立的原因:
     两次ISR会争同一个 counter=0 装载边界, 后一次覆盖前一次, 一半控制量被丢弃。
     控制周期必须与开关周期一致 (control.c 顶部有 #error 守卫)。 */
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  /* All timers are software-reset then started together. Do not reset a slave
     at the same instant as its own period event: that would make event order
     dependent and is unnecessary because they share the same HRTIM clock. */
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* 上电中性态: pulse = PER/2 (50%占空), 居中 -> CMP1=PER/4, CMP2=3*PER/4 */
  pCompareCfg.CompareValue = HRTIM_PWM_PERIOD_TICKS / 4U;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (HRTIM_PWM_PERIOD_TICKS * 3U) / 4U;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* 死区已示波器实测约206ns/边, 与 280 tick @ tDTG=0.735ns 一致
     (即本配置下 fDTG = 1.36GHz)。
     !! 降开关频率不需要改这里 !! RM0440: tDTG = 2^DTPRSC * (tHRTIM/8), 只与
     fHRTIM(170MHz) 和 DTPRSC 有关, 与计数预分频 CKPSC 无关。故 MUL4->MUL2 后
     死区仍是 280*0.735ns = 205.8ns。(这条错了是桥臂直通, 上机请示波器复核一次)
     占一个10kHz周期的0.206%/边, 对应基波压降约0.2V —— 比20kHz时减半,
     仍由有效值慢环兜住, 无需额外死区补偿。 */
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV8;
  pDeadTimeCfg.RisingValue = 280;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 280;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  /* 居中脉冲: 每相在自己的 CMP1 置位、CMP2 复位, 二者关于 PER/2 对称
     (CMP1 = (PER-pulse)/2, CMP2 = (PER+pulse)/2, 见 update_hrtim_duty)。
     000 零矢量因此落在周期两端、111 落在中间, 波形对 counter=0 严格对称,
     使 counter=0 处的采样值等于电感电流的周期平均值(推导见 hrtim.h)。
     注意不能再用 MASTERPER 置位: 那会把上升沿钉在周期边界, 即左对齐。 */
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* ===== 互补输出 TA2/TB2/TC2 =====
     交叉开关置空: DTEN=1 时输出2由死区发生器从"输出1的发生器信号"导出,
     其交叉开关被硬件忽略(RM0440 28.3.4), 置空是正确做法, 不是遗漏。
     !! 极性必须刻意选, 不能靠复用 TA1 的结构体继承 !! 见 hrtim.h 的
     HRTIM_COMP_OUTPUT_ACTIVE_LOW: 若低边驱动低有效而这里留 HIGH,
     低边管全程关断 —— 现象就是"CHx2 无波形/与CHx1同相"。
     其余字段(IdleMode/IdleLevel/FaultLevel/Chopper)显式重述一遍,
     使这段不再依赖上面 TA1 分支遗留的结构体内容。 */
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
#if HRTIM_COMP_OUTPUT_ACTIVE_LOW
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_LOW;
#else
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
#endif
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = HRTIM_PWM_PERIOD_TICKS / 4U;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (HRTIM_PWM_PERIOD_TICKS * 3U) / 4U;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = HRTIM_PWM_PERIOD_TICKS / 4U;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (HRTIM_PWM_PERIOD_TICKS * 3U) / 4U;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */
  //===== ADC采样与载波对齐 =====
  /* 触发点取 PER-400, 使四通道扫描(800 tick @340MHz)居中于对称点 counter=0。
     居中脉冲下该点是电感电流纹波的平均点(推导见 hrtim.h)。 */
  pCompareCfg.CompareValue = HRTIM_ADC_SAMPLE_DELAY_TICKS;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER,
                                       HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  //One HRTIM trigger starts one four-channel ADC scan per complete PWM cycle.
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  pADCTriggerCfg.Trigger      = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP1;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* 互补波的开关是 OUTxR 的 DTEN 位。RM0440 规定它在 TxCEN 置位或输出已使能后
     不可再改, 所以只能在这里(计数器未启动、OENR 仍为0)设置。上面的
     WaveformTimerConfig 已经写过, 这里做一次落地确认: 若三路里有任何一路
     DTEN=0, 后面就不可能有互补波, 且运行中无法补救 —— 直接停在这里,
     比带着单边驱动上电安全。 */
  if (((hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].OUTxR & HRTIM_OUTR_DTEN) == 0U) ||
      ((hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].OUTxR & HRTIM_OUTR_DTEN) == 0U) ||
      ((hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].OUTxR & HRTIM_OUTR_DTEN) == 0U))
  {
    Error_Handler();
  }
  HRTIM_CaptureRegs();
  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspInit 0 */

  /* USER CODE END HRTIM1_MspInit 0 */
    /* HRTIM1 clock enable */
    __HAL_RCC_HRTIM1_CLK_ENABLE();
  /* USER CODE BEGIN HRTIM1_MspInit 1 */

  /* USER CODE END HRTIM1_MspInit 1 */
  }
}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspPostInit 0 */

  /* USER CODE END HRTIM1_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**HRTIM1 GPIO Configuration
    PB12     ------> HRTIM1_CHC1
    PB13     ------> HRTIM1_CHC2
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2
    PA10     ------> HRTIM1_CHB1
    PA11     ------> HRTIM1_CHB2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

  /* USER CODE END HRTIM1_MspPostInit 1 */
  }

}

void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspDeInit 0 */

  /* USER CODE END HRTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HRTIM1_CLK_DISABLE();
  /* USER CODE BEGIN HRTIM1_MspDeInit 1 */

  /* USER CODE END HRTIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

volatile HRTIM_RegSnapshot hrtim_regs;

/* 抄一份互补输出相关的寄存器现值。调试器 watch hrtim_regs 即可判读:
 *   outar/outbr/outcr 应为 0x00000100
 *     bit8  DTEN=1   <- 死区发生器使能, 这是互补波的开关
 *     bit1  POL1=0   <- CHx1 高有效
 *     bit17 POL2=0   <- CHx2 高有效 (HRTIM_COMP_OUTPUT_ACTIVE_LOW=1 时应为 0x00020100)
 *   dtar/dtbr/dtcr 应为 0x01180118
 *     DTR[8:0]=0x118=280, DTF[24:16]=0x118=280, DTPRSC[12:10]=0, SDTR/SDTF=0
 *   oenr  在按下确认键(WaveformOutputStart)之后应为 0x0000003F (六路全开)
 *   mcr   计数器启动后 bit16..19 (MCEN/TACEN/TBCEN/TCCEN) 应置位
 *
 * 判读方法:
 *   DTEN=0            -> 死区发生器没开, CHx2 恒为非活动电平(无波形)
 *   DTEN=1 但 CHx2 无波 -> 不是 HRTIM 的问题, 查 POL2 与驱动板输入极性, 或走线
 *   oenr 不含对应位   -> 输出没使能(没进 UI_Measure, 或被 WaveformOutputStop 关了)
 */
void HRTIM_CaptureRegs(void)
{
    hrtim_regs.outar = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].OUTxR;
    hrtim_regs.outbr = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].OUTxR;
    hrtim_regs.outcr = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].OUTxR;
    hrtim_regs.dtar  = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].DTxR;
    hrtim_regs.dtbr  = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].DTxR;
    hrtim_regs.dtcr  = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].DTxR;
    hrtim_regs.oenr  = hhrtim1.Instance->sCommonRegs.OENR;
    hrtim_regs.mcr   = hhrtim1.Instance->sMasterRegs.MCR;
}

/* USER CODE END 1 */
