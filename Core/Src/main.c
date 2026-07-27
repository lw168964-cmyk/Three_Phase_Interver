/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "suanfa.h"
#include "inital.h"
#include "sample.h"
#include "key.h"
#include "OLED.h"
#include "oledui.h"
#include "arm_math.h"
#include "stdio.h"
#include <stdarg.h>
#define DMA_TX_BUFFER_SIZE 128
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 /** ===========================================================================================================================
 * @brief usart的dma发送函数
 * @note 类似printf
 */
static char dma_tx_buffer[DMA_TX_BUFFER_SIZE];

/**
 * @brief  无阻塞 DMA 格式化打印函数
 * @param  huart: 串口句柄 (比如 &huart4)
 * @param  fmt: 格式化字符串
 */
void printf_DMA(const char *fmt, ...)
{
    // 1. 【生死防线】检查 DMA 是否正在忙着搬运上一次的数据？
    // 如果在忙，直接丢弃本次打印！这能保证 CPU 绝对不会被卡住，也不会发乱码。
    if (huart4.gState != HAL_UART_STATE_READY) 
    {
        return; 
    }

    // 2. 将传入的参数格式化为字符串，存入缓冲区
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(dma_tx_buffer, DMA_TX_BUFFER_SIZE, fmt, args);
    va_end(args);

    // 3. 启动 DMA 后台发送
    if (len > 0) 
    {
        HAL_UART_Transmit_DMA(&huart4, (uint8_t *)dma_tx_buffer, len);
    }
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t ADC1_Value[4]={0};//原始数据
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */
	OLEDUI_Init();
	
	fixed_angle_init(&dianjiaodu,Line_f1,CTRL_FREQUENCY); //初始化电角度结构体

	//角度发生器与PR谐振系数统一由Line_f1推出,避免改默认值后两边脱节
	OLEDUI_Apply_Freq();
	my_svpwm_Init(&SVPWM);
	
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);//ADC校准
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC1_Value, 4);//开启转换
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//	HAL_TIM_Base_Start_IT(&htim6);    //开启定时中断
//	
//	HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_MASTER);//开启通道输出
//	HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_TIMER_A);
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);
//	HAL_HRTIM_SimplePWMStart(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);


//	HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_TIMER_B);
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
//	HAL_HRTIM_SimplePWMStart(&hhrtim1,HRTIM_TIMERINDEX_TIMER_B,HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
//	
//	HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_TIMER_C);
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
//	HAL_HRTIM_SimplePWMStart(&hhrtim1,HRTIM_TIMERINDEX_TIMER_C,HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
	
  while (1)
  {
		OLEDUI_Key_Handle();
    OLED_Update();
    OLEDUI_Refresh();
//		printf_DMA("A:%f B:%f C:%f\n",ADC1_Value[0]*3.3/4066/16,ADC1_Value[1]*3.3/4066/16,ADC1_Value[2]*3.3/4066/16);
//		HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
