/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <espATcommands.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BATMAXV 2200
#define BATMINV 2110

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

osThreadId PS_MeasureHandle;
osThreadId PC_13_LEDHandle;
osThreadId PC_14_LEDHandle;
osThreadId EspCommunicationHandle;
/* USER CODE BEGIN PV */
uint16_t batteryVoltage[10]; // Battery voltage.
EventGroupHandle_t pc13EventGroup;
EventGroupHandle_t pc14EventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
void psMeasure(void const * argument);
void pc13LedCntrl(void const * argument);
void pc14LedCntrl(void const * argument);
void espCommunication(void const * argument);

/* USER CODE BEGIN PFP */
void ADC1_configuration(void);

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  ADC1_configuration();


  pc13EventGroup = xEventGroupCreate();
  pc14EventGroup = xEventGroupCreate();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of PS_Measure */
  osThreadDef(PS_Measure, psMeasure, osPriorityNormal, 0, 128);
  PS_MeasureHandle = osThreadCreate(osThread(PS_Measure), NULL);

  /* definition and creation of PC_13_LED */
  osThreadDef(PC_13_LED, pc13LedCntrl, osPriorityNormal, 0, 128);
  PC_13_LEDHandle = osThreadCreate(osThread(PC_13_LED), NULL);

  /* definition and creation of PC_14_LED */
  osThreadDef(PC_14_LED, pc14LedCntrl, osPriorityNormal, 0, 128);
  PC_14_LEDHandle = osThreadCreate(osThread(PC_14_LED), NULL);

  /* definition and creation of EspCommunication */
  osThreadDef(EspCommunication, espCommunication, osPriorityNormal, 0, 256);
  EspCommunicationHandle = osThreadCreate(osThread(EspCommunication), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1024;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1024;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN12_Pin|_1A_Pin|_2A_Pin|EN34_Pin
                          |_3A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, _4A_Pin|RST_ESP_Pin|EN_ESP_Pin|SPI2_CS_L_Pin
                          |SPI2_CS_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN12_Pin EN34_Pin */
  GPIO_InitStruct.Pin = EN12_Pin|EN34_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : _1A_Pin _2A_Pin _3A_Pin */
  GPIO_InitStruct.Pin = _1A_Pin|_2A_Pin|_3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : _4A_Pin SPI2_CS_L_Pin SPI2_CS_R_Pin */
  GPIO_InitStruct.Pin = _4A_Pin|SPI2_CS_L_Pin|SPI2_CS_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_ESP_Pin EN_ESP_Pin */
  GPIO_InitStruct.Pin = RST_ESP_Pin|EN_ESP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ADC1_configuration(void)
{
	//NVIC configuration
	NVIC->IP[11] |= 0x70; //Priority 7
	NVIC->ISER[0] |= (1<<11); //Enable DMA1 channel 1 interrupt
	NVIC->IP[18] |= 0x70; //Priority 7
	NVIC->ISER[0] |= (1<<18); //Enable ADC1 global interrupt
	//DMA configuration for ADC1
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable the DMA1 clock
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;

	DMA1_Channel1->CCR |= (1<<10); // Bits 11:10 MSIZE[1:0]: Memory size  01: 16-bits
	DMA1_Channel1->CCR |= (1<<8); // Bits 9:8 PSIZE[1:0]: Peripheral size  01: 16-bits
	DMA1_Channel1->CCR |= DMA_CCR_MINC;
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_TCIE;
	DMA1_Channel1->CCR &= ~DMA_CCR_HTIE;
	DMA1_Channel1->CCR &= ~DMA_CCR_TEIE;

	DMA1_Channel1->CNDTR = 10;
	DMA1_Channel1->CPAR = &(ADC1->DR);
	DMA1_Channel1->CMAR = &batteryVoltage[0];

	//  DMA1_Channel1->CCR |= DMA_CCR_EN;

	//ADC starting
//	MX_ADC1_Init();

//	ADC1->TR1 = 0;
//	ADC1->TR1 |= 2100;
//	ADC1->TR1 |= (2240 << 16);
//	ADC1->CFGR |= (3<<26); //Analog watchdog 1 channel selection
//	ADC1->CFGR |= ADC_CFGR_AWD1EN;  //Analog watchdog 1 enable on regular channels
//	ADC1->CFGR |= ADC_CFGR_AWD1SGL; //Analog watchdog 1 enabled on a single channel
//	ADC1->IER |= ADC_IER_AWD1IE; //Analog watchdog 1 interrupt enable
//	ADC1->CFGR |= ADC_CFGR_DMACFG;
//	ADC1->CFGR |= ADC_CFGR_DMAEN;
//	ADC1->CR |= ADC_CR_ADEN;
//	while((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY) __asm__ volatile("NOP");
//	ADC1->CR |= ADC_CR_ADSTART;
//	HAL_Delay(20);
//	DMA1_Channel1->CCR |= DMA_CCR_EN;
	__asm__ volatile("NOP");
	if(batteryVoltage[0] == 0) {
		if((ADC1->CR & ADC_CR_ADEN) == ADC_CR_ADEN) {
			DMA1_Channel1->CCR &= ~DMA_CCR_EN;
			ADC1->CR |= ADC_CR_ADSTP;
			while((ADC1->CR & ADC_CR_ADSTP) == ADC_CR_ADSTP) __asm__ volatile("NOP");
			HAL_Delay(5);
			ADC1->CR |= ADC_CR_ADDIS;
			while((ADC1->CR & ADC_CR_ADDIS) == ADC_CR_ADDIS) __asm__ volatile("NOP");
			HAL_Delay(20);
		}
		MX_ADC1_Init();
		ADC1->CFGR |= ADC_CFGR_DMACFG;
		ADC1->CFGR |= ADC_CFGR_DMAEN;
		ADC1->CR |= ADC_CR_ADEN;
		while((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY) __asm__ volatile("NOP");
		ADC1->CR |= ADC_CR_ADSTART;
		HAL_Delay(20);
		DMA1_Channel1->CCR |= DMA_CCR_EN;
		__asm__ volatile("NOP");
	}
	return;
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_psMeasure */
/**
  * @brief  Function implementing the PS_Measure thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_psMeasure */
void psMeasure(void const * argument)
{
  /* USER CODE BEGIN 5 */
	char measureResult[30];
	memset(measureResult, ' ', sizeof(measureResult));
  /* Infinite loop */
  for(;;)
  {
    osDelay(8);
    uint16_t avrBatVoltage = 0;
    for(uint8_t i = 0; i < (sizeof(batteryVoltage)/sizeof(batteryVoltage[0])); i++) {
    	avrBatVoltage += batteryVoltage[i];
    }
    avrBatVoltage = avrBatVoltage / (sizeof(batteryVoltage)/sizeof(batteryVoltage[0]));
    if(avrBatVoltage > BATMAXV) {
    	memset(measureResult, ' ', sizeof(measureResult));
    	strcpy(measureResult, "Voltage > MAX");
    	xEventGroupClearBits(pc13EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc13EventGroup, 0x1);
    	xEventGroupClearBits(pc14EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc14EventGroup, 0x1);
    	__asm__ volatile("NOP");
    } else if(avrBatVoltage < BATMINV) {
    	memset(measureResult, ' ', sizeof(measureResult));
    	strcpy(measureResult, "Voltage < MIN");
    	xEventGroupClearBits(pc13EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc13EventGroup, 0x2);
    	xEventGroupClearBits(pc14EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc14EventGroup, 0x2);
    	__asm__ volatile("NOP");
    } else {
    	memset(measureResult, ' ', sizeof(measureResult));
    	xEventGroupClearBits(pc13EventGroup, 0xFFFFFF);
    	xEventGroupClearBits(pc14EventGroup, 0xFFFFFF);
    	__asm__ volatile("NOP");
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pc13LedCntrl */
/**
* @brief Function implementing the PC_13_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pc13LedCntrl */
void pc13LedCntrl(void const * argument)
{
  /* USER CODE BEGIN pc13LedCntrl */
	EventBits_t eventBits;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    eventBits = xEventGroupGetBits(pc13EventGroup);
    switch (eventBits) {
		case 0x1:
			osDelay(100);
			GPIOC->ODR ^= GPIO_ODR_13;
			__asm__ volatile("NOP");
			break;
		case 0x2:
			osDelay(200);
			GPIOC->ODR ^= GPIO_ODR_13;
			__asm__ volatile("NOP");
			break;
		default:
			GPIOC->ODR &= ~GPIO_ODR_13;
			osDelay(5);
			__asm__ volatile("NOP");
			break;
	}
  }
  /* USER CODE END pc13LedCntrl */
}

/* USER CODE BEGIN Header_pc14LedCntrl */
/**
* @brief Function implementing the PC_14_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pc14LedCntrl */
void pc14LedCntrl(void const * argument)
{
  /* USER CODE BEGIN pc14LedCntrl */
	EventBits_t eventBits;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    eventBits = xEventGroupGetBits(pc14EventGroup);
    switch (eventBits) {
		case 0x1:
			osDelay(100);
			GPIOC->ODR ^= GPIO_ODR_14;
			__asm__ volatile("NOP");
			break;
		case 0x2:
			osDelay(200);
			GPIOC->ODR ^= GPIO_ODR_14;
			__asm__ volatile("NOP");
			break;
		default:
			GPIOC->ODR &= ~GPIO_ODR_14;
			osDelay(5);
			__asm__ volatile("NOP");
			break;
	}
  }
  /* USER CODE END pc14LedCntrl */
}

/* USER CODE BEGIN Header_espCommunication */
/**
* @brief Function implementing the EspCommunication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_espCommunication */
void espCommunication(void const * argument)
{
  /* USER CODE BEGIN espCommunication */
	vTaskDelay( pdMS_TO_TICKS( 100 ) );
	  GPIOB->ODR |= (1<<1);
	  GPIOB->ODR |= (1<<2);
	  char txBuffer[80] = {'A', 'T', '\r', '\n'};
	  char rxBuffer[80];
	  char controlAnsw[] = "AT\r\r\n\r\nOK\r\n";
	  while(strcmp(rxBuffer, controlAnsw) != 0) {
		  for(uint8_t i = 0; i < sizeof(rxBuffer); i++) {
			  rxBuffer[i] = 0;
		  }
		  HAL_UART_Transmit(&huart3, txBuffer, sizeof(txBuffer), 250);
		  HAL_UART_Receive(&huart3, rxBuffer, sizeof(rxBuffer), 250);
		  __asm__ volatile("NOP");
	  }
	  sendATCommand(&huart3, "ATE0\r\n", 6 , rxBuffer, sizeof(rxBuffer), 250);
	  sendATCommand(&huart3, "AT+CWMODE=1\r\n", 13 , rxBuffer, sizeof(rxBuffer), 250);
	  memset(txBuffer, '\0', sizeof(txBuffer));
	  strcpy(txBuffer, "AT+CWJAP_CUR=\"RedmiGiGidra\",\"DimaDimaDimon\"\r\n");
	  sendATCommand(&huart3, txBuffer, sizeof(txBuffer) , rxBuffer, sizeof(rxBuffer), 10000);
	  vTaskDelay( pdMS_TO_TICKS( 2000 ) );
	  __asm__ volatile("NOP");

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay( pdMS_TO_TICKS( 100 ) );
  }
  /* USER CODE END espCommunication */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
