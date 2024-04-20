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
#include <math.h>
#include <espATcommands.h>
#include <adxl345.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId PS_MeasureHandle;
osThreadId PC_13_LEDHandle;
osThreadId PC_14_LEDHandle;
osThreadId EspCommunicationHandle;
osThreadId AccelerometerHandle;
/* USER CODE BEGIN PV */
volatile uint16_t batteryVoltage[10]; // Battery voltage.
uint16_t avrBatVoltage;
uint8_t BatChargeState;
EventGroupHandle_t pc13EventGroup;
EventGroupHandle_t pc14EventGroup;

struct EncoderStr EncoderR, EncoderL;
struct SoftPWM SoftPwmR, SoftPwmL;

float kToRpm;

char txBuffer[ESPTXBUFFERSIZE];
volatile char rxBuffer[ESPRXBUFFERSIZE];
volatile uint16_t rxBufferHead;
uint8_t receivedBytes = 0;
char rxBufferCopy[256];

TimerHandle_t timerForDataSending;
EventGroupHandle_t timerFdsEventGroup;

//int8_t joyX;
//int8_t joyY;
float tSpeed;
float aSpeed;

int16_t accelValueR[3];
int16_t accelValueL[3];

uint32_t debugVar;
uint32_t debugVar2;

volatile uint32_t cycle_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
void psMeasure(void const * argument);
void pc13LedCntrl(void const * argument);
void pc14LedCntrl(void const * argument);
void espCommunication(void const * argument);
void accelerometer(void const * argument);

/* USER CODE BEGIN PFP */
void ADC1_configuration(void);
void TIM1_configuration(void);
void TIM2_configuration(void);
void TIM15_additional_configuration(void);
void TIM17_additional_configuration(void);
void USART3_additional_configuration(void);

void timerForSendDataCallback(TimerHandle_t xTimer);

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
	avrBatVoltage = 0;
	BatChargeState = 0;

//	EncoderR.timeNew = 0;
//	EncoderR.timeOld = 0;
//	EncoderR.timeUpdate = 0;
	EncoderR.positionNew = 0;
	EncoderR.positionOld = 0;
	EncoderR.posCntUpdate = 0;
	EncoderR.rpm = 0;

//	EncoderL.timeNew = 0;
//	EncoderL.timeOld = 0;
//	EncoderL.timeUpdate = 0;
	EncoderL.positionNew = 0;
	EncoderL.positionOld = 0;
	EncoderL.posCntUpdate = 0;
	EncoderL.rpm = 0;

	SoftPwmR.errorValue = 0;
	SoftPwmR.sumValue = 0;
	SoftPwmR.pValue = 0;
	SoftPwmR.iValue = 0;
	SoftPwmR.pwmFloatValue = 0;
	SoftPwmR.WheelSpeed = 0;
	SoftPwmR.reqValueTemp = 0;
	SoftPwmR.curValue = 0;
	SoftPwmR.reqValue = 0;
	SoftPwmR.pwmValue = 0;
	SoftPwmR.status = 0;

	SoftPwmL.errorValue = 0;
	SoftPwmL.sumValue = 0;
	SoftPwmL.pValue = 0;
	SoftPwmL.iValue = 0;
	SoftPwmL.pwmFloatValue = 0;
	SoftPwmL.WheelSpeed = 0;
	SoftPwmL.reqValueTemp = 0;
	SoftPwmL.curValue = 0;
	SoftPwmL.reqValue = 0;
	SoftPwmL.pwmValue = 0;
	SoftPwmL.status = 0;

	kToRpm = (1000*60)/1024;

	memset(txBuffer, '\0', sizeof(txBuffer));
	memset(rxBuffer, '\0', sizeof(rxBuffer));
	rxBufferHead = 0;
	memset(rxBufferCopy, '\0', sizeof(rxBufferCopy));

//	joyX = 0;
//	joyY = 0;
	tSpeed = 0;
	aSpeed = 0;

	memset(accelValueR, '0', sizeof(accelValueR));
	memset(accelValueL, '0', sizeof(accelValueL));

	debugVar = 0;
	debugVar2 = 0;

	cycle_count = 0;

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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  ADC1_configuration();
  TIM1_configuration();
  TIM2_configuration();
  TIM17_additional_configuration();
  TIM15_additional_configuration();
  USART3_additional_configuration();

  GPIOA->ODR |= (1<<6); //EN34
  GPIOA->ODR |= (1<<3); //EN12

  DWT->CTRL |= (1<<0);  //Enable cycle counter
  DWT->CTRL |= (1<<0);  //Enable cycle counter


  pc13EventGroup = xEventGroupCreate();
  pc14EventGroup = xEventGroupCreate();
  timerFdsEventGroup = xEventGroupCreate();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  timerForDataSending = xTimerCreate("TimerForDataSending", pdMS_TO_TICKS(211), pdTRUE, 1, timerForSendDataCallback); //211
  xTimerStart(timerForDataSending, portMAX_DELAY);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of PS_Measure */
  osThreadDef(PS_Measure, psMeasure, osPriorityNormal, 0, 96);
  PS_MeasureHandle = osThreadCreate(osThread(PS_Measure), NULL);

  /* definition and creation of PC_13_LED */
  osThreadDef(PC_13_LED, pc13LedCntrl, osPriorityNormal, 0, 96);
  PC_13_LEDHandle = osThreadCreate(osThread(PC_13_LED), NULL);

  /* definition and creation of PC_14_LED */
  osThreadDef(PC_14_LED, pc14LedCntrl, osPriorityNormal, 0, 96);
  PC_14_LEDHandle = osThreadCreate(osThread(PC_14_LED), NULL);

  /* definition and creation of EspCommunication */
  osThreadDef(EspCommunication, espCommunication, osPriorityAboveNormal, 0, 256);
  EspCommunicationHandle = osThreadCreate(osThread(EspCommunication), NULL);

  /* definition and creation of Accelerometer */
  osThreadDef(Accelerometer, accelerometer, osPriorityNormal, 0, 96);
  AccelerometerHandle = osThreadCreate(osThread(Accelerometer), NULL);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM15|RCC_PERIPHCLK_TIM17
                              |RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 15;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 400;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 639;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart3.Init.BaudRate = 230400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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

  /*Configure GPIO pins : EN12_Pin _1A_Pin _2A_Pin EN34_Pin
                           _3A_Pin */
  GPIO_InitStruct.Pin = EN12_Pin|_1A_Pin|_2A_Pin|EN34_Pin
                          |_3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : _4A_Pin */
  GPIO_InitStruct.Pin = _4A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(_4A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_ESP_Pin EN_ESP_Pin */
  GPIO_InitStruct.Pin = RST_ESP_Pin|EN_ESP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_L_Pin SPI2_CS_R_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_L_Pin|SPI2_CS_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
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
//	NVIC->ISER[0] |= (1<<11); //Enable DMA1 channel 1 interrupt
	NVIC->IP[18] |= 0x70; //Priority 7
//	NVIC->ISER[0] |= (1<<18); //Enable ADC1 global interrupt
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
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)&batteryVoltage[0];

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

void TIM1_configuration(void)
{
    // Enable the TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Configure GPIO pins for TIM1 PA9-CH2 and PA8-CH1
    GPIOA->MODER |= (1<<17);
    GPIOA->MODER |= (1<<19);
    GPIOA->PUPDR |= (1<<17); //Pull-down
    GPIOA->PUPDR |= (1<<19); //Pull-down
    GPIOA->AFR[1] |= (6); //AF6
    GPIOA->AFR[1] |= (6<<4); //AF6

    // Configure TIM1
	TIM1->CR1 |= TIM_CR1_ARPE; //ARPE: Auto-reload preload enable
	TIM1->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0); // Encoder mode 3
	TIM1->DIER |= TIM_DIER_UIE; //UIE: Update interrupt enable

	TIM1->CCMR1 |= (1<<0);
	TIM1->CCMR1 |= (1<<8);
	TIM1->CCER |= (1<<0);
	TIM1->CCER |= (1<<4);

    // Enable the TIM1 interrupt
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    // Set priority for the TIM1 interrupt
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 5); // Adjust priority as needed

//    TIM1->CNT = PRELOADENC;
    TIM1->PSC = 7; //7+1 = 8
    // Enable the TIM1 counter
    TIM1->CR1 |= TIM_CR1_CEN;
}

void TIM2_configuration(void)
{
    // Enable the TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure GPIO pins for TIM2 PA1-CH2 and PA0-CH1
    GPIOA->MODER |= (1<<1);
    GPIOA->MODER |= (1<<3);
    GPIOA->PUPDR |= (1<<1); //Pull-down
    GPIOA->PUPDR |= (1<<3); //Pull-down
    GPIOA->AFR[0] |= (1); //AF1
    GPIOA->AFR[0] |= (1<<4); //AF1

    // Configure TIM2
	TIM2->CR1 |= TIM_CR1_ARPE; //ARPE: Auto-reload preload enable
	TIM2->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0); // Encoder mode 3
	TIM2->DIER |= TIM_DIER_UIE; //UIE: Update interrupt enable

	TIM2->CCMR1 |= (1<<0);
	TIM2->CCMR1 |= (1<<8);
	TIM2->CCER |= (1<<0);
	TIM2->CCER |= (1<<4);

	TIM2->PSC = 7; //7+1 = 8
	TIM2->ARR = 0xFFFF; //65535

    // Enable the TIM2 interrupt
    NVIC_EnableIRQ(TIM2_IRQn);

    // Set priority for the TIM2 interrupt
    NVIC_SetPriority(TIM2_IRQn, 5); // Adjust priority as needed

    // Enable the TIM2 counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM15_additional_configuration(void)
{
	TIM15->DIER |= TIM_DIER_CC1IE;
	TIM15->DIER |= TIM_DIER_CC2IE;
	TIM15->DIER |= TIM_DIER_UIE; //UIE: Update interrupt enable

    // Enable the TIM15 interrupt
    NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    // Set priority for the TIM15 interrupt
    NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 5);

	TIM15->CR1 |= TIM_CR1_CEN;
}

void TIM17_additional_configuration(void)
{
	TIM17->DIER |= TIM_DIER_UIE; //UIE: Update interrupt enable

    // Enable the TIM17 interrupt
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    // Set priority for the TIM17 interrupt
    NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 5);

	TIM17->CR1 |= TIM_CR1_CEN;
}

void USART3_additional_configuration(void)
{
	//DMA configuration for UART3
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable the DMA1 clock

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;

	DMA1_Channel3->CCR |= DMA_CCR_MINC;
	DMA1_Channel3->CCR |= DMA_CCR_CIRC;
	DMA1_Channel3->CCR &= ~DMA_CCR_TCIE;
	DMA1_Channel3->CCR &= ~DMA_CCR_HTIE;
	DMA1_Channel3->CCR &= ~DMA_CCR_TEIE;

	DMA1_Channel3->CNDTR = ESPRXBUFFERSIZE;
	DMA1_Channel3->CPAR = (uint32_t)&(USART3->RDR);
	DMA1_Channel3->CMAR = (uint32_t)&rxBuffer[0];

	USART3->CR3 &= ~USART_CR3_DMAR;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;

	DMA1_Channel2->CCR &= ~DMA_CCR_EN;

	DMA1_Channel2->CCR |= DMA_CCR_MINC;
	DMA1_Channel2->CCR &= ~DMA_CCR_CIRC;
	DMA1_Channel2->CCR |= DMA_CCR_DIR;
	DMA1_Channel2->CCR |= DMA_CCR_TCIE;
	DMA1_Channel2->CCR &= ~DMA_CCR_HTIE;
	DMA1_Channel2->CCR &= ~DMA_CCR_TEIE;

	DMA1_Channel2->CNDTR = 0;
	DMA1_Channel2->CPAR = (uint32_t)&(USART3->TDR);
	DMA1_Channel2->CMAR = (uint32_t)&txBuffer[0];

	USART3->CR3 &= ~USART_CR3_DMAT;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
}

void timerForSendDataCallback(TimerHandle_t xTimer)
{
	xEventGroupSetBits(timerFdsEventGroup, 0x1);
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

  /* Infinite loop */
  for(;;)
  {
    osDelay(8);

    for(uint8_t i = 0; i < (sizeof(batteryVoltage)/sizeof(batteryVoltage[0])); i++) {
    	avrBatVoltage += batteryVoltage[i];
    }
    avrBatVoltage = avrBatVoltage / (sizeof(batteryVoltage)/sizeof(batteryVoltage[0]));
    BatChargeState = ((float)(avrBatVoltage - BATMINV)/(BATMAXV - BATMINV))*100;

    if(avrBatVoltage > BATMAXV) {
    	xEventGroupClearBits(pc13EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc13EventGroup, OVERVOLTAGEONLED);
    	xEventGroupClearBits(pc14EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc14EventGroup, OVERVOLTAGEONLED);
    	__asm__ volatile("NOP");
    } else if(avrBatVoltage < BATMINV) {
    	xEventGroupClearBits(pc13EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc13EventGroup, UNDERVOLTAGEONLED);
    	xEventGroupClearBits(pc14EventGroup, 0xFFFFFF);
    	xEventGroupSetBits(pc14EventGroup, UNDERVOLTAGEONLED);
    	__asm__ volatile("NOP");
    } else {
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
		case OVERVOLTAGEONLED:
			osDelay(200);
			GPIOC->ODR ^= GPIO_ODR_13;
			__asm__ volatile("NOP");
			break;
		case UNDERVOLTAGEONLED:
			osDelay(300);
			GPIOC->ODR ^= GPIO_ODR_13;
			__asm__ volatile("NOP");
			break;
		default:
			GPIOC->ODR &= ~GPIO_ODR_13;
			osDelay(200);
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
		case OVERVOLTAGEONLED:
			osDelay(200);
			GPIOC->ODR ^= GPIO_ODR_14;
			__asm__ volatile("NOP");
			break;
		case UNDERVOLTAGEONLED:
			osDelay(300);
			GPIOC->ODR ^= GPIO_ODR_14;
			__asm__ volatile("NOP");
			break;
		default:
			GPIOC->ODR &= ~GPIO_ODR_14;
			osDelay(200);
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
	vTaskDelay( pdMS_TO_TICKS( 600 ) );
	taskENTER_CRITICAL();
	  GPIOB->ODR |= (1<<1); //RST_ESP
	  GPIOB->ODR |= (1<<2); //EN_ESP
	  sprintf(txBuffer, "AT\r\n");
	  char controlAnsw[] = "AT\r\r\n\r\nOK\r\n";
	  while(strcmp(rxBuffer, controlAnsw) != 0) {
		  HAL_UART_Transmit(&huart3, (uint8_t*)txBuffer, strlen(txBuffer), 250);
		  HAL_UART_Receive(&huart3, (uint8_t*)rxBuffer, strlen(controlAnsw), 250);
		  vTaskDelay( pdMS_TO_TICKS( 100 ) );
		  __asm__ volatile("NOP");
	  }

	  sprintf(txBuffer, "AT+CWMODE_CUR=1\r\n");
	  sendATCommand(&huart3, "AT+CWMODE_CUR=1\r\n", strlen(txBuffer) , 250);
	  receiveAnswer(&huart3, rxBuffer, sizeof(rxBuffer), 250);

	  sprintf(txBuffer, "AT+CWJAP_CUR=\"RedmiGiGidra\",\"DimaDimaDimon\"\r\n");
	  sendATCommand(&huart3, txBuffer, strlen(txBuffer), 1000);
	  receiveAnswer(&huart3, rxBuffer, sizeof(rxBuffer), 10000);

	  taskEXIT_CRITICAL();

	  memset(rxBuffer, '\0', sizeof(rxBuffer));
	  USART3->CR3 |= USART_CR3_DMAR;
	  DMA1_Channel3->CCR |= DMA_CCR_EN; //Starting continuous DMA on RX

	  vTaskDelay( pdMS_TO_TICKS( 2000 ) );

	  sprintf(txBuffer, "ATE0\r\n");
	  sendATCommand(&huart3, txBuffer, strlen(txBuffer), 250);
	  vTaskDelay( pdMS_TO_TICKS( 100 ) );

	  sprintf(txBuffer, "AT+CIPCLOSE\r\n");
	  sendATCommand(&huart3, txBuffer, strlen(txBuffer), 250);
	  vTaskDelay( pdMS_TO_TICKS( 250 ) );

	  sprintf(txBuffer, "AT+CIPSTART=\"TCP\",\"192.168.137.1\",8080\r\n");
	  sendATCommand(&huart3, txBuffer, strlen(txBuffer), 250);
	  vTaskDelay( pdMS_TO_TICKS( 40 ) );

	  memset(txBuffer, '\0', sizeof(txBuffer));
      // Creating the entire GET request string
	  sprintf(txBuffer, "GET /robot HTTP/1.1\r\n"
						  "Host: 192.168.137.1\r\n");
	  int getRequestLength = strlen(txBuffer);
	  char pDataBuf[20];
	  sprintf(pDataBuf, "AT+CIPSEND=%d\r\n", getRequestLength);
	  sendATCommand(&huart3, pDataBuf, sizeof(pDataBuf), 250);
	  vTaskDelay( pdMS_TO_TICKS( 10 ) );
	  sendATCommand(&huart3, txBuffer, getRequestLength, 250);

	  memset(txBuffer, '\0', sizeof(txBuffer));
	  vTaskDelay( pdMS_TO_TICKS( 50 ) );

	  __asm__ volatile("NOP");

  /* Infinite loop */
  for(;;)
  {
	  debugVar = 1;
	  // Calculate the number of bytes received since the last processing
//	  uint8_t receivedBytes = 0;
	  if((ESPRXBUFFERSIZE - DMA1_Channel3->CNDTR) < rxBufferHead) {
		  receivedBytes = (ESPRXBUFFERSIZE - rxBufferHead + ESPRXBUFFERSIZE - DMA1_Channel3->CNDTR) % ESPRXBUFFERSIZE;
	  } else {
		  receivedBytes = (ESPRXBUFFERSIZE - DMA1_Channel3->CNDTR -rxBufferHead) % ESPRXBUFFERSIZE;
	  }

	  uint16_t sizeRxBufCopy = sizeof(rxBufferCopy);
	  memset(rxBufferCopy, '\0', sizeRxBufCopy);
	  for(uint16_t i = 0; i < receivedBytes; i++) {
		  rxBufferCopy[i] = rxBuffer[(rxBufferHead + i)%sizeRxBufCopy];
	  }

	  debugVar = 5;

	  // Process the received data
	  for(uint16_t i = 0; i < receivedBytes; i++) {
		  debugVar = 6;
		  debugVar2 = i;
		  if((rxBuffer[(rxBufferHead + i)%ESPRXBUFFERSIZE] == 'T') &&
		     (rxBuffer[(rxBufferHead + i + 1)%ESPRXBUFFERSIZE] == 'S') &&
			 (rxBuffer[(rxBufferHead + i + 2)%ESPRXBUFFERSIZE] == 'P')) {
			  if(rxBuffer[(rxBufferHead + i + 3)%ESPRXBUFFERSIZE] == '-') {
				  if(rxBuffer[(rxBufferHead + i + 8)%ESPRXBUFFERSIZE] != '_') continue;
			  } else {
				  if(rxBuffer[(rxBufferHead + i + 7)%ESPRXBUFFERSIZE] != '_') continue;
			  }
			  tSpeed = parseFloat(rxBuffer, ((rxBufferHead + i + 3)%ESPRXBUFFERSIZE));
		  }
		  debugVar = 10;
		  if((rxBuffer[(rxBufferHead + i)%ESPRXBUFFERSIZE] == 'A') &&
		     (rxBuffer[(rxBufferHead + i + 1)%ESPRXBUFFERSIZE] == 'S') &&
			 (rxBuffer[(rxBufferHead + i + 2)%ESPRXBUFFERSIZE] == 'P')) {
			  if(rxBuffer[(rxBufferHead + i + 3)%ESPRXBUFFERSIZE] == '-') {
				  if(rxBuffer[(rxBufferHead + i + 8)%ESPRXBUFFERSIZE] != '_') continue;
			  } else {
				  if(rxBuffer[(rxBufferHead + i + 7)%ESPRXBUFFERSIZE] != '_') continue;
			  }
			  aSpeed = parseFloat(rxBuffer, ((rxBufferHead + i + 3)%ESPRXBUFFERSIZE));
		  }

		  debugVar = 12;
		  if((rxBuffer[(rxBufferHead + i)%ESPRXBUFFERSIZE] == 'E') &&
		     (rxBuffer[(rxBufferHead + i + 1)%ESPRXBUFFERSIZE] == 'R') &&
			 (rxBuffer[(rxBufferHead + i + 2)%ESPRXBUFFERSIZE] == 'R') &&
			 (rxBuffer[(rxBufferHead + i + 3)%ESPRXBUFFERSIZE] == 'O') &&
			 (rxBuffer[(rxBufferHead + i + 4)%ESPRXBUFFERSIZE] == 'R')) {
			  debugVar = 121;
			  espRecon(&huart3);
		  }
		  debugVar = 13;
	  }


	  debugVar = 15;
      // Update the buffer head index
      rxBufferHead = ((rxBufferHead + receivedBytes) % ESPRXBUFFERSIZE);

      if(xEventGroupGetBitsFromISR(timerFdsEventGroup) == 0x1) {
    	  debugVar = 20;
    	    // Create the JSON content with variable values
    	    sprintf(txBuffer, "POST /robot_data HTTP/1.1\r\n"
    	                         "Host: 192.168.137.1\r\n"
    	                         "Content-Type: application/json\r\n"
//    	                         "Content-Length: %d\r\n\r\n"
    	                         "{\"avrBatVoltage\": \"%d\", \"EncoderR.rpm\": \"%d\", \"EncoderL.rpm\": \"%d\", \"SoftPwmR.pwmValue\": \"%d\", \"SoftPwmL.pwmValue\": \"%d\", \"tSpeed\": \"%f\", \"aSpeed\": \"%f\", \"rReqValue\": \"%d\", \"lReqValue\": \"%d\", \"accelRX\": \"%d\", \"accelRY\": \"%d\", \"accelRZ\": \"%d\", \"accelLX\": \"%d\", \"accelLY\": \"%d\", \"accelLZ\": \"%d\", \"cycle_count\": \"%d\"}",
								 BatChargeState, EncoderR.rpm, EncoderL.rpm, SoftPwmR.pwmValue, SoftPwmL.pwmValue, tSpeed, aSpeed, SoftPwmR.reqValue, SoftPwmL.reqValue, accelValueR[0], accelValueR[1], accelValueR[2], accelValueL[0], accelValueL[1], accelValueL[2], cycle_count);

    	    // Calculate the number of characters in the POST request
    	    int postRequestLength = strlen(txBuffer);
    	    debugVar = 25;
    	  sprintf(pDataBuf, "AT+CIPSEND=%d\r\n", postRequestLength);
    	  while(atSend_USART3_DMA(pDataBuf, strlen(pDataBuf)) != 0) {
    	    vTaskDelay( pdMS_TO_TICKS( 10 ) );
    	  }
    	  debugVar = 30;
    	  while(atSend_USART3_DMA(txBuffer, postRequestLength) != 0) {
    	    vTaskDelay( pdMS_TO_TICKS( 10 ) );
    	  }
    	  debugVar = 35;
    	  xEventGroupClearBits(timerFdsEventGroup, 0xFFFFFF);
      }

	  vTaskDelay( pdMS_TO_TICKS( 25 ) );
  }
  /* USER CODE END espCommunication */
}

/* USER CODE BEGIN Header_accelerometer */
/**
* @brief Function implementing the Accelerometer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_accelerometer */
void accelerometer(void const * argument)
{
  /* USER CODE BEGIN accelerometer */
	vTaskDelay(120);
	adxl345_bw_rate_setup(&hspi2, GPIOB, 14, 0xA);
	vTaskDelay(20);
	adxl345_data_format(&hspi2, GPIOB, 14, 0x42);
	vTaskDelay(20);
	adxl345_measure_mode(&hspi2, GPIOB, 14);
	vTaskDelay(20);
	adxl345_bw_rate_setup(&hspi2, GPIOB, 12, 0xA);
	vTaskDelay(20);
	adxl345_data_format(&hspi2, GPIOB, 12, 0x42);
	vTaskDelay(20);
	adxl345_measure_mode(&hspi2, GPIOB, 12);
  /* Infinite loop */
  for(;;)
  {
	  adxl345_read_data(&hspi2, GPIOB, 14, (uint8_t*)accelValueR);
	  adxl345_read_data(&hspi2, GPIOB, 12, (uint8_t*)accelValueL);
	  vTaskDelay(20);
  }
  /* USER CODE END accelerometer */
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
