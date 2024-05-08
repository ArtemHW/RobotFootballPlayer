/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
extern struct EncoderStr EncoderR;
extern struct EncoderStr EncoderL;

extern struct SoftPWM SoftPwmR;
extern struct SoftPWM SoftPwmL;

extern float kToRpm;

extern volatile uint8_t rxBufferHead;

extern float tSpeed;
extern float aSpeed;

extern volatile uint32_t cycle_count;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	GPIOC->ODR &= ~GPIO_ODR_13;
	uint16_t cntLed = 0;

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
	  if(cntLed == 60000) {
		  GPIOC->ODR ^= GPIO_ODR_14;
	  }
	  cntLed++;

    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	USART3->CR3 &= ~USART_CR3_DMAT;
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void DMA1_Channel1_IRQHandler(void)
{
	__asm__ volatile("NOP");
	DMA1->IFCR |= DMA_IFCR_CGIF1;
	__asm__ volatile("NOP");
}

void ADC1_IRQHandler(void)
{
	__asm__ volatile("NOP");
	ADC1->ISR |= ADC_ISR_AWD1;
	__asm__ volatile("NOP");
}

void TIM1_UP_TIM16_IRQHandler(void)
{
	if((TIM1->SR & TIM_SR_UIF) == TIM_SR_UIF) {
		EncoderR.posCntUpdate = POSUPDATED;
		TIM1->SR &= ~(TIM_SR_UIF);
	}
}

void TIM2_IRQHandler(void)
{
	EncoderL.posCntUpdate = POSUPDATED;
	TIM2->SR &= ~(TIM_SR_UIF);
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
	if((TIM15->SR & TIM_SR_UIF) == TIM_SR_UIF) {
		__asm__ volatile("NOP");

		GPIOA->ODR &= ~(1<<7); //_3A
		GPIOB->ODR &= ~(1<<0); //_4A

		GPIOA->ODR &= ~(1<<4); //_1A
		GPIOA->ODR &= ~(1<<5); //_2A

		TIM15->SR &= ~(TIM_SR_UIF);
		if((TIM15->SR & TIM_SR_UIF) == 0) {
			__asm__ volatile("NOP");
		}
		__asm__ volatile("NOP");
	}

	if((TIM15->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF) {
		__asm__ volatile("NOP");

		if(SoftPwmR.status == 1) {
			GPIOA->ODR |= (1<<7); //_3A
			GPIOB->ODR &= ~(1<<0); //_4A
		} else if(SoftPwmR.status == 2) {
			GPIOA->ODR &= ~(1<<7); //_3A
			GPIOB->ODR |= (1<<0); //_4A
		}

		TIM15->SR &= ~(TIM_SR_CC1IF);
		if((TIM15->SR & TIM_SR_CC1IF) == 0) {
			__asm__ volatile("NOP");
		}
		__asm__ volatile("NOP");
	}

	if((TIM15->SR & TIM_SR_CC2IF) == TIM_SR_CC2IF) {
		__asm__ volatile("NOP");

		if(SoftPwmL.status == 1) {
			GPIOA->ODR &= ~(1<<4); //_1A
			GPIOA->ODR |= (1<<5); //_2A
		} else if(SoftPwmL.status == 2) {
			GPIOA->ODR |= (1<<4); //_1A
			GPIOA->ODR &= ~(1<<5); //_2A
		}


		TIM15->SR &= ~(TIM_SR_CC2IF);
		if((TIM15->SR & TIM_SR_CC2IF) == 0) {
			__asm__ volatile("NOP");
		}
		__asm__ volatile("NOP");
	}
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
	if((TIM17->SR & TIM_SR_UIF) == TIM_SR_UIF) {
		  uint32_t start_cycle = DWT->CYCCNT;

		EncoderR.pulses = TIM1->CNT;
		if(EncoderR.pulses > 32767) {
			EncoderR.pulses = (65535 - EncoderR.pulses);
		} else {
			EncoderR.pulses = -EncoderR.pulses;
		}
		TIM1->EGR |= TIM_EGR_UG;
//		EncoderR.sumPulses += EncoderR.pulses;
//		if((EncoderR.pulses > 5) || (EncoderR.pulses < -5)) EncoderR.iter += 1;
		EncoderR.rpm = (EncoderR.pulses * 1000 * 60) / 10240; //RPM
//		uint32_t end_cycle = DWT->CYCCNT;
//		cycle_count = end_cycle - start_cycle;


//		if(TIM2->CNT > TIM2->ARR) {
//		  TIM2->EGR |= TIM_EGR_UG;
//		}
		EncoderL.pulses = TIM2->CNT;
		if(EncoderL.pulses > 32767) {
			EncoderL.pulses = -(65535 - EncoderL.pulses);
		}
		TIM2->EGR |= TIM_EGR_UG;
		EncoderL.rpm = (EncoderL.pulses * 1000 * 60) / 10240; //RPM

//			uint32_t end_cycle = DWT->CYCCNT;
//			cycle_count = end_cycle - start_cycle;


//		SoftPwmR.WheelSpeed = tSpeed - aSpeed*DISBETWHEELS/2;
//		  uint32_t start_cycle = DWT->CYCCNT;
//		SoftPwmR.reqValueTemp = (SoftPwmR.WheelSpeed*60)/(2*3.14*RWHEEL);
//			uint32_t end_cycle = DWT->CYCCNT;
//			cycle_count = end_cycle - start_cycle;
//		if((SoftPwmR.reqValueTemp >= - 50) && (SoftPwmR.reqValueTemp <= 50)) {
//			SoftPwmR.reqValueTemp = 0;
//		} else if((SoftPwmR.reqValueTemp > MAXRPM)) {
//			SoftPwmR.reqValueTemp = MAXRPM;
//		} else if((SoftPwmR.reqValueTemp < -MAXRPM)) {
//			SoftPwmR.reqValueTemp = -MAXRPM;
//		}
//		SoftPwmR.reqValue = (int16_t)SoftPwmR.reqValueTemp;
//
//		SoftPwmR.curValue = EncoderR.rpm;
//		SoftPwmR.errorValue = SoftPwmR.reqValue - SoftPwmR.curValue;
//		SoftPwmR.pValue = KP * SoftPwmR.errorValue;
//		SoftPwmR.iValue += KI * SoftPwmR.errorValue;
//		if(SoftPwmR.iValue > ((float)MAXRPM)) SoftPwmR.iValue = MAXRPM;
//		else if(SoftPwmR.iValue < ((float)-MAXRPM)) SoftPwmR.iValue = -MAXRPM;
//		if((SoftPwmR.iValue <= 50) && (SoftPwmR.iValue >= -50)) {
//			SoftPwmR.sumValue = SoftPwmR.pValue;
//		} else {
//			SoftPwmR.sumValue = (SoftPwmR.pValue + SoftPwmR.iValue);
//		}
//		SoftPwmR.pwmFloatValue += ((((float)SoftPwmR.sumValue)/((float)MAXRPM))*PWMVAL);
//		if(SoftPwmR.pwmFloatValue > PWMVAL) SoftPwmR.pwmFloatValue = PWMVAL;
//		else if(SoftPwmR.pwmFloatValue < -PWMVAL) SoftPwmR.pwmFloatValue = -PWMVAL;
//		SoftPwmR.pwmValue = (int16_t)SoftPwmR.pwmFloatValue;
//		if(SoftPwmR.reqValue == 0) {
//		  if(SoftPwmR.pwmValue > 0) {
//			  SoftPwmR.pwmFloatValue -= 1;
//		  }
//		  if(SoftPwmR.pwmValue < 0) {
//			  SoftPwmR.pwmFloatValue += 1;
//		  }
//		}
//
//		if(SoftPwmR.pwmValue < 0) {
//		  TIM15->CCR1 = TIM15->ARR - SoftPwmR.pwmValue*(-1);
//		  SoftPwmR.status = 2;
//		} else if(SoftPwmR.pwmValue > 0){
//		  TIM15->CCR1 = TIM15->ARR - SoftPwmR.pwmValue;
//		  SoftPwmR.status = 1;
//		} else {
//		  SoftPwmR.status = 0;
//		}

		SoftPwmR.WheelSpeed = tSpeed - aSpeed*DISBETWHEELS/2;
		SoftPwmR.reqValueTemp = ((SoftPwmR.WheelSpeed)/(2*3.14*RWHEEL))*10240/1000;
		if((SoftPwmR.reqValueTemp >= - 4) && (SoftPwmR.reqValueTemp <= 4)) {
			SoftPwmR.reqValueTemp = 0;
		} else if((SoftPwmR.reqValueTemp > MAXPULSES)) {
			SoftPwmR.reqValueTemp = MAXPULSES;
		} else if((SoftPwmR.reqValueTemp < -MAXPULSES)) {
			SoftPwmR.reqValueTemp = -MAXPULSES;
		}
		SoftPwmR.reqValue = (int16_t)SoftPwmR.reqValueTemp;

		SoftPwmR.curValue = EncoderR.pulses;
		SoftPwmR.errorValue = (SoftPwmR.reqValue - SoftPwmR.curValue)*1000000;
//		SoftPwmR.pValue = KP * SoftPwmR.errorValue;
//		SoftPwmR.iValue += KI * SoftPwmR.errorValue;
		SoftPwmR.pVal = SoftPwmR.errorValue / KPI * KPM;
//		SoftPwmR.iVal += SoftPwmR.errorValue / KII * KIM;
//		if(SoftPwmR.iValue > ((float)MAXPULSES)) SoftPwmR.iValue = MAXPULSES;
//		else if(SoftPwmR.iVal < ((float)-MAXPULSES)) SoftPwmR.iValue = -MAXPULSES;
//		if((SoftPwmR.iValue <= 4) && (SoftPwmR.iValue >= -4)) {
//			SoftPwmR.sumValue = SoftPwmR.pValue;
//		} else {
//			SoftPwmR.sumValue = (SoftPwmR.pValue + SoftPwmR.iValue);
//		}
		SoftPwmR.sumVal += (SoftPwmR.pVal + SoftPwmR.iVal);
//		SoftPwmR.pwmFloatValue += ((((float)SoftPwmR.sumValue)/((float)MAXRPM))*PWMVAL);
//		if(SoftPwmR.pwmFloatValue > PWMVAL) SoftPwmR.pwmFloatValue = PWMVAL;
//		else if(SoftPwmR.pwmFloatValue < -PWMVAL) SoftPwmR.pwmFloatValue = -PWMVAL;
//		SoftPwmR.pwmValue = (int16_t)SoftPwmR.pwmFloatValue;
		SoftPwmR.pwmValue = ((SoftPwmR.sumVal / ((MAXPULSES*1000000)/1000))*PWMVAL)/1000;
		if(SoftPwmR.pwmValue > PWMVAL) SoftPwmR.pwmValue = PWMVAL;
		else if(SoftPwmR.pwmValue < -PWMVAL) SoftPwmR.pwmValue = -PWMVAL;

//		if(SoftPwmR.reqValue == 0) {
//		  if(SoftPwmR.pwmValue > 0) {
//			  SoftPwmR.pwmFloatValue -= 1;
//		  }
//		  if(SoftPwmR.pwmValue < 0) {
//			  SoftPwmR.pwmFloatValue += 1;
//		  }
//		}

		if(SoftPwmR.pwmValue < 0) {
		  TIM15->CCR1 = TIM15->ARR - SoftPwmR.pwmValue*(-1);
		  SoftPwmR.status = 2;
		} else if(SoftPwmR.pwmValue > 0){
		  TIM15->CCR1 = TIM15->ARR - SoftPwmR.pwmValue;
		  SoftPwmR.status = 1;
		} else {
		  SoftPwmR.status = 0;
		}

//		uint32_t end_cycle = DWT->CYCCNT;
//		cycle_count = end_cycle - start_cycle;

		SoftPwmL.WheelSpeed = tSpeed + aSpeed*DISBETWHEELS/2;
		SoftPwmL.reqValueTemp = ((SoftPwmL.WheelSpeed)/(2*3.14*RWHEEL))*10240/1000;
		if((SoftPwmL.reqValueTemp >= - 4) && (SoftPwmL.reqValueTemp <= 4)) {
			SoftPwmL.reqValueTemp = 0;
		} else if((SoftPwmL.reqValueTemp > MAXPULSES)) {
			SoftPwmL.reqValueTemp = MAXPULSES;
		} else if((SoftPwmL.reqValueTemp < -MAXPULSES)) {
			SoftPwmL.reqValueTemp = -MAXPULSES;
		}
		SoftPwmL.reqValue = (int16_t)SoftPwmL.reqValueTemp;

		SoftPwmL.curValue = EncoderL.pulses;
		SoftPwmL.errorValue = (SoftPwmL.reqValue - SoftPwmL.curValue)*1000000;
		SoftPwmL.pVal = SoftPwmL.errorValue / KPI;
//		SoftPwmL.iVal += SoftPwmL.errorValue / KII;
		SoftPwmL.sumVal += (SoftPwmL.pVal + SoftPwmL.iVal);
		SoftPwmL.pwmValue = ((SoftPwmL.sumVal / ((MAXPULSES*1000000)/1000))*PWMVAL)/1000;
		if(SoftPwmL.pwmValue > PWMVAL) SoftPwmL.pwmValue = PWMVAL;
		else if(SoftPwmL.pwmValue < -PWMVAL) SoftPwmL.pwmValue = -PWMVAL;

		if(SoftPwmL.pwmValue < 0) {
		  TIM15->CCR2 = TIM15->ARR - SoftPwmL.pwmValue*(-1);
		  SoftPwmL.status = 2;
		} else if(SoftPwmL.pwmValue > 0) {
		  TIM15->CCR2 = TIM15->ARR - SoftPwmL.pwmValue;
		  SoftPwmL.status = 1;
		} else {
		  SoftPwmL.status = 0;
		}


		TIM17->SR &= ~(TIM_SR_UIF);
		uint32_t end_cycle = DWT->CYCCNT;
		cycle_count = end_cycle - start_cycle;

	}
}

/* USER CODE END 1 */
