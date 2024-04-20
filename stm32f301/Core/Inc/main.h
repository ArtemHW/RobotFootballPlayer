/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct EncoderStr {
//	int timeNew;
//	int timeOld;
//	uint8_t timeUpdate;
	int32_t positionNew;
	int32_t positionOld;
	uint8_t posCntUpdate;
	int rpm;
};

struct SoftPWM {
	int errorValue;
	float sumValue;
	float pValue;
	float iValue;
	float pwmFloatValue;
	float WheelSpeed;
	float reqValueTemp;
	int16_t reqValue;
	int16_t curValue;
	int16_t pwmValue;
	uint8_t status;

};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN12_Pin GPIO_PIN_3
#define EN12_GPIO_Port GPIOA
#define _1A_Pin GPIO_PIN_4
#define _1A_GPIO_Port GPIOA
#define _2A_Pin GPIO_PIN_5
#define _2A_GPIO_Port GPIOA
#define EN34_Pin GPIO_PIN_6
#define EN34_GPIO_Port GPIOA
#define _3A_Pin GPIO_PIN_7
#define _3A_GPIO_Port GPIOA
#define _4A_Pin GPIO_PIN_0
#define _4A_GPIO_Port GPIOB
#define RST_ESP_Pin GPIO_PIN_1
#define RST_ESP_GPIO_Port GPIOB
#define EN_ESP_Pin GPIO_PIN_2
#define EN_ESP_GPIO_Port GPIOB
#define SPI2_CS_L_Pin GPIO_PIN_12
#define SPI2_CS_L_GPIO_Port GPIOB
#define SPI2_CS_R_Pin GPIO_PIN_14
#define SPI2_CS_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BATMAXV 2385
#define BATMINV 2000
#define OVERVOLTAGEONLED 0x1
#define UNDERVOLTAGEONLED 0x2
#define KP 0.001f
#define KI  0.0000001f   //0.0001f
#define MAXRPM 500
#define PWMVAL 400
#define DISBETWHEELS 0.070
#define RWHEEL 0.0175
#define ENCDELAY 1
#define ESPRXBUFFERSIZE 511
#define ESPTXBUFFERSIZE 511
#define IPSERVER "127.16.19.7"
#define PORTSERVER "8080"
#define POSUPDATED 1
#define TIMEUPDATED 2
#define PRELOADENC 30000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
