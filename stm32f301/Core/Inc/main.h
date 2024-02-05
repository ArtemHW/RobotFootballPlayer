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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
