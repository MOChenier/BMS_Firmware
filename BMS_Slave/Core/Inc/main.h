/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define BMS_BOOT_Pin GPIO_PIN_13
#define BMS_BOOT_GPIO_Port GPIOC
#define BalanceEnableCell0_Pin GPIO_PIN_14
#define BalanceEnableCell0_GPIO_Port GPIOC
#define BalanceEnableCell1_Pin GPIO_PIN_15
#define BalanceEnableCell1_GPIO_Port GPIOC
#define BalanceThermistor1_Pin GPIO_PIN_0
#define BalanceThermistor1_GPIO_Port GPIOA
#define BalanceThermistor2_Pin GPIO_PIN_1
#define BalanceThermistor2_GPIO_Port GPIOA
#define BalanceEnableCell2_Pin GPIO_PIN_2
#define BalanceEnableCell2_GPIO_Port GPIOA
#define BalanceEnableCell3_Pin GPIO_PIN_3
#define BalanceEnableCell3_GPIO_Port GPIOA
#define BalanceEnableCell4_Pin GPIO_PIN_4
#define BalanceEnableCell4_GPIO_Port GPIOA
#define BalanceEnableCell5_Pin GPIO_PIN_5
#define BalanceEnableCell5_GPIO_Port GPIOA
#define BalanceEnableCell6_Pin GPIO_PIN_6
#define BalanceEnableCell6_GPIO_Port GPIOA
#define BalanceEnableCell7_Pin GPIO_PIN_7
#define BalanceEnableCell7_GPIO_Port GPIOA
#define BalanceEnableCell8_Pin GPIO_PIN_2
#define BalanceEnableCell8_GPIO_Port GPIOB
#define BalanceEnableCell9_Pin GPIO_PIN_12
#define BalanceEnableCell9_GPIO_Port GPIOB
#define BMS_SCL_Pin GPIO_PIN_13
#define BMS_SCL_GPIO_Port GPIOB
#define BMS_SDA_Pin GPIO_PIN_14
#define BMS_SDA_GPIO_Port GPIOB
#define BMS_Alert_Pin GPIO_PIN_15
#define BMS_Alert_GPIO_Port GPIOB
#define BalanceEnableCell10_Pin GPIO_PIN_8
#define BalanceEnableCell10_GPIO_Port GPIOA
#define BalanceEnableCell11_Pin GPIO_PIN_9
#define BalanceEnableCell11_GPIO_Port GPIOA
#define BalanceEnableCell12_Pin GPIO_PIN_10
#define BalanceEnableCell12_GPIO_Port GPIOA
#define BalanceEnableCell13_Pin GPIO_PIN_11
#define BalanceEnableCell13_GPIO_Port GPIOA
#define BalanceEnableCell14_Pin GPIO_PIN_12
#define BalanceEnableCell14_GPIO_Port GPIOA
#define YELLOW_LED_Pin GPIO_PIN_15
#define YELLOW_LED_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_5
#define RED_LED_GPIO_Port GPIOB
#define BalaResCheck_Pin GPIO_PIN_6
#define BalaResCheck_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
