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
#include "stm32f4xx_hal.h"

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
#define ISOLATION_FAULT_DETECT_Pin GPIO_PIN_13
#define ISOLATION_FAULT_DETECT_GPIO_Port GPIOC
#define PRECHARGE_CONTACTOR_Pin GPIO_PIN_0
#define PRECHARGE_CONTACTOR_GPIO_Port GPIOC
#define MAIN_CONTACTOR_Pin GPIO_PIN_1
#define MAIN_CONTACTOR_GPIO_Port GPIOC
#define GPIO0_Pin GPIO_PIN_2
#define GPIO0_GPIO_Port GPIOC
#define CHARGER_DETECT_Pin GPIO_PIN_3
#define CHARGER_DETECT_GPIO_Port GPIOC
#define CURRENT_Pin GPIO_PIN_2
#define CURRENT_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_3
#define LIGHT_GPIO_Port GPIOA
#define CHANNEL_STATUS_Pin GPIO_PIN_6
#define CHANNEL_STATUS_GPIO_Port GPIOA
#define EMERGENCY_STOP_Pin GPIO_PIN_4
#define EMERGENCY_STOP_GPIO_Port GPIOC
#define IGNITION_Pin GPIO_PIN_5
#define IGNITION_GPIO_Port GPIOC
#define ISOLATION_FAULT_DETECTB0_Pin GPIO_PIN_0
#define ISOLATION_FAULT_DETECTB0_GPIO_Port GPIOB
#define LV_BATTERY_VOLTAGE_Pin GPIO_PIN_1
#define LV_BATTERY_VOLTAGE_GPIO_Port GPIOB
#define GPIO3_Pin GPIO_PIN_10
#define GPIO3_GPIO_Port GPIOB
#define YELLOW_LED_Pin GPIO_PIN_14
#define YELLOW_LED_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_15
#define RED_LED_GPIO_Port GPIOB
#define UART_MCU_TO_DEBUG_RX_Pin GPIO_PIN_10
#define UART_MCU_TO_DEBUG_RX_GPIO_Port GPIOA
#define UART_MCU_TO_DEBUG_TX_Pin GPIO_PIN_15
#define UART_MCU_TO_DEBUG_TX_GPIO_Port GPIOA
#define EEPROM_I2C_SCL_Pin GPIO_PIN_6
#define EEPROM_I2C_SCL_GPIO_Port GPIOB
#define EEPROM_I2C_SDA_Pin GPIO_PIN_7
#define EEPROM_I2C_SDA_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_9
#define GPIO2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
