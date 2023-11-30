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
#define _1_SECOND   1000
#define _500_MS     500
#define _100_MS     100
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_CAN_A_RX_Pin GPIO_PIN_13
#define LED_CAN_A_RX_GPIO_Port GPIOC
#define BTN_Pin GPIO_PIN_14
#define BTN_GPIO_Port GPIOC
#define LED_CAN_A_TX_Pin GPIO_PIN_15
#define LED_CAN_A_TX_GPIO_Port GPIOC
#define LED_CAN_B_TX_Pin GPIO_PIN_5
#define LED_CAN_B_TX_GPIO_Port GPIOA
#define LED_CAN_B_RX_Pin GPIO_PIN_7
#define LED_CAN_B_RX_GPIO_Port GPIOA
#define CS_CAN_C_Pin GPIO_PIN_2
#define CS_CAN_C_GPIO_Port GPIOB
#define RST_CANC_IC_Pin GPIO_PIN_11
#define RST_CANC_IC_GPIO_Port GPIOB
#define INT_CAN_C_Pin GPIO_PIN_8
#define INT_CAN_C_GPIO_Port GPIOA
#define INT_CAN_C_EXTI_IRQn EXTI4_15_IRQn
#define LED_CAN_C_RX_Pin GPIO_PIN_15
#define LED_CAN_C_RX_GPIO_Port GPIOA
#define LED_CAN_C_TX_Pin GPIO_PIN_3
#define LED_CAN_C_TX_GPIO_Port GPIOD
#define LED_BLINK_Pin GPIO_PIN_3
#define LED_BLINK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
