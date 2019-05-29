/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SM2_STEP_Pin GPIO_PIN_0
#define SM2_STEP_GPIO_Port GPIOA
#define SM2_DIR_Pin GPIO_PIN_1
#define SM2_DIR_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define SWITCH_1_Pin GPIO_PIN_3
#define SWITCH_1_GPIO_Port GPIOA
#define SWITCH_2_Pin GPIO_PIN_4
#define SWITCH_2_GPIO_Port GPIOA
#define SENSOR_Pin GPIO_PIN_8
#define SENSOR_GPIO_Port GPIOA
#define SWITCH_H_Pin GPIO_PIN_9
#define SWITCH_H_GPIO_Port GPIOA
#define SWITCH_V_Pin GPIO_PIN_10
#define SWITCH_V_GPIO_Port GPIOA
#define BUCK_ENABLE_Pin GPIO_PIN_11
#define BUCK_ENABLE_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define SM1_STEP_Pin GPIO_PIN_4
#define SM1_STEP_GPIO_Port GPIOB
#define SM1_DIR_Pin GPIO_PIN_5
#define SM1_DIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// Controle LED Nucleo :
#define LEDON  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LEDOFF HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
// Lecture interrupteurs sur carte :
#define SW1 HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin)
#define SW2 HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin)
// Controle du buck qui genere 5V pour l'enceinte
#define BUCKON  HAL_GPIO_WritePin(BUCK_ENABLE_GPIO_Port, BUCK_ENABLE_Pin, GPIO_PIN_SET)
#define BUCKOFF HAL_GPIO_WritePin(BUCK_ENABLE_GPIO_Port, BUCK_ENABLE_Pin, GPIO_PIN_RESET)
// Lecture du capteur à ultrasons
#define SENSOR HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
