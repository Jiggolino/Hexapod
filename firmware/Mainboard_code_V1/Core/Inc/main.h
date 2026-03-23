/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define LED_GREEN_Pin GPIO_PIN_2
#define LED_GREEN_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOE
#define Battery_Voltage_Pin GPIO_PIN_3
#define Battery_Voltage_GPIO_Port GPIOF
#define Left_Current_Sense_Pin GPIO_PIN_0
#define Left_Current_Sense_GPIO_Port GPIOC
#define Right_Current_Sense_Pin GPIO_PIN_0
#define Right_Current_Sense_GPIO_Port GPIOA
#define Left_A2_Pin GPIO_PIN_4
#define Left_A2_GPIO_Port GPIOA
#define Left_A1_Pin GPIO_PIN_5
#define Left_A1_GPIO_Port GPIOA
#define Left_A0_Pin GPIO_PIN_6
#define Left_A0_GPIO_Port GPIOA
#define Right_Enable_Pin GPIO_PIN_0
#define Right_Enable_GPIO_Port GPIOB
#define Right_A2_Pin GPIO_PIN_15
#define Right_A2_GPIO_Port GPIOE
#define Right_A1_Pin GPIO_PIN_10
#define Right_A1_GPIO_Port GPIOB
#define Right_A0_Pin GPIO_PIN_11
#define Right_A0_GPIO_Port GPIOB
#define User_Button_Pin GPIO_PIN_10
#define User_Button_GPIO_Port GPIOD
#define XSHUT_Pin GPIO_PIN_15
#define XSHUT_GPIO_Port GPIOD
#define GPIO_1_TOF_Pin GPIO_PIN_2
#define GPIO_1_TOF_GPIO_Port GPIOG
#define Left_Enable_Pin GPIO_PIN_3
#define Left_Enable_GPIO_Port GPIOG
#define DIN_WS2812B_1_Pin GPIO_PIN_8
#define DIN_WS2812B_1_GPIO_Port GPIOA
#define DIN_WS2812B_2_Pin GPIO_PIN_9
#define DIN_WS2812B_2_GPIO_Port GPIOA
#define LED_CAM_Pin GPIO_PIN_2
#define LED_CAM_GPIO_Port GPIOD
#define UART_TX_SERIAL_Pin GPIO_PIN_6
#define UART_TX_SERIAL_GPIO_Port GPIOB
#define UART_RX_SERIAL_Pin GPIO_PIN_7
#define UART_RX_SERIAL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
