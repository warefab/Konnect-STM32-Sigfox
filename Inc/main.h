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
#include "stm32l0xx_hal.h"

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
#define GPS_RXD_Pin GPIO_PIN_2
#define GPS_RXD_GPIO_Port GPIOA
#define GPS_TXD_Pin GPIO_PIN_3
#define GPS_TXD_GPIO_Port GPIOA
#define MEM_CS_Pin GPIO_PIN_4
#define MEM_CS_GPIO_Port GPIOA
#define MEM_CLK_Pin GPIO_PIN_5
#define MEM_CLK_GPIO_Port GPIOA
#define MEM_DO_Pin GPIO_PIN_6
#define MEM_DO_GPIO_Port GPIOA
#define MEM_DI_Pin GPIO_PIN_7
#define MEM_DI_GPIO_Port GPIOA
#define BZR_Pin GPIO_PIN_0
#define BZR_GPIO_Port GPIOB
#define MIC_Pin GPIO_PIN_1
#define MIC_GPIO_Port GPIOB
#define SFX_RST_Pin GPIO_PIN_2
#define SFX_RST_GPIO_Port GPIOB
#define SFX_RX_Pin GPIO_PIN_10
#define SFX_RX_GPIO_Port GPIOB
#define SFX_TX_Pin GPIO_PIN_11
#define SFX_TX_GPIO_Port GPIOB
#define DBG_TX_Pin GPIO_PIN_9
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_10
#define DBG_RX_GPIO_Port GPIOA
#define LDR_Pin GPIO_PIN_15
#define LDR_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_3
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_8
#define BTN1_GPIO_Port GPIOB
#define BTN1_EXTI_IRQn EXTI4_15_IRQn
#define BTN2_Pin GPIO_PIN_9
#define BTN2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
