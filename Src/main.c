/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/*
 * main.c
 *
 * Warefab Konnect STM32L0-SIGFOX Development Board
 *
 * Created on: Nov 8, 2019
 * Author: Muchiri John
 * (c) wwww.warefab.com
 *
 * This software component is licensed by warefab under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 * opensource.org/licenses/BSD-3-Clause
 *
 */

#include <stdio.h>
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "L70R.h"
#include "sht30x.h"
#include "LIS2DH12.h"
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

/* USER CODE BEGIN PV */
uint16_t adc_val_mic;
uint8_t ldr;
uint32_t tick_;
uint8_t gps_flag;
//uint16_t adc_val_amb;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sendMessage();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum {
	adc_buf_len = 4
};
uint32_t adc_data[adc_buf_len];
char buffer[126];
uint8_t state;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	//adc_val_amb = 0;
	ldr = 0;
	tick_ = 0;
	gps_flag = 0;

	for (adc_val_mic = 0; adc_val_mic < 512; adc_val_mic++) {
		uart_buf[adc_val_mic] = 0;
	}
	adc_val_mic = 0;
	buf_cur_pos = 0;
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
	MX_LPUART1_UART_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	//MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	//HAL_TIM_PWM_MspInit(&htim2);
	HAL_ADC_Start_DMA(&hadc, adc_data, adc_buf_len);
	sht30x_init(0x44);
	Lis2dh12_init(0x19);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(SFX_RST_GPIO_Port, SFX_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(SFX_RST_GPIO_Port, SFX_RST_Pin, GPIO_PIN_SET);

	HAL_TIM_Base_Stop_IT(&htim2);

	l70_standby(0);
	l70_mode(1);

	while (1) {
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// LDR and MIC
		ldr = HAL_GPIO_ReadPin(LDR_GPIO_Port, LDR_Pin);
		sprintf(buffer, "\r\nMIC : %04d, LDR : %d\r\n", adc_val_mic, ldr);
		usart_puts(&huart1, buffer);

		//SHT30
		if (sht30x_get() == 1) {
			sprintf(buffer, "SHT30 -> C : %d, F : %d, H : %d\r\n",
					sht30x->cTemp, sht30x->fTemp, sht30x->humidity);
			usart_puts(&huart1, buffer);
		}

		//LIS2DH12
		state = Lis2dh12_whoIAm();
		if (state == 0x33) {
			Lis2dh12_getXYZ();
			sprintf(buffer, "LIS2DH12 - X: %03d,  Y: %03d, Z: %0d\r\n",
					Lis2dh12->x, Lis2dh12->y, Lis2dh12->z);
			usart_puts(&huart1, buffer);
		}

		//L70R
		sprintf(buffer,
				"L70R - TIME: %ld,  LAT: %ld, LNG : %ld, SPEED : %d, DATE : %ld\r\n",
				l70->time, l70->lat, l70->lng, l70->speed, l70->date);
		usart_puts(&huart1, buffer);

		//HAL_Delay(1000);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BTN1_Pin) {
		if ((HAL_GetTick() - tick_) > 2000) {
			(gps_flag == 0) ? (gps_flag = 1) : (gps_flag = 0);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, gps_flag);
			l70_standby(gps_flag);
			tick_ = HAL_GetTick();
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	uint8_t x;
	//uint8_t smp = adc_buf_len / 2;
	uint32_t adc_m = 0;
	//uint32_t adc_l = 0;

	for (x = 0; x < adc_buf_len; x++) {
		adc_m += adc_data[x];
		//adc_l += adc_data[x + 1];
		//x++;
	}

	adc_val_mic = (uint16_t) (adc_m / adc_buf_len);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		HAL_GPIO_TogglePin(BZR_GPIO_Port, BZR_Pin);
	}
}

void sendMessage(uint8_t *msg) {
	//uint8_t msg[12] = "wf-lt-sg";
	uint8_t *ch = &msg[0];
	char chm[3];

	usart_puts(&hlpuart1, (char*) "AT$SF=");
	while (*ch) {
		sprintf(chm, "%02X", *ch++);
		usart_puts(&hlpuart1, chm);
	}
	//usart_puts(&hlpuart1, (char*) "C0FFEE");
	//usart_puts(&hlpuart1, ch);
	usart_send(&hlpuart1, '\r');
	//HAL_Delay(30000);

	/*
	 //HAL_TIM_Base_Start_IT(&htim2);
	 //usart_puts(&huart1, (char*) "\nVER - ");
	 //usart_puts(&hlpuart1, (char*) "AT$I=0\r\n");
	 //HAL_Delay(25);
	 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	 //usart_puts(&huart1, (char*) "ID - ");
	 //usart_puts(&hlpuart1, (char*) "AT$I=10\r\n"); //get id
	 //HAL_Delay(500);
	 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	 HAL_Delay(500);
	 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	 //usart_puts(&huart1, (char *)"BLUE LED\n");
	 //usart_puts(&huart1, (char*) "PAC - ");
	 //usart_puts(&hlpuart1, (char*) "AT$I=11\r\n"); //get pac
	 //HAL_Delay(500);
	 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	 HAL_Delay(500);*/

	 //HAL_TIM_Base_Stop_IT(&htim2);
	 /*if (x >= 40) {
	 x = 0;
	 HAL_TIM_Base_Start_IT(&htim2);
	 sendMessage((uint8_t*) "wf-lt-sg");
	 HAL_Delay(10000);
	 } else {
	 x++;
	 }*/

	//HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
/* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
