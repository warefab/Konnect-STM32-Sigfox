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
#include <stdlib.h>
#include <math.h>
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
#include "sigfox.h"
#include "buzzer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  mic_sample_time 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t mic_output;
volatile uint16_t ldr_output;
uint32_t gps_tick_;
uint8_t gps_flag;
int32_t sigfox_tick_;
uint8_t sigfox_flag;
uint8_t sigfox_delay;
uint16_t pwm_value, step;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t getMicValue();
void sendSigfoxPacket();
void getSigfoxPACID();
char* dateTimeFormat(char *bf_tk, uint32_t, char);
inline uint8_t adcToPercent(uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum {
	adc_buf_len = 8
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
	gps_tick_ = 0;
	gps_flag = 0;
	sigfox_tick_ = 0;
	sigfox_flag = 0;
	sigfox_delay = 0;

	pwm_value = step = 100;

	for (mic_output = 0; mic_output < 512; mic_output++) {
		uart_buf[mic_output] = 0;
	}
	mic_output = 0;
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
	l70_init();
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
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	//set buzzer frequency, 200Hz
	setBuzzerFreq(200);
	//init sht30x temp&hum sensor
	sht30x_init(0x44);
	//init lis2dh12 3-axis sensor
	Lis2dh12_init(0x19);
	//set buzzer frequency, 400Hz
	HAL_Delay(500);
	setBuzzerFreq(400);
	//init l70R gps module
	l70_mode(1);
	//reset sigfox module
	HAL_GPIO_WritePin(SFX_RST_GPIO_Port, SFX_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(SFX_RST_GPIO_Port, SFX_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	//set buzzer frequency, 600Hz
	setBuzzerFreq(600);
	//print wisol info, ID/PAC
	getSigfoxPACID();
	//set buzzer frequency, 800Hz
	setBuzzerFreq(800);
	//adc start dma conversion
	HAL_ADC_Start_DMA(&hadc, adc_data, adc_buf_len);
	HAL_Delay(500);
	//stop buzzer
	stopBuzzer();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//pulse green led
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		//send packets to sigfox cloud
		sendSigfoxPacket();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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

uint16_t getMicValue() {
	uint16_t mic_max = 0;
	uint16_t mic_min = 4095;
	uint8_t x = 0;
	uint16_t mic_ptp;

	for (; x < 36; x++) {
		uint32_t p_time = HAL_GetTick();
		mic_ptp = 0;
		while ((HAL_GetTick() - p_time) < mic_sample_time) {
			if (mic_output < 4095) {
				if (mic_output > mic_max) {
					mic_max = mic_output;
				} else if (mic_output < mic_min) {
					mic_min = mic_output;
				}
			}
		}
		mic_ptp = mic_max - mic_min;
	}
	return mic_ptp;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BTN1_Pin) {
		if ((HAL_GetTick() - gps_tick_) > 2000) {
			(gps_flag == 0) ? (gps_flag = 1) : (gps_flag = 0);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, gps_flag);
			l70_standby(gps_flag);
			gps_tick_ = HAL_GetTick();
		}
	} else if (GPIO_Pin == BTN2_Pin) {
		if ((HAL_GetTick() - gps_tick_) > 2000) {
			(sigfox_flag == 0) ? (sigfox_flag = 1) : (sigfox_flag = 0);
			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, sigfox_flag);
			sigfox_delay = 0;
			gps_tick_ = HAL_GetTick();
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	uint8_t x;
	uint32_t adc_m = 0;
	uint32_t adc_l = 0;
	for (x = 0; x < adc_buf_len; x++) {
		adc_l += adc_data[x];
		adc_m += adc_data[x + 1];
		x++;
	}
	ldr_output = (uint16_t) (adc_l / (adc_buf_len / 2));
	mic_output = (uint16_t) (adc_m / (adc_buf_len / 2));
}

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 if (htim->Instance == TIM2) {
 HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
 }
 }*/

void sendSigfoxPacket() {
	//char packets[24];
	uint8_t flags = 0;
	//date/time
	char time[12] = { 0 };
	char date[12] = { 0 };
	char gps_ns, gps_ew;
	//mems mic, noise = 1, silent = 0
	uint16_t mic_val = getMicValue();
	uint8_t mic_status;
	//Ambient sensor, light = 1, dark = 0
	uint8_t ldr_status;
	//sht30 temperature and humidity
	sht30x_get();
	//lis2dh12 acc x, y, z values
	state = Lis2dh12_whoIAm();
	if (state == 0x33) {
		Lis2dh12_getXYZ();
	}
	//send readable data to debug port else to wisol module
	if (sigfox_flag == 0) {
		dateTimeFormat(time, l70->time, ':');
		dateTimeFormat(date, l70->date, '-');
		gps_ns = (l70->lat_ns == 0) ? ' ' : l70->lat_ns;
		gps_ew = (l70->lng_ew == 0) ? ' ' : l70->lng_ew;
		//mic and ldr
		sprintf(buffer, "\r\nMIC: %04d, LDR : %04d\r\n", mic_val, ldr_output);
		usart_puts(&huart1, buffer);

		//SHT30
		sprintf(buffer, "SHT30 -> C : %d, F : %d, H : %d\r\n", sht30x->cTemp,
				sht30x->fTemp, sht30x->humidity);
		usart_puts(&huart1, buffer);

		//LIS2DH12
		sprintf(buffer, "LIS2DH12 -> X: %03d,  Y: %03d, Z: %0d\r\n",
				Lis2dh12->x, Lis2dh12->y, Lis2dh12->z);
		usart_puts(&huart1, buffer);

		//L70R
		sprintf(buffer,
				"L70R -> TIME: %s,  LAT: %.5f%c, LNG : %.5f%c, SPEED : %d, DATE : %s\r\n",
				time, (float) (l70->lat / 100000.0), gps_ns,
				(float) (l70->lng / 100000.0), gps_ew, l70->speed, date);
		usart_puts(&huart1, buffer);
	} else {
		//check mic, if noise/not
		mic_status = (mic_val > 8) ? 1 : 0;
		if (mic_status == 1) { //bit 0
			flags |= (uint8_t) (1 << 0);
		}
		//light threshold, on/off
		ldr_status = (ldr_output >= 255) ? 1 : 0;
		if (ldr_status == 1) { //bit 1
			flags |= (uint8_t) (1 << 1);
		}
		//check if acc x axis is negative
		if (Lis2dh12->x < 0) { //bit 2
			flags |= (uint8_t) (1 << 2);
		}
		//check if acc y axis is negative
		if (Lis2dh12->y < 0) { //bit 3
			flags |= (uint8_t) (1 << 3);
		}
		//gps standby mode on/off
		if (gps_flag == 1) { //bit 4
			flags |= (uint8_t) (1 << 4);
		}
		//speed threshold
		if (l70->speed >= 80) { //bit 5
			flags |= (uint8_t) (1 << 5);
		}
		//latitude pos, if N=0, S=1
		if (l70->lat_ns == 'S') { //bit 5
			flags |= (uint8_t) (1 << 6);
		}
		//longitude pos, if E=0, W=1
		if (l70->lng_ew == 'W') { //bit 5
			flags |= (uint8_t) (1 << 7);
		}

		//format bytes to hex
		sprintf(buffer, "%06lX%06lX%02X%02X%02X%02X%02X%02X", l70->lat,
				l70->lng, (uint8_t) adcToPercent(ldr_output),
				(uint8_t) abs(Lis2dh12->x), (uint8_t) abs(Lis2dh12->y),
				sht30x->cTemp, sht30x->humidity, flags);
		//send to debug port
		usart_puts(&huart1, buffer);
		usart_puts(&huart1, (char*) "\r\n");
		//send to sigfox cloud, 36 sec delay
		if (sigfox_delay >= 18) {
			sendSigfoxMessage((char*) buffer);
			sigfox_delay = 0;
		} else {
			sigfox_delay++;
		}
	}
}

void getSigfoxPACID() {
	CheckSigfoxVersion(DV_VERSION);
	HAL_Delay(200);
	CheckSigfoxVersion(DV_ID);
	HAL_Delay(200);
	CheckSigfoxVersion(DV_PAC);
	HAL_Delay(200);
}

char* dateTimeFormat(char *buf_tk, uint32_t dt_, char sep) {
	uint8_t tk1 = (dt_ / 10000.0);
	uint8_t tk2 = ((uint16_t) (dt_ / 100.0)) % 100;
	uint8_t tk3 = (uint8_t) (dt_ % 100);
	sprintf(buf_tk, "%02d%c%02d%c%02d", tk1, sep, tk2, sep, tk3);
	return buf_tk;
}

inline uint8_t adcToPercent(uint16_t val) {
	uint8_t pct = ceil((val * 100) / 4095.0);
	return pct;
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
