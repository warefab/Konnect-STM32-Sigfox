/*
 * buzzer.c
 *
 * Buzzer functions
 * Warefab Konnect STM32L0-SIGFOX Development Board
 *
 * Created on: Dec 16, 2019
 * Author: Muchiri John
 * (c) wwww.warefab.com
 *
 * This software component is licensed by warefab under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 * opensource.org/licenses/BSD-3-Clause
 */
#include "buzzer.h"

void setChannel(uint16_t freq, uint8_t channel) {
	uint16_t period, pulse;

	if (freq < 50)
		freq = 50;
	else if (freq > 2000)
		freq = 2000;

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	period = (BUZZER_TIM_FREQ / freq) - 1;
	pulse = period / 2;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = (SystemCoreClock / BUZZER_TIM_FREQ) - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = period;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (channel == 1) {
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
				!= HAL_OK) {
			Error_Handler();
		}

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	} else {
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
				!= HAL_OK) {
			Error_Handler();
		}
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	}
}
/*
 * @brief set buzzer frequency, range 50-2000Hz
 * @retval none
 */
void setBuzzerFreq(uint16_t freq) {
	setChannel(freq, 1);
}

void stopBuzzer() {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void driveMotor(uint16_t freq) {
	if (freq < 50) {
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	} else {
		setChannel(freq, 2);
	}

}
