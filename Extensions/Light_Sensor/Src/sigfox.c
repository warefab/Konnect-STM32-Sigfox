/*
 * sigfox.c
 *
 * sigfox Wisol SFM10R1 functions
 * Warefab Konnect STM32L0-SIGFOX Development Board
 *
 * Created on: Nov 19, 2019
 * Author: Muchiri John
 * (c) wwww.warefab.com
 *
 * This software component is licensed by warefab under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 * opensource.org/licenses/BSD-3-Clause
 */
#include "sigfox.h"

const char *at_messages[] = {"AT$I=0", "AT$I=10", "AT$I=11", "AT$SF="};

/*
 * send a formatted(hex) message to sigfox cloud
 */
void sendSigfoxMessage(char *data) {
	usart_puts(&hlpuart1, (char*) at_messages[DV_TOKEN]);
	while (*data) {
		usart_send(&hlpuart1, *data++);
	}
	usart_send(&hlpuart1, '\r');
}

/*
 * check sifox version and ID
 * helper function
 */
void CheckSigfoxVersion(enum Message type_) {
	if (type_ == DV_VERSION) {
		usart_puts(&huart1, (char*) "\nVER - ");
	} else if (type_ == DV_ID) {
		usart_puts(&huart1, (char*) "ID - ");
	} else if (type_ == DV_PAC) {
		usart_puts(&huart1, (char*) "PAC - ");
	}
	usart_puts(&hlpuart1, (char*) at_messages[type_]); //get pac
	usart_send(&hlpuart1, '\r');
}

/*
 * get sigfox wisol module ID and PAC
 * logged to debug serial port, usart1
 */
void getSigfoxPACID() {
	CheckSigfoxVersion(DV_VERSION);
	HAL_Delay(500);
	CheckSigfoxVersion(DV_ID);
	HAL_Delay(500);
	CheckSigfoxVersion(DV_PAC);
	HAL_Delay(500);
}

/*
 * reset sigfox module
 */
void resetSigfoxModule() {
	HAL_GPIO_WritePin(SFX_RST_GPIO_Port, SFX_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(SFX_RST_GPIO_Port, SFX_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(250);
}
