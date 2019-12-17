/*
 * sigfox.c
 *
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

void sendSigfoxMessage(char *data) {
	usart_puts(&hlpuart1, (char*) at_messages[DV_TOKEN]);
	while (*data) {
		usart_send(&hlpuart1, *data++);
	}
	usart_send(&hlpuart1, '\r');
}

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
