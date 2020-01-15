/*
 * sht30x.c
 *
 * sht30 humidity and temperature sensor funtions
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

#include "usart.h"
#include "i2c.h"
#include "sht30x.h"

void sht30x_init(uint8_t address) {
	sht30x = &sht30x_info;

	sht30x->address = address << 1;
	sht30x->cTemp = 0;
	sht30x->fTemp = 0;
	sht30x->humidity = 0;
}

uint8_t sht30x_get() {
	uint8_t data[6] = { 0 };
	uint8_t config[2] = { 0 };
	uint8_t state = 0;

	int temp = 0;

	//memset(data, 0, 6);

	state = HAL_I2C_IsDeviceReady(&hi2c1, sht30x->address, 2, 2000);

	if (state != HAL_OK) {
		//usart_puts(&huart1, (char*) "SHT30 DEVICE NOT READY\n");
		return 0;
	}

	config[0] = 0x2c;
	config[1] = 0x06;
	//send config data
	state = HAL_I2C_Master_Transmit(&hi2c1, sht30x->address, &config[0], 2,
			1500);

	if (state != HAL_OK) {
		//usart_puts(&huart1, (char*) "SHT30 DEVICE TRANSMIT ERROR\n");
		return 0;
	}
	HAL_Delay(5);
	//receive bytes
	state = HAL_I2C_Master_Receive(&hi2c1, sht30x->address, &data[0], 6, 1500);

	if (state != HAL_OK) {
		//usart_puts(&huart1, (char*) "SHT30 DEVICE RECEIVE ERROR\n");
		return 0;
	}
	HAL_Delay(50);

	temp = (data[0] * 256 + data[1]);
	sht30x->cTemp = (short int) (-45 + (float) (175.0 * (temp / 65535.0)));
	sht30x->fTemp = (short int) (-49 + (float) (315.0 * (temp / 65535.0)));
	sht30x->humidity = (uint8_t) (100
			* (float) ((data[3] * 256 + data[4]) / 65535.0));

	return 1;
}
