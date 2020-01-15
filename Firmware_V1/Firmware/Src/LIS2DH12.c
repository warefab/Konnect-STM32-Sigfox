/*
 * LIS2DH12.c
 *
 * lis2dh12 3-axis sensor library
 * for Warefab Konnect STM32L0-SIGFOX Development Board
 *
 * Created on: Nov 13, 2019
 * Author: Muchiri John
 * (c) wwww.warefab.com
 *
 * Based on: https://os.mbed.com/users/jurica238814/code/Lis2dh12/
 *
 * This software component is licensed by warefab under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 * opensource.org/licenses/BSD-3-Clause
 */

#include <stdio.h>

#include "usart.h"
#include "i2c.h"
#include "LIS2DH12.h"

void Lis2dh12_init(uint8_t address) {
	uint8_t state = 0;
	Lis2dh12 = &Lis2dh12_info;

	Lis2dh12->address = address << 1;
	Lis2dh12->x = 0;
	Lis2dh12->y = 0;
	Lis2dh12->z = 0;

	//check if device ready and setup
	state = Lis2dh12_whoIAm();
	if (state == 0x33) {
		Lis2dh12_int1Setup(INT1_CFG);
		Lis2dh12_setMode(HIGH_RES);
		Lis2dh12_setScale(_8g);
		Lis2dh12_setODR(ODR_10Hz);
		Lis2dh12_enableAxes(X_axis);
		Lis2dh12_enableAxes(Y_axis);
		Lis2dh12_enableAxes(Z_axis);
	}
}

uint8_t Lis2dh12_whoIAm() {

	uint8_t config;
	uint8_t state = 0;

	//char buf[12];

	state = HAL_I2C_IsDeviceReady(&hi2c1, Lis2dh12->address, 2, 2000);

	if (state != HAL_OK) {
		//usart_puts(&huart1, (char*) "LIS2DH DEVICE NOT READY\n");
		return 0;
	}

	state = HAL_I2C_Mem_Read(&hi2c1, Lis2dh12->address, WHO_AM_I, 1, &config, 1,
			1500);

	if (config != 0x33) {
		//usart_puts(&huart1, (char*) " LISD2DH NOT FOUND\n");
		return 0;
	}
	//sprintf(buf, "READ - 0x%X\n", config);
	//usart_puts(&huart1, buf);

	return config;
}

uint8_t Lis2dh12_getXYZ() {
	Lis2dh12->x = Lis2dh12_readXAxis();
	HAL_Delay(1);
	Lis2dh12->y = Lis2dh12_readYAxis();
	HAL_Delay(1);
	Lis2dh12->z = Lis2dh12_readZAxis();
	HAL_Delay(1);

	return 1;
}

uint8_t Lis2dh12_setMode(enum Mode_ mode) {
	char ctrl1Copy;
	char ctrl4Copy;
	uint8_t success;

	Lis2dh12_readFromReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	Lis2dh12_readFromReg((char) CTRL_REG4, (uint8_t*) &ctrl4Copy, 1);

	switch (mode) {
	case HIGH_RES:
		ctrl1Copy &= 0xF7;
		ctrl4Copy |= 0x08;
		break;
	case NORMAL:
		ctrl1Copy &= 0xF7;
		ctrl4Copy &= 0xF7;
		break;
	case LOW_POWER:
		ctrl1Copy |= 0x08;
		ctrl4Copy &= 0xF7;
		break;
	default:
		return 0;
	}
	Lis2dh12_writeToReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	success = Lis2dh12_writeToReg((char) CTRL_REG4, (uint8_t*) &ctrl4Copy, 1);
	return success;
}

uint8_t Lis2dh12_enableAxes(enum Axis_ axis) {
	char ctrl1Copy;
	Lis2dh12_readFromReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	ctrl1Copy |= axis;
	Lis2dh12_writeToReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	return 0;
}

uint8_t Lis2dh12_disableAxes(enum Axis_ axis) {
	char ctrl1Copy;
	Lis2dh12_readFromReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	ctrl1Copy &= ~(1 << axis);
	Lis2dh12_writeToReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	return 0;
}

int16_t Lis2dh12_readXAxis() {
	int16_t rawData;
	uint8_t tempData;
	// Make sure new data is ready
	do {
		Lis2dh12_readFromReg((uint8_t) STATUS, (uint8_t*) &tempData, 1);
	} while (!(tempData & 0x08));
	do {
		Lis2dh12_readFromReg((uint8_t) STATUS, (uint8_t*) &tempData, 1);
	} while (!(tempData & 0x80));
	// Same data have been overwritten

	//Lis2dh12_readFromReg((char)OUT_X_H, (uint8_t*)&tempData, 1);
	//rawData = (int8_t)tempData << 8;
	Lis2dh12_readFromReg((uint8_t) OUT_X_L, (uint8_t*) &rawData, 1);
	Lis2dh12_readFromReg((uint8_t) OUT_X_H, ((uint8_t*) &rawData) + 1, 1);

	if (rawData >= 0)
		rawData = (rawData >> 4);
	else
		rawData = (rawData >> 4) | 0xF000;

	return rawData;
}

int16_t Lis2dh12_readYAxis() {
	int16_t rawData;
	//char tempData;
	//Lis2dh12_readFromReg((char)OUT_Y_H, (uint8_t*)&tempData, 1);
	//rawData = (int8_t)tempData << 8;
	Lis2dh12_readFromReg((uint8_t) OUT_Y_L, (uint8_t*) &rawData, 1);
	Lis2dh12_readFromReg((uint8_t) OUT_Y_H, ((uint8_t*) &rawData) + 1, 1);

	if (rawData >= 0)
		rawData = (rawData >> 4);
	else
		rawData = (rawData >> 4) | 0xF000;

	return rawData;
}

int16_t Lis2dh12_readZAxis() {
	int16_t rawData = 0;
	//char tempData;
	//Lis2dh12_readFromReg((char)OUT_Z_H, (uint8_t*)&tempData, 1);
	//rawData = (int8_t)tempData << 8;
	Lis2dh12_readFromReg((uint8_t) OUT_Z_L, (uint8_t*) &rawData, 1);
	Lis2dh12_readFromReg((uint8_t) OUT_Z_H, ((uint8_t*) &rawData) + 1, 1);

	if (rawData >= 0)
		rawData = (rawData >> 4);
	else
		rawData = (rawData >> 4) | 0xF000;

	return rawData;
}

uint8_t Lis2dh12_setODR(enum Odr_ odr) {
	char ctrl1Copy;
	Lis2dh12_readFromReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	ctrl1Copy |= (odr << 4);
	Lis2dh12_writeToReg((char) CTRL_REG1, (uint8_t*) &ctrl1Copy, 1);
	return 0;
}

uint8_t Lis2dh12_setScale(enum Scale_ scale) {
	char ctrl4Copy;
	Lis2dh12_readFromReg((char) CTRL_REG4, (uint8_t*) &ctrl4Copy, 1);
	ctrl4Copy |= (scale << 4);
	Lis2dh12_writeToReg((char) CTRL_REG4, (uint8_t*) &ctrl4Copy, 1);
	return 0;
}

/* Interrupt activity 1 driven to INT1 pad */
uint8_t Lis2dh12_int1Setup(uint8_t setup) {
	char data = setup;
	Lis2dh12_writeToReg((char) CTRL_REG3, (uint8_t*) &data, 1);
	return 0;
}

uint8_t Lis2dh12_int1Latch(uint8_t enable) {
	char ctrl5Copy;
	Lis2dh12_readFromReg((char) CTRL_REG5, (uint8_t*) &ctrl5Copy, 1);
	ctrl5Copy |= enable;
	Lis2dh12_writeToReg((char) CTRL_REG5, (uint8_t*) &ctrl5Copy, 1);
	return 0;
}

uint8_t Lis2dh12_int1Duration(uint8_t duration) {
	char copy = duration;
	Lis2dh12_writeToReg((char) INT1_DURATION, (uint8_t*) &copy, 1);
	return 0;
}

uint8_t Lis2dh12_int1Threshold(uint8_t threshold) {
	char copy = threshold;
	Lis2dh12_writeToReg((char) INT1_THS, (uint8_t*) &copy, 1);
	return 0;
}

uint8_t Lis2dh12_int1Config(uint8_t config) {
	char copy = config;
	Lis2dh12_writeToReg((char) INT1_CFG, (uint8_t*) &copy, 1);
	return 0;
}

void Lis2dh12_clearIntFlag() {
	char data;
	Lis2dh12_readFromReg((char) INT1_SRC, (uint8_t*) &data, 1);
}

uint8_t Lis2dh12_readFromReg(uint8_t regAddr, uint8_t *buff, uint8_t buffSize) {
	/*HAL_I2C_Mem_Read(&hi2c1, Lis2dh12->address, WHO_AM_I, 1, &config,
	 1, 1500);*/
	//HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
	uint8_t retVal = 0;

	retVal = HAL_I2C_Mem_Read(&hi2c1, Lis2dh12->address, regAddr, 1, buff,
			buffSize, 1000);
	//retVal = i2c.readFromReg((char)regAddr, (char*)buff, buffSize);

	return retVal;
}

uint8_t Lis2dh12_writeToReg(uint8_t regAddr, uint8_t *buff, uint8_t buffSize) {
	uint8_t retVal = 0;

	//HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)

	retVal = HAL_I2C_Mem_Write(&hi2c1, Lis2dh12->address, regAddr, 1, buff,
			buffSize, 1000);

	//retVal = i2c.writeToReg((char)regAddr, (char*)buff, buffSize);

	return retVal;
}
