/*
 * LIS2DH12.h
 *
 * lis2dh12 3-axis sensor functions
 * Warefab Konnect STM32L0-SIGFOX Development Board
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

#ifndef LIS2DH12_H_
#define LIS2DH12_H_

#define WHO_AM_I        (0x0F)
#define CTRL_REG0       (0x1E)
#define CTRL_REG1       (0x20)
#define CTRL_REG2       (0x21)
#define CTRL_REG3       (0x22)
#define CTRL_REG4       (0x23)
#define CTRL_REG5       (0x24)
#define CTRL_REG6       (0x25)
#define REFERENCE       (0x26)
#define STATUS          (0x27)
#define OUT_X_L         (0x28)
#define OUT_X_H         (0x29)
#define OUT_Y_L         (0x2A)
#define OUT_Y_H         (0x2B)
#define OUT_Z_L         (0x2C)
#define OUT_Z_H         (0x2D)
#define FIFO_CTRL_REG   (0x2E)
#define FIFO_SRC_REG    (0x2F)
#define INT1_CFG        (0x30)
#define INT1_SRC        (0x31)
#define INT1_THS        (0x32)
#define INT1_DURATION   (0x33)
#define INT2_CFG        (0x34)
#define INT2_SRC        (0x35)
#define INT2_THS        (0x36)
#define INT2_DURATION   (0x37)
#define CLICK_CFG       (0x38)
#define CLICK_SRC       (0x39)
#define CLICK_THS       (0x3A)
#define TIME_LIMIT      (0x3B)
#define TIME_LATENCY    (0x3C)
#define TIME_WINDOW     (0x3D)
#define ACT_THS         (0x3E)
#define ACT_DUR         (0x3F)

struct Lis2dh12_data {
	uint8_t address;
	int16_t x;
	int16_t y;
	int16_t z;
} Lis2dh12_info;

struct Lis2dh12_data *Lis2dh12;

enum Mode_ {
	HIGH_RES = 0, NORMAL, LOW_POWER,
};

enum Axis_ {
	X_axis = 0x01, Y_axis = 0x02, Z_axis = 0x04,
};

enum Odr_ {
	PowerDown = 0x00,
	ODR_1Hz = 0x01,
	ODR_10Hz = 0x02,
	ODR_25Hz = 0x03,
	ODR_50Hz = 0x04,
	ODR_100Hz = 0x05,
	ODR_200Hz = 0x06,
	ODR_400Hz = 0x07,
	ODR_1620Hz = 0x08,
	ODR_Max = 0x09,         // HighRes/Normal -> 1.344kHz, LowPower -> 5.376kHz
};

enum Scale_ {
	_2g = 0x00, _4g = 0x01, _8g = 0x02, _16g = 0x03,
};
uint8_t Lis2dh12_readFromReg(uint8_t regAddr, uint8_t *buff, uint8_t buffSize);
uint8_t Lis2dh12_writeToReg(uint8_t regAddr, uint8_t *buff, uint8_t buffSize);

void Lis2dh12_init(uint8_t address);
uint8_t Lis2dh12_whoIAm();
uint8_t Lis2dh12_setMode(enum Mode_ mode);
uint8_t Lis2dh12_enableAxes(enum Axis_ axis);
uint8_t Lis2dh12_disableAxes(enum Axis_ axis);
int16_t Lis2dh12_readXAxis();
int16_t Lis2dh12_readYAxis();
int16_t Lis2dh12_readZAxis();
uint8_t Lis2dh12_setODR(enum Odr_ odr);
uint8_t Lis2dh12_setScale(enum Scale_ scale);
uint8_t Lis2dh12_int1Setup(uint8_t setup);
uint8_t Lis2dh12_int1Latch(uint8_t enable);
uint8_t Lis2dh12_int1Duration(uint8_t duration);
uint8_t Lis2dh12_int1Threshold(uint8_t threshold);
uint8_t Lis2dh12_int1Config(uint8_t config);
void clearIntFlag();

uint8_t Lis2dh12_getXYZ();

#endif /* LIS2DH12_H_ */
