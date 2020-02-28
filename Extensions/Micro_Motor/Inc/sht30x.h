/*
 * sht30x.h
 *
 * sht30 humidity and temperature sensor functions
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
 */

#ifndef SHT30X_H_
#define SHT30X_H_

#include "i2c.h"

void sht30x_init(uint8_t address);
uint8_t sht30x_get();
struct sht30x_data{
	uint8_t address;
	short int cTemp;
	short int fTemp;
	uint8_t humidity;
}sht30x_info;

struct sht30x_data *sht30x;

#endif /* SHT30X_H_ */
