/*
 * L70R.h
 *
 * L70R gps module functions
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

#ifndef L70R_H_
#define L70R_H_

#include "usart.h"
#include "gpio.h"

#define L70_STDY  "$PMTK161,0*28"
#define RMC_1_PFIX "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define RMC_5_PFIX "$PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2D"
#define NMEA_ALL "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define NMEA_NONE "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define L70_RST "$PMTK314,-1*04"
#define NMEA_FIX "$PMTK220,3000*1D"
#define NMEA_PPS "$PMTK220,3000*1D"
struct l70_data {
	uint32_t time;
	uint32_t lat;
	uint8_t lat_ns;
	uint32_t lng;
	uint8_t lng_ew;
	uint8_t speed;
	uint32_t date;
} l70_info;

struct l70_data *l70;

uint8_t gpsbuf[128];
uint8_t *p_gps;

void l70_init();
void l70_standby(uint8_t);
void l70_mode(uint8_t);
void l70_wakeup();
uint8_t l70_parse(uint8_t*, uint16_t);
uint32_t convertRawCoords(uint32_t);
#endif /* L70R_H_ */
