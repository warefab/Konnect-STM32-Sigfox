/*
 * utils.c
 *
 *  Created on: Jan 15, 2020
 *      Author: Muchiri John
 */
#include "utils.h"

char* dateTimeFormat(char *buf_tk, uint32_t dt_, char sep) {
	uint8_t tk1 = (dt_ / 10000.0);
	uint8_t tk2 = ((uint16_t) (dt_ / 100.0)) % 100;
	uint8_t tk3 = (uint8_t) (dt_ % 100);
	sprintf(buf_tk, "%02d%c%02d%c%02d", tk1, sep, tk2, sep, tk3);
	return buf_tk;
}

uint8_t adcToPercent(uint16_t val) {
	uint8_t pct = ceil((val * 100) / 4095.0);
	return pct;
}
