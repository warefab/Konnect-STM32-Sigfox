/*
 * utils.h
 *
 *  Created on: Jan 15, 2020
 *      Author: Muchiri John
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

char* dateTimeFormat(char *bf_tk, uint32_t, char);
uint8_t adcToPercent(uint16_t);

#endif /* UTILS_H_ */
