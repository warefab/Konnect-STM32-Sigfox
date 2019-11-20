/*
 * sigfox.h
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

#ifndef SIGFOX_H_
#define SIGFOX_H_

#include "usart.h"

enum Message{
	VERSION,
	ID,
	PAC,
	TOKEN
};

void sendSigfoxMessage(char*);
void CheckSigfoxVersion(enum Message);


#endif /* SIGFOX_H_ */
