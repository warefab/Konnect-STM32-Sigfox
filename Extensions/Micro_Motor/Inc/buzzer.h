/*
 * buzzer.h
 *
 * Buzzer functions
 * Warefab Konnect STM32L0-SIGFOX Development Board
 *
 * Created on: Dec 16, 2019
 * Author: Muchiri John
 * (c) wwww.warefab.com
 *
 * This software component is licensed by warefab under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 * opensource.org/licenses/BSD-3-Clause
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include <stdint.h>
#include "tim.h"

#define BUZZER_TIM_FREQ 100000
#define BUZZER_MAX_FREQ 2000
#define BUZZER_MIN_FREQ 50
void setChannel(uint16_t, uint8_t);
void setBuzzerFreq(uint16_t);
void stopBuzzer();
void driveMotor(uint16_t);

#endif /* BUZZER_H_ */
