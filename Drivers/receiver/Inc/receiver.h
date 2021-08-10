/*
 * receiver.h
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#ifndef RECEIVER_INC_RECEIVER_H_
#define RECEIVER_INC_RECEIVER_H_

#include <SheldroneConfig.h>

#define ROLL 		0
#define PITCH 		1
#define THROTTLE 	2
#define YAW 		3
#define SWITCHER 	4
#define TWISTER 	5

#define CHANNEL_MIN 	1020
#define CHANNEL_MAX 	1980
#define CHANNEL_MID_MAX 1520
#define CHANNEL_MID_MIN 1480

#define ATMIN(n) (n<=CHANNEL_MIN)
#define ATMAX(n) (n>=CHANNEL_MAX)
#define ATMID(n) (n>=CHANNEL_MID_MIN && n<=CHANNEL_MID_MAX)

#define ATHIGH(n) 	(n>=CHANNEL_MID_MAX)
#define ATLOW(n)	(n<=CHANNEL_MID_MIN)

extern uint16_t channels[6];
extern TIM_HandleTypeDef htim11;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* RECEIVER_INC_RECEIVER_H_ */
