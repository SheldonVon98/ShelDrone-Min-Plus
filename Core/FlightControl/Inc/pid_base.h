/*
 * pid.h
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#ifndef ALGORITHM_PID_INC_PID_BASE_H_
#define ALGORITHM_PID_INC_PID_BASE_H_

#include "pid.h"

#define DEFAULT_PID_INTEGRATION_LIMIT 		400.0
#define DEFAULT_PID_OUTPUT_LIMIT      		400.0

void pidInit(PIDObject *pid,
			 const pidInit_t pidParam,
			 const float dt);
float pidUpdate(PIDObject* pid,
				const float error,
				const float desire);
void pidReset(PIDObject* pid);

#endif /* ALGORITHM_PID_INC_PID_BASE_H_ */
