/*
 * attitudeEstimate.h
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */

#ifndef FLIGHTCONTROL_INC_ATTITUDEESTIMATE_H_
#define FLIGHTCONTROL_INC_ATTITUDEESTIMATE_H_

#include <SheldroneConfig.h>
#include <sensors.h>
#include "fusion.h"

void attitudeEstimateTaskInit();
void attitudeEstimateTask();

#endif /* FLIGHTCONTROL_INC_ATTITUDEESTIMATE_H_ */
