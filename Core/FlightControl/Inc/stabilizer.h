/*
 * stabilizer.h
 *
 *  Created on: Aug 9, 2021
 *      Author: sheldonvon
 */

#ifndef FLIGHTCONTROL_INC_STABILIZER_H_
#define FLIGHTCONTROL_INC_STABILIZER_H_

#include <SheldroneConfig.h>
#include "attitudeEstimate.h"
#include "pid_process.h"

void stabilizerTaskInit();
void stabilizerTask();
void _stabilizeAttitude();
void _stabilizeAltitude();
void _stabilizeHovering();

#endif /* FLIGHTCONTROL_INC_STABILIZER_H_ */
