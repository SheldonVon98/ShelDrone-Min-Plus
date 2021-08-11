/*
 * pid_process.h
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#ifndef ALGORITHM_PID_INC_PID_PROCESS_H_
#define ALGORITHM_PID_INC_PID_PROCESS_H_

#include <droneStatus.h>
#include <algorithm.h>
#include "pid.h"
#include "sensors.h"

#define PITCH_ANGLE_MAX 15
#define ROLL_ANGLE_MAX 15
#define YAW_SPEED_MAX 0.1f

typedef struct{
	float roll;
	float pitch;
	float yaw;

	float roll_speed;
	float pitch_speed;
	float yaw_speed;
} SetPoints;

void pidsInit();
void pidProcess();
void setPointReset(SetPoints *sp);

extern PIDObject pid_pitch_speed, pid_roll_speed, pid_yaw_speed;
#endif /* ALGORITHM_PID_INC_PID_PROCESS_H_ */
