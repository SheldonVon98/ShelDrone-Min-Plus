/*
 * pic.h
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#ifndef ALGORITHM_PID_INC_PID_H_
#define ALGORITHM_PID_INC_PID_H_


typedef struct{
	float desired;		//< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	float dt;           //< delta-time dt
	float out;			//< out
}PIDObject;


typedef struct{
	float kp;
	float ki;
	float kd;
}pidInit_t;

#include "pid_base.h"
#include "pid_process.h"

extern PIDObject pid_pitch;
extern PIDObject pid_roll;
extern PIDObject pid_yaw;
#endif /* ALGORITHM_PID_INC_PID_H_ */
