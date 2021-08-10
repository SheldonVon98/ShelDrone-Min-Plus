/*
 * motor.h
 *
 *  Created on: Jul 17, 2021
 *      Author: sheldonvon
 */

#ifndef MOTOR_INC_MOTOR_H_
#define MOTOR_INC_MOTOR_H_

#include <droneStatus.h>
#include <SheldroneConfig.h>

#define MOTOR_STOP	1000
#define MOTOR_READY	1200
#define MOTOR_MAX	1980
/*
 * Motors Orientation.
 *
 * 			  Back
 *		   CCW     CW
 * 			m1     m2
 *			  \   /
 * 			   \ /
 * 	  Right	   |||    Left
 * 			   / \
 * 			  /	  \
 * 			m3	   m4
 *			CW	   CCW
 *			  Front
 * */

/*
 *	+pitch -> tilting down.
 *	+roll  -> right wing moving down.
 *	+yaw  -> turning left.
 * */

typedef struct{
	__IO uint32_t *M1;
	__IO uint32_t *M2;
	__IO uint32_t *M3;
	__IO uint32_t *M4;
	uint16_t *throttleManual;
	float batteryCompensate;
} MotorData;

void motorInit(TIM_HandleTypeDef *tim, uint16_t *throttleChannel);
void setMotorThrottle(__IO uint32_t *m, uint16_t throttle);
void mapChannel2Motor();
void setMotors3D(int16_t pitch,
				int16_t roll,
				int16_t yaw);

void setMotors6D(int16_t pitch,
				int16_t roll,
				int16_t yaw,
				int16_t x,
				int16_t y,
				int16_t z);


#endif /* MOTOR_INC_MOTOR_H_ */
