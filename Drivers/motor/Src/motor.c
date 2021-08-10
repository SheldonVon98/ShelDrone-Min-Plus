/*
 * motor.c
 *
 *  Created on: Jul 17, 2021
 *      Author: sheldonvon
 */

#include "motor.h"

MotorData motorData;

void motorInit(TIM_HandleTypeDef *tim, uint16_t *throttleChannel){
	motorData.M1 = &tim->Instance->CCR1;
	motorData.M2 = &tim->Instance->CCR2;
	motorData.M3 = &tim->Instance->CCR3;
	motorData.M4 = &tim->Instance->CCR4;
	motorData.throttleManual = throttleChannel;
	motorData.batteryCompensate = 0;
}

void setMotorThrottle(__IO uint32_t *m, uint16_t throttle){
	if(droneStatus.flightBaseStatus == STANDBY
			|| droneStatus.flightBaseStatus == STARTING
			|| droneStatus.flightBaseStatus == SHUTTING
			|| droneStatus.batteryStatus == DEAD){
		*m = MOTOR_STOP;
	} else if(droneStatus.flightBaseStatus == READY || droneStatus.flightBaseStatus == TAKEOFF){
		if(throttle < MOTOR_READY) throttle = MOTOR_READY;
		else if(throttle < MOTOR_MAX){
			if(batteryData.voltagePC > EXTREME_LOW && batteryData.voltagePC < HIGH){
				motorData.batteryCompensate = ((float)HIGH - batteryData.voltagePC)*VOLTAGE_COM_CONST;
			} else motorData.batteryCompensate = 0;
			throttle += motorData.batteryCompensate;
		}
		if(throttle > MOTOR_MAX) throttle = MOTOR_MAX;
		*m = throttle;
	}
}

void mapChannel2Motor(){
	*motorData.M1 = *motorData.throttleManual;
	*motorData.M2 = *motorData.throttleManual;
	*motorData.M3 = *motorData.throttleManual;
	*motorData.M4 = *motorData.throttleManual;
}

void setMotors3D(int16_t pitch,
				int16_t roll,
				int16_t yaw){

	setMotorThrottle(motorData.M1, *motorData.throttleManual+pitch-roll-yaw);
	setMotorThrottle(motorData.M2, *motorData.throttleManual+pitch+roll+yaw);
	setMotorThrottle(motorData.M3, *motorData.throttleManual-pitch-roll+yaw);
	setMotorThrottle(motorData.M4, *motorData.throttleManual-pitch+roll-yaw);

}

void setMotors6D(int16_t pitch,
				int16_t roll,
				int16_t yaw,
				int16_t x,
				int16_t y,
				int16_t z){

}
