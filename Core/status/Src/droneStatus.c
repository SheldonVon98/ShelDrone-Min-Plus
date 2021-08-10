/*
 * controlStatus.c
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#include <droneStatus.h>

DroneStatus droneStatus;

void droneStatusInit(){
	controlStatusInit();
	batteryStatusInit();
}

void detectDroneStatus(){
	detectControlStatus();
	detectBatteryStatus();
}

void batteryStatusInit(){
	droneStatus.batteryStatus = FULL;
}

void controlStatusInit(){
	droneStatus.flightBaseStatus = STANDBY;
	droneStatus.autoStatus = MANUAL;
}

void detectBatteryStatus(){
	if(droneStatus.batteryStatus == DEAD){
		if(batteryData.voltagePC>NORMAL) droneStatus.batteryStatus = NORMAL;
	} else {
		if(batteryData.voltagePC>FULL) 		droneStatus.batteryStatus = FULL;
		else if(batteryData.voltagePC>HIGH) 	droneStatus.batteryStatus = HIGH;
		else if(batteryData.voltagePC>NORMAL) 	droneStatus.batteryStatus = NORMAL;
		else if(batteryData.voltagePC>LOW) 	droneStatus.batteryStatus = LOW;
		else if(batteryData.voltagePC>EXTREME_LOW) droneStatus.batteryStatus = EXTREME_LOW;
		else if(batteryData.voltagePC>DEAD) droneStatus.batteryStatus = DEAD;
	}
}

void detectControlStatus(){
	switch(droneStatus.flightBaseStatus){
	case STANDBY:
		if(ATMIN(channels[THROTTLE]) && ATMAX(channels[YAW])){
			droneStatus.flightBaseStatus = STARTING;
		}
		break;
	case STARTING:
		if(ATMIN(channels[THROTTLE]) && ATMID(channels[YAW])){
			droneStatus.flightBaseStatus = READY;
		}
		break;
	case READY:
		if(!ATMIN(channels[THROTTLE])){
			droneStatus.flightBaseStatus = TAKEOFF;
		} else if(ATMIN(channels[THROTTLE]) && ATMIN(channels[YAW])){
			droneStatus.flightBaseStatus = SHUTTING;
		}
		break;
	case TAKEOFF:
		if(ATMIN(channels[THROTTLE])){
			droneStatus.flightBaseStatus = READY;
		}
		break;
	case SHUTTING:
		if(ATMIN(channels[THROTTLE]) && ATMIN(channels[YAW])){
			droneStatus.flightBaseStatus = STANDBY;
		}
		break;
	}
}
