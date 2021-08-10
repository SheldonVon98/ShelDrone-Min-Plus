/*
 * controlStatus.h
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#ifndef STATUS_CONTROLSTATUS_INC_CONTROLSTATUS_H_
#define STATUS_CONTROLSTATUS_INC_CONTROLSTATUS_H_

#include "receiver.h"
#include <stdbool.h>
#include "battery.h"

#define VOLTAGE_COM_CONST 4.0

enum FLIGHT_BASE_STATUS{
	STANDBY,
	STARTING,
	READY,
	TAKEOFF,
	SHUTTING
};

enum AUTO_STATUS{
	MANUAL,
	ALTITUDE_HOLD,
	ALTITUDE_POSITION_HOLD,
};

enum BATTERY_STATUS{
	FULL=90, // 90-100%
	HIGH=80, // 80-90%
	NORMAL=50, // 50-80%
	LOW=15, // 15-50%
	EXTREME_LOW=1, // 1-15%
	DEAD=0 // 0-1%
};

typedef struct{
	enum FLIGHT_BASE_STATUS flightBaseStatus;
	enum AUTO_STATUS autoStatus;
	enum BATTERY_STATUS batteryStatus;
}DroneStatus;

extern DroneStatus droneStatus;

void droneStatusInit();
void detectDroneStatus();
void batteryStatusInit();
void detectBatteryStatus();
void controlStatusInit();
void detectControlStatus();

#endif /* STATUS_CONTROLSTATUS_INC_CONTROLSTATUS_H_ */
