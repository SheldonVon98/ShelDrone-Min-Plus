/*
 * sensors.h
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 *
 *  The interfaces to all sensors
 */

#ifndef SENSORS_INC_SENSORS_H_
#define SENSORS_INC_SENSORS_H_

#include <SheldroneConfig.h>
#include "sensor_types.h"

#define USE_MPU9250

#ifdef USE_MPU9250
#include "mpu9250.h"
#endif

void sensorsInit();
void sensorsRead();

extern SensorData sensorData;
#endif /* SENSORS_INC_SENSORS_H_ */
