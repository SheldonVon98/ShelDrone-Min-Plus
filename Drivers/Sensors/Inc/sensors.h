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

//#define USE_MPU9250
#define USE_MPU6050

#ifdef USE_MPU9250
#include "mpu9250.h"
typedef I2C_HandleTypeDef 	COM_PROTC;
#endif
#ifdef USE_MPU6050
#include "mpu6050.h"
typedef I2C_HandleTypeDef 	COM_PROTC;
typedef MPU6050_CONFIG		IMU_CONFIG;
#endif

void sensorsInit();
void sensorsRead();
void sensorCompose();
extern SensorData sensorData;
#endif /* SENSORS_INC_SENSORS_H_ */
