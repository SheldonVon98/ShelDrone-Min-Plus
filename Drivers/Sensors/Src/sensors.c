/*
 * sensors.c
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */


#include "sensors.h"

extern I2C_HandleTypeDef hi2c1;
SensorData sensorData;

void sensorsInit(){
#ifdef USE_MPU9250
	MPU9250Init(&hi2c1);
#endif
}

void sensorsRead(){
#ifdef USE_MPU9250
	MPU9250ReadData(&sensorData.gyro, &sensorData.acc, &sensorData.mag);
#endif
}
