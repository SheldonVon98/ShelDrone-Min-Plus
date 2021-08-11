/*
 * sensors.c
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */


#include "sensors.h"

extern I2C_HandleTypeDef hi2c1;
SensorData sensorData;

void sensorsInit(COM_PROTC *com_porotc){
#ifdef USE_MPU9250
	MPU9250Init(com_porotc);
#endif
#ifdef USE_MPU6050
	IMU_CONFIG imu_config;
	imu_config.powerOn = MPU6050_POWERON;
	imu_config.sampleRate = IMU_RATE_1000;
	imu_config.DLPF_config = DLPF_43HZ;
	imu_config.ACC_config = ACCEL_RANGE_1000;
	imu_config.GYRO_config = GYRO_RANGE_4G;
#ifdef USE_IT
	imu_config.enable_it = INT_ENABLE_ENABLE;
#else
	imu_config.enable_it = INT_ENABLE_DISABLE;
#endif
	MPU6050_Init(com_porotc, imu_config);
#endif
}

void sensorsRead(){
#ifdef USE_MPU9250
	MPU9250ReadData(&sensorData.gyro, &sensorData.acc, &sensorData.mag);
#endif
#ifdef USE_MPU6050
	MPU6050_ReadData(&sensorData);
#endif
}

void sensorCompose(){
#ifdef USE_MPU6050
	MPU6050_Compose_Export(&sensorData);
#endif
}
