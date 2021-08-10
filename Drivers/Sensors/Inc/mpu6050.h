/*
 * mpu6050.h
 *
 *  Created on: 20 Jul 2021
 *      Author: sheldonvon
 */

#ifndef SENSOR_INC_MPU6050_H_
#define SENSOR_INC_MPU6050_H_

#include <SheldroneConfig.h>
#include <algorithm.h>
#include <fusion.h>
#include <math.h>
#include <stdlib.h>
#include <error.h>
#include "cmsis_os.h"
//#define CALIBRATE_MPU6050
#define MPU6050_USE_DMA
#define USE_IT

#define A_X_BIAS -957
#define A_Y_BIAS 5308
#define A_Z_BIAS 0

#define G_X_BIAS -185
#define G_Y_BIAS 126
#define G_Z_BIAS -20

#define WHO_AM_I_REG 		0x75
#define MPU6050_ADDR		0xD0
#define PWR_MGMT_1_REG 		0x6B
#define MPU6050_POWERON		0x00

#define SMPLRT_DIV_REG 		0x19
#define IMU_RATE_250		0x03
#define IMU_RATE_1000		0x00


#define INT_ENABLE_REG 		0x38
#define INT_ENABLE_ENABLE	0x01
#define INT_ENABLE_DISABLE	0x00

#define TEMP_OUT_H_REG 		0x41

#define CONFIG_REG			0x1A
#define DLPF_43HZ			0x03
#define DLPF_256HZ			0x01

#define ACCEL_CONFIG_REG 	0x1C
#define ACCEL_XOUT_H_REG 	0x3B
#define ACCEL_RANGE_250		0x00
#define ACCEL_RANGE_500		0x08
#define ACCEL_RANGE_1000	0x10
#define ACCEL_RANGE_2000	0x18

#define GYRO_CONFIG_REG 	0x1B
#define GYRO_XOUT_H_REG 	0x43
#define GYRO_RANGE_2G		0x00
#define GYRO_RANGE_4G		0x08
#define GYRO_RANGE_8G		0x10
#define GYRO_RANGE_16G		0x18

#define MPU6050_DATASTART_REG ACCEL_XOUT_H_REG

typedef struct{
	uint8_t powerOn;
	uint8_t sampleRate;
	uint8_t DLPF_config;
	uint8_t ACC_config;
	uint8_t GYRO_config;
	uint8_t enable_it;
}MPU6050_CONFIG;

typedef struct{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax; // roll
    double Ay; // pitch
    double Az; // yaw
    double Ax2;
    double Ay2;
    double Az2;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx; //pitch
    double Gy; //roll
    double Gz; //yaw

    float Temperature;

    const double Ax_cali;
    const double Ay_cali;
    const double Az_cali;
    double Gx_cali;
    double Gy_cali;
    double Gz_cali;

}MPU6050_Data;

#include "sensors.h"
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_CONFIG config);
uint8_t MPU_Read(uint8_t memAddr,
				uint8_t *pData,
				uint8_t Size);
uint8_t MPU6050_Write(uint8_t Regs_Addr,
					uint8_t *pData,
					uint8_t Size);
void MPU6050_Read_Gyro(MPU6050_Data *data);
void MPU6050_Read_Acc(MPU6050_Data *data);
void MPU6050_ReadData(SensorData *data);
void MPU6050_Compose(SensorData *sData, MPU6050_Data *data, uint8_t *raw);
void MPU6050_Compose_Export();
void mpu6050_gyro_calibrate();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#ifdef USE_CALIBRATE
void MPU6050_manual_calibate(MPU6050_Data *data);
#endif
#endif /* SENSOR_INC_MPU6050_H_ */
