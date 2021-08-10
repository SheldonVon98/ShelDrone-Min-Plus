/*
 * mpu9250.c
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */


#include "mpu9250.h"

MPU9250_DEVICE mpu9250;

void MPU9250Init(I2C_HandleTypeDef *hi2c){

	mpu9250.i2c = hi2c;
	mpu9250.status = MPU9250_OK;
	HAL_Delay(10);
	MPU9250Reset(); // Hard reset IMU
	if(MPU9250_ID == MPU9250GetDeviceID()){
		MPU9250Sleep(false); // Wake up IMU
		HAL_Delay(10);
		// Set x-axis gyro as clock source
		MPU9250SetClockSource(MPU9250_CLOCK_PLL_XGYRO);
		HAL_Delay(10);
		// Disable interruption
		MPU9250SetInt(false);
		// Connect mag and baro to main IIC
		MPU9250SetBypass(true);
		// Set Gyro full scale
		MPU9250SetFullScaleGyroRange(MPU9250_GYRO_FS_2000);
		// Set Acc full scale
		MPU9250SetFullScaleAccelRange(MPU9250_ACCEL_FS_16);
		// Set Acc digit low pass filter
		MPU9250SetAccelDLPF(MPU9250_ACCEL_DLPF_BW_41);
		// Set sample rate: 1000 / (1 + 0) = 1000Hz
		MPU9250SetRate(0);
		// Set Gyro digit low pass filter
		MPU9250SetDLPFMode(MPU9250_DLPF_BW_98);// 设置陀螺数字低通滤波

//		for (u8 i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
//		{
//			lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
//			lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
//		}

		MPU9250_gyro_calibrate();

	} else {
		mpu9250.status = MPU9250_ID_ERR;
	}
}

void MPU9250ReadData(GyroData *gyroData,
					AccData *accData,
					MagData *magData){
	MPU9250_Read(MPU9250_RA_ACCEL_XOUT_H, 14);
	mpu9250.timer = HAL_GetTick();
	MPU9250_Map_Gyro(gyroData);
	MPU9250_Map_Acc(accData);
	gyroData->dt = (float)(HAL_GetTick() - mpu9250.timer) / 1000;
	accData->dt = gyroData->dt;
}

void MPU9250_Map_Gyro(GyroData *gyroData){
	gyroData->x = ((int16_t)(mpu9250.buffer[8] << 8 | mpu9250.buffer[9])-mpu9250.gyroCali.x) * MPU9250_DEG_PER_LSB_2000;
	gyroData->y = ((int16_t)(mpu9250.buffer[10] << 8 | mpu9250.buffer[11])-mpu9250.gyroCali.y) * MPU9250_DEG_PER_LSB_2000;
	gyroData->z = ((int16_t)(mpu9250.buffer[12] << 8 | mpu9250.buffer[13])-mpu9250.gyroCali.z) * MPU9250_DEG_PER_LSB_2000;
}

void MPU9250_Map_Acc(AccData *accData){
	accData->x = ((int16_t)(mpu9250.buffer[0] << 8 | mpu9250.buffer[1]));
	accData->y = ((int16_t)(mpu9250.buffer[2] << 8 | mpu9250.buffer[3]));
	accData->z = ((int16_t)(mpu9250.buffer[4] << 8 | mpu9250.buffer[5]));
}

void MPU9250_gyro_calibrate(){
	int32_t Gx=0, Gy=0, Gz=0;
	const uint16_t caliNum = 1000;
	for(uint16_t count=0; count<caliNum; count++){
		MPU9250_Read(MPU9250_RA_ACCEL_XOUT_H, 14);
		Gx += ((int16_t)(mpu9250.buffer[8] << 8 | mpu9250.buffer[9]));
		Gy += ((int16_t)(mpu9250.buffer[10] << 8 | mpu9250.buffer[11]));
		Gz += ((int16_t)(mpu9250.buffer[12] << 8 | mpu9250.buffer[13]));
		HAL_Delay(1);
	}
	mpu9250.gyroCali.x = (float)Gx/caliNum;
	mpu9250.gyroCali.y = (float)Gy/caliNum;
	mpu9250.gyroCali.z = (float)Gz/caliNum;
}


uint8_t MPU9250GetDeviceID(){
  MPU9250_Read(MPU9250_RA_WHO_AM_I, 1);
  return mpu9250.buffer[0];
}

void MPU9250Reset(){
	mpu9250.buffer[0] = (0x01 << MPU9250_PWR1_DEVICE_RESET_BIT);
	MPU9250_Write(MPU9250_RA_PWR_MGMT_1, mpu9250.buffer, 1);
	osDelay(100);
}

void MPU9250Sleep(bool enable){
	mpu9250.buffer[0] = enable ? (0x01 << MPU9250_PWR1_SLEEP_BIT): 0x00;
	MPU9250_Write(MPU9250_RA_PWR_MGMT_1, mpu9250.buffer, 1);
}

void MPU9250SetClockSource(uint8_t source){
	mpu9250.buffer[0] = source;
	MPU9250_Write(MPU9250_RA_PWR_MGMT_1, mpu9250.buffer, 1);
}

void MPU9250SetInt(bool enable){
	mpu9250.buffer[0] = enable ? 0x01: 0x00;
	MPU9250_Write(MPU9250_RA_INT_ENABLE, mpu9250.buffer, 1);
}

void MPU9250SetBypass(bool enable){
	mpu9250.buffer[0] = enable ? (0x01 << MPU9250_INTCFG_I2C_BYPASS_EN_BIT): 0x00;
	MPU9250_Write(MPU9250_RA_INT_PIN_CFG, mpu9250.buffer, 1);
}

void MPU9250SetFullScaleGyroRange(uint8_t range){
	mpu9250.buffer[0] = range << 3;
	MPU9250_Write(MPU9250_RA_GYRO_CONFIG, mpu9250.buffer, 1);
}
void MPU9250SetFullScaleAccelRange(uint8_t range){
	mpu9250.buffer[0] = range << 3;
	MPU9250_Write(MPU9250_RA_ACCEL_CONFIG, mpu9250.buffer, 1);
}

void MPU9250SetAccelDLPF(uint8_t range){
	mpu9250.buffer[0] = range;
	MPU9250_Write(MPU9250_RA_ACCEL_CONFIG_2, mpu9250.buffer, 1);
}

void MPU9250SetRate(uint8_t rate){
	mpu9250.buffer[0] = rate;
  	MPU9250_Write(MPU9250_RA_SMPLRT_DIV, mpu9250.buffer, 1);
}

void MPU9250SetDLPFMode(uint8_t mode){
	mpu9250.buffer[0] = mode;
	MPU9250_Write(MPU9250_RA_CONFIG, mpu9250.buffer, 1);
}

uint8_t MPU9250_Read(uint8_t memAddr,
					uint8_t Size){
	mpu9250.i2c_status = HAL_I2C_Mem_Read(mpu9250.i2c,
										MPU9250_ADDR<<1,
										memAddr,
										I2C_MEMADD_SIZE_8BIT,
										mpu9250.buffer,
										Size,
										HAL_MAX_DELAY);
	if(mpu9250.i2c_status != HAL_OK) mpu9250.status = MPU9250_READ_ERR;
	return mpu9250.i2c_status;
}

uint8_t MPU9250_Write(uint8_t Regs_Addr,
					uint8_t *pData,
					uint8_t Size){
	mpu9250.i2c_status = HAL_I2C_Mem_Write(mpu9250.i2c,
										MPU9250_ADDR<<1,
										Regs_Addr,
										I2C_MEMADD_SIZE_8BIT,
										pData,
										Size,
										HAL_MAX_DELAY);
	if(mpu9250.i2c_status != HAL_OK) mpu9250.status = MPU9250_WRITE_ERR;
	return mpu9250.i2c_status;
}
