/*
 * HMC5883L.c
 *
 *  Created on: Jul 23, 2021
 *      Author: sheldonvon
 */

#include "HMC5883L.h"

I2C_HandleTypeDef *i2c_multi;
uint8_t	magBuf[12];
float mag_angle, x, y, z = 0;
int16_t mx, my, mz = 0;

void HMC5883L_Init(I2C_HandleTypeDef *hi2c) {
	i2c_multi = hi2c;
	// Magnetometer HMC5883L Init
	// Continuous measurement mode for Magnetometer
	magBuf[0] = 0x78;
//	magBuf[1] = 0x20;
//	magBuf[2] = 0x00;
//	HMC5883L_Write(HMC5883L_RA_MODE, 0x00, 1);
	HMC5883L_Write(HMC5883L_RA_CONFIG_A, magBuf, 3);

	HAL_Delay(10);
}

uint8_t HMC5883L_Write(uint8_t Regs_Addr, uint8_t *pData, uint8_t Size){
	return HAL_I2C_Mem_Write(i2c_multi, (uint16_t)HMC5883L_ADDRESS << 1, Regs_Addr, I2C_MEMADD_SIZE_8BIT, pData, Size, I2C_TIMEOUT);
}

uint8_t HMC5883L_Read(uint8_t memAddr, uint8_t Size){
	return HAL_I2C_Mem_Read(i2c_multi, (uint16_t)HMC5883L_ADDRESS << 1, memAddr, I2C_MEMADD_SIZE_8BIT, magBuf, Size, I2C_TIMEOUT);
}


uint16_t HMC5883L_GetAngle(void) {

	HMC5883L_Read(HMC5883L_RA_DATAX_H, 6);
	mx = (int16_t)magBuf[0]<<8 | magBuf[1];
	mz = (int16_t)magBuf[2]<<8 | magBuf[3];
	my = (int16_t)magBuf[4]<<8 | magBuf[5];

	x = mx * 0.92;
	y = my * 0.92;
	z = mz * 0.92;

	// Calculate angle
	mag_angle = atan2(y, x);
	if (mag_angle < 0)
		mag_angle += 2*M_PI;

	if (mag_angle > 2*M_PI)
		mag_angle -= 2*M_PI;

	mag_angle *= (180 / M_PI);

	return mag_angle;
}
