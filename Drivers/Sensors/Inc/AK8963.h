/*
 * AK8963.h
 *
 *  Created on: 6 Aug 2021
 *      Author: sheldonvon
 */

#ifndef SENSORS_INC_AK8963_H_
#define SENSORS_INC_AK8963_H_

#include <SheldroneConfig.h>
#include "sensor_types.h"
#include <stdbool.h>

typedef struct{
	I2C_HandleTypeDef *i2c;
	uint8_t buffer[6];
//	enum AK8963_STATUS status;
	uint8_t i2c_status;
	uint32_t timer;
}AK8963_DEVICE;

void AK8963Init(I2C_HandleTypeDef *hi2c);
void AK8963ReadData(MagData *magData);
void AK8963_read();
void AK8963_write();

#endif /* SENSORS_INC_AK8963_H_ */
