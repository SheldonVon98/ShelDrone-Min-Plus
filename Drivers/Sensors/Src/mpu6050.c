/*
 * mpu6050.c
 *
 *  Created on: 20 Jul 2021
 *      Author: sheldonvon
 */

#include "mpu6050.h"

MPU6050_Data mpu6050Data;
I2C_HandleTypeDef *i2c;
uint8_t rawData[14];
uint8_t status;

uint32_t timer;
double dt;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_CONFIG config){
	i2c = hi2c;
    uint8_t check;
    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, I2C_TIMEOUT);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        MPU6050_Write(PWR_MGMT_1_REG, &config.powerOn, 1);
        HAL_Delay(100);
        MPU6050_Write(ACCEL_CONFIG_REG, &config.ACC_config, 1);
        MPU6050_Write(GYRO_CONFIG_REG, &config.GYRO_config, 1);
        MPU6050_Write(CONFIG_REG, &config.DLPF_config, 1);
		MPU6050_Write(INT_ENABLE_REG, &config.enable_it, 1);
		MPU6050_Write(SMPLRT_DIV_REG, &config.sampleRate, 1);
		HAL_Delay(100);
	    LEDSetTimes(RED, 4);
	    mpu6050_gyro_calibrate();
	    LEDSetTimes(RED, 2);
	    LEDSetTimes(GREEN, 3);
        return 0;
    }

    return 1;
}

uint8_t MPU6050_Read(uint8_t memAddr, uint8_t Size){
#ifdef MPU6050_USE_DMA
	status = HAL_I2C_Mem_Read_DMA(i2c, MPU6050_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, rawData, Size);
#else
	status = HAL_I2C_Mem_Read(i2c, MPU6050_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, rawData, Size, I2C_TIMEOUT);
#endif
	return status;
}

uint8_t MPU6050_Write(uint8_t Regs_Addr, uint8_t *pData, uint8_t Size){
	status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, Regs_Addr, I2C_MEMADD_SIZE_8BIT, pData, Size, I2C_TIMEOUT);
	return status;
}

void MPU6050_Map_Acc(MPU6050_Data *data, uint8_t *raw){
	data->Accel_X_RAW = (int16_t)(raw[0] << 8 | raw[1]);
	data->Accel_Y_RAW = (int16_t)(raw[2] << 8 | raw[3]);
	data->Accel_Z_RAW = (int16_t)(raw[4] << 8 | raw[5]);

	data->Ax = data->Accel_X_RAW - A_X_BIAS;
	data->Ay = data->Accel_Y_RAW - A_Y_BIAS;
	data->Az = data->Accel_Z_RAW - A_Z_BIAS;
}

void MPU6050_Map_Temperature(MPU6050_Data *data, uint8_t *raw){
	data->Temperature = (int16_t)(raw[6] << 8 | raw[7]);
}

void MPU6050_Map_Gyro(MPU6050_Data *data, uint8_t *raw){
	data->Gyro_X_RAW = (int16_t)(raw[8] << 8 | raw[9]);
	data->Gyro_Y_RAW = (int16_t)(raw[10] << 8 | raw[11]);
	data->Gyro_Z_RAW = (int16_t)(raw[12] << 8 | raw[13]);

	data->Gx = (data->Gyro_X_RAW-data->Gx_cali) / 65.5;
	data->Gy = (data->Gyro_Y_RAW-data->Gy_cali) / 65.5;
	data->Gz = (data->Gyro_Z_RAW-data->Gz_cali) / 65.5;
}

void MPU6050_ReadData(SensorData *data){
	MPU6050_Read(MPU6050_DATASTART_REG, 14);
#ifndef MPU6050_USE_DMA
	MPU6050_Compose(data, &mpu6050Data, rawData);
#endif
}

//void MPU6050_Compose(SensorData *sData, MPU6050_Data *data, uint8_t *raw){
//	MPU6050_Map_Acc(&mpu6050Data, raw);
//	MPU6050_Map_Gyro(&mpu6050Data, raw);
//	MPU6050_Map_Temperature(&mpu6050Data, raw);
//
//	data->Ax2 = data->Ax*data->Ax;
//	data->Ay2 = data->Ay*data->Ay;
//	data->Az2 = data->Az*data->Az;
//
//	data->Ax = (float) atan(data->Ax/sqrt(data->Ay2+data->Az2)) * RAD_TO_DEG;
//	data->Ay = (float) atan(data->Ay/sqrt(data->Ax2+data->Az2)) * RAD_TO_DEG;
//	data->Az = (float) atan(data->Az/sqrt(data->Ay2+data->Ax2)) * RAD_TO_DEG;
//
//	data->Gy = -data->Gy;
//
//	complimentaryFilter(&sData->gyro_roll, data->Gy, 0.3);
//	complimentaryFilter(&sData->gyro_pitch, data->Gx, 0.3);
//	complimentaryFilter(&sData->gyro_yaw, data->Gz, 0.3);
//
//	dt = (double)(HAL_GetTick() - timer) / 1000;
//	timer = HAL_GetTick();
//
//	sData->angle_pitch += (float)sData->gyro_pitch * 0.004;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
//	sData->angle_roll += (float)sData->gyro_roll * 0.004;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
//
//	//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
//	sData->angle_pitch -= sData->angle_roll * sin(sData->gyro_yaw * 0.00006981317007977319);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
//	sData->angle_roll += sData->angle_pitch * sin(sData->gyro_yaw * 0.00006981317007977319);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
//
//	complimentaryFilter(&sData->angle_pitch, data->Ay, 0.0004);
//	complimentaryFilter(&sData->angle_roll, data->Ax, 0.0004);
//
//	KalmanAngleY = Kalman_getAngle(&KalmanY, data->Ay, data->Gx, dt);
//    KalmanAngleX = Kalman_getAngle(&KalmanX, data->Ax, data->Gy, dt);
//
//}


void MPU6050_Compose(SensorData *sData, MPU6050_Data *data, uint8_t *raw){
	MPU6050_Map_Acc(&mpu6050Data, raw);
	MPU6050_Map_Gyro(&mpu6050Data, raw);
	MPU6050_Map_Temperature(&mpu6050Data, raw);

	dt = (double)(HAL_GetTick() - timer) / 1000.0;
	timer = HAL_GetTick();

	complimentaryFilter(&sData->gyro.y, data->Gy, 0.3);
	complimentaryFilter(&sData->gyro.x, data->Gx, 0.3);
	complimentaryFilter(&sData->gyro.z, data->Gz, 0.3);

	sData->acc.x = data->Ax;
	sData->acc.y = data->Ay;
	sData->acc.z = data->Az;

	IMUFusion(&fusionData,
			sData->gyro.x*DEG2RAD,
			sData->gyro.y*DEG2RAD,
			sData->gyro.z*DEG2RAD,
			sData->acc.x,
			sData->acc.y,
			sData->acc.z, 0.001);
}

void MPU6050_Compose_Export(SensorData *data){
	MPU6050_Compose(data, &mpu6050Data, rawData);
}

void mpu6050_gyro_calibrate(){
	int32_t Gx=0, Gy=0, Gz=0;
	const uint16_t caliNum = 1000;
	mpu6050Data.Gx_cali = 0;
	mpu6050Data.Gy_cali = 0;
	mpu6050Data.Gz_cali = 0;
	for(uint16_t count=0; count<caliNum; count++){
		HAL_I2C_Mem_Read(i2c,
				MPU6050_ADDR,
				MPU6050_DATASTART_REG,
				I2C_MEMADD_SIZE_8BIT,
				rawData,
				14, 0xff);
		MPU6050_Map_Gyro(&mpu6050Data, rawData);
		Gx += mpu6050Data.Gyro_X_RAW;
		Gy += mpu6050Data.Gyro_Y_RAW;
		Gz += mpu6050Data.Gyro_Z_RAW;
		osDelay(1);
	}
	mpu6050Data.Gx_cali = Gx/caliNum;
	mpu6050Data.Gy_cali = Gy/caliNum;
	mpu6050Data.Gz_cali = Gz/caliNum;
}

#ifdef CALIBRATE_MPU6050
void MPU6050_manual_calibate(MPU6050_Data *data){
	int32_t Ax=0, Ay=0, Az=0, Gx=0, Gy=0, Gz=0;
	const uint16_t caliNum = 1000;
	for(data->count=0; data->count<caliNum; data->count++){
		MPU6050_readall(data);
		Gx += data->Gyro_X_RAW;
		Gy += data->Gyro_Y_RAW;
		Gz += data->Gyro_Z_RAW;
		Ax += data->Accel_X_RAW;
		Ay += data->Accel_Y_RAW;
		Az += data->Accel_Z_RAW;
	}

	data->Gx_cali = Gx/caliNum;
	data->Gy_cali = Gy/caliNum;
	data->Gz_cali = Gz/caliNum;
	data->Ax_cali = Ax/caliNum;
	data->Ay_cali = Ay/caliNum;
	data->Az_cali = Az/caliNum;
	while(1){};
}
#endif

