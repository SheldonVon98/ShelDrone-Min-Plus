/*
 * sensor_types.h
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */

#ifndef SENSORS_INC_SENSOR_TYPES_H_
#define SENSORS_INC_SENSOR_TYPES_H_

typedef struct{
	float x, y, z, dt;
}SensorXYZData;

typedef struct{
	float x, y, dt;
}SensorXYData;

typedef struct{
	float h, dt;
}SensorHData;

typedef SensorXYZData GyroData;
typedef SensorXYZData AccData;
typedef SensorXYZData MagData;
typedef SensorXYData OpticalFlowData;
typedef SensorHData LaserData;
typedef SensorHData BaroData;

typedef struct{
	GyroData gyro;
	AccData acc;
	MagData mag;
	OpticalFlowData of;
	LaserData las;
	BaroData baro;
}SensorData;

#endif /* SENSORS_INC_SENSOR_TYPES_H_ */
