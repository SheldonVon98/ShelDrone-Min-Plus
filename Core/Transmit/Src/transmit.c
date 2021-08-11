/*
 * transmit.c
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */
#include "transmit.h"

uint8_t transfer_buffer[TRANS_LENGTH];
DoubleU8 dc;
double check;
int16_t temp;

extern UART_HandleTypeDef huart1;


//void print(char *fmt, ...){
//	while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=USBD_OK);
//	va_list args;
//	va_start(args, fmt);
//	vsprintf((char *)transfer_buffer, fmt, args);
//	va_end(args);
////	CDC_Transmit_FS((uint8_t *)transfer_buffer, strlen(transfer_buffer));
//	HAL_UART_Transmit_DMA(&huart1, transfer_buffer, strlen(transfer_buffer));
//}

//void printFloat(float f){
//	d2c(f, &dc);
//	print("%d.%d", dc.high, dc.low);
//}



extern double KalmanAngleX;
extern double KalmanAngleY;
extern double KalmanAngleZ;
extern float mag_angle;
void double2DU8(double d, DoubleU8 *dc, uint16_t scale){
	temp = d*scale;
	dc->low = temp & 0xFF;
	dc->high = (temp >> 8) & 0xFF;
}

void fillArrayPosition_16Bit(double value, uint16_t index, uint16_t scale){
	double2DU8(value, &dc, scale);
	transfer_buffer[index] = dc.high;
	transfer_buffer[index+1] = dc.low;
}


extern Quaternion q_err, q_target;
void fillDebugBuffer(){
	transfer_buffer[TRANS_START_HIGH] = 'S';
	transfer_buffer[TRANS_START_LOW] = 'V';
	transfer_buffer[TRANS_END_HIGH] = 'R';
	transfer_buffer[TRANS_END_LOW] = 'N';

//	fillArrayPosition_16Bit(sensorData.gyro_roll, TRANS_GYRO_ROLL_HIGH, 100);
//	fillArrayPosition_16Bit(sensorData.gyro_pitch, TRANS_GYRO_PITCH_HIGH, 100);
//	fillArrayPosition_16Bit(sensorData.gyro_yaw, TRANS_GYRO_YAW_HIGH, 100);
//
//	fillArrayPosition_16Bit(sensorData.angle_roll, TRANS_ANG_ROLL_HIGH, 1);
//	fillArrayPosition_16Bit(sensorData.angle_pitch, TRANS_ANG_PITCH_HIGH, 1);
//	fillArrayPosition_16Bit(sensorData.angle_yaw, TRANS_ANG_YAW_HIGH, 1);
//
//	fillArrayPosition_16Bit(KalmanAngleY, TRANS_KAL_ROLL_HIGH, 100);
//	fillArrayPosition_16Bit(KalmanAngleX, TRANS_KAL_PITCH_HIGH, 100);
//	fillArrayPosition_16Bit(KalmanAngleZ, TRANS_KAL_YAW_HIGH, 100);
//
//	fillArrayPosition_16Bit(mag_angle, TRANS_MAG_ANG_HIGH, 100);

		fillArrayPosition_16Bit(sensorData.gyro.x, TRANS_GYRO_ROLL_HIGH, 100);
		fillArrayPosition_16Bit(sensorData.gyro.y, TRANS_GYRO_PITCH_HIGH, 100);
		fillArrayPosition_16Bit(sensorData.gyro.z, TRANS_GYRO_YAW_HIGH, 100);

		fillArrayPosition_16Bit(0, TRANS_ANG_ROLL_HIGH, 1);
		fillArrayPosition_16Bit(0, TRANS_ANG_PITCH_HIGH, 1);
		fillArrayPosition_16Bit(0, TRANS_ANG_YAW_HIGH, 1);

		fillArrayPosition_16Bit(0, TRANS_KAL_ROLL_HIGH, 100);
		fillArrayPosition_16Bit(0, TRANS_KAL_PITCH_HIGH, 100);
		fillArrayPosition_16Bit(0, TRANS_KAL_YAW_HIGH, 100);

		fillArrayPosition_16Bit(0, TRANS_MAG_ANG_HIGH, 100);

	fillArrayPosition_16Bit(q_target.w, TRANS_Q1_HIGH, 10000);
	fillArrayPosition_16Bit(q_target.x, TRANS_Q2_HIGH, 10000);
	fillArrayPosition_16Bit(q_target.y, TRANS_Q3_HIGH, 10000);
	fillArrayPosition_16Bit(q_target.z, TRANS_Q4_HIGH, 10000);
}

void transmitDebugData(){
	fillDebugBuffer();
	if(huart1.gState == HAL_UART_STATE_READY){
		HAL_UART_Transmit_DMA(&huart1, transfer_buffer, TRANS_LENGTH);
	}
}
