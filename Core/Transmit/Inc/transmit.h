/*
 * transmit.h
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */

#ifndef TRANSMIT_INC_TRANSMIT_H_
#define TRANSMIT_INC_TRANSMIT_H_

#include <SheldroneConfig.h>
#include <string.h>
#include <stdarg.h>
#include <sensors.h>
#include <fusion.h>
#include "attitudeEstimate.h"



#define TRANS_START_HIGH		0
#define TRANS_START_LOW			1

#define TRANS_GYRO_ROLL_HIGH	2
#define TRANS_GYRO_ROLL_LOW		3
#define TRANS_GYRO_PITCH_HIGH	4
#define TRANS_GYRO_PITCH_LOW	5
#define TRANS_GYRO_YAW_HIGH		6
#define TRANS_GYRO_YAW_LOW		7

#define TRANS_ANG_ROLL_HIGH		8
#define TRANS_ANG_ROLL_LOW		9
#define TRANS_ANG_PITCH_HIGH	10
#define TRANS_ANG_PITCH_LOW		11
#define TRANS_ANG_YAW_HIGH		12
#define TRANS_ANG_YAW_LOW		13

#define TRANS_KAL_ROLL_HIGH		14
#define TRANS_KAL_ROLL_LOW		15
#define TRANS_KAL_PITCH_HIGH	16
#define TRANS_KAL_PITCH_LOW		17
#define TRANS_KAL_YAW_HIGH		18
#define TRANS_KAL_YAW_LOW		19

#define TRANS_MAG_ANG_HIGH		20
#define TRANS_MAG_ANG_LOW		21

#define TRANS_Q1_HIGH			22
#define TRANS_Q1_LOW			23
#define TRANS_Q2_HIGH			24
#define TRANS_Q2_LOW			25

#define TRANS_Q3_HIGH			26
#define TRANS_Q3_LOW			27
#define TRANS_Q4_HIGH			28
#define TRANS_Q4_LOW			29

#define TRANS_END_HIGH			30
#define TRANS_END_LOW			31

#define TRANS_LENGTH			TRANS_END_LOW+1

typedef struct{
	uint8_t high;
	uint8_t low;
} DoubleU8;


void print(char *fmt, ...);
void printFloat(float f);
void double2DU8(double d, DoubleU8 *dc, uint16_t scale);
void fillArrayPosition_16Bit(double value,
							uint16_t index,
							uint16_t scale);
void fillDebugBuffer();
void transmitDebugData();

#endif /* TRANSMIT_INC_TRANSMIT_H_ */
