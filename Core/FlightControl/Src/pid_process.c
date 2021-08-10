/*
 * pid_process.c
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#include "pid_process.h"
#include "mpu6050.h"
#include <math.h>
//PIDObject pid_pitch;
//PIDObject pid_roll;
//PIDObject pid_yaw;
//
//pidInit_t pit_pitch_param;
//pidInit_t pit_roll_param;
//pidInit_t pit_yaw_param;
//
//SetPoints setpoints;
//extern double KalmanAngleY, KalmanAngleX;
//
//float pitch_level_adjust, roll_level_adjust;
//void pidsInit(){
//	pit_pitch_param.kp = 1.5;
//	pit_pitch_param.ki = 0;
//	pit_pitch_param.kd = 0.0;
//	pidInit(&pid_pitch, pit_pitch_param, 0.004);
//
//	pit_roll_param.kp = 1.5;
//	pit_roll_param.ki = 0;
//	pit_roll_param.kd = 0.0;
//	pidInit(&pid_roll, pit_roll_param, 0.004);
//
//	pit_yaw_param.kp = 2.0;
//	pit_yaw_param.ki = 0.008;
//	pit_yaw_param.kd = 0;
//	pidInit(&pid_yaw, pit_yaw_param, 0.004);
//}
//
//void pidProcess(){
//	if(controlStatus.flightBaseStatus == STANDBY){
//		pidReset(&pid_pitch);
//		pidReset(&pid_roll);
//		pidReset(&pid_yaw);
//	} else if (controlStatus.flightBaseStatus == TAKEOFF){
//
//		setPointReset(&setpoints);
//
//		roll_level_adjust = 20 * KalmanAngleX;// sensorData.angle_roll;//
//		if(ATHIGH(channels[ROLL])) setpoints.roll = channels[ROLL] - CHANNEL_MID_MAX;
//		else if(ATLOW(channels[ROLL])) setpoints.roll = channels[ROLL] - CHANNEL_MID_MIN;
//		setpoints.roll -= roll_level_adjust;
//		setpoints.roll /= 6.0;
//
//
//		pitch_level_adjust = 20 * KalmanAngleY;// sensorData.angle_pitch;//
//		if(ATHIGH(channels[PITCH])) setpoints.pitch = CHANNEL_MID_MAX - channels[PITCH];
//		else if(ATLOW(channels[PITCH])) setpoints.pitch = CHANNEL_MID_MIN - channels[PITCH];
//		setpoints.pitch -= pitch_level_adjust;
//		setpoints.pitch /= 6.0;
//
//		if(ATHIGH(channels[YAW])) setpoints.yaw = CHANNEL_MID_MAX - channels[YAW];
//		else if(ATLOW(channels[YAW])) setpoints.yaw = CHANNEL_MID_MIN - channels[YAW];
//		setpoints.yaw /= 3;
//
//		pidUpdate(&pid_pitch, sensorData.gyro_pitch, setpoints.pitch);
//		pidUpdate(&pid_roll, sensorData.gyro_roll, setpoints.roll);
//		pidUpdate(&pid_yaw, sensorData.gyro_yaw, setpoints.yaw);
//	}
//}

PIDObject pid_pitch_angle;
PIDObject pid_roll_angle;
PIDObject pid_yaw_angle;

PIDObject pid_pitch_speed;
PIDObject pid_roll_speed;
PIDObject pid_yaw_speed;

SetPoints setpoints;

Quaternion q_target, q_err;

void pidsInit(){

	pidInit_t pid_pitch_angle_param={
		.kp = 10,
		.ki = 0.005,
		.kd = 0.01,
	};
	pidInit_t pid_roll_angle_param = pid_pitch_angle_param;
	pidInit_t pid_yaw_angle_param={
		.kp = 10,
		.ki = 0.0,
		.kd = 0.01,
	};

	pidInit(&pid_pitch_angle, pid_pitch_angle_param, 0.004);
	pidInit(&pid_roll_angle, pid_roll_angle_param, 0.004);
	pidInit(&pid_yaw_angle, pid_yaw_angle_param, 0.004);
	pid_yaw_angle.outputLimit = 200;


	pidInit_t pid_pitch_speed_param={
		.kp = 0.7,
		.ki = 0.0,
		.kd = 0.01,
	};
	pidInit_t pid_roll_speed_param = pid_pitch_speed_param;
	pidInit_t pid_yaw_speed_param={
		.kp = 1.7,
		.ki = 0.01,
		.kd = 0.0,
	};

	pidInit(&pid_pitch_speed, pid_pitch_speed_param, 0.004);
	pidInit(&pid_roll_speed, pid_roll_speed_param, 0.004);
	pidInit(&pid_yaw_speed, pid_yaw_speed_param, 0.004);
	pid_yaw_speed.outputLimit = 200;

}

void pidProcess(){
	if(droneStatus.flightBaseStatus == STANDBY){
		pidReset(&pid_pitch_angle);
		pidReset(&pid_roll_angle);
		pidReset(&pid_yaw_angle);

		pidReset(&pid_pitch_speed);
		pidReset(&pid_roll_speed);
		pidReset(&pid_yaw_speed);

	} else if (droneStatus.flightBaseStatus == TAKEOFF){
		setPointReset(&setpoints);
		if(ATHIGH(channels[ROLL])) setpoints.roll = (channels[ROLL] - CHANNEL_MID_MAX) * ROLL_ANGLE_MAX / (CHANNEL_MAX - CHANNEL_MID_MAX);
		else if(ATLOW(channels[ROLL])) setpoints.roll = (channels[ROLL] - CHANNEL_MID_MIN) * ROLL_ANGLE_MAX / (CHANNEL_MID_MIN - CHANNEL_MIN);

		if(ATHIGH(channels[PITCH])) setpoints.pitch = (CHANNEL_MID_MAX - channels[PITCH]) * PITCH_ANGLE_MAX / (CHANNEL_MAX - CHANNEL_MID_MAX);
		else if(ATLOW(channels[PITCH])) setpoints.pitch = (CHANNEL_MID_MIN - channels[PITCH]) * PITCH_ANGLE_MAX / (CHANNEL_MID_MIN - CHANNEL_MIN);

		if(ATHIGH(channels[YAW])) setpoints.yaw += (CHANNEL_MID_MAX - channels[YAW]) * YAW_SPEED_MAX / (CHANNEL_MAX - CHANNEL_MID_MAX);
		else if(ATLOW(channels[YAW])) setpoints.yaw += (CHANNEL_MID_MIN - channels[YAW]) * YAW_SPEED_MAX / (CHANNEL_MID_MIN - CHANNEL_MIN);

		if(setpoints.yaw > 720) setpoints.yaw = 0;
		else if(setpoints.yaw < 0) setpoints.yaw = 720;

		q_target = RPY2quaternion(-setpoints.roll, setpoints.pitch, setpoints.yaw);
		q_err = quaternionCross(q_target, fusionData.q);

		pidUpdate(&pid_pitch_angle, q_err.x * 100.0, 0);
		pidUpdate(&pid_roll_angle, q_err.y * 100.0, 0);
		pidUpdate(&pid_yaw_angle, q_err.z * 100.0, 0);

		pidUpdate(&pid_pitch_speed, sensorData.gyro.x, pid_pitch_angle.out);
		pidUpdate(&pid_roll_speed, -sensorData.gyro.y, -pid_roll_angle.out);
		pidUpdate(&pid_yaw_speed, sensorData.gyro.z, pid_yaw_angle.out);

	}
}

void setPointReset(SetPoints *sp){
	sp->roll = 0;
	sp->pitch = 0;
	sp->roll_speed = 0;
	sp->pitch_speed = 0;
	sp->yaw_speed = 0;
}
