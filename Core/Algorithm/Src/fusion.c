/*
 * fusion.c
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */

#include "fusion.h"

FusionData fusionData;

void complimentaryFilter(float *value, float update, float updatePerc){
	*value = (*value * (1-updatePerc)) + update*updatePerc;
}

void IMUFusion(FusionData *fData, float gx, float gy, float gz, float ax, float ay, float az, double dt) {
	Quaternion qDot, qRate;
	float ex, ey, ez;

	quaternion2RotMat(fData->q, fData->rMat);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		ex = (ay * fData->rMat[2][2] - az * fData->rMat[2][1]);
		ey = (az * fData->rMat[2][0] - ax * fData->rMat[2][2]);
		ez = (ax * fData->rMat[2][1] - ay * fData->rMat[2][0]);
		fData->exInt += GRAVITY_Ki * ex * dt ;
		fData->eyInt += GRAVITY_Ki * ey * dt ;
		fData->ezInt += GRAVITY_Ki * ez * dt ;
		gx += GRAVITY_Kp * ex + fData->exInt;
		gy += GRAVITY_Kp * ey + fData->eyInt;
		gz += GRAVITY_Kp * ez + fData->ezInt;
	}

	// Rate of change of quaternion from gyroscope
	qRate.w = 0;
	qRate.x = gx;
	qRate.y = gy;
	qRate.z = gz;
	qDot = quaternionCross(fData->q, qRate);

	// Integrate rate of change of quaternion to yield quaternion
	fData->q.w += qDot.w * dt * 0.5f;
	fData->q.x += qDot.x * dt * 0.5f;
	fData->q.y += qDot.y * dt * 0.5f;
	fData->q.z += qDot.z * dt * 0.5f;

	// Normalise quaternion
	quaternionNorm(&fData->q);
}

