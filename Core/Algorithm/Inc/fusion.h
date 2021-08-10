/*
 * fusion.h
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */

#ifndef FLIGHTCONTROL_INC_FUSION_H_
#define FLIGHTCONTROL_INC_FUSION_H_

#include "algorithm.h"
#include "quaternion.h"

#define GRAVITY_Kp 	0.002f
#define GRAVITY_Ki 	0.00001f

typedef struct{
	Quaternion q;
	float exInt;
	float eyInt;
	float ezInt;
	float rMat[3][3];
}FusionData;

void complimentaryFilter(float *value,
						float update,
						float updatePerc);


void IMUFusion(FusionData *fData,
				float gx,
				float gy,
				float gz,
				float ax,
				float ay,
				float az,
				double dt);
float invSqrt(float x);

extern FusionData fusionData;
#endif /* FLIGHTCONTROL_INC_FUSION_H_ */
