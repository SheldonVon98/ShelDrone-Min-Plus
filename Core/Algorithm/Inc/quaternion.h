/*
 * math.h
 *
 *  Created on: Aug 9, 2021
 *      Author: sheldonvon
 */

#ifndef ALGORITHM_INC_QUATERNION_H_
#define ALGORITHM_INC_QUATERNION_H_



typedef struct{
	float w, x, y, z;
}Quaternion;

void quaternion2RotMat(Quaternion q,
		float rotationMat[3][3]);
Quaternion quaternionCross(Quaternion q1,
						Quaternion q2);
Quaternion RPY2quaternion(float roll,
						float pitch,
						float yaw);
void quaternionNorm(Quaternion *q);

float invSqrt(float x);
#include "algorithm.h"
#endif /* ALGORITHM_INC_QUATERNION_H_ */
