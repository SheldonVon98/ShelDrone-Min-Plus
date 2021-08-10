/*
 * math.c
 *
 *  Created on: Aug 9, 2021
 *      Author: sheldonvon
 */

#include "quaternion.h"

Quaternion q_result;

void quaternion2RotMat(Quaternion q, float rotationMat[3][3]){
    float q1q1 = q.x * q.x;
    float q2q2 = q.y * q.y;
    float q3q3 = q.z * q.z;

    float q0q1 = q.w * q.x;
    float q0q2 = q.w * q.y;
    float q0q3 = q.w * q.z;
    float q1q2 = q.x * q.y;
    float q1q3 = q.x * q.z;
    float q2q3 = q.y * q.z;

    rotationMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rotationMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rotationMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rotationMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rotationMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rotationMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rotationMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rotationMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rotationMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

Quaternion quaternionCross(Quaternion a, Quaternion b){
	q_result.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
	q_result.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
	q_result.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
	q_result.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
	return q_result;
}

Quaternion RPY2quaternion(float roll, float pitch, float yaw){
    double cy = cos(yaw * DEG2RAD* 0.5);
    double sy = sin(yaw * DEG2RAD* 0.5);
    double cp = cos(roll * DEG2RAD* 0.5);
    double sp = sin(roll * DEG2RAD* 0.5);
    double cr = cos(pitch * DEG2RAD* 0.5);
    double sr = sin(pitch * DEG2RAD* 0.5);
    q_result.w = cr * cp * cy + sr * sp * sy;
    q_result.x = sr * cp * cy - cr * sp * sy;
    q_result.y = cr * sp * cy + sr * cp * sy;
    q_result.z = cr * cp * sy - sr * sp * cy;
    return q_result;
}

void quaternionNorm(Quaternion *q){
	float recipNorm = invSqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	q->w *= recipNorm;
	q->x *= recipNorm;
	q->y *= recipNorm;
	q->z *= recipNorm;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
