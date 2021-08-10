/*
 * attitudeEstimate.c
 *
 *  Created on: Aug 5, 2021
 *      Author: sheldonvon
 */


#include "attitudeEstimate.h"

extern SensorData sensorData;
extern FusionData fusionData;


void attitudeEstimateTaskInit(){
	fusionData.q.w = 1;
}
void attitudeEstimateTask(){
	pidProcess();
}
