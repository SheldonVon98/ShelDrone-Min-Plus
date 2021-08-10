/*
 * stabilizer.c
 *
 *  Created on: Aug 9, 2021
 *      Author: sheldonvon
 */


#include "stabilizer.h"


void stabilizerTaskInit(){
	attitudeEstimateTaskInit();
	pidsInit();
}

void stabilizerTask(){
	_stabilizeAttitude();
	_stabilizeAltitude();
	_stabilizeHovering();
}

void _stabilizeAttitude(){
	attitudeEstimateTask();
}

void _stabilizeAltitude(){
}

void _stabilizeHovering(){
}
