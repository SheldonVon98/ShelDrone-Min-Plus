/*
 * error.c
 *
 *  Created on: 31 Jul 2021
 *      Author: sheldonvon
 */


#include "error.h"

void errorInit(){
	LEDSignalInit();
	LEDToggle(ALL_LED, true);
}

void errorSignal(){
	LEDSignal();
}
