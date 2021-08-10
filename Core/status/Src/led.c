/*
 * led.c
 *
 *  Created on: 31 Jul 2021
 *      Author: sheldonvon
 */

#include "led.h"

LEDData ledR, ledG, ledB;

void LEDSignalInit(){
	DS_FALL(DS_LED_R);
	DS_FALL(DS_LED_G);
	DS_FALL(DS_LED_B);
	ledR.pin = DS_LED_R;
	ledG.pin = DS_LED_G;
	ledB.pin = DS_LED_B;
	LEDToggle(ALL_LED, false);
}

void LEDSignal(){
	_singleLEDSignal(&ledR);
	_singleLEDSignal(&ledG);
	_singleLEDSignal(&ledB);
}

void _singleLEDSignal(LEDData *led){
	if(led->start){
		if(led->count <= LED_BLINK_PERIOD){
			if(led->count>0&&led->count%led->switchPoint==0) led->on = !led->on;
		} else led->on = false;
		if(led->on) DS_RISE(led->pin);
		else DS_FALL(led->pin);
		if(led->count == LED_FULL_PERIOD/LED_UNIT_PERIOD){
			led->count = 0;
			led->on = true;
		} else {
			led->count++;
		}
	}
}

void _LEDStartStop(LEDData *led, bool start){
	led->start = start;
	led->on = start;
}

void LEDToggle(enum LED led, bool start){
	switch (led) {
		case RED:
			_LEDStartStop(&ledR, start);
			break;
		case GREEN:
			_LEDStartStop(&ledG, start);
			break;
		case BLUE:
			_LEDStartStop(&ledB, start);
			break;
		case ALL_LED:
			_LEDStartStop(&ledR, start);
			_LEDStartStop(&ledG, start);
			_LEDStartStop(&ledB, start);
			break;
	}
}

void _setSingleLEDTimes(LEDData *led, uint8_t times){
	led->switchPoint = LED_BLINK_PERIOD/(times*2-1);
}

void LEDSetTimes(enum LED led, uint8_t times){
	if(times > LED_FREQ_MAX) times = LED_FREQ_MAX;
	if(times < LED_FREQ_Min) times = LED_FREQ_Min;
	switch (led) {
		case RED:
			_setSingleLEDTimes(&ledR, times);
			break;
		case GREEN:
			_setSingleLEDTimes(&ledG, times);
			break;
		case BLUE:
			_setSingleLEDTimes(&ledB, times);
			break;
		case ALL_LED:
			_setSingleLEDTimes(&ledR, times);
			_setSingleLEDTimes(&ledG, times);
			_setSingleLEDTimes(&ledB, times);
			break;
	}
}
