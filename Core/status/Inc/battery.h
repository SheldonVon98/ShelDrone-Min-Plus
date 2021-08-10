/*
 * battery.h
 *
 *  Created on: Jul 25, 2021
 *      Author: sheldonvon
 */

#ifndef STATUS_INC_BATTERY_H_
#define STATUS_INC_BATTERY_H_

#include <SheldroneConfig.h>

/*
 * Battery configuration
 *
 * resistor ratio = 15220Ω : 990Ω = 15.374 : 1
 * 4095 = 3.3v (battery pin) = 15.374*3.3v (battery) = 50.7342v (battery)
 * 4095 / 50.7342 = 80.7148
 * 80.7148 / (1 - 0.92) = 1008.935
 * */

#define BATTERY_RATIO 			80.7148
#define BATTERY_FILTER_RATIO 	1008.935
#define BATTERY_VOLTAGE_MAX		16.71
#define BATTERY_VOLTAGE_MIN		12.4

typedef struct{
	float voltage;
	float voltagePC; // voltage percentage.
	uint16_t rawBatteryVol;
}BatteryData;

void batteryInit(ADC_HandleTypeDef *h_adc);
void watchBattery();
void getBatteryVoltage();

extern BatteryData batteryData;
#endif /* STATUS_INC_BATTERY_H_ */
