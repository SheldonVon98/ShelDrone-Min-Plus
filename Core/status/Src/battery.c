/*
 * battery.c
 *
 *  Created on: Jul 25, 2021
 *      Author: sheldonvon
 */

#include "battery.h"

ADC_HandleTypeDef *hadc;
BatteryData batteryData;

void batteryInit(ADC_HandleTypeDef *h_adc){
	hadc = h_adc;
	getBatteryVoltage();
	batteryData.voltage = (float)batteryData.rawBatteryVol / BATTERY_RATIO;
}

void watchBattery(){
	getBatteryVoltage();
	batteryData.voltage = batteryData.voltage * 0.92 + (float)batteryData.rawBatteryVol / BATTERY_FILTER_RATIO;
	batteryData.voltagePC = (batteryData.voltage-BATTERY_VOLTAGE_MIN)/(BATTERY_VOLTAGE_MAX-BATTERY_VOLTAGE_MIN)*100;
}

void getBatteryVoltage(){
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	batteryData.rawBatteryVol = HAL_ADC_GetValue(hadc);
}
