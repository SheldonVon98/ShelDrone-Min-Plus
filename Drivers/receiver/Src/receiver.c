/*
 * receiver.c
 *
 *  Created on: 18 Jul 2021
 *      Author: sheldonvon
 */

#include "receiver.h"

int32_t measured_time, measured_time_start, receiver_watchdog;
uint8_t channel_select_counter;
uint16_t channels[6];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	  measured_time = htim11.Instance->CCR1 - measured_time_start;
	  if (measured_time < 0)measured_time += 0xFFFF;
	  measured_time_start = htim11.Instance->CCR1;
	  if (measured_time > 3000) {
	    channel_select_counter = 0;
	    receiver_watchdog = 0;
	  }
	  else channel_select_counter++;
	  channels[channel_select_counter-1] = measured_time;
}
