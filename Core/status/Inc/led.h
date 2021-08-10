/*
 * led.h
 *
 *  Created on: 31 Jul 2021
 *      Author: sheldonvon
 */

#ifndef STATUS_INC_LED_H_
#define STATUS_INC_LED_H_

#include <SheldroneConfig.h>
#include <stdbool.h>

#define DS_LED_B		GPIO_PIN_13
#define DS_LED_G		GPIO_PIN_14
#define DS_LED_R		GPIO_PIN_15
#define DS_RISE(n) 	GPIOC->ODR |= n
#define DS_FALL(n) 	GPIOC->ODR &= ~n

typedef struct{
	uint16_t pin;
	uint8_t count;
	uint8_t switchPoint;
	bool on;
	bool start;
}LEDData;

/*
 * Signal full period: 2000ms
 * Interval: 500ms
 * Blink Period: 2000ms-500ms=1500ms
 * Time unit: 50ms
 * */
#define LED_FULL_PERIOD 	2000
#define LED_INTERVAL 		400
#define LED_UNIT_PERIOD 	20
#define LED_BLINK_PERIOD 	(LED_FULL_PERIOD-LED_INTERVAL)/LED_UNIT_PERIOD
#define LED_FREQ_MAX		10
#define LED_FREQ_Min		1

enum LED{
	RED,
	GREEN,
	BLUE,
	ALL_LED
};

void LEDSignalInit();
void _singleLEDSignal(LEDData *led);
void LEDSignal();
void _LEDStartStop(LEDData *led, bool start);
void LEDToggle(enum LED led, bool start);
void _setSingleLEDTimes(LEDData *led, uint8_t times);
void LEDSetTimes(enum LED led, uint8_t times);

#endif /* STATUS_INC_LED_H_ */
