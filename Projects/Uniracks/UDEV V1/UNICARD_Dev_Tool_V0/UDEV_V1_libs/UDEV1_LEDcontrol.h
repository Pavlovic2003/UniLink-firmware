/*
 * UDEV1_LEDcontrol.h
 *
 *  Created on: Jan 22, 2025
 *      Author: Admin
 */

#ifndef UDEV1_LEDCONTROL_H_
#define UDEV1_LEDCONTROL_H_

#include <stdint.h>

/*typedef struct {
TIM_HandleTypeDef Rtimer;
uint32_t RtimerChannel;

TIM_HandleTypeDef Gtimer;
uint32_t GtimerChannel;

TIM_HandleTypeDef Btimer;
uint32_t BtimerChannel;
} LEDhardware_enum;*/

typedef enum {
	EFFECT_OFF = 0,
	EFFECT_ON,
	EFFECT_BLINK,
	EFFECT_PULSE,
	EFFECT_BREATHE,
	EFFECT_COLORSHIFT,
} LEDeffect_enum;

typedef enum {
	TRANSITION_INSTANT = 0,
	TRANSITION_LINEAR,
	TRANSITION_FADE
} LEDtransition_enum;

typedef struct {
	float R;
	float G;
	float B;
	float phase;
	float transitionTime;
} RGB_struct;

typedef struct {
	LEDeffect_enum type;
	uint16_t period;
	float dutycycle;
} EFFECT_struct;

typedef struct {
	LEDtransition_enum type;
	uint16_t time;
} TRANSITION_struct;

typedef struct {
	float R;
	float G;
	float B;
	EFFECT_struct effect;
	TRANSITION_struct transition;
} RGBLEDS_struct;

typedef enum {
	MainLED1 = 0,
	MainLED2,
	CardLED
} LEDpositions;

extern RGBLEDS_struct leds[3];

void LED_init();
//void LED_Set(RGBLEDS_struct RGBLEDS);
void LED_DisplayChannel(uint8_t channel);
void LED_UpdateTransition();
void LED_UpdateEffect();
void LED_Iterate();
void LED_PWMSET(float R, float G, float B);
void LED_PWMCLEAR();

#endif /* UDEV1_LEDCONTROL_H_ */
