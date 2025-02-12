/*
 * UDEV1_LEDcontrol.c
 *
 *  Created on: Jan 22, 2025
 *      Author: Admin
 */

#include "UDEV1_LEDcontrol.h"
#include "main.h"
#include <stdint.h>
#include "math.h"

uint8_t LEDPosition = 0;
RGBLEDS_struct leds[3] = {0}; // user accesible
RGB_struct actualRGB[3] = {0}; // stores data of actual RGB settings with added effects
RGB_struct transitionRGB[3] = {0}; // stores post-transition RGB data without effects
uint32_t period = 0;

void LED_init(uint32_t updatePeriod)
{
	period = updatePeriod;
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);	//R
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	//G
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	//B
}

void LED_DisplayChannel(uint8_t channel)
{
	HAL_GPIO_WritePin(LED1_OUT_GPIO_Port, LED1_OUT_Pin, channel==0);
	HAL_GPIO_WritePin(LED2_OUT_GPIO_Port, LED2_OUT_Pin, channel==1);
	HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, channel==2);
}

void LED_UpdateTransition()
{
	for(uint8_t channel = 0; channel <= 2; channel++)
	{
		float newR = 0, newG = 0, newB = 0;
		if(leds[channel].effect.type != EFFECT_OFF)
		{
			newR = leds[channel].R >= 0.01 ? leds[channel].R : 0.0;
			newG = leds[channel].G >= 0.01 ? leds[channel].G : 0.0;
			newB = leds[channel].B >= 0.01 ? leds[channel].B : 0.0;
		}

		switch (leds[channel].transition.type)
		{
			default: //TRANSITION_INSTANT
				transitionRGB[channel].R = newR;
				transitionRGB[channel].G = newG;
				transitionRGB[channel].B = newB;
			break;

			case TRANSITION_FADE:
				float val = period/(float)(leds[channel].transition.time);
				transitionRGB[channel].R = (1 - val) * transitionRGB[channel].R + val * newR;
				transitionRGB[channel].G = (1 - val) * transitionRGB[channel].G + val * newG;
				transitionRGB[channel].B = (1 - val) * transitionRGB[channel].B + val * newB;
			break;
		}
	}
}

void LED_UpdateEffect()
{
	for(uint8_t channel = 0; channel <= 2; channel++)
	{
		switch (leds[channel].effect.type)
		{
			default: //OFF
			case EFFECT_ON:
				actualRGB[channel].R = transitionRGB[channel].R;
				actualRGB[channel].G = transitionRGB[channel].G;
				actualRGB[channel].B = transitionRGB[channel].B;
			break;

			case EFFECT_BLINK:
				actualRGB[channel].R = actualRGB[channel].phase > 0.5 ? transitionRGB[channel].R : 0;
				actualRGB[channel].G = actualRGB[channel].phase > 0.5 ? transitionRGB[channel].G : 0;
				actualRGB[channel].B = actualRGB[channel].phase > 0.5 ? transitionRGB[channel].B : 0;
			break;

			case EFFECT_PULSE:
				actualRGB[channel].R = actualRGB[channel].phase > 1-leds[channel].effect.dutycycle ? transitionRGB[channel].R : 0;
				actualRGB[channel].G = actualRGB[channel].phase > 1-leds[channel].effect.dutycycle ? transitionRGB[channel].G : 0;
				actualRGB[channel].B = actualRGB[channel].phase > 1-leds[channel].effect.dutycycle ? transitionRGB[channel].B : 0;
			break;

			case EFFECT_BREATHE:
				float val = 0.5 + 0.5 * cosf(6.28 * actualRGB[channel].phase);
				actualRGB[channel].R = transitionRGB[channel].R * val;
				actualRGB[channel].G = transitionRGB[channel].G * val;
				actualRGB[channel].B = transitionRGB[channel].B * val;
			break;

			case EFFECT_COLORSHIFT:
				actualRGB[channel].R = transitionRGB[channel].R * (0.55 + 0.45 * cosf(6.28 * actualRGB[channel].phase));
				actualRGB[channel].G = transitionRGB[channel].G * (0.55 + 0.45 * cosf(6.28 * actualRGB[channel].phase + 2.1));
				actualRGB[channel].B = transitionRGB[channel].B * (0.55 + 0.45 * cosf(6.28 * actualRGB[channel].phase + 4.2));
			break;
		}
		actualRGB[channel].phase += period/(float)(leds[channel].effect.period);
		if(actualRGB[channel].phase > 1) actualRGB[channel].phase = 0;
	}
}

void LED_Iterate()
{
	LEDPosition++;
	if(LEDPosition > 2) LEDPosition = 0;

	LED_PWMSET(actualRGB[LEDPosition].R, actualRGB[LEDPosition].G, actualRGB[LEDPosition].B);

	uint8_t enabler = (actualRGB[LEDPosition].R + actualRGB[LEDPosition].G + actualRGB[LEDPosition].B) > 0.005 ? 1 : 0;
	HAL_GPIO_WritePin(LED1_OUT_GPIO_Port, LED1_OUT_Pin, LEDPosition==0 && enabler);
	HAL_GPIO_WritePin(LED2_OUT_GPIO_Port, LED2_OUT_Pin, LEDPosition==1 && enabler);
	HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, LEDPosition==2 && enabler);
}

void LED_PWMSET(float R, float G, float B)
{
	TIM5->CCR1 = 1000 - (uint16_t)(R * 1000.0);// set Red
	TIM3->CCR4 = 1000 - (uint16_t)(G * 1000.0);// set Green
	TIM3->CCR3 = 1000 - (uint16_t)(B * 1000.0);// set Blue
}

void LED_PWMCLEAR()
{
	TIM5->CCR1 = 1000;// set Red
	TIM3->CCR4 = 1000;// set Green
	TIM3->CCR3 = 1000;// set Blue
}
