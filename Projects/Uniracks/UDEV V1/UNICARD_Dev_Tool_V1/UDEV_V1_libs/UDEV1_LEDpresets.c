/*
 * UDEV1_LEDpresets.c
 *
 *  Created on: Jan 23, 2025
 *      Author: Admin
 */

#include "UDEV1_LEDpresets.h"

RGBLEDS_struct LED_Preset(LEDpreset_enum LEDpreset)
{
	RGBLEDS_struct presetStruct;
	presetStruct.R = 0;
	presetStruct.G = 0;
	presetStruct.B = 0;
	presetStruct.effect.type = EFFECT_OFF;
	presetStruct.effect.period = 1000;
	presetStruct.effect.dutycycle = 0.5;
	presetStruct.transition.type = TRANSITION_FADE;
	presetStruct.transition.time = 25;

	switch (LEDpreset)
	{
		default: //OFF
			presetStruct.transition.type = TRANSITION_FADE;
			presetStruct.transition.time = 50;
			break;
		case PRESET_OVERCURRENT:
			presetStruct.R = 1;
			presetStruct.effect.type = EFFECT_BLINK;
			presetStruct.effect.period = 200;
			presetStruct.transition.type = TRANSITION_INSTANT;
			break;
		case PRESET_NOTLINKEDWITHPC:
			presetStruct.R = 1;
			presetStruct.G = 0.5;
			presetStruct.effect.type = EFFECT_BREATHE;
			presetStruct.effect.period = 1400;
			break;
		case PRESET_LINKEDTOPC:
			presetStruct.G = 0.5;
			presetStruct.effect.type = EFFECT_ON;
			break;
		case PRESET_CARDRECOGNISED:
			presetStruct.G = 1;
			presetStruct.effect.type = EFFECT_ON;
			break;
		case PRESET_CARDNOTDETECTED:
			presetStruct.R = 1;
			presetStruct.G = 0.5;
			presetStruct.effect.type = EFFECT_BREATHE;
			presetStruct.effect.period = 2000;
			break;
		case PRESET_CARDNOTRECOGNISED:
			presetStruct.R = 1;
			presetStruct.G = 0.5;
			presetStruct.effect.type = EFFECT_COLORSHIFT;
			presetStruct.effect.period = 400;
			break;
		case PRESET_CARDCOMFAIL:
			presetStruct.R = 1;
			presetStruct.B = 1;
			presetStruct.effect.type = EFFECT_COLORSHIFT;
			presetStruct.effect.period = 400;
			break;
	}
	return presetStruct;
}
