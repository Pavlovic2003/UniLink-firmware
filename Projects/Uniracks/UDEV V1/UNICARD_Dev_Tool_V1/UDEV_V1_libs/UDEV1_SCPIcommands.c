/*
 * UDEV1_SCPIcommands.c
 *
 *  Created on: Feb 12, 2025
 *      Author: Admin
 */

#include "UDEV1_SCPIcommands.h"

void SEND(struct subword* subwords, int length)
{
	if(length != 1) return;

	if(subwords[0].type != params) return;
	struct subword subword = subwords[0];
	char* LEDstatus = Blink ? "Blinking" : "Not Blinking";

	switch (subword.paramType)
	{
		case OTHER_P:
			Variable = subword.otherParam;
			strcpy(TXbuff,"OK\n\r");
			break;
	}
}
