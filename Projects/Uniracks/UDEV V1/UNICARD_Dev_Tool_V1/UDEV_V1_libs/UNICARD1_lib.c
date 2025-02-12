/*
 * UNICARD1_lib.c
 *
 *  Created on: Jan 27, 2025
 *      Author: Admin
 */

#include "UNICARD1_lib.h"

uint8_t UC1_CheckCardValidity(UNICARD1_struct *UNICARD1)
{
	if(UNICARD1->DeviceName[0] == '\0') return 0;
	else if(UNICARD1->DeviceType == UC1dev_unknown) return 0;
	else if(UNICARD1->serialNumber == 0) return 0;

	return 1;
}

