/*
 * UDEV1_DeviceState.h
 *
 *  Created on: Jan 25, 2025
 *      Author: Admin
 */

#ifndef UDEV1_DEVICESTATE_H_
#define UDEV1_DEVICESTATE_H_

typedef enum {
	UDEV1_OFF = 0,
	UDEV1_ON_NC,
	UDEV1_ON_CON
} UDEV1_MainStates;

typedef enum {
	UDEV1CARD_OFF = 0,
	UDEV1CARD_NC,
	UDEV1CARD_NOTRECOGNISED,
	UDEV1CARD_CON,
	UDEV1CARD_OVERCURRENT,
	UDEV1CARD_COMFAIL
} UDEV1_CardStates;

#endif /* UDEV1_DEVICESTATE_H_ */
