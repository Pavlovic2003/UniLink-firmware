/*
 * UDEV1_RS485.h
 *
 *  Created on: Jan 30, 2025
 *      Author: Admin
 */

#ifndef UDEV1_RS485_H_
#define UDEV1_RS485_H_

#include "RS485.h"
#include "cmsis_os.h"
#include "stdint.h"

typedef enum{
	OK = 0,
	BUSY
} UDEV1RS485_status;

UDEV1RS485_status UDEV1RS485_TxRx(RS485cfg_struct *RS485cfg);

#endif /* UDEV1_RS485_H_ */
