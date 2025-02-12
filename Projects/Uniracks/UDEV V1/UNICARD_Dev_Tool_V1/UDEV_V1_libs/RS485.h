/*
 * RS485.h
 *
 *  Created on: Nov 11, 2024
 *      Author: ARYELLE
 */

#ifndef RS485_H_
#define RS485_H_

#include "stm32g4xx_hal.h"

typedef struct {
	UART_HandleTypeDef *huart;

	char *TXbuffer;
	uint16_t TXbuffSize;

	char *RXbuffer;
	uint16_t RXbuffSize;

	// add DE polarity etc.

}RS485cfg_struct;

extern uint8_t RS485receiveFlag;

void RS485_Init(RS485cfg_struct *RS485cfg);
void RS485_Transmit_DMA(RS485cfg_struct *RS485cfg);
uint8_t RS485_TransmitDone(RS485cfg_struct *RS485cfg);
void RS485_Receive(RS485cfg_struct *RS485cfg); // call this after youre done with decoding received message
uint8_t RS485_IsReceived(); //test in IF statement if the message is received

#endif /* RS485_H_ */
