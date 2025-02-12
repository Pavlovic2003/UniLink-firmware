/*
 * RS485.c
 *
 *  Created on: Nov 11, 2024
 *      Author: ARYELLE
 */

#include "RS485.h"
#include "string.h"
#include "stm32g4xx_hal.h"

uint8_t RS485receiveFlag = 0;

void RS485_Init(RS485cfg_struct *RS485cfg){
	HAL_RS485Ex_Init(RS485cfg->huart, UART_DE_POLARITY_HIGH, 2, 2);
	RS485receiveFlag = 0;
	HAL_UARTEx_ReceiveToIdle_IT(RS485cfg->huart, RS485cfg->RXbuffer, RS485cfg->RXbuffSize-1);
}

void RS485_Transmit_DMA(RS485cfg_struct *RS485cfg)
{
	uint16_t msglength = strlen(RS485cfg->TXbuffer) + 2;
	if(msglength > RS485cfg->TXbuffSize) return;

	strcat(RS485cfg->TXbuffer, "\r\n");

	HAL_StatusTypeDef RS485status = HAL_UART_Transmit_DMA(RS485cfg->huart, RS485cfg->TXbuffer, msglength);
	if (RS485status != HAL_OK) {
	        Error_Handler();
	}

	RS485receiveFlag = 0;
	HAL_UARTEx_ReceiveToIdle_IT(RS485cfg->huart, RS485cfg->RXbuffer, RS485cfg->RXbuffSize-1);

	return;
}

uint8_t RS485_TransmitDone(RS485cfg_struct *RS485cfg)
{
	HAL_StatusTypeDef val = HAL_UART_GetError(RS485cfg->huart);
	return (HAL_OK == val);
}

void RS485_Receive(RS485cfg_struct *RS485cfg)
{
	RS485receiveFlag = 0;
	HAL_UARTEx_ReceiveToIdle_IT(RS485cfg->huart, RS485cfg->RXbuffer, RS485cfg->RXbuffSize-1);
}

uint8_t RS485_IsReceived()
{
	return RS485receiveFlag;
}
