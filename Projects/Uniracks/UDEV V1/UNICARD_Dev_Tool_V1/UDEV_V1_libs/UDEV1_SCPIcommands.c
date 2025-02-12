/*
 * UDEV1_SCPIcommands.c
 *
 *  Created on: Feb 12, 2025
 *      Author: Admin
 */

#include "UDEV1_SCPIcommands.h"
#include "main.h"
#include "UDEV1_board.h"
#include "cmsis_os.h"
#include "RS485.h"

void SENDfunc(struct subword* subwords, int length)
{
	if(length != 1) return;

	if(subwords[0].type != params) return;
	struct subword subword = subwords[0];

	uint8_t USBtoRS485_DONE = 0;

	switch (subword.paramType)
	{
			case OTHER_P:

					if(MainState != UDEV1_OFF && CardState > UDEV1CARD_NOTRECOGNISED)
				  	{
				  	  	  for(uint8_t i = 0; i < 100 && !USBtoRS485_DONE; i++)
				  	  	  {
				  			  if(osMutexAcquire(mutex_RS485Handle, osWaitForever) == osOK)
				  			  {
				  				  sprintf(RS485TXbuffer, "000:%s",subword.otherParam);
				  				  RS485_Transmit_DMA(&RS485cfg);

				  				  for(uint16_t timer = RS485maxTimeToTransmit_ms; timer > 0 && !RS485_TransmitDone(&RS485cfg); timer--) osDelay(1);

				  				  for(uint16_t timer = RS485maxTimeToReceive_ms; timer > 0 && !RS485_IsReceived(); timer--) osDelay(1);

				  				  if(RS485_IsReceived())// && UC1_CheckCardValidity(&UNICARD1data))
				  				  {
				  					  	sprintf(USBTXbuffer, "RESPONSE:%s", RS485RXbuffer);
				  						if(ComFailCounter > 0 && !strncmp(RS485RXbuffer, "OK\r\n", 2)) ComFailCounter -= Card_acceptableFailedComRatio;
				  						USBtoRS485_DONE = 1;
				  				  }
				  				  else if(ComFailCounter < 10) ComFailCounter += (1-Card_acceptableFailedComRatio);

				  				  RS485receiveFlag = 0;

				  				  osMutexRelease(mutex_RS485Handle);
				  			  }
				  		  }
				  	  	  if(!USBtoRS485_DONE) sprintf(USBTXbuffer, "ERR:Timeout");

				  		  osDelay(1);
				  	 }

			break;

			default:
				sprintf(USBTXbuffer, "ERR:Parameter");
				break;
	}
}

void UDEV1SCPI_init()
{
	addFunction("SEND", SENDfunc);
}
