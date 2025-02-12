/*
 * UDEV1_RS485.c
 *
 *  Created on: Jan 30, 2025
 *      Author: Admin
 */

#include "UDEV1_RS485.h"

/*UDEV1RS485_status UDEV1RS485_TxRx(RS485cfg_struct *RS485cfg)
{
	for(uint8_t i = 0; i < 10 && !task_CardConnectionCheck_DONE; i++)
	{
		if(osMutexAcquire(mutex_RS485Handle, osWaitForever) == osOK)
		{
			sprintf(TXbuffer, "SYS:CON?");
			RS485_Transmit_DMA(&RS485cfg);

			for(uint16_t timer = RS485maxTimeToTransmit_ms; timer > 0 && !RS485_TransmitDone(&RS485cfg); timer--) osDelay(1);

			for(uint16_t timer = RS485maxTimeToReceive_ms; timer > 0 && !RS485_IsReceived(); timer--) osDelay(1);

			if(RS485_IsReceived())// && UC1_CheckCardValidity(&UNICARD1data))
			{
				if(!strncmp(RXbuffer, "OK\r\n", 2))
				{
					CardCon_Watchdog = CardCon_Watchdog_TIME;
					CardState = UDEV1CARD_CON;
				}
				RS485receiveFlag = 0;
			}
			task_CardConnectionCheck_DONE = 1;
			osMutexRelease(mutex_RS485Handle);
		}
		osDelay(1);
	}
}*/
