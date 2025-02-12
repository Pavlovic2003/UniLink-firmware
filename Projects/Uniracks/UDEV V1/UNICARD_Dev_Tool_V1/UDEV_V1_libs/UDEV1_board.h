/*
 * UDEV1_board.h
 *
 *  Created on: Jan 28, 2025
 *      Author: Admin
 */

#ifndef UDEV1_BOARD_H_
#define UDEV1_BOARD_H_

#define Vref 2.048

#define V_IN_GAIN	22.277	// (100k+4k7)/4k7
#define I_3V3_GAIN	2.35	// V = I*20m*100k/4k7 -> I = V*4k7/(20m*100k)
#define I_5V_GAIN	2.35	// V = I*20m*100k/4k7 -> I = V*4k7/(20m*100k)
#define I_VP_GAIN	3.7		// I = 3700/1k

#define I_3V3_MAX	2.5
#define I_5V_MAX	2.5
#define I_VP_MAX	5.0

#define RXbuffSIZE	1500
#define TXbuffSIZE	1500

#define RS485maxTimeToReceive_ms	100 // how much time does Card have to respond (whole number)
#define RS485maxTimeToTransmit_ms	10	// (whole number)

#define PCcon_Watchdog_TIME 		10 	// *.1s (whole number)
#define CardCon_Watchdog_TIME 		10 	// *.1s (whole number)

#define Card_ConnectionCheckPeriod_ms	200 //(whole number)
#define Card_StatusUpdatePeriod_ms		500 //(whole number)

#define Card_CommMaxRetryCount			5	// defines how many times UDEV retries communication if Card response isn't acceptable
#define Card_acceptableFailedComRatio	0.1	// (0-1) represents what percentage of failed messages is acceptable (0 meaning no tolerated failures, 1 failures are ignored)

#endif /* UDEV1_BOARD_H_ */
