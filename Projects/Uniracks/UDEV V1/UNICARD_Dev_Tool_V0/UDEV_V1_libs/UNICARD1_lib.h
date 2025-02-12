/*
 * UNICARD1_lib.h
 *
 *  Created on: Jan 27, 2025
 *      Author: Admin
 */

#ifndef UNICARD1_LIB_H_
#define UNICARD1_LIB_H_

#include "stdint.h"

// RS485 baudrate
#define UC1_BaudRate 115200

typedef enum {
	UC1dev_unknown = 0,
	UC1dev_custom,
	UC1dev_devboard,
	UC1dev_voltmeter,
	UC1dev_ampermeter,
	UC1dev_RLCmeter,
	UC1dev_oscilloscope,
	UC1dev_powerSupply,
	UC1dev_powerLoad,
	UC1dev_counter
} UC1deviceType;

typedef enum {
	UC1OCR_manual = 0,	// requires user to send command to restart device, minimum time to restart	 defined by uint16_t OCRrecoveryTime_ms;
	UC1OCR_automatic,	// automatically restarts VP after time period defined by uint16_t OCRrecoveryTime_ms;
	UC1OCR_shutdown 	// turns off every power supply, manual card ejection and reinsertion required
} UC1overCurrentRecovery;

typedef struct {
	// mandatory
	char DeviceName[26]; 			// must be defined, for example "Shock-o-Meter 9000"
	char DeviceDescription[36]; 	// describe device, for example "6.5 digit, TrueRMS, 4 channel voltmeter"
	uint8_t DeviceMajorVersion; 	// if the device version is V1.2, this number is 1
	uint8_t DeviceMinorVersion; 	// if the device version is V1.2, this number is 2

	UC1deviceType DeviceType; 		// cannot be unknown
	uint32_t serialNumber;			// serial number must be greater than 0

	// parameters for high power line
	// to use high power line, all parameters except VPidealVoltage_mV must be defined
	// if Rack cannot provide requested parameters, VP wont be used
	uint16_t VPmaxVoltage_mV;
	uint16_t VPminVoltage_mV;
	uint16_t VPidealVoltage_mV;
	// 24V

	uint16_t VPmaxCurrent_mA;
	UC1overCurrentRecovery OCR;		// defines how will power line behave in overcurrent scenario
	uint16_t OCRrecoveryTime_ms;

} UNICARD1_struct;

uint8_t UC1_CheckCardValidity(UNICARD1_struct *UNICARD1);

#endif /* UNICARD1_LIB_H_ */
