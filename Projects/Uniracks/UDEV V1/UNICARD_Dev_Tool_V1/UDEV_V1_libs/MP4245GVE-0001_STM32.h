/*
 * MP4245GVE-0001_STM32.h
 *
 *  Created on: Feb 1, 2025
 *      Author: Admin
 */

#ifndef MP4245GVE_0001_STM32_H_
#define MP4245GVE_0001_STM32_H_

#include "stm32g4xx_hal_i2c.h"

typedef enum {
    DRV_VOLTAGE_5_5V = 0b00,  // 5.5V
    DRV_VOLTAGE_6_0V = 0b01,  // 6V (default)
    DRV_VOLTAGE_6_2V = 0b10,  // 6.2V
    DRV_VOLTAGE_6_5V = 0b11   // 6.5V
} DRV_VOLTAGE_t;

typedef enum {
    LINE_DROP_COMP_0UA_A   = 0b000, // 0 µA/A
    LINE_DROP_COMP_0_5UA_A = 0b001, // 0.5 µA/A
    LINE_DROP_COMP_1UA_A   = 0b010, // 1 µA/A (default)
    LINE_DROP_COMP_2UA_A   = 0b011, // 2 µA/A
    LINE_DROP_COMP_4UA_A   = 0b100, // 4 µA/A
    LINE_DROP_COMP_8UA_A   = 0b101  // 8 µA/A
} LINE_DROP_COMP_GAIN_t;

typedef enum {
    OTP_THRESHOLD_110C = 0b000, // 110°C
    OTP_THRESHOLD_120C = 0b001, // 120°C
    OTP_THRESHOLD_130C = 0b010, // 130°C
    OTP_THRESHOLD_140C = 0b011, // 140°C
    OTP_THRESHOLD_150C = 0b100, // 150°C
    OTP_THRESHOLD_160C = 0b101, // 160°C (default)
    OTP_THRESHOLD_170C = 0b110, // 170°C
    OTP_THRESHOLD_180C = 0b111  // 180°C
} OTP_THRESHOLD_t;

typedef enum {
    OT_WARNING_60C     = 0b000, // 60°C
    OT_WARNING_70C     = 0b001, // 70°C
    OT_WARNING_80C     = 0b010, // 80°C
    OT_WARNING_90C     = 0b011, // 90°C
    OT_WARNING_100C    = 0b100, // 100°C
    OT_WARNING_DISABLED = 0b101, // Disabled
    OT_WARNING_120C    = 0b110, // 120°C (default)
    OT_WARNING_130C    = 0b111  // 130°C
} OT_WARNING_THRESHOLD_t;

typedef enum {
    GND_SENSE_5m  = 0b00, // 5mΩ
    GND_SENSE_10m = 0b01, // 10mΩ
    GND_SENSE_20m = 0b10, // 20mΩ (default)
    GND_SENSE_30m = 0b11  // 30mΩ
} GND_SENSE_RESISTOR_t;

typedef enum {
    HICCUP_TIMER_500mS = 0b0, // 500ms (default)
    HICCUP_TIMER_2S    = 0b1  // 2 seconds
} HICCUP_TIMER_t;

typedef enum {
    PFM_PWM_MODE_AUTO    = 0b0, // Auto-PFM/PWM mode
    PFM_PWM_MODE_FORCED  = 0b1  // Forced PWM mode (default)
} PFM_PWM_MODE_t;

typedef struct{
	I2C_HandleTypeDef hi2c4;

	uint8_t DITHER_ENABLE;
	uint8_t CLK_MODE;
	GND_SENSE_RESISTOR_t GND_SENSE_RESISTOR;
	uint8_t OUTPUT_DISCHARGE_EN;
	DRV_VOLTAGE_t DRV_VOLTAGE;
	LINE_DROP_COMP_GAIN_t LINE_DROP_COMP_GAIN;
	OTP_THRESHOLD_t OTP_THRESHOLD;
	OT_WARNING_THRESHOLD_t OT_WARNING_THRESHOLD;
} MP4245GVE0001setup_struct;

uint8_t MP4245GVE0001_Init(MP4245GVE0001setup_struct *MP4245GVE0001setup);

uint8_t MP4245GVE0001_SetVoltage(float voltage);
uint8_t MP4245GVE0001_SetCurrentLimit(float currentLimit);


#endif /* MP4245GVE_0001_STM32_H_ */
