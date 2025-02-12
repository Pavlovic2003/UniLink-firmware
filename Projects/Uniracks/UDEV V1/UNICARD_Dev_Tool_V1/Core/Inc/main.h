/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "UDEV1_DeviceState.h"
#include "UDEV1_board.h"
#include "RS485.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern osMutexId_t mutex_RS485Handle;
extern UDEV1_MainStates MainState;
extern UDEV1_CardStates CardState;

extern RS485cfg_struct RS485cfg;

extern char RS485TXbuffer[TXbuffSIZE];
extern char RS485RXbuffer[RXbuffSIZE];
extern char USBTXbuffer[TXbuffSIZE];

extern float ComFailCounter;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USB_CDC_RxHandler(uint8_t*, uint32_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEST_BUTTON_Pin GPIO_PIN_13
#define TEST_BUTTON_GPIO_Port GPIOC
#define RLANESEL1_OUT_Pin GPIO_PIN_14
#define RLANESEL1_OUT_GPIO_Port GPIOC
#define RLANEEN_OUT_Pin GPIO_PIN_15
#define RLANEEN_OUT_GPIO_Port GPIOC
#define PWR3V3_ISENSE_ADC1IN10_Pin GPIO_PIN_0
#define PWR3V3_ISENSE_ADC1IN10_GPIO_Port GPIOF
#define RLANESEL2_OUT_Pin GPIO_PIN_1
#define RLANESEL2_OUT_GPIO_Port GPIOF
#define TDI_OUT_Pin GPIO_PIN_0
#define TDI_OUT_GPIO_Port GPIOC
#define TDO_IN_Pin GPIO_PIN_1
#define TDO_IN_GPIO_Port GPIOC
#define TCK_OUT_Pin GPIO_PIN_2
#define TCK_OUT_GPIO_Port GPIOC
#define TMS_OUT_Pin GPIO_PIN_3
#define TMS_OUT_GPIO_Port GPIOC
#define TLANESEL0_OUT_Pin GPIO_PIN_0
#define TLANESEL0_OUT_GPIO_Port GPIOA
#define RS485_USART2DE_Pin GPIO_PIN_1
#define RS485_USART2DE_GPIO_Port GPIOA
#define RS485_USART2TX_Pin GPIO_PIN_2
#define RS485_USART2TX_GPIO_Port GPIOA
#define RS485_USART2RX_Pin GPIO_PIN_3
#define RS485_USART2RX_GPIO_Port GPIOA
#define PWR5VISENSE_ADC2IN17_Pin GPIO_PIN_4
#define PWR5VISENSE_ADC2IN17_GPIO_Port GPIOA
#define TLANEEN_OUT_Pin GPIO_PIN_5
#define TLANEEN_OUT_GPIO_Port GPIOA
#define TLANESEL2_OUT_Pin GPIO_PIN_6
#define TLANESEL2_OUT_GPIO_Port GPIOA
#define TLANESEL1_OUT_Pin GPIO_PIN_7
#define TLANESEL1_OUT_GPIO_Port GPIOA
#define PWR5VEN_OUT_Pin GPIO_PIN_4
#define PWR5VEN_OUT_GPIO_Port GPIOC
#define FLR_USART1RX_Pin GPIO_PIN_5
#define FLR_USART1RX_GPIO_Port GPIOC
#define LEDB_TIM3CH3_Pin GPIO_PIN_0
#define LEDB_TIM3CH3_GPIO_Port GPIOB
#define LEDG_TIM3CH4_Pin GPIO_PIN_1
#define LEDG_TIM3CH4_GPIO_Port GPIOB
#define LEDR_TIM5CH1_Pin GPIO_PIN_2
#define LEDR_TIM5CH1_GPIO_Port GPIOB
#define LED1_OUT_Pin GPIO_PIN_10
#define LED1_OUT_GPIO_Port GPIOB
#define LED2_OUT_Pin GPIO_PIN_11
#define LED2_OUT_GPIO_Port GPIOB
#define VIN_SENSE_ADC1_IN11_Pin GPIO_PIN_12
#define VIN_SENSE_ADC1_IN11_GPIO_Port GPIOB
#define BTN_IN_Pin GPIO_PIN_8
#define BTN_IN_GPIO_Port GPIOC
#define PWR3V3_EN_OUT_Pin GPIO_PIN_9
#define PWR3V3_EN_OUT_GPIO_Port GPIOC
#define VCC_ISENSE_ADC5_IN1_Pin GPIO_PIN_8
#define VCC_ISENSE_ADC5_IN1_GPIO_Port GPIOA
#define FLT_USART1TX_Pin GPIO_PIN_9
#define FLT_USART1TX_GPIO_Port GPIOA
#define DFUEN_OUT_Pin GPIO_PIN_10
#define DFUEN_OUT_GPIO_Port GPIOA
#define ESP32_EN_OUT_Pin GPIO_PIN_15
#define ESP32_EN_OUT_GPIO_Port GPIOA
#define ESP32_TX_UART4_Pin GPIO_PIN_10
#define ESP32_TX_UART4_GPIO_Port GPIOC
#define ESP32_RX_UART4_Pin GPIO_PIN_11
#define ESP32_RX_UART4_GPIO_Port GPIOC
#define DEN_OUT_Pin GPIO_PIN_12
#define DEN_OUT_GPIO_Port GPIOC
#define PWRVCC_EN_OUT_Pin GPIO_PIN_2
#define PWRVCC_EN_OUT_GPIO_Port GPIOD
#define CardDetect_IN_Pin GPIO_PIN_5
#define CardDetect_IN_GPIO_Port GPIOB
#define LED3_OUT_Pin GPIO_PIN_9
#define LED3_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
