/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usbpd.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UniLink_SCPI.h"
#include "UDEV1_board.h"

#include "UDEV1_LEDcontrol.h"
#include "UDEV1_LEDpresets.h"
#include "UDEV1_DeviceState.h"
#include "UNICARD1_lib.h"

#include "RS485.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc5;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc5;

I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for task_USBcom */
osThreadId_t task_USBcomHandle;
const osThreadAttr_t task_USBcom_attributes = {
  .name = "task_USBcom",
  .priority = (osPriority_t) osPriorityRealtime7,
  .stack_size = 128 * 4
};
/* Definitions for LEDcontrol */
osThreadId_t LEDcontrolHandle;
const osThreadAttr_t LEDcontrol_attributes = {
  .name = "LEDcontrol",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Current_Check */
osThreadId_t Current_CheckHandle;
const osThreadAttr_t Current_Check_attributes = {
  .name = "Current_Check",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for BUTTON */
osThreadId_t BUTTONHandle;
const osThreadAttr_t BUTTON_attributes = {
  .name = "BUTTON",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for StateMachine */
osThreadId_t StateMachineHandle;
const osThreadAttr_t StateMachine_attributes = {
  .name = "StateMachine",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for PCconnectionWat */
osThreadId_t PCconnectionWatHandle;
const osThreadAttr_t PCconnectionWat_attributes = {
  .name = "PCconnectionWat",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for CardConnectionW */
osThreadId_t CardConnectionWHandle;
const osThreadAttr_t CardConnectionW_attributes = {
  .name = "CardConnectionW",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for CardConnectionC */
osThreadId_t CardConnectionCHandle;
const osThreadAttr_t CardConnectionC_attributes = {
  .name = "CardConnectionC",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for CardUpdateStatu */
osThreadId_t CardUpdateStatuHandle;
const osThreadAttr_t CardUpdateStatu_attributes = {
  .name = "CardUpdateStatu",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for mutex_RS485 */
osMutexId_t mutex_RS485Handle;
const osMutexAttr_t mutex_RS485_attributes = {
  .name = "mutex_RS485"
};
/* Definitions for mutex_USB */
osMutexId_t mutex_USBHandle;
const osMutexAttr_t mutex_USB_attributes = {
  .name = "mutex_USB"
};
/* USER CODE BEGIN PV */

// ADC
uint32_t adc1_data[2] = {0}, adc2_data = 0, adc5_data = 0;
float V_IN = 0, I_VP = 0, I_5V = 0, I_3V3 = 0;

uint16_t OCRtimer = 0;
uint8_t I_3V3_fault = 0, I_5V_fault = 0, I_VP_fault = 0;

uint16_t lastButtonPressTime = 0;

uint8_t check_ConnectionToPC = 1;
uint8_t check_ConnectionToCard = 1;

uint8_t connectedToPC = 1;
uint8_t connectedToCard = 1;
uint8_t cardRecognised = 1;

UDEV1_MainStates MainState = UDEV1_ON_NC;
UDEV1_CardStates CardState = UDEV1CARD_OFF;

UNICARD1_struct UNICARD1data = {0};

RS485cfg_struct RS485cfg = {0};

char RS485TXbuffer[TXbuffSIZE] = {0};
char RS485RXbuffer[RXbuffSIZE] = {0};

int PCcon_Watchdog = 0, CardCon_Watchdog = 0;
float ComFailCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UCPD1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC5_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void task_LEDcontrol(void *argument);
void task_Current_Check(void *argument);
void task_BUTTON(void *argument);
void task_StateMachine(void *argument);
void task_PCconnectionWatchDog(void *argument);
void taks_CardConnectionWatchDog(void *argument);
void task_CardConnectionCheck(void *argument);
void task_CardUpdateStatus(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_UCPD1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC5_Init();
  MX_I2C4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  //------------------------------------------------------------------------------------- INIT

  while(HAL_GPIO_ReadPin(TEST_BUTTON_GPIO_Port, TEST_BUTTON_Pin)){}

  LED_init(4);

  UNICARD1data.OCR = UC1OCR_manual;
  UNICARD1data.OCRrecoveryTime_ms = 1000;

  RS485cfg.huart = &huart2;
  RS485cfg.TXbuffSize = TXbuffSIZE;
  RS485cfg.TXbuffer = RS485TXbuffer;
  RS485cfg.RXbuffSize = RXbuffSIZE;
  RS485cfg.RXbuffer = RS485RXbuffer;

  RS485_Init(&RS485cfg);

  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_data, 2);

  //HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  //HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&adc2_data, 1);

  //HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
  //HAL_ADC_Start_DMA(&hadc5, (uint32_t *)&adc5_data, 1);

  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* USBPD initialisation ---------------------------------*/
  MX_USBPD_Init();
  /* Create the mutex(es) */
  /* creation of mutex_RS485 */
  mutex_RS485Handle = osMutexNew(&mutex_RS485_attributes);

  /* creation of mutex_USB */
  mutex_USBHandle = osMutexNew(&mutex_USB_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_USBcom */
  task_USBcomHandle = osThreadNew(StartDefaultTask, NULL, &task_USBcom_attributes);

  /* creation of LEDcontrol */
  LEDcontrolHandle = osThreadNew(task_LEDcontrol, NULL, &LEDcontrol_attributes);

  /* creation of Current_Check */
  Current_CheckHandle = osThreadNew(task_Current_Check, NULL, &Current_Check_attributes);

  /* creation of BUTTON */
  BUTTONHandle = osThreadNew(task_BUTTON, NULL, &BUTTON_attributes);

  /* creation of StateMachine */
  StateMachineHandle = osThreadNew(task_StateMachine, NULL, &StateMachine_attributes);

  /* creation of PCconnectionWat */
  PCconnectionWatHandle = osThreadNew(task_PCconnectionWatchDog, NULL, &PCconnectionWat_attributes);

  /* creation of CardConnectionW */
  CardConnectionWHandle = osThreadNew(taks_CardConnectionWatchDog, NULL, &CardConnectionW_attributes);

  /* creation of CardConnectionC */
  CardConnectionCHandle = osThreadNew(task_CardConnectionCheck, NULL, &CardConnectionC_attributes);

  /* creation of CardUpdateStatu */
  CardUpdateStatuHandle = osThreadNew(task_CardUpdateStatus, NULL, &CardUpdateStatu_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = DISABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20B0D9FF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 10-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 18000000;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**UCPD1 GPIO Configuration
  PB4   ------> UCPD1_CC2
  PB6   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UCPD1 DMA Init */

  /* UCPD1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_UCPD1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMAMUX_REQ_UCPD1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD1 interrupt Init */
  NVIC_SetPriority(UCPD1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(UCPD1_IRQn);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA1_Channel8_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel8_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel8_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RLANESEL1_OUT_Pin|RLANEEN_OUT_Pin|TDI_OUT_Pin|TCK_OUT_Pin
                          |TMS_OUT_Pin|PWR5VEN_OUT_Pin|PWR3V3_EN_OUT_Pin|DEN_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RLANESEL2_OUT_GPIO_Port, RLANESEL2_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TLANESEL0_OUT_Pin|TLANEEN_OUT_Pin|TLANESEL2_OUT_Pin|TLANESEL1_OUT_Pin
                          |DFUEN_OUT_Pin|ESP32_EN_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_OUT_Pin|LED2_OUT_Pin|LED3_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWRVCC_EN_OUT_GPIO_Port, PWRVCC_EN_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TEST_BUTTON_Pin TDO_IN_Pin */
  GPIO_InitStruct.Pin = TEST_BUTTON_Pin|TDO_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RLANESEL1_OUT_Pin RLANEEN_OUT_Pin TDI_OUT_Pin TCK_OUT_Pin
                           TMS_OUT_Pin PWR5VEN_OUT_Pin PWR3V3_EN_OUT_Pin DEN_OUT_Pin */
  GPIO_InitStruct.Pin = RLANESEL1_OUT_Pin|RLANEEN_OUT_Pin|TDI_OUT_Pin|TCK_OUT_Pin
                          |TMS_OUT_Pin|PWR5VEN_OUT_Pin|PWR3V3_EN_OUT_Pin|DEN_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RLANESEL2_OUT_Pin */
  GPIO_InitStruct.Pin = RLANESEL2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RLANESEL2_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TLANESEL0_OUT_Pin TLANEEN_OUT_Pin TLANESEL2_OUT_Pin TLANESEL1_OUT_Pin
                           DFUEN_OUT_Pin ESP32_EN_OUT_Pin */
  GPIO_InitStruct.Pin = TLANESEL0_OUT_Pin|TLANEEN_OUT_Pin|TLANESEL2_OUT_Pin|TLANESEL1_OUT_Pin
                          |DFUEN_OUT_Pin|ESP32_EN_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_OUT_Pin LED2_OUT_Pin LED3_OUT_Pin */
  GPIO_InitStruct.Pin = LED1_OUT_Pin|LED2_OUT_Pin|LED3_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_IN_Pin */
  GPIO_InitStruct.Pin = BTN_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWRVCC_EN_OUT_Pin */
  GPIO_InitStruct.Pin = PWRVCC_EN_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWRVCC_EN_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CardDetect_IN_Pin */
  GPIO_InitStruct.Pin = CardDetect_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CardDetect_IN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//---------------------------------- a delay that can be used without irq enabled
void delay(uint32_t delay_ms) {
    // Configure the SysTick timer
    SysTick->LOAD = (SystemCoreClock / 1000) * delay_ms - 1; // 50 ms delay
    SysTick->VAL = 0;                                   // Clear current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    // Wait until the COUNTFLAG is set
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    // Disable the SysTick timer
    SysTick->CTRL = 0;
}

/*void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
    CDC_Transmit_FS(Buf, Len);
}*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == RS485cfg.huart)
	{
		RS485receiveFlag = 1;
		//RS485_Receive(&RS485cfg);
	}
}
//----------------------------------------------------------------------------------------- adc conversion
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// convert units
	I_3V3 = I_3V3_GAIN * Vref * adc1_data[0]/(float)(1 << 12);
	V_IN = V_IN_GAIN * Vref * adc1_data[1]/(float)(1 << 12);
	I_5V = I_5V_GAIN * Vref * adc2_data/(float)(1 << 12);
	I_VP = I_VP_GAIN * Vref * adc5_data/(float)(1 << 12);

	I_3V3_fault = (I_3V3 > I_3V3_MAX) ? 1 : 0;
	I_5V_fault = (I_5V > I_5V_MAX) ? 1 : 0;
	//I_VP_fault = (I_VP > I_VP_MAX) || (I_VP > (UNICARD1data.VPmaxCurrent_mA * 1000.0)) ? 1 : 0;
	I_VP_fault = (I_VP > I_VP_MAX) ? 1 : 0;

	if(I_VP_fault + I_5V_fault + I_3V3_fault > 0)
	{
		CardState = UDEV1CARD_OVERCURRENT;
		OCRtimer = UNICARD1data.OCRrecoveryTime_ms + 1;
	}
}

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	// add SCPI

	//ReformatString(RXbuff, RS485BUFFSIZE);

	strcpy((char*)buf, "ERR\n\r");

	struct word word = generateWordDirect((char*)buf);

	executeWord(word);

	for(int i = word.subwordsCount; i > 0 ; i--)
	{
		if (word.subwords[i].paramType == OTHER_P && word.subwords[i].otherParam != NULL)
		{
			free(word.subwords[i].otherParam);
			word.subwords[i].otherParam = NULL;
		}
	}
	free(word.subwords);
	word.subwords = NULL;

	/*const char* response = "Hello, World!\r\n";
	uint32_t response_len = strlen(response);*/

	CDC_Transmit_FS(Buf, response_len);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the task_USBcom thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_Delay(1000);
	//Error_Handler();
	/*if(osMutexAcquire(mutex_RS485Handle, osWaitForever) == osOK)
	{
				//RS485
				sprintf(RS485TXbuffer, "cigan");
				RS485_Transmit_DMA(&RS485cfg);
				osDelay(1);
				osMutexRelease(mutex_RS485Handle);
	}*/
	//CardState = UDEV1CARD_OVERCURRENT;
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_LEDcontrol */
/**
* @brief Function implementing the LEDcontrol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_LEDcontrol */
void task_LEDcontrol(void *argument)
{
  /* USER CODE BEGIN task_LEDcontrol */
  osDelay(500);
  /* Infinite loop */
  for(;;)
  {
	  LED_Iterate();
	  LED_UpdateEffect();
	  LED_UpdateTransition();
	  osDelay(3);
	  LED_PWMCLEAR();
	  osDelay(1);
  }
  /* USER CODE END task_LEDcontrol */
}

/* USER CODE BEGIN Header_task_Current_Check */
/**
* @brief Function implementing the Current_Check thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_Current_Check */
void task_Current_Check(void *argument)
{
  /* USER CODE BEGIN task_Current_Check */
	uint8_t outEN = 0;
  /* Infinite loop */
  for(;;)
  {
	  // this function only triggers adc sampling, check void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	  HAL_ADC_Start_DMA(&hadc1, adc1_data, 2);
	  HAL_ADC_Start_DMA(&hadc2, &adc2_data, 1);
	  HAL_ADC_Start_DMA(&hadc5, &adc5_data, 1);

	  // overcurrent recovery handling
	  if(OCRtimer > 0)
	  {
		  if(OCRtimer == 1 && !I_VP_fault)
		  {
			  switch (UNICARD1data.OCR)
			  {
			  	  default: /*UC1OCR_manual*/
			  		  //CardState = UDEV1CARD_OFF;
			  		  break;
			  	  case UC1OCR_automatic:
			  		  if(CardState != UDEV1CARD_OFF) CardState = UDEV1CARD_CON;
			  		  break;
			  	  case UC1OCR_shutdown:
			  		  CardState = UDEV1CARD_OFF;
			  		  break;
			  }
		  }
		  OCRtimer--;
	  }

	  outEN = !((CardState == UDEV1CARD_OFF) || (CardState == UDEV1CARD_OVERCURRENT));

	  HAL_GPIO_WritePin(PWR3V3_EN_OUT_GPIO_Port, PWR3V3_EN_OUT_Pin, outEN);
	  HAL_GPIO_WritePin(PWR5VEN_OUT_GPIO_Port, PWR5VEN_OUT_Pin, outEN);
	  HAL_GPIO_WritePin(PWRVCC_EN_OUT_GPIO_Port, PWRVCC_EN_OUT_Pin, outEN);

	  osDelay(1);
  }
  /* USER CODE END task_Current_Check */
}

/* USER CODE BEGIN Header_task_BUTTON */
/**
* @brief Function implementing the BUTTON thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_BUTTON */
void task_BUTTON(void *argument)
{
  /* USER CODE BEGIN task_BUTTON */
	uint16_t ButtonTimer = 0;
	uint8_t WaitingForRelease = 0;
  /* Infinite loop */
  for(;;)
  {
	if(HAL_GPIO_ReadPin(TEST_BUTTON_GPIO_Port, TEST_BUTTON_Pin))
	{
		if(ButtonTimer <= 20 && !WaitingForRelease)
			ButtonTimer++;
		else
		{
			WaitingForRelease = 1;
			lastButtonPressTime = ButtonTimer;
			ButtonTimer = 0;
		}
	}
	else
	{
		WaitingForRelease = 0;
		lastButtonPressTime = ButtonTimer;
		ButtonTimer = 0;
	}

    osDelay(50);
  }
  /* USER CODE END task_BUTTON */
}

/* USER CODE BEGIN Header_task_StateMachine */
/**
* @brief Function implementing the StateMachine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_StateMachine */
void task_StateMachine(void *argument)
{
  /* USER CODE BEGIN task_StateMachine */
  /* Infinite loop */
  for(;;)
  {
	  //---------------------------------------------- Device state machine
	  switch (MainState)
	  {
	  	  default: //OFF
	  		  leds[0] = LED_Preset(PRESET_OFF);
	  		  leds[1] = leds[0];

	  		  CardState = UDEV1CARD_OFF;
	  		  if(lastButtonPressTime >= 1)
	  		  {
	  			/*lastButtonPressTime = 0;
	  			MainState = UDEV1_ON_CON;
	  			CardState = UDEV1CARD_CON;*/
	  			NVIC_SystemReset();
	  		  }
	  		  break;
	  	  case UDEV1_ON_NC:
	  		  leds[0] = LED_Preset(PRESET_NOTLINKEDWITHPC);
	  		  leds[1] = leds[0];

	  		  if(PCcon_Watchdog > 0) MainState = UDEV1_ON_CON;

	  		  if(lastButtonPressTime >= 10)
	  		  {
	  			lastButtonPressTime = 0;
	  			MainState = UDEV1_OFF;
	  		  }
	  		  break;
	  	  case UDEV1_ON_CON:
	  		  leds[0] = LED_Preset(PRESET_LINKEDTOPC);
	  		  leds[1] = leds[0];

	  		  if(lastButtonPressTime >= 10)
	  		  {
	  			lastButtonPressTime = 0;
	  			MainState = UDEV1_OFF;
	  		  }
	  		  break;
	  }
	  //---------------------------------------------- Card state machine
	  switch (CardState)
	  {
	  	  default: //UDEV1CARD_OFF
	  		  leds[2] = LED_Preset(PRESET_OFF);

	  		  if(lastButtonPressTime >= 1 && MainState != UDEV1_OFF && OCRtimer <= 1)
	  		  {
	  			  lastButtonPressTime = 0;
	  			  CardState = !HAL_GPIO_ReadPin(CardDetect_IN_GPIO_Port, CardDetect_IN_Pin) ? UDEV1CARD_CON : UDEV1CARD_NC;
	  		  }
	  		  break;
	  	  case UDEV1CARD_NC:
	  		  leds[2] = LED_Preset(PRESET_CARDNOTDETECTED);

	  		  if(lastButtonPressTime >= 1)
	  		  {
	  			  lastButtonPressTime = 0;
	  			  CardState = UDEV1CARD_OFF;
	  		  }
	  		  break;
	  	  case UDEV1CARD_NOTRECOGNISED:
	  		  leds[2] = LED_Preset(PRESET_CARDNOTRECOGNISED);

	  		  if(lastButtonPressTime >= 1)
	  		  {
	  			  lastButtonPressTime = 0;
	  			  CardState = UDEV1CARD_OFF;
	  		  }
		  	  break;

	  	case UDEV1CARD_CON:
	  		  leds[2] = LED_Preset(PRESET_CARDRECOGNISED);

	  		  if(ComFailCounter > 5) CardState = UDEV1CARD_COMFAIL;

	  		  if(lastButtonPressTime >= 1)
	  		  {
	  			  lastButtonPressTime = 0;
	  			  CardState = UDEV1CARD_OFF;
	  		  }
	  		  break;
	  	case UDEV1CARD_OVERCURRENT:
	  		  leds[2] = LED_Preset(PRESET_OVERCURRENT);

	  		  if(lastButtonPressTime >= 1)
	  		  {
	  			  lastButtonPressTime = 0;
	  			  CardState = UDEV1CARD_OFF;
	  		  }
	  		  break;

		case UDEV1CARD_COMFAIL:
		  	leds[2] = LED_Preset(PRESET_CARDCOMFAIL);

		  	if(ComFailCounter < 3)  CardState = UDEV1CARD_CON;

		  	if(lastButtonPressTime >= 1)
		  	{
		  		lastButtonPressTime = 0;
		  		CardState = UDEV1CARD_OFF;
		  	}
		  	break;
	  }
	  osDelay(10);
  }
  /* USER CODE END task_StateMachine */
}

/* USER CODE BEGIN Header_task_PCconnectionWatchDog */
/**
* @brief Function implementing the PCconnectionWat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_PCconnectionWatchDog */
void task_PCconnectionWatchDog(void *argument)
{
  /* USER CODE BEGIN task_PCconnectionWatchDog */
  /* Infinite loop */
  for(;;)
  {
	if(MainState != UDEV1_OFF)
	{
		if(PCcon_Watchdog > 0)	PCcon_Watchdog--;
		else 					MainState = UDEV1_ON_NC;
	}
	else PCcon_Watchdog = PCcon_Watchdog_TIME;
    osDelay(100);
  }
  /* USER CODE END task_PCconnectionWatchDog */
}

/* USER CODE BEGIN Header_taks_CardConnectionWatchDog */
/**
* @brief Function implementing the CardConnectionW thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taks_CardConnectionWatchDog */
void taks_CardConnectionWatchDog(void *argument)
{
  /* USER CODE BEGIN taks_CardConnectionWatchDog */
  /* Infinite loop */
  for(;;)
  {
	if(MainState != UDEV1_OFF && CardState > UDEV1CARD_NC)
	{
		if(CardCon_Watchdog > 0)	CardCon_Watchdog--;
		else 						CardState = UDEV1CARD_NOTRECOGNISED;
	}
	else CardCon_Watchdog = 0;
    osDelay(100);
  }
  /* USER CODE END taks_CardConnectionWatchDog */
}

/* USER CODE BEGIN Header_task_CardConnectionCheck */
/**
* @brief Function implementing the CardConnectionC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_CardConnectionCheck */
void task_CardConnectionCheck(void *argument)
{
  /* USER CODE BEGIN task_CardConnectionCheck */

	uint8_t task_CardConnectionCheck_DONE = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(!HAL_GPIO_ReadPin(CardDetect_IN_GPIO_Port, CardDetect_IN_Pin))
	  {
		  if(CardState == UDEV1CARD_NC) CardState = UDEV1CARD_CON;
	  }
	  else if(CardState != UDEV1CARD_OFF) CardState = UDEV1CARD_NC;

	  task_CardConnectionCheck_DONE = 0;

	  if(MainState != UDEV1_OFF && (CardState == UDEV1CARD_NOTRECOGNISED || CardState == UDEV1CARD_CON))
	  {
	  	  for(uint8_t i = 0; i < Card_CommMaxRetryCount && !task_CardConnectionCheck_DONE; i++)
	  	  {
			  if(osMutexAcquire(mutex_RS485Handle, osWaitForever) == osOK)
			  {
				  sprintf(RS485TXbuffer, "SYS:CON?");
				  RS485_Transmit_DMA(&RS485cfg);

				  for(uint16_t timer = RS485maxTimeToTransmit_ms; timer > 0 && !RS485_TransmitDone(&RS485cfg); timer--) osDelay(1);

				  for(uint16_t timer = RS485maxTimeToReceive_ms; timer > 0 && !RS485_IsReceived(); timer--) osDelay(1);

				  if(RS485_IsReceived())// && UC1_CheckCardValidity(&UNICARD1data))
				  {
					  if(!strncmp(RS485RXbuffer, "OK\r\n", 2))
					  {
						  CardCon_Watchdog = CardCon_Watchdog_TIME;
						  CardState = UDEV1CARD_CON;
					  }
					  RS485receiveFlag = 0;
				  }
				  task_CardConnectionCheck_DONE = 1;
				  osMutexRelease(mutex_RS485Handle);
			  }
		  }
		  osDelay(1);
	  }
	  osDelay(Card_ConnectionCheckPeriod_ms);
  }
  /* USER CODE END task_CardConnectionCheck */
}

/* USER CODE BEGIN Header_task_CardUpdateStatus */
/**
* @brief Function implementing the CardUpdateStatu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_CardUpdateStatus */
void task_CardUpdateStatus(void *argument)
{
  /* USER CODE BEGIN task_CardUpdateStatus */
	uint8_t task_CardUpdateStatus_DONE = 0;
  /* Infinite loop */
  for(;;)
  {
	  	  task_CardUpdateStatus_DONE = 0;
	  	  if(MainState != UDEV1_OFF && CardState > UDEV1CARD_NOTRECOGNISED)
	  	  {
	  	  	  for(uint8_t i = 0; i < Card_CommMaxRetryCount && !task_CardUpdateStatus_DONE; i++)
	  	  	  {
	  			  if(osMutexAcquire(mutex_RS485Handle, osWaitForever) == osOK)
	  			  {
	  				  sprintf(RS485TXbuffer, "SYS:STATUS:%d",(uint8_t)CardState);
	  				  RS485_Transmit_DMA(&RS485cfg);

	  				  for(uint16_t timer = RS485maxTimeToTransmit_ms; timer > 0 && !RS485_TransmitDone(&RS485cfg); timer--) osDelay(1);

	  				  for(uint16_t timer = RS485maxTimeToReceive_ms; timer > 0 && !RS485_IsReceived(); timer--) osDelay(1);

	  				  if(RS485_IsReceived() && !strncmp(RS485RXbuffer, "OK\r\n", 2))// && UC1_CheckCardValidity(&UNICARD1data))
	  				  {
	  						if(ComFailCounter > 0) ComFailCounter -= Card_acceptableFailedComRatio;
	  						task_CardUpdateStatus_DONE = 1;
	  				  }
	  				  else if(ComFailCounter < 10) ComFailCounter += (1-Card_acceptableFailedComRatio);

	  				  RS485receiveFlag = 0;

	  				  osMutexRelease(mutex_RS485Handle);
	  			  }
	  		  }
	  		  osDelay(1);
	  	  }
    osDelay(Card_StatusUpdatePeriod_ms);
  }
  /* USER CODE END task_CardUpdateStatus */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM20 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM20) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  uint8_t toggler = 0;
  int rstTimer = 0;
  while (1)
  {
	  HAL_GPIO_WritePin(PWR3V3_EN_OUT_GPIO_Port, PWR3V3_EN_OUT_Pin, 0);
	  HAL_GPIO_WritePin(PWR5VEN_OUT_GPIO_Port, PWR5VEN_OUT_Pin, 0);
	  HAL_GPIO_WritePin(PWRVCC_EN_OUT_GPIO_Port, PWRVCC_EN_OUT_Pin, 0);

	  LED_PWMSET(1, 0, 0);
	  HAL_GPIO_WritePin(LED1_OUT_GPIO_Port, LED1_OUT_Pin, !toggler);
	  HAL_GPIO_WritePin(LED2_OUT_GPIO_Port, LED2_OUT_Pin, toggler);
	  HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, 0);
	  toggler = !toggler;

	  if(rstTimer >= 1 && !HAL_GPIO_ReadPin(TEST_BUTTON_GPIO_Port, TEST_BUTTON_Pin)) NVIC_SystemReset();

	  if(HAL_GPIO_ReadPin(TEST_BUTTON_GPIO_Port, TEST_BUTTON_Pin)) rstTimer++;
	  else if(rstTimer > 0) rstTimer--;
	  else rstTimer = 0;

	  delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
