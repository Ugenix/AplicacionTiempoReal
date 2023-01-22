/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "lcd.h"
#include "ds1307.h"
#include "ssd1306.h"
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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLED_Task */
osThreadId_t OLED_TaskHandle;
const osThreadAttr_t OLED_Task_attributes = {
  .name = "OLED_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RGB_Task */
osThreadId_t RGB_TaskHandle;
const osThreadAttr_t RGB_Task_attributes = {
  .name = "RGB_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADQUISICION_Tas */
osThreadId_t ADQUISICION_TasHandle;
const osThreadAttr_t ADQUISICION_Tas_attributes = {
  .name = "ADQUISICION_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = {
  .name = "LCD_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ALARMA_Task */
osThreadId_t ALARMA_TaskHandle;
const osThreadAttr_t ALARMA_Task_attributes = {
  .name = "ALARMA_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LOG_data_Task */
osThreadId_t LOG_data_TaskHandle;
const osThreadAttr_t LOG_data_Task_attributes = {
  .name = "LOG_data_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Adquisicion_data */
osMessageQueueId_t Adquisicion_dataHandle;
const osMessageQueueAttr_t Adquisicion_data_attributes = {
  .name = "Adquisicion_data"
};
/* Definitions for Alarma_data */
osMessageQueueId_t Alarma_dataHandle;
const osMessageQueueAttr_t Alarma_data_attributes = {
  .name = "Alarma_data"
};
/* Definitions for LOG_data */
osMessageQueueId_t LOG_dataHandle;
const osMessageQueueAttr_t LOG_data_attributes = {
  .name = "LOG_data"
};
/* Definitions for Alarma_mutex */
osMutexId_t Alarma_mutexHandle;
const osMutexAttr_t Alarma_mutex_attributes = {
  .name = "Alarma_mutex"
};
/* Definitions for OLED_UART_flag */
osEventFlagsId_t OLED_UART_flagHandle;
const osEventFlagsAttr_t OLED_UART_flag_attributes = {
  .name = "OLED_UART_flag"
};
/* Definitions for RGB_flag */
osEventFlagsId_t RGB_flagHandle;
const osEventFlagsAttr_t RGB_flag_attributes = {
  .name = "RGB_flag"
};
/* Definitions for LCD_DISPLAY_flag */
osEventFlagsId_t LCD_DISPLAY_flagHandle;
const osEventFlagsAttr_t LCD_DISPLAY_flag_attributes = {
  .name = "LCD_DISPLAY_flag"
};
/* Definitions for Alarm_flag */
osEventFlagsId_t Alarm_flagHandle;
const osEventFlagsAttr_t Alarm_flag_attributes = {
  .name = "Alarm_flag"
};
/* Definitions for Alarm_active_flag */
osEventFlagsId_t Alarm_active_flagHandle;
const osEventFlagsAttr_t Alarm_active_flag_attributes = {
  .name = "Alarm_active_flag"
};
/* Definitions for RGB_Active_flag */
osEventFlagsId_t RGB_Active_flagHandle;
const osEventFlagsAttr_t RGB_Active_flag_attributes = {
  .name = "RGB_Active_flag"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
void UART_Task_function(void *argument);
void OLED_Task_function(void *argument);
void RGB_Task_function(void *argument);
void ADQUISICION_Task_function(void *argument);
void LCD_Task_function(void *argument);
void ALARMA_Task_function(void *argument);
void LOG_data_Task_function(void *argument);

/* USER CODE BEGIN PFP */
void ssd1306_PutText();
void set_RGB(uint8_t red, uint8_t green, uint8_t blue);
void lcd_config();
void lcd_rgb_init(LCD_t *lcd_dir);
void ds1307_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LCD_t lcd;
uint8_t buffer[8]; //buffer para almacenar datos del UART
uint8_t H_data = 0;

struct horas{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
};

struct fechas{
	uint8_t week_day;
	uint8_t day;
	uint8_t month;
	uint8_t year;
};
struct log{
	char Hora[16];
	char Fecha[16];
};

struct horas alarma = {10, 0, 0};
uint8_t set_hora[3] = {0, 0, 0};
uint8_t set_date[4] = {1, 1, 1, 1};
char *RESPUESTAS[2] = {"L:", "R:Y"};

struct horas set_horas;
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
  lcd_config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 5);
  ssd1306_WriteString("Hello", Font_16x26, White);
  ssd1306_SetCursor(30, 35);
  ssd1306_WriteString("World!", Font_16x26, White);
  ssd1306_UpdateScreen();
  lcd_rgb_init(&lcd);
  ds1307_init();
  ssd1306_PutText();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Alarma_mutex */
  Alarma_mutexHandle = osMutexNew(&Alarma_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Adquisicion_data */
  Adquisicion_dataHandle = osMessageQueueNew (32, sizeof(float), &Adquisicion_data_attributes);

  /* creation of Alarma_data */
  Alarma_dataHandle = osMessageQueueNew (128, sizeof(int), &Alarma_data_attributes);

  /* creation of LOG_data */
  LOG_dataHandle = osMessageQueueNew (128, 32, &LOG_data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(UART_Task_function, NULL, &UART_Task_attributes);

  /* creation of OLED_Task */
  OLED_TaskHandle = osThreadNew(OLED_Task_function, NULL, &OLED_Task_attributes);

  /* creation of RGB_Task */
  RGB_TaskHandle = osThreadNew(RGB_Task_function, NULL, &RGB_Task_attributes);

  /* creation of ADQUISICION_Tas */
  ADQUISICION_TasHandle = osThreadNew(ADQUISICION_Task_function, NULL, &ADQUISICION_Tas_attributes);

  /* creation of LCD_Task */
  LCD_TaskHandle = osThreadNew(LCD_Task_function, NULL, &LCD_Task_attributes);

  /* creation of ALARMA_Task */
  ALARMA_TaskHandle = osThreadNew(ALARMA_Task_function, NULL, &ALARMA_Task_attributes);

  /* creation of LOG_data_Task */
  LOG_data_TaskHandle = osThreadNew(LOG_data_Task_function, NULL, &LOG_data_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of OLED_UART_flag */
  OLED_UART_flagHandle = osEventFlagsNew(&OLED_UART_flag_attributes);

  /* creation of RGB_flag */
  RGB_flagHandle = osEventFlagsNew(&RGB_flag_attributes);

  /* creation of LCD_DISPLAY_flag */
  LCD_DISPLAY_flagHandle = osEventFlagsNew(&LCD_DISPLAY_flag_attributes);

  /* creation of Alarm_flag */
  Alarm_flagHandle = osEventFlagsNew(&Alarm_flag_attributes);

  /* creation of Alarm_active_flag */
  Alarm_active_flagHandle = osEventFlagsNew(&Alarm_active_flag_attributes);

  /* creation of RGB_Active_flag */
  RGB_Active_flagHandle = osEventFlagsNew(&RGB_Active_flag_attributes);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RGB_green_Pin|RGB_red_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_EN_Pin|LCD_RS_Pin|D7_Pin|D6_Pin
                          |D5_Pin|D4_Pin|zumbador_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_blue_GPIO_Port, RGB_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RGB_green_Pin RGB_red_Pin PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|RGB_green_Pin|RGB_red_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RS_Pin D7_Pin D6_Pin
                           D5_Pin D4_Pin zumbador_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RS_Pin|D7_Pin|D6_Pin
                          |D5_Pin|D4_Pin|zumbador_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_blue_Pin */
  GPIO_InitStruct.Pin = RGB_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BotonLCD_Pin BotonRGB_Pin BotonAlarma_Pin */
  GPIO_InitStruct.Pin = BotonLCD_Pin|BotonRGB_Pin|BotonAlarma_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
**Funcion callback de Rx de UART
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	int comand = 0;
	int type_flag = 0;
	if(buffer[1] == 58){
		if(buffer[2] >= 48 && buffer[2] <= 57) H_data = (buffer[2] - 48); //decimal
		if(buffer[3] >= 48 && buffer[3] <= 57){H_data = (H_data * 10); H_data += (buffer[3] - 48);} //unidad
		comand = buffer[0];
		switch(comand)
		{
		case 72: //H-->hora
			rtc_get_time(&set_horas.hour, &set_horas.min, &set_horas.sec);
			set_horas.hour = H_data;
			type_flag = 1;
			break;
		case 77: //M-->min
			rtc_get_time(&set_horas.hour, &set_horas.min, &set_horas.sec);
			set_horas.min = H_data;
			type_flag = 1;
			break;
		case 83: //S-->seg
			rtc_get_time(&set_horas.hour, &set_horas.min, &set_horas.sec);
			set_horas.sec = H_data;
			type_flag = 1;
			break;
		case 100: // d-->dia
			set_date[0] = H_data;
			type_flag = 0;
			break;
		case 109: // m-->mes
			set_date[1] = H_data;
			type_flag = 0;
			break;
		case 121: // y-->year
			set_date[2] = H_data;
			type_flag = 2;
			break;
		case 119: // w-->week day
			set_date[3] = H_data;
			type_flag = 0;
			break;
		case 88: // X--> H alarma
			osMutexAcquire(Alarma_mutexHandle, osWaitForever);
			alarma.hour = H_data;
			osMutexRelease(Alarma_mutexHandle);
			type_flag = 0;
			break;
		case 89: // Y--> M alarma
			osMutexAcquire(Alarma_mutexHandle, osWaitForever);
			alarma.min = H_data;
			osMutexRelease(Alarma_mutexHandle);
			type_flag = 0;
			break;
		case 90: // Z--> S alarma
			osMutexAcquire(Alarma_mutexHandle, osWaitForever);
			alarma.sec = H_data;
			osMutexRelease(Alarma_mutexHandle);
			type_flag = 0;
			break;
		}
	}
	if(type_flag == 1) rtc_set_time(set_horas.hour, set_horas.min, set_horas.sec);
	else if(type_flag == 2) rtc_set_date(set_date[3], set_date[0], set_date[1], set_date[2]);
	osEventFlagsSet(OLED_UART_flagHandle, 0x01U);
	HAL_UART_Receive_IT(&huart2, buffer, 5);
}

/*
**Funcion callback de Tx de UART
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
}

/*
**Funcion callback de las interrupciones EXTI
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
	if(GPIO_pin == GPIO_PIN_11) //Boton RGB
	{
		switch(osEventFlagsGet(RGB_flagHandle))
		{
			case 0x80U:
				osEventFlagsClear(RGB_flagHandle, 0x80U);
				osEventFlagsSet(RGB_flagHandle, 0x01U);
				break;
			case 0x01U:
				osEventFlagsClear(RGB_flagHandle, 0x01U);
				osEventFlagsSet(RGB_flagHandle, 0x02U);
				break;
			case 0x02U:
				osEventFlagsClear(RGB_flagHandle, 0x02U);
				osEventFlagsSet(RGB_flagHandle, 0x04U);
				break;
			case 0x04U:
				osEventFlagsClear(RGB_flagHandle, 0x04U);
				osEventFlagsSet(RGB_flagHandle, 0x08U);
				break;
			case 0x08U:
				osEventFlagsClear(RGB_flagHandle, 0x08U);
				osEventFlagsSet(RGB_flagHandle, 0x10U);
				break;
			case 0x10U:
				osEventFlagsClear(RGB_flagHandle, 0x10U);
				osEventFlagsSet(RGB_flagHandle, 0x20U);
				break;
			case 0x20U:
				osEventFlagsClear(RGB_flagHandle, 0x20U);
				osEventFlagsSet(RGB_flagHandle, 0x40U);
				break;
			case 0x40U:
				osEventFlagsClear(RGB_flagHandle, 0x40U);
				osEventFlagsSet(RGB_flagHandle, 0x80U);
				break;
			case 0x00U:
				osEventFlagsSet(RGB_flagHandle, 0x01U);
				break;
		}
		osEventFlagsSet(Alarm_active_flagHandle, 0x01U);
	}
	else if(GPIO_pin == GPIO_PIN_10) //Boton display
	{
		if((osEventFlagsGet(LCD_DISPLAY_flagHandle))==0x01U)
		{
			ssd1306_SetDisplayOn(0);
			lcd_noDisplay(&lcd);
			osEventFlagsClear(LCD_DISPLAY_flagHandle, 0x01U);
			osEventFlagsSet(LCD_DISPLAY_flagHandle, 0x02U);
		}
		else
		{
			ssd1306_SetDisplayOn(1);
			lcd_display(&lcd);
			osEventFlagsClear(LCD_DISPLAY_flagHandle, 0x02U);
			osEventFlagsSet(LCD_DISPLAY_flagHandle, 0x01U);
		}
	}
	else if(GPIO_pin == GPIO_PIN_12){
		if((osEventFlagsGet(Alarm_flagHandle))==0x01U)
		{
			osEventFlagsClear(Alarm_flagHandle, 0x01U);
			osEventFlagsSet(Alarm_flagHandle, 0x00U);
		}
		else if((osEventFlagsGet(Alarm_flagHandle))==0x00U)
		{
			osEventFlagsClear(Alarm_flagHandle, 0x00U);
			osEventFlagsSet(Alarm_flagHandle, 0x01U);
		}
	}
	else if(GPIO_pin == GPIO_PIN_13){
		if((osEventFlagsGet(OLED_UART_flagHandle)&0x08U)==0x00U)
		{
			osEventFlagsSet(OLED_UART_flagHandle, 0x08U);
			osEventFlagsSet(OLED_UART_flagHandle, 0x04U);
		}
		else if((osEventFlagsGet(OLED_UART_flagHandle)&0x08U)==0x08U)
		{
			osEventFlagsClear(OLED_UART_flagHandle, 0x08U);
			osEventFlagsClear(OLED_UART_flagHandle, 0x04U);
		}
	}
	else __NOP();
}

/*
**Funcion la pantalla OLED
*/
void ssd1306_PutText()
{
    char buff[10];
    char *alarm_text[2] = {"OFF", "ON "};
    char *log_msg[10] = {"------------------", "Guardando datos..."};

    ssd1306_Fill(Black);

	ssd1306_SetCursor(10, 0);
	ssd1306_WriteString("Comandos:", Font_6x8, White);
	ssd1306_SetCursor(74, 0);
	ssd1306_WriteString(buffer, Font_11x18, White);
	ssd1306_SetCursor(95, 0);
	sprintf(buff ,"%02d", H_data);
	ssd1306_WriteString(buff, Font_16x26, White);
	ssd1306_SetCursor(10, 10);
	ssd1306_WriteString("Alarm:", Font_6x8, White);
	ssd1306_SetCursor(10, 22);
	sprintf(buff ,"%02d:%02d:%02d", alarma.hour, alarma.min, alarma.sec);
	ssd1306_WriteString(buff, Font_11x18, White);
	ssd1306_SetCursor(100, 30);
	ssd1306_WriteString(alarm_text[osEventFlagsGet(Alarm_flagHandle)], Font_6x8, White);
	ssd1306_SetCursor(12, 45);
	ssd1306_WriteString(log_msg[((osEventFlagsGet(OLED_UART_flagHandle)>>3))], Font_6x8, White);

    ssd1306_UpdateScreen();
}

/*
**Funcion para la configuracion del color del led RGB
*/
void set_RGB(uint8_t red, uint8_t green, uint8_t blue) //
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, blue);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, green);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, red);
}

/*
**Funcion para la configuracion del LCD
*/
void lcd_config() {
	lcd.RS_port= LCD_RS_GPIO_Port;
	lcd.RS_pin = LCD_RS_Pin;
	lcd.EN_port = LCD_EN_GPIO_Port;
	lcd.EN_pin = LCD_EN_Pin;
	lcd.D4_port = D4_GPIO_Port;
	lcd.D4_pin = D4_Pin;
	lcd.D5_port = D5_GPIO_Port;
	lcd.D5_pin = D5_Pin;
	lcd.D6_port = D6_GPIO_Port;
	lcd.D6_pin = D6_Pin;
	lcd.D7_port = D7_GPIO_Port;
	lcd.D7_pin = D7_Pin;
}

/*
**Inicializacion del reloj y el led RGB
*/
void lcd_rgb_init(LCD_t *lcd_dir) {
	lcd_begin(lcd_dir, 16, 2, LCD_5x8DOTS);

	// Mensaje inicial
	lcd_home(lcd_dir);
	lcd_setCursor(lcd_dir, 0, 0);
	lcd_print(lcd_dir, "  Tiempo Real   ");
	lcd_setCursor(lcd_dir, 0, 1);
	lcd_print(lcd_dir, "  Xiang & Alex  ");
	set_RGB(1,1,1);
	HAL_Delay(3000);
	lcd_clear(lcd_dir);
}

/*
**Inicializacion del modulo DS1307
*/
void ds1307_init()
{
	rtc_init(0,1,0);
	rtc_set_time(00,00,00);
	//rtc_set_date(1, 11, 11, 11);
	rtc_set_date(set_date[3], set_date[0], set_date[1], set_date[2]);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_UART_Task_function */
/**
  * @brief  Function implementing the UART_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_UART_Task_function */
void UART_Task_function(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_UART_Receive_IT(&huart2, buffer, 5);
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(OLED_UART_flagHandle, 0x02U, osFlagsNoClear, osWaitForever);
	  if((osEventFlagsGet(OLED_UART_flagHandle)&0x04U)==0x00U){
		  HAL_UART_Transmit_IT(&huart2, RESPUESTAS[1], strlen(RESPUESTAS[1]));
	  }
	  osEventFlagsClear(OLED_UART_flagHandle, 0x00000002U);
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_OLED_Task_function */
/**
* @brief Function implementing the OLED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_Task_function */
void OLED_Task_function(void *argument)
{
  /* USER CODE BEGIN OLED_Task_function */
  /* Infinite loop */
  for(;;)
  {
	  //osEventFlagsWait(OLED_UART_flagHandle, 0x00000001U, osFlagsNoClear, osWaitForever);
	  ssd1306_PutText();
	  if((osEventFlagsGet(OLED_UART_flagHandle)&0x01U)==0x01U){
		  osEventFlagsSet(OLED_UART_flagHandle, 0x02U);
		  osEventFlagsClear(OLED_UART_flagHandle, 0x01U);
	  }
    osDelay(100);
  }
  /* USER CODE END OLED_Task_function */
}

/* USER CODE BEGIN Header_RGB_Task_function */
/**
* @brief Function implementing the RGB_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RGB_Task_function */
void RGB_Task_function(void *argument)
{
  /* USER CODE BEGIN RGB_Task_function */
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(Alarm_active_flagHandle, 0x01U, osFlagsWaitAny, osWaitForever);
	  switch(osEventFlagsGet(RGB_flagHandle))
  	  {
  		case 0x01U:
			set_RGB(1,0,0); // R
	  		break;
  		case 0x02U:
  			set_RGB(0,1,0); // G
  			break;
  		case 0x04U:
  			set_RGB(0,0,1); // B
  			break;
  		case 0x08U:
  			set_RGB(1,1,0); // R+G
			break;
  		case 0x10U:
  			set_RGB(1,0,1); // R+B
  			break;
  		case 0x20U:
  			set_RGB(0,1,1); // G+B
  			break;
  		case 0x40U:
  			set_RGB(1,1,1); // R+G+B
  			break;
  		case 0x80U:
  			set_RGB(0,0,0); // apagado
  			break;
	  	}
    osDelay(100);
  }
  /* USER CODE END RGB_Task_function */
}

/* USER CODE BEGIN Header_ADQUISICION_Task_function */
/**
* @brief Function implementing the ADQUISICION_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADQUISICION_Task_function */
void ADQUISICION_Task_function(void *argument)
{
  /* USER CODE BEGIN ADQUISICION_Task_function */
	HAL_StatusTypeDef status;
	uint8_t temp;
	float temp_celsius;
  /* Infinite loop */
  for(;;)
  {
  		//Adquisicion ADC
		HAL_ADC_Start(&hadc1);
		status = HAL_ADC_PollForConversion(&hadc1, 1);
		if(status==HAL_OK)
			temp = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		
		//Conversion
		temp_celsius = (((0.001221)*temp)-0.5)/(0.01);
		
		//Meter el dato en la cola
		osMessageQueuePut(Adquisicion_dataHandle, &temp_celsius, osPriorityNormal, osWaitForever);
    osDelay(1000);
  }
  /* USER CODE END ADQUISICION_Task_function */
}

/* USER CODE BEGIN Header_LCD_Task_function */
/**
* @brief Function implementing the LCD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCD_Task_function */
void LCD_Task_function(void *argument)
{
  /* USER CODE BEGIN LCD_Task_function */
	struct horas hora_value;
	struct fechas fecha_value;
	float temp_celsius;
	char *dayName[] = {"LUN", "MAR", "MIE", "JUE", "VIE", "SAB", "DOM"};
	char str[16];
	struct log Log;
  /* Infinite loop */
  for(;;)
  {
  		//Adquisición de la temperatura
	  	osMessageQueueGet(Adquisicion_dataHandle, &temp_celsius, NULL, osWaitForever);
	  	
	  	//Adquisición de la hora y el día
		rtc_get_time(&hora_value.hour, &hora_value.min, &hora_value.sec);
		rtc_get_date(&fecha_value.week_day, &fecha_value.day, &fecha_value.month, &fecha_value.year);
		
		//Mostrar datos por pantalla LCD
		lcd_home(&lcd);
		lcd_setCursor(&lcd, 0, 0);
		memset(Log.Fecha, 0, sizeof(Log.Hora));
		sprintf(Log.Fecha,"%02d/%02d/%02d   %s", fecha_value.day, fecha_value.month, fecha_value.year, dayName[fecha_value.week_day-1]);
		lcd_print(&lcd, Log.Fecha);
		lcd_home(&lcd);
		lcd_setCursor(&lcd, 0, 1);
		memset(Log.Hora, 0, sizeof(Log.Hora));
		sprintf(Log.Hora,"%02d:%02d:%02d %d.%01dC", hora_value.hour, hora_value.min, hora_value.sec, (int)temp_celsius, abs((int)((temp_celsius - (int)temp_celsius)*10)));
		lcd_print(&lcd, Log.Hora);
		
		//Insertar los datos en sus respectivas colas
		if((osEventFlagsGet(OLED_UART_flagHandle)&0x08U) == 0x08U){
			osMessageQueuePut(LOG_dataHandle, &Log, osPriorityNormal, osWaitForever);
		}
		if(osEventFlagsGet(Alarm_flagHandle) == 0x01U){
			osMessageQueuePut(Alarma_dataHandle, &hora_value, osPriorityNormal, osWaitForever);
		}
    osDelay(20);
  }
  /* USER CODE END LCD_Task_function */
}

/* USER CODE BEGIN Header_ALARMA_Task_function */
/**
* @brief Function implementing the ALARMA_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ALARMA_Task_function */
void ALARMA_Task_function(void *argument)
{
  /* USER CODE BEGIN ALARMA_Task_function */
	struct horas alarma_value;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(Alarma_dataHandle, &alarma_value, NULL, osWaitForever);
	  
	  //Comparacion
	  if(alarma.hour == alarma_value.hour && alarma.min == alarma_value.min && alarma.sec == alarma_value.sec){
		  osEventFlagsSet(Alarm_active_flagHandle, 0x01U);
	  }
	  
	  if(osEventFlagsGet(Alarm_active_flagHandle) == 0x01U && osEventFlagsGet(Alarm_flagHandle) == 0x01U){
		  while(osEventFlagsGet(Alarm_flagHandle) == 0x01U){
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			  osDelay(200);
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			  osDelay(50);
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			  osDelay(200);
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			  osDelay(490);
		  }
	  }
	  osEventFlagsClear(Alarm_active_flagHandle, 0x01U);

    osDelay(10);
  }
  /* USER CODE END ALARMA_Task_function */
}

/* USER CODE BEGIN Header_LOG_data_Task_function */
/**
* @brief Function implementing the LOG_data_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LOG_data_Task_function */
void LOG_data_Task_function(void *argument)
{
  /* USER CODE BEGIN LOG_data_Task_function */
	struct log Log_data;
	char Log_str[63] = {};
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(OLED_UART_flagHandle, 0x08U, osFlagsNoClear, osWaitForever);
	  while((osEventFlagsGet(OLED_UART_flagHandle)&0x08U) == 0x08U){
		  osMessageQueueGet(LOG_dataHandle, &Log_data, NULL, osWaitForever);
		  memset(Log_str, 0, sizeof(Log_str));
		  strcat(Log_str, RESPUESTAS[0]);
		  strcat(Log_str, Log_data.Fecha);
		  strcat(Log_str, "	");
		  strcat(Log_str, Log_data.Hora);
		  strcat(Log_str, "\n");
		  HAL_UART_Transmit_IT(&huart2, Log_str, strlen(Log_str));
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
		  osDelay(1000);
	  }
    osDelay(100);
  }
  /* USER CODE END LOG_data_Task_function */
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
  while (1)
  {
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
