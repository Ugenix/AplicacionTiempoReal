/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_1
#define LCD_EN_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_12
#define D7_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_13
#define D6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_14
#define D5_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_15
#define D4_GPIO_Port GPIOB
#define RGB_blue_Pin GPIO_PIN_7
#define RGB_blue_GPIO_Port GPIOC
#define RGB_green_Pin GPIO_PIN_9
#define RGB_green_GPIO_Port GPIOA
#define RGB_red_Pin GPIO_PIN_10
#define RGB_red_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BotonLCD_Pin GPIO_PIN_10
#define BotonLCD_GPIO_Port GPIOC
#define BotonLCD_EXTI_IRQn EXTI15_10_IRQn
#define BotonRGB_Pin GPIO_PIN_11
#define BotonRGB_GPIO_Port GPIOC
#define BotonRGB_EXTI_IRQn EXTI15_10_IRQn
#define BotonAlarma_Pin GPIO_PIN_12
#define BotonAlarma_GPIO_Port GPIOC
#define BotonAlarma_EXTI_IRQn EXTI15_10_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define zumbador_Pin GPIO_PIN_4
#define zumbador_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
