/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TFT_CS_Pin GPIO_PIN_0
#define TFT_CS_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_1
#define TFT_RST_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_2
#define TFT_DC_GPIO_Port GPIOB
#define TP_CS_Pin GPIO_PIN_12
#define TP_CS_GPIO_Port GPIOB
#define TP_IRQ_Pin GPIO_PIN_8
#define TP_IRQ_GPIO_Port GPIOA
#define SelectButton_Pin GPIO_PIN_5
#define SelectButton_GPIO_Port GPIOB
#define SelectButton_EXTI_IRQn EXTI9_5_IRQn
#define RightButton_Pin GPIO_PIN_6
#define RightButton_GPIO_Port GPIOB
#define RightButton_EXTI_IRQn EXTI9_5_IRQn
#define LeftButton_Pin GPIO_PIN_7
#define LeftButton_GPIO_Port GPIOB
#define LeftButton_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
void drawGraticule(uint16_t xDivCount, uint16_t yDivCount, uint16_t divLength);
void findTrigger();
void graphData(uint16_t w, uint16_t h, uint32_t color);
void displayStats(uint8_t spacing);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
