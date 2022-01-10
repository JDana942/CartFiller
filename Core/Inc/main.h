/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define STP_X_Pin GPIO_PIN_2
#define STP_X_GPIO_Port GPIOC
#define STEPPER_ENABLE_Pin GPIO_PIN_3
#define STEPPER_ENABLE_GPIO_Port GPIOC
#define FET_HEAT_Pin GPIO_PIN_0
#define FET_HEAT_GPIO_Port GPIOA
#define FET_BED_Pin GPIO_PIN_1
#define FET_BED_GPIO_Port GPIOA
#define FET_FAN_Pin GPIO_PIN_2
#define FET_FAN_GPIO_Port GPIOA
#define LIM_X_Pin GPIO_PIN_5
#define LIM_X_GPIO_Port GPIOA
#define LIM_Y_Pin GPIO_PIN_6
#define LIM_Y_GPIO_Port GPIOA
#define LIM_Z_Pin GPIO_PIN_7
#define LIM_Z_GPIO_Port GPIOA
#define TB_Pin GPIO_PIN_4
#define TB_GPIO_Port GPIOC
#define TH_Pin GPIO_PIN_5
#define TH_GPIO_Port GPIOC
#define LCD_BUTTON_Pin GPIO_PIN_2
#define LCD_BUTTON_GPIO_Port GPIOB
#define LCD_ENC_B_Pin GPIO_PIN_10
#define LCD_ENC_B_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_11
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_ENC_A_Pin GPIO_PIN_14
#define LCD_ENC_A_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define LCD_BEEPER_Pin GPIO_PIN_6
#define LCD_BEEPER_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DIR_E_Pin GPIO_PIN_3
#define DIR_E_GPIO_Port GPIOB
#define STP_E_Pin GPIO_PIN_4
#define STP_E_GPIO_Port GPIOB
#define DIR_Z_Pin GPIO_PIN_5
#define DIR_Z_GPIO_Port GPIOB
#define STP_Z_Pin GPIO_PIN_6
#define STP_Z_GPIO_Port GPIOB
#define DIR_Y_Pin GPIO_PIN_7
#define DIR_Y_GPIO_Port GPIOB
#define STP_Y_Pin GPIO_PIN_8
#define STP_Y_GPIO_Port GPIOB
#define DIR_X_Pin GPIO_PIN_9
#define DIR_X_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
