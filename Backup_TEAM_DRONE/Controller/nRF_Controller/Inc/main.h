/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define button_Pin GPIO_PIN_13
#define button_GPIO_Port GPIOC
#define button_EXTI_IRQn EXTI15_10_IRQn
#define THROTTLE_2_Pin GPIO_PIN_0
#define THROTTLE_2_GPIO_Port GPIOC
#define THROTTLE_1_Pin GPIO_PIN_1
#define THROTTLE_1_GPIO_Port GPIOC
#define JOYSTICK_1_Pin GPIO_PIN_0
#define JOYSTICK_1_GPIO_Port GPIOA
#define JOYSTICK_2_Pin GPIO_PIN_1
#define JOYSTICK_2_GPIO_Port GPIOA
#define CSNpin_Pin GPIO_PIN_5
#define CSNpin_GPIO_Port GPIOA
#define CEpin_Pin GPIO_PIN_6
#define CEpin_GPIO_Port GPIOA
#define button2_Pin GPIO_PIN_10
#define button2_GPIO_Port GPIOC
#define button2_EXTI_IRQn EXTI15_10_IRQn
#define button3_Pin GPIO_PIN_12
#define button3_GPIO_Port GPIOC
#define button3_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
