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
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Forward_Pin GPIO_PIN_0
#define Forward_GPIO_Port GPIOC
#define Forward_EXTI_IRQn EXTI0_IRQn
#define Reverse_Pin GPIO_PIN_1
#define Reverse_GPIO_Port GPIOC
#define Reverse_EXTI_IRQn EXTI1_IRQn
#define Right_Pin GPIO_PIN_2
#define Right_GPIO_Port GPIOC
#define Right_EXTI_IRQn EXTI2_IRQn
#define Left_Pin GPIO_PIN_3
#define Left_GPIO_Port GPIOC
#define Left_EXTI_IRQn EXTI3_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Pump_Button_Pin GPIO_PIN_6
#define Pump_Button_GPIO_Port GPIOC
#define Pump_Button_EXTI_IRQn EXTI9_5_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LeftMotor_Pin GPIO_PIN_4
#define LeftMotor_GPIO_Port GPIOB
#define RightMotor_Pin GPIO_PIN_5
#define RightMotor_GPIO_Port GPIOB
#define Pump_Control_Pin GPIO_PIN_7
#define Pump_Control_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void MotorControlFWD();
void MotorControlREV();
void MotorControlLEFT();
void MotorControlRIGHT();
void MotorControlSTOP();

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
