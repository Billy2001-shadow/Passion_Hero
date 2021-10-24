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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define CAN2_RX_Pin GPIO_PIN_5
#define CAN2_RX_GPIO_Port GPIOB
#define USART6_TX_PC_Pin GPIO_PIN_14
#define USART6_TX_PC_GPIO_Port GPIOG
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_6
#define CAN2_TX_GPIO_Port GPIOB
#define CAN1_RX_Pin GPIO_PIN_0
#define CAN1_RX_GPIO_Port GPIOD
#define DBUS_RX_Pin GPIO_PIN_11
#define DBUS_RX_GPIO_Port GPIOC
#define RED_IT_Pin GPIO_PIN_7
#define RED_IT_GPIO_Port GPIOI
#define RED_IT_EXTI_IRQn EXTI9_5_IRQn
#define USART6_RX_PC_Pin GPIO_PIN_9
#define USART6_RX_PC_GPIO_Port GPIOG
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define LASTER_Pin GPIO_PIN_8
#define LASTER_GPIO_Port GPIOC
#define IST8310_RST_Pin GPIO_PIN_6
#define IST8310_RST_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define IST8310_DRDY_Pin GPIO_PIN_3
#define IST8310_DRDY_GPIO_Port GPIOG
#define IST8310_DRDY_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI0_IRQn
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
