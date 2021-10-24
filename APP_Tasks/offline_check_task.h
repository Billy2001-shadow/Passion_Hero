#ifndef	OFFLINE_CHECK_TASK_H
#define	OFFLINE_CHECK_TASK_H
/* 包含头文件 ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "cmsis_os.h"

/* 本模块向外部提供的宏定义 -------------------------------------------------*/
#define LED1_ON()     HAL_GPIO_WritePin( GPIOH,GPIO_PIN_10,GPIO_PIN_SET)
#define LED1_OFF()    HAL_GPIO_WritePin (GPIOH,GPIO_PIN_10,GPIO_PIN_RESET)
#define	LED1_TOG() 	  HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10)

#define LED2_ON()     HAL_GPIO_WritePin (GPIOH,GPIO_PIN_11,GPIO_PIN_SET)
#define LED2_OFF()    HAL_GPIO_WritePin (GPIOH,GPIO_PIN_11,GPIO_PIN_RESET)
#define	LED2_TOG() 	  HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11)

#define LED3_ON()     HAL_GPIO_WritePin (GPIOH,GPIO_PIN_12,GPIO_PIN_SET)
#define LED3_OFF()    HAL_GPIO_WritePin (GPIOH,GPIO_PIN_12,GPIO_PIN_RESET)
#define	LED3_TOG() 	  HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12)
/* 本模块向外部提供的结构体/枚举定义 ----------------------------------------*/

/* 本模块向外部提供的变量声明 -----------------------------------------------*/

/* 本模块向外部提供的自定义数据类型变量声明 ---------------------------------*/

/* 本模块向外部提供的接口函数原型声明 ---------------------------------------*/





#endif

