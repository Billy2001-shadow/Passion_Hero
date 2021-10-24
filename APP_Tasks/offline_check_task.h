#ifndef	OFFLINE_CHECK_TASK_H
#define	OFFLINE_CHECK_TASK_H
/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "cmsis_os.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/
#define LED1_ON()     HAL_GPIO_WritePin( GPIOH,GPIO_PIN_10,GPIO_PIN_SET)
#define LED1_OFF()    HAL_GPIO_WritePin (GPIOH,GPIO_PIN_10,GPIO_PIN_RESET)
#define	LED1_TOG() 	  HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10)

#define LED2_ON()     HAL_GPIO_WritePin (GPIOH,GPIO_PIN_11,GPIO_PIN_SET)
#define LED2_OFF()    HAL_GPIO_WritePin (GPIOH,GPIO_PIN_11,GPIO_PIN_RESET)
#define	LED2_TOG() 	  HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11)

#define LED3_ON()     HAL_GPIO_WritePin (GPIOH,GPIO_PIN_12,GPIO_PIN_SET)
#define LED3_OFF()    HAL_GPIO_WritePin (GPIOH,GPIO_PIN_12,GPIO_PIN_RESET)
#define	LED3_TOG() 	  HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12)
/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/

/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/

/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/





#endif

