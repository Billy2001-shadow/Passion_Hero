#ifndef __RC_H
#define __RC_H
/* 包含头文件 ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "cmsis_os.h"
/* 本模块向外部提供的宏定义 -------------------------------------------------*/
#define DR16_RX_BUFFER_SIZE 36u
#define DR16_DATA_LEN 18u

#define KEY_PRESSED_OFFSET_W     ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R     ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F     ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G     ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X     ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C     ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V     ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B     ((uint16_t)1 << 15)


#define Left_Press          (rc_ctrl.mouse.press_l==1)
#define Left__NoPress       (rc_ctrl.mouse.press_l==0)
#define Right_Press         (rc_ctrl.mouse.press_r==1)
#define Right_NoPress       (rc_ctrl.mouse.press_r==0)

#define W_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)
#define S_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)
#define A_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)
#define D_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)

#define SHIFT_Press         (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)
#define CTRL_Press          (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)

#define Q_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
#define E_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
#define R_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
#define F_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
#define G_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)
#define Z_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
#define X_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
#define C_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
#define V_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
#define B_Press             (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)

/* 本模块向外部提供的结构体/枚举定义 ----------------------------------------*/
//遥控器数据结构体
typedef __packed struct
{
	__packed struct
	{
			int16_t ch0;
			int16_t ch1;
			int16_t ch2;
			int16_t ch3;
			uint8_t s1;
		  uint8_t s2;
	} rc;
	__packed struct
	{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
	}mouse;
	__packed struct
	{
			uint16_t v;
	} key;

} RC_ctrl_t;

/* 本模块向外部提供的变量声明 -----------------------------------------------*/

/* 本模块向外部提供的自定义数据类型变量声明 ---------------------------------*/
extern RC_ctrl_t rc_ctrl;//遥控器数据结构体


/* 本模块向外部提供的接口函数原型声明 ---------------------------------------*/
void Remote_Control_USART_Init(void);
void DR16_UART_IRQHandler(void);

#endif
