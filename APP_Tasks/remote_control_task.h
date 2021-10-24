#ifndef  __REMOTE_CONTORL_TASK_H
#define	 __REMOTE_CONTORL_TASK_H
/* 包含头文件 ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* 本模块向外部提供的宏定义 -------------------------------------------------*/

/* 本模块向外部提供的结构体/枚举定义 ----------------------------------------*/
typedef enum
{
 STOP_SHOOT,      //关闭发射
	SHOOT_READY,     //准备发射
 SHOOT_ON,        //开启发射
	SHOOT_OFF,       //发射结束
	SHOOTING,        //正向拨弹（射击）
	BACK             //反向拨弹（调整下一颗子弹位置）
}SHOOT_MODE;

typedef enum
{
 GROY,          //陀螺仪
 ENCODER,        //编码器
 PC,
	LOCK,
}GIMBAL_MODE;
typedef enum
{
 MOUSE=1,         //键鼠
 RC_CONTROL,      //遥控器
 PC_CONTROL       //视觉
}CONTROL_MODE;
typedef struct
{
 uint8_t gimbal; 
 uint8_t gun;
 uint8_t mode;
}C_Board_t;
typedef struct
{
	float yaw_gory;
	float pitch_gory;
	float yaw_encoder;
	float pitch_encoder;
	float yaw_lock;
	float pitch_lock;
	float yaw_pc;
	float pitch_pc;
	float initial_yaw;
	float initial_pitch;
	float Shoot_speed;
	float trigger_angle;
	float plate_angle;
}expect_t;
/* 本模块向外部提供的变量声明 -----------------------------------------------*/

/* 本模块向外部提供的自定义数据类型变量声明 ---------------------------------*/
extern C_Board_t C_Board;
extern expect_t  expect;
/* 本模块向外部提供的接口函数原型声明 ---------------------------------------*/



#endif
