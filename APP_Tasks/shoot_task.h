#ifndef  __SHOOT_FRICTION_TASK_H
#define  __SHOOT_FRICTION_TASK_H
/* 包含头文件 ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "user_lib.h"
#include "cmsis_os.h"

/* 本模块向外部提供的宏定义 -------------------------------------------------*/

/* 本模块向外部提供的结构体/枚举定义 ----------------------------------------*/
typedef struct
{
 uint8_t Bullets_in;       //进入弹舱的数量
 uint8_t Bullets_out;      //射出子弹的数量
 uint8_t Bullets_Check;    //弹舱剩余子弹数量
 uint8_t Motor_State;      //拨盘电机堵转标志位
 uint8_t Stall_time;       //堵转判断时间
 uint8_t Reverse_flag;     //电机反转标志位
 int8_t  Reverse_time;     //电机反转时间
 int32_t Angle_storage;    //暂存拨盘角度
 uint16_t wait_fall; 
}plate_t;
typedef struct
{
 uint16_t heat;
	uint16_t Max_speed;
	float bullet_speed;
}Referee_t;
typedef enum
{
 FALL = 1,
	RISE = 2
}PLATE_INIT;
typedef enum
{
 CHECKED,
 CHECKING
}Bullets_Check;
typedef enum 
{
 OPEN,
 STALL
}Motor_State;
typedef enum
{
 FORWARD,
 BACKWARD	
}Reverse_flag;
typedef struct
{
 uint8_t State;
 uint8_t Pin;
 uint8_t Margin;
 uint16_t Times;
 uint16_t Check_Times;
}Plate_Init_t;
/* 本模块向外部提供的变量声明 -----------------------------------------------*/
extern plate_t plate;
extern Referee_t Referee;
extern uint8_t Check_state;
extern Plate_Init_t Plate_Init;
/* 本模块向外部提供的自定义数据类型变量声明 ---------------------------------*/
/* 本模块向外部提供的接口函数原型声明 ---------------------------------------*/
#endif



