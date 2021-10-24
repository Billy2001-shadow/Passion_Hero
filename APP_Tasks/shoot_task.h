#ifndef  __SHOOT_FRICTION_TASK_H
#define  __SHOOT_FRICTION_TASK_H
/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "user_lib.h"
#include "cmsis_os.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/
typedef struct
{
 uint8_t Bullets_in;       //���뵯�յ�����
 uint8_t Bullets_out;      //����ӵ�������
 uint8_t Bullets_Check;    //����ʣ���ӵ�����
 uint8_t Motor_State;      //���̵����ת��־λ
 uint8_t Stall_time;       //��ת�ж�ʱ��
 uint8_t Reverse_flag;     //�����ת��־λ
 int8_t  Reverse_time;     //�����תʱ��
 int32_t Angle_storage;    //�ݴ沦�̽Ƕ�
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
/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/
extern plate_t plate;
extern Referee_t Referee;
extern uint8_t Check_state;
extern Plate_Init_t Plate_Init;
/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/
/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/
#endif



