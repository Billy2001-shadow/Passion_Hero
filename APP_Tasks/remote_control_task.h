#ifndef  __REMOTE_CONTORL_TASK_H
#define	 __REMOTE_CONTORL_TASK_H
/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/
typedef enum
{
 STOP_SHOOT,      //�رշ���
	SHOOT_READY,     //׼������
 SHOOT_ON,        //��������
	SHOOT_OFF,       //�������
	SHOOTING,        //���򲦵��������
	BACK             //���򲦵���������һ���ӵ�λ�ã�
}SHOOT_MODE;

typedef enum
{
 GROY,          //������
 ENCODER,        //������
 PC,
	LOCK,
}GIMBAL_MODE;
typedef enum
{
 MOUSE=1,         //����
 RC_CONTROL,      //ң����
 PC_CONTROL       //�Ӿ�
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
/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/

/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/
extern C_Board_t C_Board;
extern expect_t  expect;
/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/



#endif
