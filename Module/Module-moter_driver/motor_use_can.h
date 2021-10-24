#ifndef __MOTOR_USE_CAN_H
#define __MOTOR_USE_CAN_H
/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "can.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/


/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/
typedef enum
{
	YAW_6020MOTOR,
	PITCH_6623MOTOR,
 SHOOTR_3508MOTOR,
	SHOOTL_3508MOTOR,
	TRIGGER_2006MOTOR,
 DRIVERPLATE_3508MOTO,
	NUM_OF_MOTOR      //���������
}motor_ID;//������ݽ��սṹ���Ӧ�洢��������

typedef enum
{
  /* can1*/
 	CAN_YAW_6020MOTOR_ID        = 0x205,
  CAN_TRIGGER_2006MOTOR_ID    = 0x201,
  CAN_DRIVERPLATE_3508MOTO_ID = 0x202,
 	CAN_CHASSIS_ID              = 0x123,
	 CAN_SHOOT_ID                = 0x130,
  /* can2*/
 	CAN_PITCH_6623MOTOR_ID      = 0x206,
  CAN_SHOOTR_3508MOTOR_ID     = 0x202,
  CAN_SHOOTL_3508MOTOR_ID     = 0x203,
	
}CAN_Message_ID;//CANͨ��ʱ��������ӦID��

typedef struct{
	int16_t	 			speed_rpm;			//ת��ת��
	int16_t  			real_current; 	//ʵ��ת�ص���
	int16_t  			given_current;	//����ת�ص���
	uint8_t  			temperature;		//����¶�
	uint16_t 			angle;					//ת�ӻ�е�Ƕ�abs angle range:[0,8191]
	uint16_t 			last_angle;			//��һ��ת�ӻ�е�Ƕ�
	uint16_t			offset_angle;		//����ϵ�ת�ӳ�ʼ��е�Ƕ�
	int16_t       		pit_angle;             
	int32_t				round_cnt;			//���ת��ת��Ȧ��
	int32_t				total_angle;		//�����ת��Ȧ��
	uint8_t				msg_cnt;				//CAN�������ݴ��������ڻ�ȡ���ת�ӳ�ʼ�Ƕȣ�
}motor_measure_t;
/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/

/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/
extern motor_measure_t  motor_get[NUM_OF_MOTOR];

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/
HAL_StatusTypeDef Gimbal_Motor6020(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
HAL_StatusTypeDef Gimbal_Motor6020_Disable(CAN_HandleTypeDef * hcan);
HAL_StatusTypeDef Gimbal_Motor6623_Calibration(CAN_HandleTypeDef * hcan);
HAL_StatusTypeDef Shoot_Motor3508( CAN_HandleTypeDef * hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
HAL_StatusTypeDef Shoot_Motor3508_Disable( CAN_HandleTypeDef * hcan);
HAL_StatusTypeDef Trigger_Motor2006(CAN_HandleTypeDef * hcan,int16_t value);
HAL_StatusTypeDef CAN_Send_RemoteDate( CAN_HandleTypeDef * hcan,uint16_t key_v, int16_t rc_ch0, int16_t rc_ch1, uint8_t rc_s1, uint8_t rc_s2);
HAL_StatusTypeDef CAN_Send_RefereeData( CAN_HandleTypeDef * hcan, uint16_t data0, uint16_t data1 , uint16_t data2 ,uint16_t data3);
HAL_StatusTypeDef CAN_Send_MouseDate( CAN_HandleTypeDef * hcan,
									                int16_t x, int16_t y, int16_t z, uint8_t press_l, uint8_t press_r);
#endif
