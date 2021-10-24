#ifndef  __PID_H
#define  __PID_H

/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <math.h>
/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/

enum PID_MODE
{
		POSITION_PID =0,
		DELTA_PID = 1,
};

enum Time
{
    LLAST	= 0,//����һ��
    LAST 	= 1,//��һ��
    NOW 	= 2,//����һ��
};

typedef enum
{
	/*��̨*/
 PID_YAW_6020MOTOR_SPEED,               //yaw�������ٶȻ�
	PID_YAW_6020MOTOR_POS,                 //yaw������λ�û�
	PID_YAW_6020MOTOR_GORY_SPEED,          //yaw�������ٶȻ�
	PID_YAW_6020MOTOR_GORY_POS,            //yaw������λ�û�
	PID_PITCH_6623MOTOR_SPEED,             //pitch�ٶȻ�
	PID_PITCH_6623MOTOR_POS,               //pitchλ�û�
	/*����*/
	PID_DRIVERPLATE_3508MOTO_SPEED,        //�����ٶȻ�
	PID_DRIVERPLATE_3508MOTO_POS,          //����λ�û�
	PID_TRIGGER_2006MOTOR_SPEED,           //�����ٶȻ�
	PID_TRIGGER_2006MOTOR_POS,             //����λ�û�
	PID_SHOOTR_3508MOTOR_SPEED,            //��Ħ�����ٶȻ�
  PID_SHOOTL_3508MOTOR_SPEED,            //��Ħ�����ٶȻ�
	/*������*/
	IMU_TEMP_PID,                          //�������¿رջ�

	NUM_OF_PID//����PID����ʱ����PID�ṹ�������
	
}PID_ID;//����ջ�����ʱ������ٶȻ���λ�û�PID�ṹ���Ӧ�洢��������


typedef struct __pid_t
{
	  uint8_t pid_mode;       	//PIDģʽ

		//PID������
    float Kp;			
    float Ki;
    float Kd;

    float set;								//Ŀ��ֵ
    float get;								//����ֵ
    float err[3];							//���	,����NOW�� LAST��LLAST

    float pout;								//p���
    float iout;								//i���
    float dout;								//d���
    float last_dout;					//��һ��d���
	
		float max_pout;						//�����޷�
		float max_iout;						//�����޷�
		float max_dout;						//΢���޷�
		float max_out;						//����޷�		
		float max_err;						//������
    float deadband;						//���� 

    float pos_out;						//����λ��ʽ���
    float delta_out;					//��������ʽ���

} pid_t;
/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/

/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/
extern pid_t   PID[NUM_OF_PID];

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/
void PID_Param_Init(pid_t *pid, uint8_t mode, float max_out, float max_iout, float Kp, float Ki, float Kd);
float PID_Calc(pid_t* pid, float get, float set);
void PID_Clear(pid_t *pid);



#endif


