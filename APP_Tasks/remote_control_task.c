/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : remote_control_task.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.12.1
  ����޸�   : 
  ��������   : ң�������ݽ�һ���������񣬰����ж��Ǽ����������ң���������ȡ�
							 ����USART1������һ������ʱ�ڿ����жϽ��н��룬����������֪ͨ,Ȼ��
							 �����񷽿����У����������������״̬������ʱ��Ϊ4294967295ms��
							 Լ����47��17Сʱ����
  �����б�   : 1) Remote_Data_Task()      ��FreeRTOS����������ϵͳ����������С�  
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "remote_control_task.h"
#include "motor_use_can.h"
#include "offline_check.h"
#include "usart_printf.h"
#include "remote_control.h"
#include "vision.h"

/* �ڲ��궨�� ----------------------------------------------------------------*/
#define Limit_angle(x) x>1020?(1020):(x<0?0:x)
/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/
C_Board_t C_Board;
expect_t  expect;
/* �ڲ����� ------------------------------------------------------------------*/
uint8_t shoot_time = 0;
/* �ڲ�����ԭ������ ----------------------------------------------------------*/
void RC_Mode_gimbal(void);
void RC_Mode_gun(void);
void Mouse_Control(void);
void Mouse_control_gimbal(void);
void Rc_control_gimbal(void);
void Control_Pc(void);
/* �������岿�� --------------------------------------------------------------*/
/**
  * @brief				ң��������
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Remote_Data_Task(void const * argument)
{
	  uint32_t NotifyValue;

		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //δ������֪ͨ��������״̬ȥ�ȴ�����֪ͨ
		CAN_Send_RemoteDate(&hcan1, rc_ctrl.key.v, rc_ctrl.rc.ch0, rc_ctrl.rc.ch1, rc_ctrl.rc.s1, rc_ctrl.rc.s2);
		CAN_Send_MouseDate(&hcan1, rc_ctrl.mouse.x, rc_ctrl.mouse.y, motor_get[PITCH_6623MOTOR].pit_angle, rc_ctrl.mouse.press_l, rc_ctrl.mouse.press_r);
		if(NotifyValue == 1)
		{
			Refresh_Task_OffLine_Time(RemoteDataTask_TOE);
			if(rc_ctrl.rc.s1 == 1)                                          //ң������̨ģʽ
			{
			 RC_Mode_gimbal();
			}
			else if(rc_ctrl.rc.s1 == 3)                                     //ң�������ģʽ
			{
			 RC_Mode_gun();
   }
   else if(rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s2 == 1)            //����ģʽ
			{
			 Mouse_Control();
			}
   else if(rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s2 == 3)            //����ģʽ
			{
			 Mouse_Control();
			}
			else if(rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s2 == 2)               //����ģʽ
			{
//			 C_Board.mode = PC_CONTROL;
			 Mouse_Control();
			}

			switch(C_Board.mode)
			{
				case MOUSE:     //����
				{
					Mouse_control_gimbal();
				}break;
				case RC_CONTROL://ң����
				{
					Rc_control_gimbal();
				}break;
				case PC_CONTROL://�Ӿ�
				{
					Control_Pc();
				}break;
	   default : break;
			}
		}
		osDelayUntil(&xLastWakeTime, 7);
	}
}
/**
  * @brief				��̨ģʽѡ��ң������
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void RC_Mode_gimbal(void)
{
  C_Board.mode = RC_CONTROL;
 
 if(C_Board.gun == SHOOT_OFF || C_Board.gun == SHOOT_READY)
	  C_Board.gun = STOP_SHOOT;
 if(rc_ctrl.rc.s2 == 1 || rc_ctrl.rc.s2 == 2)                      //������
 {
  C_Board.gimbal = GROY;
 }
 else if(rc_ctrl.rc.s2 == 3)                                       //������
 {
  C_Board.gimbal = ENCODER;
 }
}
/**
  * @brief				����״̬ѡ��ң������
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void RC_Mode_gun(void)
{
  C_Board.mode = RC_CONTROL;

 if(C_Board.gun == STOP_SHOOT) 
	C_Board.gun = SHOOT_OFF;
 if(rc_ctrl.rc.s2 == 3 && C_Board.gun == SHOOT_OFF)                //׼������
 {
  C_Board.gun = SHOOT_READY;
 }
 else if(rc_ctrl.rc.s2 == 1 && C_Board.gun == SHOOT_READY)         //����
 {
  C_Board.gun = SHOOT_ON;
 }
}
/**
  * @brief				��̨ģʽѡ�񣨼���
  * @param[in]		
	* @param[out]
  * @retval				none
*/
void Mouse_Control(void)
{
/*ģʽ*/
	if(Q_Press)		    C_Board.gimbal = ENCODER;                 //���̷���
	if(C_Press)      C_Board.gimbal = GROY;                    //������
	if(F_Press)      C_Board.gimbal = GROY;                    //���̸���
	if(X_Press)      C_Board.gimbal = GROY;                    //���̸���45
	if(R_Press && expect.pitch_lock != 0)      C_Board.gimbal = LOCK;                      
  C_Board.mode = MOUSE;
/*Ħ����*/
   if(CTRL_Press && Left_Press)
   {
	   C_Board.gun = STOP_SHOOT;
   }
   else if(Left_Press && C_Board.gun == STOP_SHOOT)
   {
    C_Board.gun = SHOOT_OFF;
		  shoot_time = 10;
	  }
   else if(Left_Press && C_Board.gun == SHOOT_OFF)
   {
		 if(shoot_time > 0) shoot_time--;
		 else	 C_Board.gun = SHOOT_ON;
   }
}
/**
  * @brief				��̨�˶�����
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Mouse_control_gimbal(void)
{
  /*�˶�*/                                                     //������޸�
	if(C_Board.gimbal == GROY)                             //������ģʽ
	{
	 expect.yaw_gory += (rc_ctrl.mouse.x/2.0f);
	 expect.pitch_encoder -= (rc_ctrl.mouse.y/2.0f);
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
	else if(C_Board.gimbal == ENCODER)                       //������ģʽ
	{
	 expect.yaw_encoder += (rc_ctrl.mouse.x/2.0f);
	 expect.pitch_encoder -= (rc_ctrl.mouse.y/2.0f);
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
	else if(C_Board.gimbal == LOCK)                       //������ģʽ
	{
	 expect.yaw_encoder = expect.yaw_lock;
	 expect.pitch_encoder = expect.pitch_lock;
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
 	expect.yaw_pc = 0;
		expect.pitch_pc = 0;
}
void Rc_control_gimbal(void)
{
  /*�˶�*/                                                     //������޸�
	if(C_Board.gimbal == GROY)                             //������ģʽ
	{
	 expect.yaw_gory += (rc_ctrl.rc.ch2/20.0f);
	 expect.pitch_encoder -= (rc_ctrl.rc.ch3/80.0f);
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
	else if(C_Board.gimbal == ENCODER)                       //������ģʽ
	{
		
	 expect.yaw_encoder += (rc_ctrl.rc.ch2/20.0f);
	 expect.pitch_encoder -= (rc_ctrl.rc.ch3/80.0f);
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
		expect.yaw_pc = 0;
		expect.pitch_pc = 0;
}
void Control_Pc(void)
{
 expect.yaw_pc += minipc_rx.angle_yaw ;
 expect.pitch_pc += minipc_rx.angle_pit ;
 C_Board.gimbal = PC; 
}
