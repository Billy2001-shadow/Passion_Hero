/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : offline_check_task.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.12.10
  ����޸�   :
  ��������   : ���߼���������ڵ���������߼�⺯����������߼�⺯�����ж�
							 ����������Ƿ���ߣ������߲�������ش���
							 ע����Ҫ��һ������
  �����б�   : 1) OffLine_Check_Task()			��FreeRTOS����������ϵͳ����������С�
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "offline_check_task.h"
#include "offline_check.h"
/* �ڲ��궨�� ----------------------------------------------------------------*/

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/* �ڲ�����ԭ������ ----------------------------------------------------------*/

/* �������岿�� --------------------------------------------------------------*/
/*********************************************************  
off_line.task_offline_flagΪ16λ����0000 0000 0000 0000
0000 0000 0000 0001��0x01��ң�����������
0000 0000 0000 0010��0x02���Ӿ��������
0000 0000 0000 0100��0x04������ϵͳ����
0000 0000 0000 1000��0x08��Ħ�����������
0000 0000 0001 0000��0x10�������������
0000 0000 0010 0000��0x20����̨�������
0000 0000 0100 0000��0x40�������������
0000 0000 1000 0000��0x80�����߼���������
*********************************************************/
/**
  * @brief				���߼������
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void OffLine_Check_Task(void const *argument)
{
	osDelay(1000);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  uint8_t LED_count, OffLine_count;
	for(;;)
	{
		Refresh_Task_OffLine_Time(OutLineCheckTask_TOE);
    uint8_t OffLine_number = 0, OffLine_group[8] = {7,7,7,7,7,7,7,7};

		Task_OffLine_Check();//������
		Device_OffLine_Check();//���߼��
    
		//������߶�Ӧ����
//		if((off_line.task_offline_flag & 0x01))//ң�����������
//		{
//			printf("Remote_Data_Task is offline\r\n");
//		}
//				
//		if((off_line.task_offline_flag & 0x02))//�Ӿ��������
//		{
//			printf("Vision_Data_Task is offline\r\n");
//		} 
//		
//		if((off_line.task_offline_flag & 0x04))//����ϵͳ����
//		{
//			printf("Referee_Data_Task is offline\r\n");
//		}	
//		
//		if((off_line.task_offline_flag & 0x08))//Ħ�����������
//		{
//			printf("Friction_Drive_Task is offline\r\n");
//		}
//		
//		if((off_line.task_offline_flag & 0x10))//�����������
//		{
//			printf("Trigger_Drive_Task is offline\r\n");
//		}	

//		if((off_line.task_offline_flag & 0x20))//��̨�������
//		{
//			printf("Gimbal_Control_Task is offline\r\n");
//		}	
//		
//		if((off_line.task_offline_flag & 0x40))//�����������
//		{
//			printf("Chassis_Control_Task is offline\r\n");
//		}	
//		
//		if((off_line.task_offline_flag & 0x80))//���߼���������
//		{
//			printf("OffLine_Check_Task is offline\r\n");
//		}
//		
		
		//������߶�Ӧ����
		if((off_line.device_offline_flag & 0x01))//Yaw�������ݽ��գ�
		{
			  OffLine_group[0] = 0;
			  OffLine_number++;
		}else OffLine_group[0] = 7;

		if((off_line.device_offline_flag & 0x02))//Pitch�������ݽ���
		{
			  OffLine_group[1] = 1;
			  OffLine_number++;
		}else OffLine_group[1] = 7;
		if((off_line.device_offline_flag & 0x04))//�������
		{
			  OffLine_group[2] = 2;
			  OffLine_number++;
		}else OffLine_group[2] = 7;

		if((off_line.device_offline_flag & 0x08))//���̵��
		{
			  OffLine_group[3] = 3;
			  OffLine_number++;
		}else OffLine_group[3] = 7;
		
		if((off_line.device_offline_flag & 0x10))//ң�������ݽ���
		{
			  OffLine_group[4] = 4;			
			  OffLine_number++;
		}else OffLine_group[4] = 7;

		if((off_line.device_offline_flag & 0x20))//�Ӿ����ݽ���
		{
			  OffLine_group[5] = 5;
			  OffLine_number++;
		}else OffLine_group[5] = 7;
		
		if((off_line.device_offline_flag & 0x40) || (off_line.device_offline_flag & 0x80))//Ħ���ֵ��
		{
			  OffLine_group[6] = 6;
			  OffLine_number++;
		}else OffLine_group[6] = 7;
   LED_count = LED_count + 4;
		 if(LED_count == 160)
		{
			OffLine_count = OffLine_count+32;
		}
		 if(OffLine_group[OffLine_count/32] == 0)
		{
		 LED1_ON();
		 LED2_ON();
		 LED3_ON();
		}
   else if(OffLine_group[OffLine_count/32] == 1)
		{
		 LED1_OFF();
		 LED2_ON();
		 LED3_ON();
		}
		else if(OffLine_group[OffLine_count/32] == 2)
		{
		 LED1_ON();
		 LED2_OFF();
		 LED3_ON();
		}else if(OffLine_group[OffLine_count/32] == 3)
		{
		 LED1_OFF();
		 LED2_OFF();
		 LED3_ON();
		}else if(OffLine_group[OffLine_count/32] == 4)
		{
		 LED1_ON();
		 LED2_ON();
		 LED3_OFF();
		}else if(OffLine_group[OffLine_count/32] == 5)
		{
		 LED1_OFF();
		 LED2_ON();
		 LED3_OFF();
		}else if(OffLine_group[OffLine_count/32] == 6)
		{
		 LED1_ON();
		 LED2_OFF();
		 LED3_OFF();
		}else if(OffLine_group[OffLine_count/32] == 7)
		{
    if(OffLine_number == 0)
			{
 			 LED1_OFF();
		   LED2_OFF();
		   LED3_OFF();
   }
  	LED_count = 156;
		}
/*���е�������*/
 		 if(OffLine_number != 0) 
		 {
      HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
		 }
		osDelayUntil(&xLastWakeTime,10);
	}
}
