/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : offline_check_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2020.12.10
  最近修改   :
  功能描述   : 断线检测任务，用于调用任务断线检测函数与外设断线检测函数来判断
							 任务或外设是否掉线，若掉线并进行相关处理。
							 注：需要进一步完善
  函数列表   : 1) OffLine_Check_Task()			【FreeRTOS函数，操作系统任务调度运行】
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "offline_check_task.h"
#include "offline_check.h"
/* 内部宏定义 ----------------------------------------------------------------*/

/* 内部自定义数据类型的变量 --------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/* 内部函数原型声明 ----------------------------------------------------------*/

/* 函数主体部分 --------------------------------------------------------------*/
/*********************************************************  
off_line.task_offline_flag为16位数据0000 0000 0000 0000
0000 0000 0000 0001即0x01：遥控器任务掉线
0000 0000 0000 0010即0x02：视觉任务掉线
0000 0000 0000 0100即0x04：裁判系统掉线
0000 0000 0000 1000即0x08：摩擦轮任务掉线
0000 0000 0001 0000即0x10：拨盘任务掉线
0000 0000 0010 0000即0x20：云台任务掉线
0000 0000 0100 0000即0x40：底盘任务掉线
0000 0000 1000 0000即0x80：断线检测任务掉线
*********************************************************/
/**
  * @brief				断线检测任务
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

		Task_OffLine_Check();//任务检测
		Device_OffLine_Check();//断线检测
    
		//任务掉线对应操作
//		if((off_line.task_offline_flag & 0x01))//遥控器任务掉线
//		{
//			printf("Remote_Data_Task is offline\r\n");
//		}
//				
//		if((off_line.task_offline_flag & 0x02))//视觉任务掉线
//		{
//			printf("Vision_Data_Task is offline\r\n");
//		} 
//		
//		if((off_line.task_offline_flag & 0x04))//裁判系统掉线
//		{
//			printf("Referee_Data_Task is offline\r\n");
//		}	
//		
//		if((off_line.task_offline_flag & 0x08))//摩擦轮任务掉线
//		{
//			printf("Friction_Drive_Task is offline\r\n");
//		}
//		
//		if((off_line.task_offline_flag & 0x10))//拨盘任务掉线
//		{
//			printf("Trigger_Drive_Task is offline\r\n");
//		}	

//		if((off_line.task_offline_flag & 0x20))//云台任务掉线
//		{
//			printf("Gimbal_Control_Task is offline\r\n");
//		}	
//		
//		if((off_line.task_offline_flag & 0x40))//底盘任务掉线
//		{
//			printf("Chassis_Control_Task is offline\r\n");
//		}	
//		
//		if((off_line.task_offline_flag & 0x80))//断线检测任务掉线
//		{
//			printf("OffLine_Check_Task is offline\r\n");
//		}
//		
		
		//外设掉线对应操作
		if((off_line.device_offline_flag & 0x01))//Yaw轴电机数据接收，
		{
			  OffLine_group[0] = 0;
			  OffLine_number++;
		}else OffLine_group[0] = 7;

		if((off_line.device_offline_flag & 0x02))//Pitch轴电机数据接收
		{
			  OffLine_group[1] = 1;
			  OffLine_number++;
		}else OffLine_group[1] = 7;
		if((off_line.device_offline_flag & 0x04))//拨弹电机
		{
			  OffLine_group[2] = 2;
			  OffLine_number++;
		}else OffLine_group[2] = 7;

		if((off_line.device_offline_flag & 0x08))//拨盘电机
		{
			  OffLine_group[3] = 3;
			  OffLine_number++;
		}else OffLine_group[3] = 7;
		
		if((off_line.device_offline_flag & 0x10))//遥控器数据接收
		{
			  OffLine_group[4] = 4;			
			  OffLine_number++;
		}else OffLine_group[4] = 7;

		if((off_line.device_offline_flag & 0x20))//视觉数据接收
		{
			  OffLine_group[5] = 5;
			  OffLine_number++;
		}else OffLine_group[5] = 7;
		
		if((off_line.device_offline_flag & 0x40) || (off_line.device_offline_flag & 0x80))//摩擦轮电机
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
/*若有掉线鸣笛*/
 		 if(OffLine_number != 0) 
		 {
      HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
		 }
		osDelayUntil(&xLastWakeTime,10);
	}
}
