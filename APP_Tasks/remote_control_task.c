/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : remote_control_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2020.12.1
  最近修改   : 
  功能描述   : 遥控器数据进一步处理任务，包括判断是键鼠操作还是遥控器操作等。
							 （当USART1接收完一组数据时在空闲中断进行解码，并发送任务通知,然后
							 该任务方可运行，否则该任务处于阻塞状态，阻塞时间为4294967295ms即
							 约等于47天17小时。）
  函数列表   : 1) Remote_Data_Task()      【FreeRTOS函数：操作系统任务调度运行】  
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "remote_control_task.h"
#include "motor_use_can.h"
#include "offline_check.h"
#include "usart_printf.h"
#include "remote_control.h"
#include "vision.h"

/* 内部宏定义 ----------------------------------------------------------------*/
#define Limit_angle(x) x>1020?(1020):(x<0?0:x)
/* 内部自定义数据类型的变量 --------------------------------------------------*/
C_Board_t C_Board;
expect_t  expect;
/* 内部变量 ------------------------------------------------------------------*/
uint8_t shoot_time = 0;
/* 内部函数原型声明 ----------------------------------------------------------*/
void RC_Mode_gimbal(void);
void RC_Mode_gun(void);
void Mouse_Control(void);
void Mouse_control_gimbal(void);
void Rc_control_gimbal(void);
void Control_Pc(void);
/* 函数主体部分 --------------------------------------------------------------*/
/**
  * @brief				遥控器任务
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
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
		CAN_Send_RemoteDate(&hcan1, rc_ctrl.key.v, rc_ctrl.rc.ch0, rc_ctrl.rc.ch1, rc_ctrl.rc.s1, rc_ctrl.rc.s2);
		CAN_Send_MouseDate(&hcan1, rc_ctrl.mouse.x, rc_ctrl.mouse.y, motor_get[PITCH_6623MOTOR].pit_angle, rc_ctrl.mouse.press_l, rc_ctrl.mouse.press_r);
		if(NotifyValue == 1)
		{
			Refresh_Task_OffLine_Time(RemoteDataTask_TOE);
			if(rc_ctrl.rc.s1 == 1)                                          //遥控器云台模式
			{
			 RC_Mode_gimbal();
			}
			else if(rc_ctrl.rc.s1 == 3)                                     //遥控器射击模式
			{
			 RC_Mode_gun();
   }
   else if(rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s2 == 1)            //键鼠模式
			{
			 Mouse_Control();
			}
   else if(rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s2 == 3)            //键鼠模式
			{
			 Mouse_Control();
			}
			else if(rc_ctrl.rc.s1 == 2 && rc_ctrl.rc.s2 == 2)               //自瞄模式
			{
//			 C_Board.mode = PC_CONTROL;
			 Mouse_Control();
			}

			switch(C_Board.mode)
			{
				case MOUSE:     //键盘
				{
					Mouse_control_gimbal();
				}break;
				case RC_CONTROL://遥控器
				{
					Rc_control_gimbal();
				}break;
				case PC_CONTROL://视觉
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
  * @brief				云台模式选择（遥控器）
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void RC_Mode_gimbal(void)
{
  C_Board.mode = RC_CONTROL;
 
 if(C_Board.gun == SHOOT_OFF || C_Board.gun == SHOOT_READY)
	  C_Board.gun = STOP_SHOOT;
 if(rc_ctrl.rc.s2 == 1 || rc_ctrl.rc.s2 == 2)                      //陀螺仪
 {
  C_Board.gimbal = GROY;
 }
 else if(rc_ctrl.rc.s2 == 3)                                       //编码器
 {
  C_Board.gimbal = ENCODER;
 }
}
/**
  * @brief				发射状态选择（遥控器）
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void RC_Mode_gun(void)
{
  C_Board.mode = RC_CONTROL;

 if(C_Board.gun == STOP_SHOOT) 
	C_Board.gun = SHOOT_OFF;
 if(rc_ctrl.rc.s2 == 3 && C_Board.gun == SHOOT_OFF)                //准备发射
 {
  C_Board.gun = SHOOT_READY;
 }
 else if(rc_ctrl.rc.s2 == 1 && C_Board.gun == SHOOT_READY)         //发射
 {
  C_Board.gun = SHOOT_ON;
 }
}
/**
  * @brief				云台模式选择（键鼠）
  * @param[in]		
	* @param[out]
  * @retval				none
*/
void Mouse_Control(void)
{
/*模式*/
	if(Q_Press)		    C_Board.gimbal = ENCODER;                 //底盘分离
	if(C_Press)      C_Board.gimbal = GROY;                    //大陀螺
	if(F_Press)      C_Board.gimbal = GROY;                    //底盘跟随
	if(X_Press)      C_Board.gimbal = GROY;                    //底盘跟随45
	if(R_Press && expect.pitch_lock != 0)      C_Board.gimbal = LOCK;                      
  C_Board.mode = MOUSE;
/*摩擦轮*/
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
  * @brief				云台运动控制
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Mouse_control_gimbal(void)
{
  /*运动*/                                                     //方向待修改
	if(C_Board.gimbal == GROY)                             //陀螺仪模式
	{
	 expect.yaw_gory += (rc_ctrl.mouse.x/2.0f);
	 expect.pitch_encoder -= (rc_ctrl.mouse.y/2.0f);
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
	else if(C_Board.gimbal == ENCODER)                       //编码器模式
	{
	 expect.yaw_encoder += (rc_ctrl.mouse.x/2.0f);
	 expect.pitch_encoder -= (rc_ctrl.mouse.y/2.0f);
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
	else if(C_Board.gimbal == LOCK)                       //编码器模式
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
  /*运动*/                                                     //方向待修改
	if(C_Board.gimbal == GROY)                             //陀螺仪模式
	{
	 expect.yaw_gory += (rc_ctrl.rc.ch2/20.0f);
	 expect.pitch_encoder -= (rc_ctrl.rc.ch3/80.0f);
	 expect.pitch_encoder = Limit_angle(expect.pitch_encoder);
	}
	else if(C_Board.gimbal == ENCODER)                       //编码器模式
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
