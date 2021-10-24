/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : shoot_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2020.12.1
  最近修改   :
  功能描述   : 射击任务，包括摩擦轮任务与拨盘任务
  函数列表   : 1) Friction_Drive_Task()		【FreeRTOS函数：操作系统任务调度运行】
							 2) Trigger_Drive_Task()		【FreeRTOS函数：操作系统任务调度运行】
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "shoot_task.h"
#include "remote_control.h"
#include "motor_use_can.h"
#include "offline_check.h"
#include "remote_control_task.h"
#include "pid.h"
#include "usart_printf.h"
#include "gpio.h"
#include "INS_task.h"
/* 内部宏定义 ----------------------------------------------------------------*/
#define ANGLE_CONVERT(X) X>8191?(X-8191):(X<0?X+8191:X)
/* 内部自定义数据类型的变量 --------------------------------------------------*/
int32_t plate_angle_error;
/* 内部变量 ------------------------------------------------------------------*/
Plate_Init_t Plate_Init;
plate_t plate;
Referee_t Referee;
float plate_current,trigger_current;
/* 内部函数原型声明 ----------------------------------------------------------*/
void Shoot_PID_Init(void);
void Stall_Deal(void);
void Trigger_PID_Init(void);
void heat_limit(uint8_t check_state);
void speed_control(float *shoot_speed,uint8_t max_speed);
void Bullets_Init(uint8_t *state, uint8_t pin,uint8_t pin_i7,uint8_t *margin,uint16_t *times);
void Bullets_Init_Check(int bullets,uint8_t *state, uint8_t pin);
uint8_t Prevent_HitRod(void);
/* 函数主体部分 --------------------------------------------------------------*/
//===================================================================================================================//
/******************************************************摩擦轮、拨弹*********************************************************/
//45 40
//===================================================================================================================//
/**
  * @brief				摩擦轮,拨弹任务
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Friction_Drive_Task(void const *argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();       
	Shoot_PID_Init();
 uint8_t  Check_state;
 uint16_t back_time;
 osDelay(1000);
	for(;;)
	{
		Refresh_Task_OffLine_Time(FrictionDriveTask_TOE);      //记录任务运行的时间点
		float expect_speed,shootr_current,shootl_current;
  int32_t trigger_angle_error = motor_get[TRIGGER_2006MOTOR].total_angle - expect.trigger_angle;
		heat_limit(Check_state);                               //热量控制
  if(C_Board.gun == STOP_SHOOT)                          //停止摩擦轮，保持拨弹电机
		{
		 expect.Shoot_speed = 0;
		}
		else                                                   //开启摩擦轮，改变拨弹电机        
		{
			speed_control(&expect.Shoot_speed, Referee.Max_speed);//射速控制
		 if(C_Board.gun == SHOOT_ON)                           //发射置位
		 {
			 expect.trigger_angle = motor_get[TRIGGER_2006MOTOR].total_angle;
				C_Board.gun = Prevent_HitRod();
		  if(C_Board.gun == SHOOTING)
				{
					expect.pitch_lock = expect.pitch_encoder;
					expect.yaw_lock = expect.yaw_encoder;
		   expect.trigger_angle -= 5400 * 45;
			  back_time = 20;
				}
			}
		 else if(C_Board.gun == SHOOTING && ABS(trigger_angle_error) < 10000)//到达目标位置
		 {
				 if(back_time > 0)
					{
						back_time--;
						expect.trigger_angle = motor_get[TRIGGER_2006MOTOR].total_angle;
     }
					else
					{
						expect.trigger_angle += 5400 * 47;
					 C_Board.gun = BACK;
					}
		 }
		 else if(C_Board.gun == BACK && ABS(trigger_angle_error) < 18000)   //发射结束
		 {
    C_Board.gun = SHOOT_OFF;
		 }
		}
	  expect_speed = PID_Calc(&PID[PID_TRIGGER_2006MOTOR_POS], motor_get[TRIGGER_2006MOTOR].total_angle, expect.trigger_angle);
		 trigger_current = PID_Calc(&PID[PID_TRIGGER_2006MOTOR_SPEED], motor_get[TRIGGER_2006MOTOR].speed_rpm, expect_speed);

	  shootr_current = PID_Calc(&PID[PID_SHOOTR_3508MOTOR_SPEED], motor_get[SHOOTR_3508MOTOR].speed_rpm, -expect.Shoot_speed);
		 shootl_current = PID_Calc(&PID[PID_SHOOTL_3508MOTOR_SPEED], motor_get[SHOOTL_3508MOTOR].speed_rpm, expect.Shoot_speed);
		 Shoot_Motor3508(&hcan2, 0, shootr_current, shootl_current, 0);
		 Shoot_Motor3508(&hcan1, trigger_current, plate_current, 0, 0);
   osDelayUntil(&xLastWakeTime,5);
	}
}
void Oscilloscope_Task(void const * argument)
{
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		Virtual_Oscilloscope(expect.trigger_angle/300.0f,motor_get[TRIGGER_2006MOTOR].total_angle/300.f,0,0,0,0,0,0);
		osDelayUntil(&xLastWakeTime, 15);
	}
}
//===================================================================================================================//
/*******************************************************拨盘**********************************************************/
//===================================================================================================================//
/**
  * @brief				拨盘任务
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Trigger_Drive_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
 Trigger_PID_Init();
 osDelay(1000);
	for(;;)
	{
		Refresh_Task_OffLine_Time(TriggerDriveTask_TOE);//记录任务运行的时间点
		float expect_speed;
		int8_t Bullets = plate.Bullets_in - plate.Bullets_out;
		Stall_Deal();                                    //堵转处理
	 Plate_Init.Pin = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9);
		uint8_t PIN_I7 = HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7);
  Bullets_Init(&Plate_Init.State, Plate_Init.Pin, PIN_I7, &Plate_Init.Margin, &Plate_Init.Times);
		Bullets_Init_Check(Bullets,&Plate_Init.State, Plate_Init.Pin);
		if(plate.Bullets_Check	== CHECKING)            //可以检查弹量
		{
    switch (Bullets)                                               
	  {
				case 0: {
					expect.plate_angle = expect.plate_angle + 1365 * 3 * 19;                /*  转3/6圈 */
				 Open_IT();
				}break;
				case 1: {
					expect.plate_angle = expect.plate_angle + 1365 * 2 * 19;                /* 转2/6圈 */
				 Open_IT();
				}break;
				case 2: {
					expect.plate_angle = expect.plate_angle + 1365 * 1 * 19;                /* 转1/6圈 */
				 Open_IT();
				}break;
				case 3: {
					expect.plate_angle = expect.plate_angle + 1365 * 0 * 19;               /*不动 */
					Close_IT();
				}break;
				default: 
				{
					Close_IT();
				}break;
	 	}
 		plate.Bullets_Check	= CHECKED;
		}
		expect_speed = PID_Calc(&PID[PID_DRIVERPLATE_3508MOTO_POS], motor_get[DRIVERPLATE_3508MOTO].total_angle, expect.plate_angle);
		if(plate.Reverse_flag == BACKWARD)  expect_speed = -800;
		plate_current = PID_Calc(&PID[PID_DRIVERPLATE_3508MOTO_SPEED], motor_get[DRIVERPLATE_3508MOTO].speed_rpm, expect_speed);
  Shoot_Motor3508(&hcan1, trigger_current, plate_current, 0, 0);
		osDelayUntil(&xLastWakeTime,5);
	}
}
void Shoot_PID_Init(void)   //待调试
{
	PID_Param_Init(&PID[PID_TRIGGER_2006MOTOR_POS], POSITION_PID, 10000, 10000,
									0.125f, 0.0f, 0.0f);
	PID_Param_Init(&PID[PID_TRIGGER_2006MOTOR_SPEED], POSITION_PID, 10000, 10000,
									8.0f, 0.0f, 0.0f);

	PID_Param_Init(&PID[PID_SHOOTR_3508MOTOR_SPEED], POSITION_PID, 16384, 16384,
									8.5f, 0.0f, 7.3f);
	PID_Param_Init(&PID[PID_SHOOTL_3508MOTOR_SPEED], POSITION_PID, 16384, 16384,
									8.5f, 0.0f, 7.3f);
}
void Trigger_PID_Init(void)
{
	PID_Param_Init(&PID[PID_DRIVERPLATE_3508MOTO_POS], POSITION_PID, 1500, 5000,
			           0.25f, 0.0f, 0.0f);
	PID_Param_Init(&PID[PID_DRIVERPLATE_3508MOTO_SPEED], POSITION_PID, 10000, 5000,//10000
			           6.0f, 0.00f, 3.0f);
}
void Stall_Deal(void)
{
/*堵转判断*/
	if(motor_get[DRIVERPLATE_3508MOTO].real_current > 8000 && plate.Reverse_flag == FORWARD)
	{
		plate.Stall_time++;
	}
	else plate.Stall_time = 0;
	if(plate.Stall_time > 16)
	{
	 plate.Motor_State = STALL;
	}
/*堵转处理*/
  if(plate.Motor_State == STALL)
	{
		plate.Angle_storage = expect.plate_angle;
	 plate.Motor_State = OPEN;
		plate.Reverse_flag = BACKWARD;
		plate.Reverse_time = 50;
	}
  if(plate.Reverse_flag == BACKWARD && plate.Reverse_time != 0)      //反转结束
	{
	 plate.Reverse_time--;
	}
	else if(plate.Reverse_flag == BACKWARD && plate.Reverse_time == 0)      //反转结束
	{
  plate.Reverse_flag = FORWARD;
  expect.plate_angle = plate.Angle_storage;
	}
  plate_angle_error = motor_get[DRIVERPLATE_3508MOTO].total_angle - expect.plate_angle;
	if(plate.Reverse_flag == FORWARD && ABS(plate_angle_error) < 3500)  /* 相对角度小于5.5度 */
	{
		plate.wait_fall ++;
		if(plate.wait_fall > 80)                                               
		{
			plate.wait_fall = 0;
			plate.Bullets_Check	= CHECKING;
		}
	}
}
/*热量控制*/
void heat_limit(uint8_t check_state)
{
	static uint8_t heat_flag;
 if(Referee.heat < 100 && C_Board.gun != STOP_SHOOT)
 {
		if(heat_flag == 0 && C_Board.gun == SHOOTING)
		{
   expect.trigger_angle += 5400 * 53;
   C_Board.gun = BACK;
		 heat_flag = 1;
		}
		else if((heat_flag == 1 && C_Board.gun == SHOOTING) || C_Board.gun == SHOOT_ON)//发射置位或者发射中
		{
		 C_Board.gun = SHOOT_OFF;
		}
	}else if(Referee.heat > 100)heat_flag = 0;
}
/*射速控制*/
void speed_control(float *shoot_speed,uint8_t max_speed)
{
 if(max_speed == 10)
	{
	 *shoot_speed = 3620;
	}
	else if(max_speed == 16)
 {
	 *shoot_speed = 5700;
	}
}
/*防打杆控制*/
uint8_t Prevent_HitRod(void)
{
	float absulot_angle_min, absulot_angle_max,angle_min, angle_max;
	angle_min = motor_get[YAW_6020MOTOR].angle  + motor_get[YAW_6020MOTOR].speed_rpm * 136.5f * 0.135f;
	absulot_angle_min = ANGLE_CONVERT(angle_min);
	angle_max = motor_get[YAW_6020MOTOR].angle + motor_get[YAW_6020MOTOR].speed_rpm * 136.5f * 0.255f;
	absulot_angle_max = ANGLE_CONVERT(angle_max);
 if((337<absulot_angle_min && absulot_angle_max<5226) || (5800<absulot_angle_min && absulot_angle_max<8018))
	{
		return SHOOTING;
	}
	else
	{
	 return SHOOT_ON;
	}
}
void Bullets_Init(uint8_t *state, uint8_t pin,uint8_t pin_i7,uint8_t *margin,uint16_t *times)
{
 if(*state == 0)   //初始化
	{
	 if(pin == 1)//子弹小于三颗 
		{
		 *margin = RISE;
   *state = 1;
	 	*times = 0;
		}
		else if(pin == 0)//子弹大于等于三颗
		{
		 *margin = FALL;
   *state = 1;
		 *times = 0;
		}
	}
	else if(*state != 0 && *state != 3)//初始化结束，还未放置好弹丸
	{
		*times = *times + 1;
	 if(*margin == RISE && pin == 0)
  {
			if(*state == 1)
			{
    *state = 2;
	 	 *times = 501;
			}
			else if(*state == 2&&*times > 600)
			{
			 plate.Bullets_in = 3;
		  plate.Bullets_out = 0;
    *state = 3;
   }
		}
	 else if(*margin == FALL && pin == 1)
  {
			if(*state == 1)
			{
    *state = 2;
	 	 *times = 501;
			}
			else if(*state == 2&&*times>600)
			{
			 plate.Bullets_in = 2;
		  plate.Bullets_out = 0;
    *state = 3;
   }
		}
		else if(*margin == RISE && pin == 1)
		{
   *state = 1;
		 if(*times > 500)
			{
    expect.plate_angle = expect.plate_angle + 1365 * 1 * 19;                /* 转1/6圈 */
				*times = 0;
			}else if(pin_i7 == 0)
			{
			 expect.plate_angle = motor_get[DRIVERPLATE_3508MOTO].total_angle;
			}
		}
	 else if(*margin == FALL && pin == 0)
  {
   *state = 1;
		 if(*times > 500)
			{
				expect.plate_angle = expect.plate_angle + 1365 * 0 * 19;                /*不动 */
 			*times = 0;
			}
		}
			plate.wait_fall = 0;  //防止上方红外开启检测
 }
}
void Bullets_Init_Check(int bullets,uint8_t *state, uint8_t pin)
{
		if(*state == 3)
	{
		if(bullets < 0 || bullets > 4)
		{
			Plate_Init.Check_Times++;
		}
		else if(bullets < 3 && pin == 0)
		{
			Plate_Init.Check_Times++;
		}
		else if(bullets > 2 && pin == 1)
		{
			Plate_Init.Check_Times++;
		}
		else
		{
			Plate_Init.Check_Times = 0;
		}
		if(Plate_Init.Check_Times > 100)
		{
			*state = 0;
			Plate_Init.Check_Times = 0;
		}
	}
}
