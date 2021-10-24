/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : gimbal_control_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2020.12.10
  最近修改   : 
  功能描述   : 云台任务,待完善
  函数列表   : 1) Gimbal_Control_Task()【FreeRTOS函数：操作系统任务调度运行】
*************************************************************************                           ******/
/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_control_task.h"
#include "offline_check.h"
#include "pid.h"
#include "motor_use_can.h"
#include "remote_control_task.h"
#include "INS_task.h"
#include "kalman.h"
#include "usart_printf.h"
#include "math.h"


/* 内部宏定义 ----------------------------------------------------------------*/
#define PC_Pitch  motor_get[PITCH_6623MOTOR].total_angle - expect.initial_pitch
#define PC_Yaw    motor_get[YAW_6020MOTOR].total_angle - expect.initial_yaw
#define GIMBAL_PERIOD 5
#define T             2.0f
/* 内部自定义数据类型的变量 --------------------------------------------------*/
/* 内部变量 ------------------------------------------------------------------*/
uint32_t time_flag=0;
float  expect_parameter;
float  set_angle,set_speed,get_speed;
float  get_pit_angle,get_pit_speed;
int16_t pitch_test_current;
float tly_speed;
float encode_speed;
float klm_pitch;
int32_t angle_get;
int32_t angle_set;
int32_t angle_err;
uint8_t test_mode =  0;
uint8_t pitch_arrival = 0;
int16_t pitch_init_angle=0;
/* 内部函数原型声明 ----------------------------------------------------------*/
void Gimble_PID_Init(void);//云台pid初始化
/* 函数主体部分 --------------------------------------------------------------*/
/*目标曲线*/
void sin_calc()    
{
	time_flag++;                
	expect_parameter = 5*sin((PI*2/T)*(time_flag*0.005));//(time_flag*0.005/T)*2PI
}
/**
  * @brief				云台任务
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Gimbal_Control_Task(void const * argument)
{
	Gimble_PID_Init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	osDelay(1000);
	for(;;)
	{
		Refresh_Task_OffLine_Time(GimbalContrlTask_TOE);
	
		if(C_Board.gimbal == ENCODER)
		{
			//这里仅作为第二次测试使用
			//该版本为PID调试版本 
			pitch_init_angle = 700;
			angle_get =  motor_get[PITCH_6623MOTOR].angle;
			angle_set = pitch_init_angle;          //给一个阶跃信号  角度500
			angle_err = angle_set - angle_get;
			
			klm_pitch = KalmanFilter(&extKalman[KLM_PITCH_6623MOTOR],angle_err);
			PID_Calc(&PID[PID_PITCH_6623MOTOR_POS],0,klm_pitch);
			set_speed = PID[PID_PITCH_6623MOTOR_POS].pos_out;
			get_speed = -bmi088_real_data.gyro[0];  //这个反馈速度是用这个吗  之前乘上一个系数是为了把单位转换为rpm
			PID_Calc(&PID[PID_PITCH_6623MOTOR_SPEED], get_speed,set_speed);
			pitch_test_current = PID[PID_PITCH_6623MOTOR_SPEED].pos_out;
			Gimbal_Motor6020(&hcan2,0,pitch_test_current); //pitch_test_current
			osDelayUntil(&xLastWakeTime,5);
		}
		else
		{
			Gimbal_Motor6020(&hcan2,0,0);
		}
		//需要看pitch轴的角度
	}
}
/**
  * @brief				PID初始化函数（6020电机）
  * @param[in]
	 * @param[out]
  * @retvalnone
*/

void Gimble_PID_Init(void)
{
	KalmanCreate(&extKalman[KLM_PITCH_6623MOTOR], 1,2.0f);
	//位置环的P对力影响较大
	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_POS], POSITION_PID, 1000, 30.0f,
								 0.8f, 0.001f, 7.0f);
	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_SPEED], POSITION_PID, 5000, 5000,
								18.0f, 0.0f, 0.0f);
}	


//void Gimble_PID_Init(void)   //待调试
//{
//	//调试下面的参数    
//	//PID_Param_Init(&PID[PID_PITCH_6623MOTOR_POS], POSITION_PID, 600, 3.0f,
////	//							 0.5f, 0.0033f, 5.8f);
////	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_SPEED], POSITION_PID, 5000, 5000,
////									35.0f, 0.0f, 0.0f);
//	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_POS], POSITION_PID, 1000, 3.0f,
//								 0.5f, 0.00f, 0.0f);
//	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_SPEED], POSITION_PID, 5000, 5000,
//									15.0f, 0.0f, 0.0f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_POS], POSITION_PID, 30000, 6,
//									0.13f, 0.065f, 1.2f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_SPEED], POSITION_PID, 30000, 30000,
//									280.0f, 0.0f, 0.0f);
//	
////	PID_Param_Init(&PID[PID_YAW_6020MOTOR_SPEED], POSITION_PID, 30000, 30000,
////									280.0f, 0.0f, 0.0f);
//	
//	
//	KalmanCreate(&extKalman[KLM_PITCH_6623MOTOR], 1,2.0f);
//	


//	KalmanCreate(&extKalman[KLM_YAW_6020MOTOR_GORY], 1,0.0f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_GORY_POS], POSITION_PID, 30000, 10,
//								 0.8f, 0.15f, 1.00f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_GORY_SPEED], POSITION_PID, 30000, 30000,
//									120.0f, 0.0f, 0.0f);
//}



//	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_POS], POSITION_PID, 5000, 80,
//								 0.3f, 0.018f, 2.70f);
//	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_SPEED], POSITION_PID, 5000, 5000,
//									100.0f, 0.0f, 0.0f);

//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_GORY_POS], POSITION_PID, 30000, 30,
//								 1.20f, 0.02f, 0.80f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_GORY_SPEED], POSITION_PID, 30000, 30000,
//									160.0f, 0.0f, 0.0f);

//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_POS], POSITION_PID, 30000, 30,
//									0.25f, 0.013f, 0.2f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_SPEED], POSITION_PID, 30000, 30000,
//									320.0f, 0.0f, 0.0f);

//	KalmanCreate(&extKalman[KLM_PITCH_6623MOTOR], 1,1.0f);
//	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_POS], POSITION_PID, 5000, 20,
//								 0.16f, 0.0065f, 1.9f);
//	PID_Param_Init(&PID[PID_PITCH_6623MOTOR_SPEED], POSITION_PID, 5000, 5000,
//									100.0f, 0.0f, 0.0f);

// KalmanCreate(&extKalman[KLM_YAW_6020MOTOR_GORY], 1,1.0f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_GORY_POS], POSITION_PID, 30000, 30,
//								 2.0f, 0.0f, 0.00f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_GORY_SPEED], POSITION_PID, 30000, 30000,
//									120.0f, 0.0f, 0.0f);

//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_POS], POSITION_PID, 30000, 30,
//									0.15f, 0.01f, 0.3f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_SPEED], POSITION_PID, 30000, 30000,
//									250.0f, 0.0f, 0.0f);

//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_POS], POSITION_PID, 30000, 6,
//									0.13f, 0.05f, 1.2f);
//	PID_Param_Init(&PID[PID_YAW_6020MOTOR_SPEED], POSITION_PID, 30000, 30000,
//									250.0f, 0.0f, 0.0f);
