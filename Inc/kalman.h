/**
  * @author  
  * 卡尔曼滤波器来自RoboMaster论坛  
  */
  
#ifndef _KALMAN_H
#define _KALMAN_H


typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
	float B;
    float Q;
    float R;
    float H;
}extKalman_t;
typedef enum
{
	KLM_YAW_6020MOTOR,                 //yaw编码器位置环
	KLM_YAW_6020MOTOR_GORY,            //yaw陀螺仪位置环
	KLM_PITCH_6623MOTOR,               //pitch位置环
	NUM_OF_KLM//所有电机进行PID运算时所需PID结构体的数量

}KLM_ID;//电机闭环控制时各电机速度环或位置环PID结构体对应存储的数组编号
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);
extern extKalman_t extKalman[NUM_OF_KLM];

#endif
