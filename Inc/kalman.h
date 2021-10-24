/**
  * @author  
  * �������˲�������RoboMaster��̳  
  */
  
#ifndef _KALMAN_H
#define _KALMAN_H


typedef struct {
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
	float B;
    float Q;
    float R;
    float H;
}extKalman_t;
typedef enum
{
	KLM_YAW_6020MOTOR,                 //yaw������λ�û�
	KLM_YAW_6020MOTOR_GORY,            //yaw������λ�û�
	KLM_PITCH_6623MOTOR,               //pitchλ�û�
	NUM_OF_KLM//���е������PID����ʱ����PID�ṹ�������

}KLM_ID;//����ջ�����ʱ������ٶȻ���λ�û�PID�ṹ���Ӧ�洢��������
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);
extern extKalman_t extKalman[NUM_OF_KLM];

#endif
