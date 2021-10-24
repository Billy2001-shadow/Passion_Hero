绝对上角度 1070  中间320(下面好像是-400多)

```
void sin_calc()
{
   expect_parameter = (time_flag*0.005));
 800*sin((PI*2/T)*//T=10s:0.001;5s:0.002;2.5s:0.004//T=2s:0.005;4s:0.0025;8s:0.00125
}
```

没有接到任务通知导致堵住了，遥控器接不到值，没进中断(重新开一下就好了)

发给pitch轴和yaw轴电机的值为-30000~30000(16位的数据2B)
		一帧有8字节，即一条指令最多可以控制4个电机

上pitch 1850
		下 pitch 3250

右极限 2876 570   1677

6020无电调

C610---2006

debug看不到局部变量
		jscope只能看局部变量？

没有使用的变量debug看不到
		

1889  3250    1800

2874  1647(total_angle = 0)  576  

+(2874-1647)  1227
		-(1647-576)   

1473   553会转过去一圈

JLink找不到是什么鬼---要直接连接电脑(不能用转接口)

#### Jlink

SEGGER公司为支持仿真ARM内核芯片推出的JTAG**仿真器** 

目标板电压范围1.2V~3.3V,兼容5V(A板工作电压  C板工作电压)

使用USB电源(但不对目标板供电)

调速度环的时候只给位置环的P(经验最小值)





### 陀螺仪任务学习

```
硬件资源关系：
BMI088：C板上使用
BMI088是一款高性能6轴惯性传感器，由16位数字三轴±24g加速度计和16位数字三轴±2000°/ s陀螺仪组成。 BMI088允许高精度测量方向和沿三个正交轴的运动检测。
IST8310 

MPU6500：A板上使用
六轴运动处理组件，内带3轴陀螺仪和3轴加速度传感器，并且含有一个IIC接口，可用于连接外部磁力计传感器。

A板上也有板载陀螺仪和磁力计(效果不怎么好)，板载的陀螺仪是6500，

MEMS  微机电陀螺仪  普遍应用

六轴加3轴磁力计的作用：不同地方的磁场强度是不一样的，通过加磁力计来消除磁场的干扰



目前英雄车上使用的资源为：

```

任务里面延时的作用？

```
名词解释：
1.AHRS：航姿态参考系(包括多个轴向传感器，能够为飞行器提供航向，横滚和侧翻信息，这类系统用来为飞行器提供准确可靠的姿态与航行信息。姿态参考系统包括基于MEMS的三轴陀螺仪，加速度计和磁强计)
航姿态参考系与IMU的区别在于，航姿态参考系包含了嵌入式的姿态数据结算单元与航向信息，惯性测量单元(IMU)仅仅提供传感器数据，并不具有提供准确可靠的姿态数据的功能

IMU也就是惯性测量单元，所有的运动都可以分解为一个直线运动和一个旋转运动，故这个惯性测量单元就是测量这两种运动，直线运动通过加速度计可以测量，旋转运动则通过陀螺。
```

用云台反馈的速度会发现震荡很严重的问题，调4号步兵的YAW轴PID时，给定一个角度，最后不能稳定，会在这个角度附近不停的摇摆

由于云台控制对于灵敏度和精度的要求比较高，而云台电机编码器反馈的数据是离散的，并且速度数据波动较大，用它来做控制并不能够达到很好的效果，所以目前的主流方案依然是通过IMU的数据来做云台的闭环控制。

准备电机闭环方案，检测到IMU离线后可以自动/手动切换成电机编码器反馈

绕X轴旋转---roll轴角速度
		绕Y轴旋转----pitch轴角速度
		绕Z轴旋转----yaw轴角速度

![image-20210706181647466](C:/Users/chenw/AppData/Roaming/Typora/typora-user-images/image-20210706181647466.png)

![image-20210706180011351](C:/Users/chenw/AppData/Roaming/Typora/typora-user-images/image-20210706180011351.png)

```
英雄云台的pitch轴反馈速度用的是 -bmi088_real_data.gyro[0]*6.0f
		yaw轴反馈速度用的是-bmi088_real_data.gyro[2]*57.3f 


-bmi088_real_data.gyro[2]*57.3f
total_angle
init_angle
```

```
PID调试程序	
	
	yaw_pos_get = motor_get[YAW_MOTOR].total_angle;
		yaw_spd_get = motor_get[YAW_MOTOR].speed_rpm;
		if(rc_ctrl.rc.s1 == 1)
		{
			yaw_pos_set = 1000;
		}
		
		yaw_spd_set = PID_Calc(&motor_pid[PID_YAW_MOTOR_START_POS], yaw_pos_get, yaw_pos_set);
		yaw_current_set = PID_Calc(&motor_pid[PID_YAW_MOTOR_START_SPD], yaw_spd_get, yaw_spd_set);

		Gimbal_Motor_Drive(&hcan1,yaw_current_set,0); 
		osDelayUntil(&xLastWakeTime,GIMBAL_PERIOD);	
```

速度环滞后严重是因为什么？

调PID离谱的时候，看一下程序是不是有问题

目前两大问题：1.积分如何限幅如何使用  2.微分如何给

```C
用陀螺仪模式时
只有yaw的期望值命名为expect.yaw_gory
pitch的期望值命名仍为expect.pitch_encoder


发射逻辑
发射标志位及意义如下：
typedef enum
{
	STOP_SHOOT,      //关闭发射
	SHOOT_READY,     //准备发射
	SHOOT_ON,        //开启发射
	SHOOT_OFF,       //发射结束
	SHOOTING,        //正向拨弹（射击）
	BACK             //反向拨弹（调整下一颗子弹位置）
}SHOOT_MODE;
1.在云台模式下：
if(发射标志位被置为SHOOT_OFF或SHOOT_READY),发射标志位被置为SHOOT_OFF
2.在发射模式下：
if(C_Board.gun == STOP_SHOOT) C_Board.gun = SHOOT_OFF;
if(s2==3 && C_Board.gun = SHOOT_OFF) C_Board.gun = SHOOT_READY; //准备发射
if(s2==1 && C_Board.gun = SHOOT_READY) C_Board.gun = SHOOT_ON; //发射
3.在键鼠模式下：
if(Ctrl+左键) C_Board.gun = STOP_SHOOT;
if(左键 && C_Board.gun == STOP_SHOOT)   C_Board.gun = SHOOT_OFF; shoot_time = 10;
if(左键 && C_Board.gun == SHOOT_OFF)  shoot_time--; 减到0之后  C_Board.gun = SHOOT_ON;



STOP_SHOOT下：expect.Shoot_speed = 0;
SHOOT_OFF
    
    
    

射击任务：

无人机云台启动代码：
一开始获取角度就可以，不解算total_angle?   直接用total_angle呗
    
debug看一下到达指定位置后的total_angle和last_angle和angle这几个的关系 还有offset_angle
感觉到达指定位置后total_angle为0
```



#### 官方云台控制任务

#### 无人机还没有搞的东西、

测速模块没有装，打多少发弹丸的程序还没有

整理一下如何和飞镖通信的程序(这边主要是飞镖来写)



4278  5400差不多最大了  给1000角度吧 5278差不多



yaw轴 4128 2014

步兵的total_angle一直为0是什么意思？



```
莫名奇妙的bug
最好不要用if(motor_get[PITCH_MOTOR].msg_cnt++<= 10)
用下面的，老老实实写
	if(motor_get[PITCH_MOTOR].msg_cnt<= 10)	
    {
    motor_get[PITCH_MOTOR].msg_cnt++;
    Get_Moto_Offset(&motor_get[PITCH_MOTOR],CAN1_RX_date);
    }
    else
    {
    Get_Moto_Measure_6020(&motor_get[PITCH_MOTOR],CAN1_RX_date);
    motor_get[YAW_MOTOR].msg_cnt = 11;
    }
```



步兵内环最大输出为400   设置最大输出的目的是？

如果最大输出为400，而积分最大输出为230，相当于一大部分是积分的作用



![image-20210708135020175](C:/Users/chenw/AppData/Roaming/Typora/typora-user-images/image-20210708135020175.png)



```
							官方C板陀螺仪程序

```

![image-20210708163612749](C:/Users/chenw/AppData/Roaming/Typora/typora-user-images/image-20210708163612749.png)

![image-20210708163737135](C:/Users/chenw/AppData/Roaming/Typora/typora-user-images/image-20210708163737135.png)

这里获取的角度的单位是：rad(弧度)  获取的是欧拉角(yaw  pitch  roll)

![image-20210708163829701](C:/Users/chenw/AppData/Roaming/Typora/typora-user-images/image-20210708163829701.png)

![image-20210708164452205](C:/Users/chenw/AppData/Roaming/Typora/typora-user-images/image-20210708164452205.png)



#### 疑问：

INS_angle在哪里赋值？

为什么分离模式下使用编码器反馈就可以了？？

PID给阶跃信号只适用于起始PID参数吗