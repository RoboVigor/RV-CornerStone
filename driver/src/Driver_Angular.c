#define __DRIVER_ANGULAR_GLOBALS
#include "Driver_Angular.h"
#include "MadgwickAHRS.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
//__DRIVER_MPU6500_EXT volatile IMU_DataType mpu6500_data;

extern float yaw_c;

//测试用
int q_0 = 0;
int q_1 = 0;
int q_2 = 0;
int q_3 = 0;
int yaw_32 = 0;
int pitch_32 = 0;
int roll_32 = 0;
int start_yaw = 0;
int final_yaw = 0;
uint8_t yaw_state = 1;
uint16_t startcount = 0;
int pitchint = 0;

//自制算法声明的变量
float SS_para1, SS_para2, SS_para3, SS_para4, SS_para5, SS_para6, SS_para7, SS_para8;
float SS_para9, SS_para10, SS_para11, SS_para12;
//float q[4]={1,0,0,0};
//float q[4]={0.5,0,0.8660254,0};   //mag,ix,iy,iz
float e_angle[3] = {0, 0, 0}; //yaw,pitch,roll
float a_speed[4] = {0, 0, 0, 0};
float last_a_speed[4] = {0, 0, 0, 0};
float Ki = 0.009;
float Kp = 8;
// float Kd=2;

float ax_acc = 0;
float ay_acc = 0;
float az_acc = 0;

float costheta1 = 1;
float sintheta1 = 0;
float costheta2 = 1;
float sintheta2 = 0;

float V_error_x_I = 0;
float V_error_y_I = 0;
float V_error_z_I = 0;
float a_sum;
float ax_nor;
float ay_nor;
float az_nor;
float vx, vy, vz;
float V_error_z, V_error_y, V_error_x;
float q_magnitude;
float T11, T12, T13, T23, T33;
float L_V_error_z, L_V_error_y, L_V_error_x;
float V_error_x_D, V_error_y_D, V_error_z_D;

float q0_;
float q1_;
float q2_;
float q3_;

void Motion_Update(void)
{
	//uint8_t count;

	//IMU_Data_Get();

	//	for(count=1;count<4;count++)
	//		last_a_speed[count]=a_speed[count];

	a_speed[1] = (float)((mpu6500_data.gx / GYRO_LSB) * PI / 180);
	a_speed[2] = (float)((mpu6500_data.gy / GYRO_LSB) * PI / 180);
	a_speed[3] = (float)((mpu6500_data.gz / GYRO_LSB) * PI / 180);
	ax_acc = (float)(mpu6500_data.ax / ACC_LSB);
	ay_acc = (float)(mpu6500_data.ay / ACC_LSB);
	az_acc = (float)(mpu6500_data.az / ACC_LSB);

	//	q_fresh(q,last_a_speed,a_speed,0.002);

	MadgwickAHRSupdateIMU(a_speed[1], a_speed[2], a_speed[3], ax_acc, ay_acc, az_acc);
	//GD算法或Madgwick算法,梯度算法,网上开源

	//	  MadgwickAHRSupdate(a_speed[1],a_speed[2],a_speed[3],ax_acc,ay_acc,az_acc,mpu6500_data.mx,mpu6500_data.my,mpu6500_data.mz);
	//	q_0=q[0]*1000;
	//	q_1=q[1]*1000;
	//	q_2=q[2]*1000;
	//	q_3=q[3]*1000;

	//JLINK测试用
	q_0 = (int)(q0 * 1000);
	q_1 = (int)(q1 * 1000);
	q_2 = (int)(q2 * 1000);
	q_3 = (int)(q3 * 1000);

	//		q0_=q0*costheta1-q2*sintheta1;
	//		q1_=q1*costheta1+q3*sintheta1;
	//		q2_=q2*costheta1;
	//		q3_=q3*costheta1-q1*sintheta1;

	//		q0_=q0*costheta1*costheta2+q1*sintheta1*costheta2-q2*costheta1*sintheta2+q3*sintheta1*sintheta2;
	//		q1_=q0*sintheta1*costheta2+q1*costheta1*costheta2+q2*sintheta1*sintheta2-q3*costheta1*sintheta2;
	//		q2_=q0*costheta1*sintheta2+q1*sintheta1*sintheta2+q2*costheta1*costheta2+q3*sintheta1*costheta2;
	//		q3_=q0*sintheta1*sintheta2+q1*costheta1*sintheta2-q2*sintheta1*costheta2+q3*costheta1*costheta2;
	//		int sum=sqrt(q0_*q0_+q1_*q1_+q2_*q2_+q3_*q3_);
	//		q0_=q0_/sum;
	//		q1_=q1_/sum;
	//		q2_=q2_/sum;
	//		q3_=q3_/sum;
	//	q0_=q0*0.819152+q2*0.57358;
	//	q1_=q1*0.819152+q3*0.57358;
	//	q2_=-q0*0.57358+q2*0.819152;
	//	q3_=q3*0.819152-q2*0.57358;
	//	q0_=q0*0.5-q2*0.8660254;
	//	q1_=q1*0.5-q3*0.8660254;
	//	q2_=q0*0.8660254+q2*0.5;
	//	q3_=q3*0.5+q2*0.8660254;
	//    q[0]=q0;
	//		q[1]=q1;
	//		q[2]=q2;
	//		q[3]=q3;
	//	q2euler(q,e_angle);

	//    const float epsilon=0.0009765625f;
	//    const float threshold=0.5f-epsilon;
	//    float judge=q0_*q2_-q1_*q3_;
	//    if(judge<-threshold||judge>threshold)
	//    {
	//        e_angle[yaw]=-2*atan2(q1_,q0_)*180/PI;
	//        e_angle[pitch]=90;
	//        e_angle[roll]=0;
	//    }
	//    else
	//    {
	//        e_angle[roll] = atan2(2 * (q0_ * q1_ + q2_ * q3_),(1-2*(pow(q1_,2.0)+pow(q2_,2.0))))*180/PI;
	//        e_angle[pitch] = asin(2 * (q0_ * q2_ - q1_ * q3_));
	//        e_angle[yaw] = atan2(2 * (q0_ * q3_) + q1_ * q2_,(1-2*(pow(q2_,2.0)+pow(q3_,2.0))))*180/PI;
	//    }

	e_angle[1] = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180 / PI;
	e_angle[0] = asin(2.0f * (q0 * q2 - q1 * q3)) * 180 / PI;
	e_angle[2] = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180 / PI;

	if (e_angle[1] >= 0)
		e_angle[1] -= 180.0;
	else
		e_angle[1] += 180.0;

	Euler_Angle.Yaw = e_angle[2];
	Euler_Angle.Pitch = -e_angle[1];
	Euler_Angle.Pitch += Euler_Angle.Pitch_offset;
	Euler_Angle.Roll = e_angle[0];

	//	if(startcount>=7000)
	//	{
	//		startcount=7000;
	//		if(Motor_Feedback.motor201Speed==0 && Motor_Feedback.motor202Speed==0 &&
	//		Motor_Feedback.motor203Speed==0 && Motor_Feedback.motor204Speed==0 && yaw_c==0 && YAW_Encoder.ecdAngle==0)
	//		{
	//			if(yaw_state)
	//			{
	//				start_yaw=(int)Euler_Angle.Yaw;
	//				yaw_state=0;
	//			}
	//			Euler_Angle.Yaw = start_yaw;
	//		}
	//		else
	//		{
	//			if(yaw_state==0)
	//			{
	//				final_yaw=(int)Euler_Angle.Yaw;
	//				Euler_Angle.Yaw_offset+=(start_yaw-final_yaw);
	//				yaw_state=1;
	//			}
	//		}
	//	}
	//	Euler_Angle.Yaw +=Euler_Angle.Yaw_offset;
	//	if(Euler_Angle.Yaw>180)
	//		Euler_Angle.Yaw -=360;
	//	else if(Euler_Angle.Yaw<-180)
	//		Euler_Angle.Yaw +=360;

	//JLINK测试用
	yaw_32 = (int32_t)(Euler_Angle.Yaw);	 //*1000);
	pitch_32 = (int32_t)(Euler_Angle.Pitch); //*1000);
	roll_32 = (int32_t)(Euler_Angle.Roll);   //*1000);
											 //pitchint=(int)Pitch_Encoder.ecdAngle;

	//	startcount++;
}

//四元数更新,队内自制,可能可以,互补滤波,一阶微分方程或二阶可选择,但没有使用
void q_fresh(float *q_num, float *a_num, float *a_num2, float time)
{
	//这是二阶四元数更新算法,没有加旋转矢量,因为是瞬时角速度,如果是角增量要采用旋转矢量
	//采用了暴力方法为了节省时间
	ax_acc = (float)(mpu6500_data.ax / ACC_LSB);
	ay_acc = (float)(mpu6500_data.ay / ACC_LSB);
	az_acc = (float)(mpu6500_data.az / ACC_LSB);

	a_sum = sqrt(pow(ax_acc, 2.0) + pow(ay_acc, 2.0) + pow(az_acc, 2.0));

	ax_nor = ax_acc / a_sum;

	ay_nor = ay_acc / a_sum;

	az_nor = az_acc / a_sum;

	vx = 2 * (q_num[1] * q_num[3] - q_num[0] * q_num[2]);
	vy = 2 * (q_num[0] * q_num[1] + q_num[2] * q_num[3]);
	//	 vz=1-2*(q_num[1]*q_num[1]+q_num[2]*q_num[2]);
	vz = q_num[0] * q_num[0] - q_num[1] * q_num[1] - q_num[2] * q_num[2] + q_num[3] * q_num[3];

	L_V_error_z = V_error_z;
	L_V_error_y = V_error_y;
	L_V_error_x = V_error_x;
	V_error_z = ax_nor * vy - ay_nor * vx;
	V_error_y = az_nor * vx - ax_nor * vz;
	V_error_x = ay_nor * vz - az_nor * vy;

	if (V_error_x != 0.0f && V_error_y != 0.0f && V_error_z != 0.0f)
	{
		V_error_x_I += (V_error_x * Ki);
		V_error_y_I += (V_error_y * Ki);
		V_error_z_I += (V_error_z * Ki);

		//		V_error_x_D=Kd*(V_error_x-L_V_error_z)/time;
		//		V_error_y_D=Kd*(V_error_y-L_V_error_y)/time;
		//		V_error_z_D=Kd*(V_error_z-L_V_error_x)/time;
		a_num[ix] += (V_error_x * Kp + V_error_x_I);
		a_num[iy] += (V_error_y * Kp + V_error_y_I);
		a_num[iz] += (V_error_z * Kp + V_error_z_I);
	}

	SS_para1 = 0.5 * (-a_num[ix] * q_num[ix] - a_num[iy] * q_num[iy] - a_num[iz] * q_num[iz]);
	SS_para2 = 0.5 * (a_num[ix] * q_num[mag] + a_num[iz] * q_num[iy] - a_num[iy] * q_num[iz]);
	SS_para3 = 0.5 * (a_num[iy] * q_num[mag] - a_num[iz] * q_num[ix] - a_num[ix] * q_num[iz]);
	SS_para4 = 0.5 * (a_num[iz] * q_num[mag] + a_num[iy] * q_num[ix] - a_num[ix] * q_num[iy]);

	q_num[mag] = q_num[mag] + time * SS_para1;
	q_num[ix] = q_num[ix] + time * SS_para2;
	q_num[iy] = q_num[iy] + time * SS_para3;
	q_num[iz] = q_num[iz] + time * SS_para4;

	//	SS_para5 = q_num[mag] + time * SS_para1;
	//	SS_para6 = q_num[ix] + time * SS_para2;
	//	SS_para7 = q_num[iy] + time * SS_para3;
	//	SS_para8 = q_num[iz] + time * SS_para4;
	//	SS_para9 = 0.5*(-a_num2[ix] * SS_para6 - a_num2[iy] * SS_para7 - a_num2[iz] * SS_para8);
	//	SS_para10 = 0.5*(a_num2[ix] * SS_para5 + a_num2[iz] * SS_para7 - a_num2[iy] * SS_para8);
	//	SS_para11 = 0.5*(a_num2[iy] * SS_para5 - a_num2[iz] * SS_para6 - a_num2[ix] * SS_para8);
	//	SS_para12 = 0.5*(a_num2[iz] * SS_para5 + a_num2[iy] * SS_para6 - a_num2[ix] * SS_para7);
	//	q_num[mag] += time / 2 * (SS_para1 + SS_para9);
	//	q_num[ix] += time / 2 * (SS_para2 + SS_para10);
	//	q_num[iy] += time / 2 * (SS_para3 + SS_para11);
	//	q_num[iz] += time / 2 * (SS_para4 + SS_para12);

	q_magnitude = sqrt(pow(q_num[mag], 2.0) + pow(q_num[ix], 2.0) + pow(q_num[iy], 2.0) + pow(q_num[iz], 2.0));
	q_num[mag] = q_num[mag] / q_magnitude;
	q_num[ix] = q_num[ix] / q_magnitude;
	q_num[iy] = q_num[iy] / q_magnitude;
	q_num[iz] = q_num[iz] / q_magnitude;
}

//四元数转欧拉2失败
void q2euler2(float *q_num, float *e_num)
{
	const float epsilon = 0.0009765625f;
	const float threshold = 0.5f - epsilon;
	float judge = q_num[0] * q_num[2] - q_num[1] * q_num[3];
	if (judge < -threshold || judge > threshold)
	{
		e_num[yaw] = -2 * atan2(q_num[1], q_num[0]) * 180 / PI;
		e_num[pitch] = 90;
		e_num[roll] = 0;
	}
	else
	{
		e_num[roll] = atan2(2 * (q_num[0] * q_num[1] + q_num[2] * q_num[3]), (1 - 2 * (pow(q_num[1], 2.0) + pow(q_num[2], 2.0)))) * 180 / PI;
		e_num[pitch] = asin(2 * (q_num[0] * q_num[2] - q_num[1] * q_num[3]));
		e_num[yaw] = atan2(2 * (q_num[0] * q_num[3] + q_num[1] * q_num[2]), (1 - 2 * (pow(q_num[2], 2.0) + pow(q_num[3], 2.0)))) * 180 / PI;
	}
}

//四元数转欧拉失败
void q2euler(float *q_num, float *e_num)
{
	//q_num是四元数,e_num是欧拉角,都是数组
	//检查四元数是否是四个
	//	if ((sizeof(q_num) / sizeof(q_num[0])) < 5)
	//	{
	//		return NULL;
	//	}
	//准备转换
	T11 = pow(q_num[0], 2.0) + pow(q_num[1], 2.0) - pow(q_num[2], 2.0) - pow(q_num[3], 2.0);
	T12 = 2 * (q_num[1] * q_num[2] + q_num[0] * q_num[3]);
	T23 = 2 * (q_num[2] * q_num[3] + q_num[0] * q_num[1]);
	T33 = pow(q_num[0], 2.0) + pow(q_num[3], 2.0) - pow(q_num[2], 2.0) - pow(q_num[1], 2.0);
	T13 = 2 * (q_num[1] * q_num[3] - q_num[0] * q_num[2]);
	//得出pitch角,俯仰角,y轴角
	e_num[pitch] = -asin(T13) * 180 / PI;
	//得出yawn角,航向角,z轴角
	if (T11 < 0.01 && T11 > -0.01)
	{
		if (T12 < 0)
			e_num[yaw] = 270;
		else
			e_num[yaw] = 90;
	}
	else if (T11 >= 0.01)
	{
		e_num[yaw] = atan(T12 / T11) * 180 / (PI);
	}
	else
	{
		e_num[yaw] = atan(T12 / T11) * 180 / (PI) + 180;
	}
	if (T33 < 0.01 && T33 > -0.01)
	{
		if (T23 < 0)
			e_num[roll] = -90;
		else
			e_num[roll] = 90;
	}
	else if (T11 >= 0.01)
	{
		e_num[roll] = atan(T12 / T11) * 180 / (PI);
	}
	else
	{
		if (T23 > 0)
			e_num[roll] = atan(T12 / T11) * 180 / (PI) + 180;
		else
			e_num[roll] = atan(T12 / T11) * 180 / (PI)-180;
	}
}

//论坛matlab滤波方法
//double Chebyshev10HzLPF(Filter_t *F)
//{
//	int i;
//	for(i=7; i>0; i--)
//	{
//		F->ybuf[i] = F->ybuf[i-1];
//		F->xbuf[i] = F->xbuf[i-1];
//	}
//	F->xbuf[0] = F->rawValue;
//	F->ybuf[0] = NUM[0] * F->xbuf[0];
//	for(i=1;i<8;i++)
//	{
//		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
//	}
//	F->filtered_value = F->ybuf[0];
//	return F->filtered_value;
//}

//2016年官方开源,互补滤波
/////////////////////////////////////////////////////////////////////////

//volatile float exInt, eyInt, ezInt;  // 误差积分
//volatile float q0 = 1.0f;
//volatile float q1 = 0.0f;
//volatile float q2 = 0.0f;
//volatile float q3 = 0.0f;

//volatile float mygetqval[9];	//用于存放传感器转换结果的数组
//static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //作用域仅在此文件中

//static volatile float q[4]; //　四元数
//volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
//volatile float angle[3] = {0};
//volatile float yaw_temp,pitch_temp,roll_temp;
//volatile float last_yaw_temp,last_pitch_temp,last_roll_temp;
//volatile float yaw_angle,pitch_angle,roll_angle; //使用到的角度值

//uint32_t Get_Time_Micros(void)
//{
//	return TIM2->CNT;
//}

//// Fast inverse square-root
///**************************实现函数********************************************
//*函数原型:	   float invSqrt(float x)
//*功　　能:	   快速计算 1/Sqrt(x)
//输入参数: 要计算的值
//输出参数: 结果
//*******************************************************************************/
//float invSqrt(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}

///**************************实现函数********************************************
//*函数原型:	   void Init_Quaternion
//*功　　能:	 初始化四元数
//输入参数: 当前的测量值.
//输出参数:没有
//*******************************************************************************/
////初始化IMU数据
//#define BOARD_DOWN 1   //板子正面朝下摆放

//void Init_Quaternion()//根据测量数据,初始化q0,q1,q2.q3,从而加快收敛速度
//{
//	int16_t hx,hy,hz;
//	hx=mpu6500_data.mx;
//	hy=mpu6500_data.my;
//	hz=mpu6500_data.mz;
//	#ifdef BOARD_DOWN
//	if(hx<0 && hy <0)   //OK
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = -0.005;
//			q1 = -0.199;
//			q2 = 0.979;
//			q3 = -0.0089;
//		}
//		else
//		{
//			q0 = -0.008;
//			q1 = -0.555;
//			q2 = 0.83;
//			q3 = -0.002;
//		}
//
//	}
//	else if (hx<0 && hy > 0) //OK
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = 0.005;
//			q1 = -0.199;
//			q2 = -0.978;
//			q3 = 0.012;
//		}
//		else
//		{
//			q0 = 0.005;
//			q1 = -0.553;
//			q2 = -0.83;
//			q3 = -0.0023;
//		}
//
//	}
//	else if (hx > 0 && hy > 0)   //OK
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = 0.0012;
//			q1 = -0.978;
//			q2 = -0.199;
//			q3 = -0.005;
//		}
//		else
//		{
//			q0 = 0.0023;
//			q1 = -0.83;
//			q2 = -0.553;
//			q3 = 0.0023;
//		}
//
//	}
//	else if (hx > 0 && hy < 0)     //OK
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = 0.0025;
//			q1 = 0.978;
//			q2 = -0.199;
//			q3 = 0.008;
//		}
//		else
//		{
//			q0 = 0.0025;
//			q1 = 0.83;
//			q2 = -0.56;
//			q3 = 0.0045;
//		}
//	}
//	#else
//		if(hx<0 && hy <0)
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = 0.195;
//			q1 = -0.015;
//			q2 = 0.0043;
//			q3 = 0.979;
//		}
//		else
//		{
//			q0 = 0.555;
//			q1 = -0.015;
//			q2 = 0.006;
//			q3 = 0.829;
//		}
//
//	}
//	else if (hx<0 && hy > 0)
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = -0.193;
//			q1 = -0.009;
//			q2 = -0.006;
//			q3 = 0.979;
//		}
//		else
//		{
//			q0 = -0.552;
//			q1 = -0.0048;
//			q2 = -0.0115;
//			q3 = 0.8313;
//		}
//
//	}
//	else if (hx>0 && hy > 0)
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = -0.9785;
//			q1 = 0.008;
//			q2 = -0.02;
//			q3 = 0.195;
//		}
//		else
//		{
//			q0 = -0.9828;
//			q1 = 0.002;
//			q2 = -0.0167;
//			q3 = 0.5557;
//		}
//
//	}
//	else if (hx > 0 && hy < 0)
//	{
//		if(fabs(hx/hy)>=1)
//		{
//			q0 = -0.979;
//			q1 = 0.0116;
//			q2 = -0.0167;
//			q3 = -0.195;
//		}
//		else
//		{
//			q0 = -0.83;
//			q1 = 0.014;
//			q2 = -0.012;
//			q3 = -0.556;
//		}
//	}
//	#endif
//
//	//根据hx hy hz来判断q的值,取四个相近的值做逼近即可,初始值可以由欧拉角转换到四元数计算得到
//
//}

//

///**************************实现函数********************************************
//*函数原型:	   void IMU_getValues(volatile float * values)
//*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值
//输入参数: 将结果存放的数组首地址
//加速度值:原始数据,-8192-+8192
//角速度值:deg/s
//磁力计值:原始数据
//输出参数:没有
//*******************************************************************************/
//void IMU_getValues(volatile float * values) {
//		int16_t accgyroval[6];
//		int i;
//	//读取加速度和陀螺仪的当前ADC

//	accgyroval[0]=mpu6500_data.ax;
//	accgyroval[1]=mpu6500_data.ay;
//	accgyroval[2]=mpu6500_data.az;
//	accgyroval[3]=mpu6500_data.gx;
//	accgyroval[4]=mpu6500_data.gy;
//	accgyroval[5]=mpu6500_data.gz;
//
//
//    for(i = 0; i<6; i++) {
//      if(i < 3) {
//        values[i] = (float) accgyroval[i];
//      }
//      else {
//        values[i] = ((float) accgyroval[i]) / GYRO_LSB; //转成度每秒
//      }
//    }
//    //读取磁力计的ADC值
//		values[6]=mpu6500_data.mx;
//		values[7]=mpu6500_data.my;
//		values[8]=mpu6500_data.mz;
//}

///**************************实现函数********************************************
//*函数原型:	   void IMU_AHRSupdate
//*功　　能:	 更新AHRS 更新四元数
//输入参数: 当前的测量值.
//输出参数:没有
//*******************************************************************************/
//#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
//void IMU_AHRSupdate(void) {
//    float norm;
//    float hx, hy, hz, bx, bz;
//    float vx, vy, vz, wx, wy, wz;
//    float ex, ey, ez,halfT;
//    float tempq0,tempq1,tempq2,tempq3;

//    float q0q0 = q0*q0;
//    float q0q1 = q0*q1;
//    float q0q2 = q0*q2;
//    float q0q3 = q0*q3;
//    float q1q1 = q1*q1;
//    float q1q2 = q1*q2;
//    float q1q3 = q1*q3;
//    float q2q2 = q2*q2;
//    float q2q3 = q2*q3;
//    float q3q3 = q3*q3;

//    gx = mygetqval[3] * PI/180;
//    gy = mygetqval[4] * PI/180;
//    gz = mygetqval[5] * PI/180;
//    ax = mygetqval[0];
//    ay = mygetqval[1];
//    az = mygetqval[2];
//    mx = mygetqval[6];
//    my = mygetqval[7];
//    mz = mygetqval[8];

//    now = Get_Time_Micros();  //读取时间 单位是us
//    if(now<lastUpdate)
//    {
//    //halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
//    }
//    else
//    {
//        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//    }
//    lastUpdate = now;	//更新时间
//    //快速求平方根算法
//    norm = invSqrt(ax*ax + ay*ay + az*az);
//    ax = ax * norm;
//    ay = ay * norm;
//    az = az * norm;
//    //把加计的三维向量转成单位向量.
//    norm = invSqrt(mx*mx + my*my + mz*mz);
//    mx = mx * norm;
//    my = my * norm;
//    mz = mz * norm;
//    // compute reference direction of flux
//    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
//    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
//    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);
//    bx = sqrt((hx*hx) + (hy*hy));
//    bz = hz;
//    // estimated direction of gravity and flux (v and w)
//    vx = 2.0f*(q1q3 - q0q2);
//    vy = 2.0f*(q0q1 + q2q3);
//    vz = q0q0 - q1q1 - q2q2 + q3q3;
//    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
//    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
//    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);
//    // error is sum of cross product between reference direction of fields and direction measured by sensors
//    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
//    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
//    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

//    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
//    {
//        exInt = exInt + ex * Ki * halfT;
//        eyInt = eyInt + ey * Ki * halfT;
//        ezInt = ezInt + ez * Ki * halfT;
//        // 用叉积误差来做PI修正陀螺零偏
//        gx = gx + Kp*ex + exInt;
//        gy = gy + Kp*ey + eyInt;
//        gz = gz + Kp*ez + ezInt;
//    }
//    // 四元数微分方程
//    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

//    // 四元数规范化
//    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
//    q0 = tempq0 * norm;
//    q1 = tempq1 * norm;
//    q2 = tempq2 * norm;
//    q3 = tempq3 * norm;

//}

///**************************实现函数********************************************
//*函数原型:	   void IMU_getQ(float * q)
//*功　　能:	 更新四元数 返回当前的四元数组值
//输入参数: 将要存放四元数的数组首地址
//输出参数:没有
//*******************************************************************************/

//void IMU_getQ(volatile float * q) {

//    IMU_getValues(mygetqval);	 //获取原始数据,加速度计和磁力计是原始值,陀螺仪转换成了deg/s
//    IMU_AHRSupdate();
//    q[0] = q0; //返回当前值
//    q[1] = q1;
//    q[2] = q2;
//    q[3] = q3;
//}

///**************************实现函数********************************************
//*函数原型:	   void IMU_getYawPitchRoll(float * angles)
//*功　　能:	 更新四元数 返回当前解算后的姿态数据
//输入参数: 将要存放姿态角的数组首地址
//输出参数:没有
//*******************************************************************************/
//void IMU_getYawPitchRoll(volatile float * angles)
//{
//    // volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向
//    IMU_getQ(q); //更新全局四元数
//    //四元数转换成欧拉角,经过三角函数计算即可
//    angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/PI; // yaw        -pi----pi
//    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/PI; // pitch    -pi/2    --- pi/2
//    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/PI; // roll       -pi-----pi
//}

//static int yaw_count = 0;
//void GetPitchYawGxGyGz(void)
//{
////	MPU6050_Real_Data.Gyro_X = mygetqval[3];
////	MPU6050_Real_Data.Gyro_Y = -mygetqval[4];
////	MPU6050_Real_Data.Gyro_Z = mygetqval[5];

//	last_yaw_temp = yaw_temp;
//	yaw_temp = angle[0];
//	if(yaw_temp-last_yaw_temp>=330)  //yaw轴角度经过处理后变成连续的
//	{
//		yaw_count--;
//	}
//	else if (yaw_temp-last_yaw_temp<=-330)
//	{
//		yaw_count++;
//	}
//	Euler_Angle.Yaw = yaw_temp + yaw_count*360;  //yaw轴角度
//	Euler_Angle.Pitch = angle[1];
//  Euler_Angle.Roll = angle[2];
//	yaw_32=(int32_t)(Euler_Angle.Yaw);//*1000);
//	pitch_32=(int32_t)(Euler_Angle.Pitch);//*1000);
//	roll_32=(int32_t)(Euler_Angle.Roll);//*1000);
//}

////////////////////////////////////////////////////////////////////////
