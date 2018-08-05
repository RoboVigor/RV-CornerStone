
#ifndef __DRIVER_ANGULAR_H
#define __DRIVER_ANGULAR_H

#include "stm32f4xx.h"
#include "math.h"
#include "mpu6500_driver.h"
#include "stdio.h"

#define Q_SIZE   4
#define E_size   3
#define PI       3.1415926
#define GYRO_LSB 16.4f
#define ACC_LSB  4096.0


//e_index是欧拉角索引,x轴向机头,y轴向机翼,yaw为z轴旋转,pitch为y轴旋转
//q_index是四元数索引,mag是实部
typedef enum {yaw,pitch,roll} i_euler;
typedef enum  {mag,ix,iy,iz} i_qud ;

typedef struct{
	float Yaw;
	int   Yaw_offset;
	int   round;
	float Yawfeedback;
	float Pitch;
	int   Pitch_offset;
	float Roll;
} Euler_AngleType;

typedef struct{
	double raw_value;
	double xbuf[8];
	double ybuf[8];
	double filtered_value;
}Filter_t;

#ifdef  __DRIVER_ANGULAR_GLOBALS
#define __DRIVER_ANGULAR_EXT
#else
#define __DRIVER_ANGULAR_EXT extern
#endif


__DRIVER_ANGULAR_EXT volatile Euler_AngleType Euler_Angle;
__DRIVER_ANGULAR_EXT i_euler e_index;
__DRIVER_ANGULAR_EXT i_qud   q_index;
extern volatile float angle[3];

void q2euler(float* q_num,float* e_num);
void q_fresh( float* q_num, float* a_num, float* a_num2,float time);
void Motion_Update(void);
double Chebyshev10HzLPF(Filter_t *F);
void q2euler2(float *q_num, float *e_num);
///////////////////////////////////////////////////
void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //更新姿态
void GetPitchYawGxGyGz(void);

#endif 


