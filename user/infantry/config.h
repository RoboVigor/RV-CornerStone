/**
 * @brief 机器人参数
 * @note  默认参数及参数列表见 default_config.h
 */

#include "default_config.h"

// Dedicated tuning build: owns USART3 and joint0; boot/STOP/fault = zero output.
// Set to 0 ONLY to explicitly build the legacy, non-tuning behavior.
#ifndef GIMBAL_TUNING_ENABLED
#define GIMBAL_TUNING_ENABLED 1
#endif

// 步兵编号
#define ROBOT_MIAO (Robot_Id == 1)
#define ROBOT_WANG (Robot_Id == 2)
#define ROBOT_SHARK (Robot_Id == 3)

//裁判系统启用
#define fricEnabled 0

// 调试
#define DEBUG_ENABLED 0          // 调试开关
#define SERIAL_DEBUG_PORT USART6 // 串口调试端口

// 运动参数
#define GIMBAL_PITCH_MIN -15
#define GIMBAL_PITCH_MAX 28
#define CHASSIS_ROTOR_SPEED 550

// 底盘配置
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  // 底盘电机减速比
#define CHASSIS_SIZE_K 0.385f               // 测量值, 机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.333f // 测量值, 麦克纳姆轮半径的倒数
#define CHASSIS_RADIUS 0.250f               // 测量值，全向轮电机到底盘俯视图几何中心的距离
// k0,k1,k2,k3,k4,k5
#define SECOND_MACLAURIN_COEFFICIENT {0.6641993412640775,     \
                                      0.006444284468539646,   \
                                      0.0001423857226262331,  \
                                      0.017644430204543864,   \
                                      0.1650143850678086,     \
                                      0.00003096721772539512   \
                                    }      

//拨弹配置
#define STIR_MOTOR_REDUCTION_RATE 36.0f // 拨弹电机减速比

//射击配置
#define FIRE_MOTOR_REDUCTION_RATE 19.2f // 射击电机减速比

//云台配置
#define HAS_SLIP_RING 1                  // 该步兵拥有滑环
#define GIMBAL_MOTOR_REDUCTION_RATE 1.0f // 云台电机减速比

// 陀螺仪设置
#define fastTriangle 1                          // 快速三角函数计算，仅用于imu解算，且下列三个角度必须为PI/2的倍数
#define angle_x 0                               // 内旋，先绕x轴旋转角度（rad）
#define angle_y 0                               // 内旋，再绕y轴旋转角度（rad）
#define angle_z (PI/2)                           // 内旋，最后绕z轴旋转角度（rad）
#if fastTriangle
  #define f_cos(para) (pow(-1, ((int)((para + PI/2)/PI*2))/2)*abs((int)((para + PI/2)/PI*2)%2))
  #define f_sin(para) (pow(-1, ((int)(para/PI*2))/2)*abs((int)(para/PI*2)%2))
#else
  #define f_cos(para) cos(para)
  #define f_sin(para) sin(para)
#endif
#define trans_matrix {f_cos(angle_y)*f_cos(angle_z), f_cos(angle_x)*f_sin(angle_z) + f_cos(angle_z)*f_sin(angle_x)*f_sin(angle_y), f_sin(angle_x)*f_sin(angle_z) - f_cos(angle_x)*f_cos(angle_z)*f_sin(angle_y), \
                      -f_cos(angle_y)*f_sin(angle_z), f_cos(angle_x)*f_cos(angle_z) - f_sin(angle_x)*f_sin(angle_y)*f_sin(angle_z), f_cos(angle_z)*f_sin(angle_x) + f_cos(angle_x)*f_sin(angle_y)*f_sin(angle_z),  \
                      f_sin(angle_y), -f_cos(angle_y)*f_sin(angle_x), f_cos(angle_x)*f_cos(angle_y)}
#define GYROSCOPE_YAW_FILTER_Coefficient 0 // 零飘修正 更换为EKF后弃用
#define GYROSCOPE_YAW_MODIFICATION -0.002f    // 零飘修正值
#define GYROSCOPE_LSB 16.384f                 // 陀螺仪敏感度 2^16/4000
#define ACCELERATE_LSB 4096.0f                // 加速度计敏感度 2^16/16

// DMA
#define DMA_BUFFER_LENGTH 128 // DMA发送接收长度

//电机相关参数
#define C2T_3508 0.3f  //3508电机转矩对电流系数
#define CurrentMap_C620 0.0012f  //c620电调电流对输入值的映射系数
#define CurrentMap_C620_Inverse 833.333f //

// 对应handle.h中的初始化事件组对应的掩码
#define INIT_EVENT_IMU        (1 << 0)
#define INIT_EVENT_ALL        INIT_EVENT_IMU