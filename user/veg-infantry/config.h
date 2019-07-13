/**
 * @brief 机器人参数
 * @note  默认参数及参数列表见 default_config.h
 */

#include "default_config.h"

// 调试
#define DEBUG_ENABLED 0 // 调试开关
// #define SERIAL_DEBUG_PORT USART6 // 串口调试端口

// 陀螺仪校准
#define IMU_GX_BIAS 13 // GX静态误差,通过平放主控板采样得到
#define IMU_GY_BIAS 4  // GY静态误差
#define IMU_GZ_BIAS 11 // GZ静态误差

// 运动参数
#define GIMBAL_PITCH_MIN -15
#define GIMBAL_PITCH_MAX 30
#define CHASSIS_ROTOR_SPEED 550

// 底盘配置
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  // 底盘电机减速比
#define CHASSIS_SIZE_K 0.385f               // 测量值, 机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16f // 测量值, 麦克纳姆轮半径的倒数

// 陀螺仪设置
#define BOARD_FRONT_IS_UP 0                     // 板子正面朝上
#define BOARD_SHORT_SIDE_IS_PARALLEL_TO_PITCH 0 // 板子短边朝下
#define GYROSCOPE_START_UP_DELAY_ENABLED 1      // 开机解算延迟开关
#define GYROSCOPE_START_UP_DELAY 200            // 开机解算延迟量
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.005f   // 零飘修正阈值
#define GYROSCOPE_LSB 16.384f                   // 陀螺仪敏感度 2^16/4000
#define ACCELERATE_LSB 4096.0f                  // 加速度计敏感度 2^16/16