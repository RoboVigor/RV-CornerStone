/**
 * @brief 机器人参数
 */

// 硬件
#define DEBUG_ENABLED 0          // 调试开关
#define BOARD_VERSION 1          // 开发板型号, 0:旧板, 1:A板
#define USER_POWER_ENABLED 1     // 24V用户电源开关
#define LASER_ENABLED 1          // 激光开关
#define USART3_ENABLED 1         // 串口3开关
#define USART3_BAUD_RATE 9600    // 串口3波特率
#define USART6_ENABLED 0         // 串口6开关
#define USART6_BAUD_RATE 9600    // 串口6波特率
#define SERIAL_DEBUG_PORT USART6 // 串口调试端口

// 陀螺仪
#define GYROSCOPE_START_UP_DELAY_ENABLED 1    //开机解算延迟开关
#define GYROSCOPE_START_UP_DELAY 1700         //开机解算延迟量
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.003f // 零飘修正阈值
#define IMU_GX_BIAS 2
#define IMU_GY_BIAS 9
#define IMU_GZ_BIAS -6
#define GYROSCOPE_LSB 16.384f  // 2^16/4000
#define ACCELERATE_LSB 4096.0f // 2^16/16
#define RPM2RPS 0.10471975f    // 2 * 3.1415926f / 60.0f round per minute  to rad per second
#define DPS2RPS 0.01745329f    // 3.1415926f / 180.0f    degree per second to rad per second
#define RPS2DPS 57.2957804f    // 180.0f / 3.1415926f    rad per second    to degree per second

// 底盘
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  //底盘电机减速比
#define CHASSIS_MAX_ROTOR_SPEED 700         //最大轮子转速,单位rad/s
#define CHASSIS_SIZE_K 0.385f               //测量值,机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16f //测量值,麦克纳姆轮半径的倒数

// 视觉
#define PS_ENABLE 0 // 视觉辅助开关
