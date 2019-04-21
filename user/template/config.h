/**
 * @brief 机器人参数
 */

// Driver版本
#define DRIVER_VERSION_1_0 // 适配版本1.0

// 硬件
#define DEBUG_ENABLED 1          // 调试开关
#define BOARD_VERSION 1          // 开发板型号, 0:旧板, 1:A板
#define USER_POWER_ENABLED 1     // 24V用户电源开关
#define USART3_ENABLED 1         // 串口3开关
#define USART3_BAUD_RATE 9600    // 串口3波特率
#define USART6_ENABLED 1         // 串口6开关
#define USART6_BAUD_RATE 9600    // 串口6波特率
#define SERIAL_DEBUG_PORT USART6 // 串口调试端口

// 陀螺仪
#define GYROSCOPE_START_UP_DELAY_ENABLED 1    //开机解算延迟开关
#define GYROSCOPE_START_UP_DELAY 500          //开机解算延迟量
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.003f // 零飘修正阈值

// 底盘
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  //底盘电机减速比
#define CHASSIS_MAX_ROTOR_SPEED 253         //最大轮子转速, 单位rad/s
#define CHASSIS_SIZE_K 0.946f               //测量值, 机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16f //测量值, 麦克纳姆轮半径的倒数

// 视觉
#define PS_ENABLE 0 // 视觉辅助开关
