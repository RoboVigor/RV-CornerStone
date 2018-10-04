/**
 * @brief 机器人参数
 */

// 全局
#define BOARD_VERSION 1 //开发板型号,0:旧板,1:A板
#define CAN1_ENABLED 1
#define CAN2_ENABLED 0
#define USER_POWER_ENABLED 1

// 陀螺仪
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.1

// 底盘
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2  //底盘电机减速比
#define CHASSIS_MAX_ROTOR_SPEED 253        //最大轮子转速,单位rad/s
#define CHASSIS_SIZE_K 0.946               //测量值,机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16 //测量值,麦克纳姆轮半径的倒数
