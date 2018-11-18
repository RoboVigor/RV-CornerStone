/**
 * @brief 机器人参数
 */

// 全局
#define ROBOT_TYPE 1         // 机器人种类, 0:甩锅小车, 1:步兵
#define DEBUG_ENABLED 1      // 调试开关
#define BOARD_VERSION 1      //开发板型号,0:旧板,1:A板
#define USER_POWER_ENABLED 1 // 24V用户电源开关

// 陀螺仪
#if ROBOT_TYPE == 1
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.0040  // 零飘修正阈值
#define GYROSCOPE_YAW_QUATERNION_ABSTRACTION 1 // 开启四元数收敛计数器
#define COUNT_QUATERNIONABSTRACTION 7000       //云台四元数收敛
#endif
#if ROBOT_TYPE == 0
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.003
#endif

// 底盘
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2  //底盘电机减速比
#define CHASSIS_MAX_ROTOR_SPEED 253        //最大轮子转速,单位rad/s
#define CHASSIS_SIZE_K 0.946               //测量值,机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16 //测量值,麦克纳姆轮半径的倒数
