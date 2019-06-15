/**
 * @brief 机器人参数
 */

// 调试
#define DEBUG_ENABLED 0 // 调试开关

// 陀螺仪
#define GYROSCOPE_YAW_START_UP_DELAY_ENABLED 0 // 开机解算延迟开关
#define GYROSCOPE_START_UP_DELAY 1700          // 开机解算延迟量
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.005f  // 零飘修正阈值
#define GYROSCOPE_LSB 16.384f                  // 陀螺仪敏感度 2^16/4000
#define ACCELERATE_LSB 4096.0f                 // 加速度计敏感度 2^16/16

// 底盘
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  //底盘电机减速比
#define CHASSIS_MAX_ROTOR_SPEED 253         //最大轮子转速, 单位rad/s
#define CHASSIS_SIZE_K 0.385f               //测量值, 机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16f //测量值, 麦克纳姆轮半径的倒数
