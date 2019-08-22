/**
 * @brief 机器人参数
 */

// 调试
#define DEBUG_ENABLED 0 // 调试开关

// 陀螺仪校准
#define IMU_GX_BIAS 32 // GX静态误差,通过平放主控板采样得到
#define IMU_GY_BIAS -6 // GY静态误差
#define IMU_GZ_BIAS -1 // GZ静态误差

// 陀螺仪
#define BOARD_FRONT_IS_UP 0                     // 板子正面朝下
#define BOARD_SHORT_SIDE_IS_PARALLEL_TO_PITCH 0 // 板子短边朝下
#define GYROSCOPE_YAW_START_UP_DELAY_ENABLED 1  // 开机解算延迟开关
#define GYROSCOPE_START_UP_DELAY 300            // 开机解算延迟量
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.005f   // 零飘修正阈值
#define GYROSCOPE_LSB 16.384f                   // 陀螺仪敏感度 2^16/4000
#define ACCELERATE_LSB 4096.0f                  // 加速度计敏感度 2^16/16

// 底盘
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  //底盘电机减速比
#define CHASSIS_MAX_ROTOR_SPEED 253         //最大轮子转速, 单位rad/s
#define CHASSIS_SIZE_K 0.385f               //测量值, 机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16f //测量值, 麦克纳姆轮半径的倒数

// 云台
#define YAW_ANGLE_MIN -110
#define YAW_ANGLE_MAX 110
#define PITCH_ANGLE_MIN -35
#define PITCH_ANGLE_MAX 15
#define AUTO_YAW_ANGLE_MIN -110
#define AUTO_YAW_ANGLE_MAX 110
#define AUTO_PITCH_ANGLE_MIN -35
#define AUTO_PITCH_ANGLE_MAX -5