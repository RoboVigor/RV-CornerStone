/**
 * @brief 机器人参数
 * @note  默认参数及参数列表见 default_config.h
 */

#include "default_config.h"

// 陀螺仪
#define BOARD_FRONT_IS_UP 1                     // 板子正面朝上
#define BOARD_SHORT_SIDE_IS_PARALLEL_TO_PITCH 1 // 板子短边朝下
#define GYROSCOPE_START_UP_DELAY_ENABLED 1      // 开机解算延迟开关
#define GYROSCOPE_START_UP_DELAY 300            // 开机解算延迟量
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.003f   // 零飘修正阈值

#ifdef STM32F427_437xx
#define GYROSCOPE_LSB 16.384f  // 陀螺仪敏感度
#define ACCELERATE_LSB 4096.0f // 加速度计敏感度
#endif
#ifdef STM32F407xx
#define ACCELERATE_LSB 1114.3f // 加速度计敏感度
#define GYROSCOPE_LSB 938.7f   // 陀螺仪敏感度
#define MAGNETIC_LSB 3.33f     // 磁力计敏感度
#endif

// DMA
#define DMA_BUFFER_LENGTH 128 // DMA发送接收长度
