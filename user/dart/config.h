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
#define GYROSCOPE_FREQ 0x03    // 陀螺仪频率 0x09/100Hz 0x04/200Hz 0x03/250Hz 0x01/500Hz 0x00/1000Hz
#define SAMPLE_FREQ 250.0f     // 结算采样频率
#endif
#ifdef STM32F40_41xxx
#define ACCELERATE_LSB 1114.3f // 加速度计敏感度
#define GYROSCOPE_LSB 938.7f   // 陀螺仪敏感度
#define MAGNETIC_LSB 3.33f     // 磁力计敏感度
#define ACCELERATE_FREQ 0x09   // 加速度计频率 0x08/100Hz 0x09/200Hz 0x0A/400Hz 0x0B/800Hz
#define GYROSCOPE_FREQ 0x04    // 陀螺仪频率 0x05/100Hz 0x04/200Hz 0x03/400Hz 0x02/1000Hz
#define SAMPLE_FREQ 200.0f     // 结算采样频率
#endif

// DMA
#define DMA_BUFFER_LENGTH 128 // DMA发送接收长度
