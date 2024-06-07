/**
 * @brief 默认机器人参数
 */

// 总线设置
#define CAN_DEVICE_ID 0x501 // 发送数据时CAN标准帧中的ID 0x501-0x503
#define CAN_TIMEOUT 100     // 电机超过该时间(ms)无数据将报警

// 调试
#define DEBUG_ENABLED 0          // 调试开关
#define SERIAL_DEBUG_PORT USART6 // 串口调试端口
#undef SERIAL_DEBUG_PORT         // 使用ITM调试

// 陀螺仪
#define BOARD_FRONT_IS_UP 1                     // 板子正面朝上
#define BOARD_SHORT_SIDE_IS_PARALLEL_TO_PITCH 1 // 板子短边朝下
#define GYROSCOPE_START_UP_DELAY_ENABLED 1      // 开机解算延迟开关
#define GYROSCOPE_START_UP_DELAY 1700           // 开机解算延迟量
#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.003f   // 零飘修正阈值
#define GYROSCOPE_LSB 16.384f                   // 陀螺仪敏感度
#define ACCELERATE_LSB 4096.0f                  // 加速度计敏感度
#define GYROSCOPE_FREQ 0x03                     // 陀螺仪频率 0x09/100Hz 0x04/200Hz 0x03/250Hz 0x01/500Hz 0x00/1000Hz
#define SAMPLE_FREQ 250.0f                      // 结算采样频率

// 底盘
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  // 底盘电机减速比
#define CHASSIS_SIZE_K 0.385f               // 测量值, 机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16f // 测量值, 麦克纳姆轮半径的倒数

// DMA
#define DMA_BUFFER_LENGTH 128 // DMA发送接收长度