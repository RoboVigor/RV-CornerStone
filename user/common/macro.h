/**
 * @brief 宏定义
 * @note 使用宏函数时参数必须是单个变量/值而不能是包含任何计算的表达式
 */

#ifndef __MACRO_H
#define __MACRO_H

// clang-format off

// 单位换算
#define RPM2RPS 0.10471975f // 2 * 3.1415926f / 60.0f, round/min->rad/s
#define DPS2RPS 0.01745329f // 3.1415926f / 180.0f,    degree/s ->rad/s
#define RPS2DPS 57.2957804f // 180.0f / 3.1415926f,    rad/s    ->degree/s

// 遥控器开关
#define LEFT_SWITCH_TOP     (remoteData.switchLeft == 1)
#define LEFT_SWITCH_MIDDLE  (remoteData.switchLeft == 3)
#define LEFT_SWITCH_BOTTOM  (remoteData.switchLeft == 2)
#define RIGHT_SWITCH_TOP    (remoteData.switchRight == 1)
#define RIGHT_SWITCH_MIDDLE (remoteData.switchRight == 3)
#define RIGHT_SWITCH_BOTTOM (remoteData.switchRight == 2)

// 数值运算
#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

// 偷懒
#ifdef STM32F427_437xx
#define LASER_ON GPIO_SetBits(GPIOG, GPIO_Pin_13) // 激光开启
#define LASER_OFF GPIO_ResetBits(GPIOG, GPIO_Pin_13) // 激光关闭
#define LASER_TOGGLE GPIO_ToggleBits(GPIOG, GPIO_Pin_13) // 激光闪烁
#endif
#ifdef STM32F40_41xxx
#define LASER_ON GPIO_SetBits(GPIOC, GPIO_Pin_8) // 激光开启
#define LASER_OFF GPIO_ResetBits(GPIOC, GPIO_Pin_8) // 激光关闭
#define LASER_TOGGLE GPIO_ToggleBits(GPIOC, GPIO_Pin_8) // 激光闪烁
#endif

// 好看
#define ESC_ID(x) (x - 0x201)

/**
 * @brief 限流
 * @note
 *   REGULATE丑得不行; LIMIT又有极限的歧义; RSTR(restrict)又好像不太容易看懂
 *   所以喵就完事了
 *   因为猫攻击力更高,所以MIAO可以直接对变量进行赋值,而WANG只能返回限流后的值
 */
#define MIAO(val, min, max)                                    \
  ((val) = ((val) > (min) ? (val)                               \
                         : ((val) = (min))) < (max) ? (val)    \
                                                    : (max))
#define WANG(val, min, max)                                    \
  (((val) > (min) ? (val)                                      \
                 : (min)) < (max) ? ((val) > (min) ? (val)     \
                                                   : (min))    \
                                  : (max))

/**
 * @brief 根据遥控器左上的开关返回一个值
 * @note
 *   只需要调用该函数就可以动态赋值,实现一次烧写测试三个数值等效果
 */
#define CHOOSE(a, b, c) (remoteData.switchLeft == 1?(a):(remoteData.switchLeft == 3?(b):(c)))
#define CHOOSEL(a, b, c) (remoteData.switchLeft == 1?(a):(remoteData.switchLeft == 3?(b):(c)))
#define CHOOSER(a, b, c) (remoteData.switchRight == 1?(a):(remoteData.switchRight == 3?(b):(c)))

/**
 * @brief 斜坡函数 (一次函数)
 * @note
 *   y=a+k(b-a) 输入起点a,终点b,进度k,返回当前位置.进度k会自动限流至0-1.
 */
#define RAMP(start, stop, progress) ((WANG(progress, 0, 1)) * ((stop) - (start)) + (start))

#endif