/**
 * @brief 宏定义
 * @note 使用宏函数时参数必须是单个变量/值而不能是包含任何计算的表达式
 */

#ifndef __MACRO_H
#define __MACRO_H

// clang-format off

#define FORWARD 1
#define BACKWARD -1

#define ABS(x) ((x) >= 0 ? (x) : -(x))

#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define MIN(a,b) ((a) < (b) ? (a) : (b))

#define LASER_ON GPIO_SetBits(GPIOG, GPIO_Pin_13) // 激光开启
#define LASER_OFF GPIO_ResetBits(GPIOG, GPIO_Pin_13) // 激光关闭

/**
 * @brief 限流
 * @note
 *   REGULATE丑得不行; LIMIT又有极限的歧义; RSTR(restrict)又好像不太容易看懂
 *   所以喵就完事了
 *   因为猫攻击力更高,所以MIAO可以直接对变量进行赋值,而WANG只能返回限流后的值
 */
#define MIAO(val, min, max)                                    \
  (val) = ((val) > (min) ? (val)                               \
                         : ((val) = (min))) < (max) ? (val)    \
                                                    : (max)
#define WANG(val, min, max)                                    \
  ((val) > (min) ? (val)                                       \
                 : (min)) < (max) ? ((val) > (min) ? (val)     \
                                                   : (min))    \
                                  : (max)

/**
 * @brief 根据遥控器左上的开关返回一个值
 * @note
 *   只需要调用该函数就可以动态赋值,实现一次烧写测试三个数值等效果
 */
#define CHOOSE(a, b, c) remoteData.switchLeft == 1?(a):(remoteData.switchLeft == 3?(b):(c))


/**
 * @brief 斜坡函数 (一次函数)
 * @note
 *   y=a+k(b-a) 输入起点a,终点b,进度k,返回当前位置.进度k会自动限流至0-1.
 */
#define RAMP(start, stop, progress) (WANG(progress, 0, 1)) * ((stop) - (start)) + (start)

#endif