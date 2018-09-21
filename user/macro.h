/**
 * @brief 宏定义
 * @note 使用宏函数时参数必须是单个变量/值而不能是包含任何计算的表达式
 */

// clang-format off

#define FORWARD 1
#define BACKWARD -1

#define ABS(x) ((x) >= 0 ? (x) : -(x))

#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define MIN(a,b) ((a) < (b) ? (a) : (b))

/**
 * @brief 限流
 * @note
 *   REGULATE丑得不行; LIMIT又有极限的歧义; RSTR(restrict)又好像不太容易看懂
 *   喵就完事了
 */
#define MIAO(val, min, max) (val) = ((val) > (min) ? (val) : ((val) = (min))) < (max) ? (val) : (max)

// #define WANG(val, min, max) 