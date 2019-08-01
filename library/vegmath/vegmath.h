#ifndef __VEGMATH_H
#define __VEGMATH_H

/**
 * @brief CORDIC算法
 * @note  I can't write such NB code.
 *        Copy from http://www.dcs.gla.ac.uk/~jhw/cordic/
 */
void cordic(int theta, int *s, int *c);

/**
 * @brief 正弦函数
 * @note  This vegtable code is writting by me.
 * @arg   deg (-Infinity to Infinity)
 */
double vegsin(float deg);

/**
 * @brief 余弦函数
 * @arg   deg (-Infinity to Infinity)
 */
double vegcos(float deg);

/**
 * @brief 低通滤波
 *
 * @param input      输入
 * @param output     输出变量指针, 与return值相同
 * @param sampleFrq  采样频率
 * @param CutFrq     截至频率
 * @return float     返回过滤后的值
 */
float FirstOrderLowPassFilter(float input, float *output, float sampleFrq, float CutFrq);

/**
 * @brief 快速log2
 * @copyright copy from ce123
 *            https:// blog.csdn.net/ce123_zhouwei/article/details/16961615
 * @param x
 * @return int
 */
int FastLog2(int x);

#endif
