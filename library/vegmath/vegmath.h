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

#endif
