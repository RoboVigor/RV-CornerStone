#ifndef __DRIVER_KALMAN_FILTER_H
#define __DRIVER_KALMAN_FILTER_H

#include "arm_math.h"

// 卡尔曼滤波临时变量参数结构体
typedef struct {
    arm_matrix_instance_f32 *AT; // 转置状态转移矩阵: XRows * XRows
    arm_matrix_instance_f32 *HT; // 转置观测矩阵: 1 * YRows

    arm_matrix_instance_f32 *Identity; // 单位矩阵: XRows * XRows

    arm_matrix_instance_f32 *Tmp_1; // 临时矩阵 1: XRows * XRows
    arm_matrix_instance_f32 *Tmp_2; // 临时矩阵 2: XRows * YRows
    arm_matrix_instance_f32 *Tmp_3; // 临时矩阵 3: YRows * XRows
    arm_matrix_instance_f32 *Tmp_4; // 临时矩阵 4: YRows * YRows
    arm_matrix_instance_f32 *Tmp_5; // 临时矩阵 5: YRows * 1
    arm_matrix_instance_f32 *Tmp_6; // 临时矩阵 6: XRows * 1
} Kalman_Temp_Mat_Type;

// 卡尔曼滤波主要参数结构体
typedef struct {
    arm_matrix_instance_f32 *X; // 状态向量

    arm_matrix_instance_f32 *Y; // 传感器观测向量

    arm_matrix_instance_f32 *A; // 状态转移矩阵
    arm_matrix_instance_f32 *P; // 协方差矩阵

    arm_matrix_instance_f32 *Q; // 过程误差矩阵
    arm_matrix_instance_f32 *R; // 测量误差矩阵

    arm_matrix_instance_f32 *H; // 观测矩阵

    arm_matrix_instance_f32 *V; // 观测噪声向量

    arm_matrix_instance_f32 *K; // 卡尔曼增益

    Kalman_Temp_Mat_Type tempMats; // 计算用临时矩阵变量结构体
} Kalman_Type;

/**
 * @brief Kalman_Init   初始化卡尔曼滤波器
 * @param kalman  滤波器结构体句柄
 * @param X 初始状态向量
 * @param A 状态转移矩阵
 * @param P 初始协方差矩阵
 * @param Q 过程误差矩阵
 * @param R 测量误差矩阵
 * @param H 观测矩阵
 * @param V 观测噪声向量
 * @param K 初始卡尔曼增益矩阵
 */
void Kalman_Init(Kalman_Type *            kalman,
                 arm_matrix_instance_f32 *X,
                 arm_matrix_instance_f32 *A,
                 arm_matrix_instance_f32 *P,
                 arm_matrix_instance_f32 *Q,
                 arm_matrix_instance_f32 *R,
                 arm_matrix_instance_f32 *H,
                 arm_matrix_instance_f32 *V,
                 arm_matrix_instance_f32 *K);

/**
 * @brief Kalman_Update   状态更新卡尔曼滤波函数
 * @param kalman  滤波器结构体句柄
 * @param Y_New 传感器观测向量
 */
void Kalman_Update(Kalman_Type *kalman, arm_matrix_instance_f32 *Y_New);

#endif