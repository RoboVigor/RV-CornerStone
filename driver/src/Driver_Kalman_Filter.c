#include "Driver_Kalman_Filter.h"

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
                 arm_matrix_instance_f32 *K) {
    // 记录状态向量维度和传感器观测向量维度
    uint16_t XRows = kalman->X->numRows;
    uint16_t YRows = kalman->Y->numRows;

    // 卡尔曼滤波器基本参数初始化
    kalman->X = X;
    kalman->A = A;
    kalman->P = P;
    kalman->Q = Q;
    kalman->R = R;
    kalman->H = H;
    kalman->V = V;
    kalman->K = K;

    // 卡尔曼滤波器计算用缓存变量初始化
    kalman->tempMats.AT = (float32_t *) pvPortMalloc((XRows * XRows) * sizeof(float32_t));
    kalman->tempMats.HT = (float32_t *) pvPortMalloc((1 * YRows) * sizeof(float32_t));

    kalman->tempMats.Identity = (float32_t *) pvPortMalloc((XRows * XRows) * sizeof(float32_t));

    kalman->tempMats.Tmp_1 = (float32_t *) pvPortMalloc((XRows * XRows) * sizeof(float32_t));
    kalman->tempMats.Tmp_2 = (float32_t *) pvPortMalloc((XRows * YRows) * sizeof(float32_t));
    kalman->tempMats.Tmp_3 = (float32_t *) pvPortMalloc((YRows * XRows) * sizeof(float32_t));
    kalman->tempMats.Tmp_4 = (float32_t *) pvPortMalloc((YRows * YRows) * sizeof(float32_t));
    kalman->tempMats.Tmp_5 = (float32_t *) pvPortMalloc((YRows * 1) * sizeof(float32_t));
    kalman->tempMats.Tmp_6 = (float32_t *) pvPortMalloc((XRows * 1) * sizeof(float32_t));

    // 定义单位矩阵
    uint8_t i = 0;
    for (; i < XRows * XRows; i++) {
        kalman->tempMats.Identity->pData[i] = 1;
    }
}

/**
 * @brief Kalman_Update   状态更新卡尔曼滤波函数
 * @param kalman  滤波器结构体句柄
 * @param Y_New 传感器观测向量
 */
void Kalman_Update(Kalman_Type *kalman, arm_matrix_instance_f32 *Y_New) {
    // 1.计算本次预测状态：X(k) = A * X(k-1)
    arm_mat_mult_f32(kalman->A, kalman->X, kalman->X);

    // 2.计算本次协方差预测：P(k) = A * P(k-1) * At + Q
    arm_mat_trans_f32(kalman->A, &kalman->tempMats.AT);
    arm_mat_mult_f32(kalman->A, kalman->P, &kalman->tempMats.Tmp_1);
    arm_mat_mult_f32(&kalman->tempMats.Tmp_1, &kalman->tempMats.AT, &kalman->tempMats.Tmp_1);

    arm_mat_add_f32(&kalman->tempMats.Tmp_1, kalman->Q, kalman->P);

    // 3.计算卡尔波增益: K(k) = P(k) * Ht * [H * P(k) * Ht + R]exp(-1)
    arm_mat_trans_f32(kalman->H, &kalman->tempMats.HT);
    arm_mat_mult_f32(kalman->P, &kalman->tempMats.HT, &kalman->tempMats.Tmp_2);

    arm_mat_mult_f32(kalman->H, kalman->P, &kalman->tempMats.Tmp_3);
    arm_mat_mult_f32(&kalman->tempMats.Tmp_3, &kalman->tempMats.HT, &kalman->tempMats.Tmp_4);
    arm_mat_add_f32(&kalman->tempMats.Tmp_4, kalman->R, &kalman->tempMats.Tmp_4);
    arm_mat_inverse_f32(&kalman->tempMats.Tmp_4, &kalman->tempMats.Tmp_4);
    arm_mat_mult_f32(&kalman->tempMats.Tmp_2, &kalman->tempMats.Tmp_4, kalman->K);

    // 4.更新本次最优状态估计：X(k) = X(k) + K(k) * [Y(k) - H * X(k) - V]
    arm_mat_mult_f32(kalman->H, kalman->X, &kalman->tempMats.Tmp_5);
    arm_mat_sub_f32(Y_New, &kalman->tempMats.Tmp_5, &kalman->tempMats.Tmp_5);
    arm_mat_sub_f32(&kalman->tempMats.Tmp_5, kalman->V, &kalman->tempMats.Tmp_5);
    arm_mat_mult_f32(kalman->K, &kalman->tempMats.Tmp_5, &kalman->tempMats.Tmp_6);
    arm_mat_add_f32(kalman->X, &kalman->tempMats.Tmp_6, kalman->X);

    // 5.更新协方差矩阵：P(k) = [1 - K(k) * H] * P(k)
    arm_mat_mult_f32(kalman->K, kalman->H, &kalman->tempMats.Tmp_1);
    arm_mat_sub_f32(&kalman->tempMats.Identity, &kalman->tempMats.Tmp_1, &kalman->tempMats.Tmp_1);
    arm_mat_mult_f32(&kalman->tempMats.Tmp_1, kalman->P, kalman->P);
}