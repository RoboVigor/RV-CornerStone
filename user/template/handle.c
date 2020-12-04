/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

static float32_t Data_Mat_X[3] = {0, 0, 0};
static float32_t Data_Mat_A[9] = {1, (REF_TIME), 0.5 * ((REF_TIME) * (REF_TIME)), 0, 1, (REF_TIME), 0, 0, 1};
static float32_t Data_Mat_P[9] = {10, 1, 1, 1, 1, 1, 1, 1, 1};
static float32_t Data_Mat_Q[9] = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0, 0.0001};
static float32_t Data_Mat_R[1] = {10};
static float32_t Data_Mat_H[3] = {0, 0, 1};
static float32_t Data_Mat_V[1] = {0};
static float32_t Data_Mat_K[6] = {0, 0, 0, 0, 0, 0};

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_LF;
    Can1_Device[ESC_ID(0x202)] = &Motor_LB;
    Can1_Device[ESC_ID(0x203)] = &Motor_RB;
    Can1_Device[ESC_ID(0x204)] = &Motor_RF;

    // 陀螺仪设置静态误差
    Gyroscope_Set_Bias(&ImuData, -4, 3, 1);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);

    // Kalman Filter
    arm_mat_init_f32(&Mat_X, 3, 1, (float32_t *) Data_Mat_X);
    arm_mat_init_f32(&Mat_A, 3, 3, (float32_t *) Data_Mat_A);
    arm_mat_init_f32(&Mat_P, 3, 3, (float32_t *) Data_Mat_P);
    arm_mat_init_f32(&Mat_Q, 3, 3, (float32_t *) Data_Mat_Q);
    arm_mat_init_f32(&Mat_R, 1, 1, (float32_t *) Data_Mat_R);
    arm_mat_init_f32(&Mat_H, 1, 3, (float32_t *) Data_Mat_H);
    arm_mat_init_f32(&Mat_V, 1, 1, (float32_t *) Data_Mat_V);
    arm_mat_init_f32(&Mat_K, 3, 2, (float32_t *) Data_Mat_K);

    Kalman_Init(&Kalman_Test, &Mat_X, &Mat_A, &Mat_P, &Mat_Q, &Mat_R, &Mat_H, &Mat_V, &Mat_K);
}
