#ifndef __DRIVER_MECARM_H
#define __DRIVER_MECARM_H

#include "Driver_PID.h"

#define ARM_L1 280
#define ARM_L2 360.25
#define ARM_L3 97.5

//声明角度控制结构体，解决不同电机方法使用参数不一致的问题
typedef struct{
    float pos_des;                  //目标角度(6020,8010)
    float vel_des;                  //目标速度(8010)
    float tau_ff;                   //前馈力矩(8010,6020计划支持)
    float kp;                       //刚度系数(8010)
    float kd;                       //阻尼系数(8010)
    PID_Type *pos_pid;              //位置PID(6020)
    PID_Type *speed_pid;            //速度PID(6020)
}Joint_Command_t;

// 前向声明
typedef struct Arm_Joint_Type Arm_Joint_Type;

// 1. 设置期望状态 (角度, 速度, 前馈力矩, 刚度/阻尼系数)
typedef void (*Set_TargetFunc)(Arm_Joint_Type *self, Joint_Command_t *joint_command);
// 2. 读取关节当前角度 (统一输出 rad)
typedef float (*Get_AngleFunc)(Arm_Joint_Type *self);
// 3. 读取关节当前转速 (统一输出 rad/s)
typedef float (*Get_SpeedFunc)(Arm_Joint_Type *self);
// 4. 检查关节是否在线/正常
typedef uint8_t (*Is_OnlineFunc)(Arm_Joint_Type *self);

typedef enum {
    JOINT_ACTUATOR_DJI_CAN,      // DJI 3508 / 6020 / 2006
    JOINT_ACTUATOR_UNITREE_UART  // 宇树 RS485 / UART 电机
} Joint_Actuator_Type_e;

// 关节控制模式
typedef enum {
    JOINT_MODE_RELAX,     // 掉电/松开
    JOINT_MODE_POSITION,  // 位置模式 (走双环 PID 或 宇树PD)
    JOINT_MODE_IMPEDANCE  // 阻抗控制模式 (位置+速度+前馈力矩)
} Joint_Control_Mode_e;

// 统一机械臂关节结构体
struct Arm_Joint_Type {
    // 基础信息
    Joint_Actuator_Type_e type;         // 电机硬件类型
    void                 *motor_handle; // 指向实际的 Motor_Type 或 Unitree_Motor_Type
    
    // 物理与安全限制
    float min_angle;                    // 关节软限位 最小角 (rad)
    float max_angle;                    // 关节软限位 最大角 (rad)
    float max_speed;                    // 最大运行角速度 (rad/s)
    float max_torque;                   // 最大力矩限制 (N·m)

    Joint_Command_t     joint_command;  //电机控制结构体
    // 抽象控制接口 (方法多态绑定)
    // 1. 设置期望状态 (角度, 速度, 前馈力矩, 刚度/阻尼系数)
    void  (*Set_Target)(Arm_Joint_Type *self, Joint_Command_t *joint_command);
    // 2. 读取关节当前角度 (统一输出 rad)
    float (*Get_Angle)(Arm_Joint_Type *self);
    // 3. 读取关节当前转速 (统一输出 rad/s)
    float (*Get_Speed)(Arm_Joint_Type *self);
    // 4. 检查关节是否在线/正常
    uint8_t (*Is_Online)(Arm_Joint_Type *self);
};

// 完整机械臂实体
typedef struct {
    Arm_Joint_Type* joints[5];           // 5 自由度机械臂关节数组
    uint8_t        joint_count;         // 关节数量
    uint8_t        is_enabled;          // 机械臂总体使能标志
    
    // 机械臂各关节目标角度
    float joint_target[5];
    // 机械臂末端笛卡尔坐标反馈 (X, Y, Z, Pitch, Yaw)
    float end_effector_pose[5];
} Manipulator_Type;

void Arm_Joint_Init(Arm_Joint_Type *ArmJoint,
                    Joint_Actuator_Type_e MotorType,
                    void *motor,
                    float min_angle,float max_angle,
                    float max_speed,float max_torque,
                    Set_TargetFunc Set_Target_cb,
                    Get_AngleFunc Get_Angle_cb,
                    Get_SpeedFunc Get_Speed_cb,
                    Is_OnlineFunc Is_Online_cb);

void MecArm_Init(Manipulator_Type *arm,
    Arm_Joint_Type* joint0,
    Arm_Joint_Type* joint1,
    Arm_Joint_Type* joint2,
    Arm_Joint_Type* joint3,
    Arm_Joint_Type* joint4,
    uint8_t joint_count,uint8_t is_enabled);

void MecArm_Calculate(Manipulator_Type *arm,
    float polarTheta,float r,float z,
    float pitch,float yaw);

void MecArm_Control(Manipulator_Type *arm);
#endif
