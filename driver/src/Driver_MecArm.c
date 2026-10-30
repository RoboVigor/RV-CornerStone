#include "handle.h"

float max(float a,float b){
    if(a>=b){
        return a;
    }else{
        return b;
    }
}

float min(float a,float b){
    if(a<=b){
        return a;
    }else{
        return b;
    }
}

uint8_t IsInRange(float a,float min,float max){
    if(a<=max && a>=min){
        return 1;
    }else{
        return 0;
    }
}

/**
 * @brief 关节节点初始化
 * @param Arm_Joint_Type* 关节节点句柄
 * @param Joint_Actuator_Type_e 电机类型
 * @param *motor 电机句柄
 * @param min_angle 关节允许最小角度
 * @param max_angle 关节允许最大角度
 * @param max_speed 关节允许最大运动速度
 * @param max_torque 关节允许最大力矩
 * @param Set_TargetFunc 函数句柄 设置期望状态 (角度, 速度, 前馈力矩, 刚度/阻尼系数)
 * @param Get_AngleFunc 函数句柄 读取关节当前角度 (统一输出 rad)
 * @param Get_SpeedFunc 函数句柄 读取关节当前转速 (统一输出 rad/s)
 * @param Is_OnlineFunc 函数句柄 检查关节是否在线/正常
 */
void Arm_Joint_Init(Arm_Joint_Type *ArmJoint,
                    Joint_Actuator_Type_e MotorType,
                    void *motor,
                    float min_angle,float max_angle,
                    float max_speed,float max_torque,
                    Set_TargetFunc Set_Target_cb,
                    Get_AngleFunc Get_Angle_cb,
                    Get_SpeedFunc Get_Speed_cb,
                    Is_OnlineFunc Is_Online_cb){
    ArmJoint->type=MotorType;
    ArmJoint->motor_handle=motor;
    ArmJoint->min_angle=min_angle;
    ArmJoint->max_angle=max_angle;
    ArmJoint->max_speed=max_speed;
    ArmJoint->max_torque=max_torque;
    ArmJoint->Set_Target=Set_Target_cb;
    ArmJoint->Get_Angle=Get_Angle_cb;
    ArmJoint->Get_Speed=Get_Speed_cb;
    ArmJoint->Is_Online=Is_Online_cb;
}

/**
 * @brief 机械臂初始化
 * @param Manipulator_Type* 机械臂句柄
 * @param Arm_Joint_Type* 关节节点句柄0
 * @param Arm_Joint_Type* 关节节点句柄1
 * @param Arm_Joint_Type* 关节节点句柄2
 * @param Arm_Joint_Type* 关节节点句柄3
 * @param Arm_Joint_Type* 关节节点句柄4
 * @param joint_count 关节节点数量
 * @param is_enabled 机械臂使能标志位
 */
void MecArm_Init(Manipulator_Type *arm,
    Arm_Joint_Type* joint0,
    Arm_Joint_Type* joint1,
    Arm_Joint_Type* joint2,
    Arm_Joint_Type* joint3,
    Arm_Joint_Type* joint4,
    uint8_t joint_count,uint8_t is_enabled){
    arm->joints[0]=joint0;
    arm->joints[1]=joint1;
    arm->joints[2]=joint2;
    arm->joints[3]=joint3;
    arm->joints[4]=joint4;
    arm->joint_count=joint_count;
    arm->is_enabled=is_enabled;
}

//TODO:解算未完成
void MecArm_Calculate(Manipulator_Type *arm,
    float polarTheta,float r,float z,
    float pitch,float yaw){

    float y1,z1,cos_theta1,sin_theta1,theta1;
    float y2,z2,cos_theta2,sin_theta2,theta2;
    float y3,z3,theta3;
    float k1,k2;

    y3=r-ARM_L3*cos(pitch);
    z3=z+ARM_L3*sin(pitch);
    
    cos_theta2=(y3*y3+z3*z3-ARM_L1*ARM_L1-ARM_L2*ARM_L2)/(2*ARM_L1*ARM_L2);
    cos_theta2=max(min(cos_theta2, 1.0), -1.0);
    sin_theta2=sqrt(1.0f - cos_theta2*cos_theta2);
    theta2=atan2(sin_theta2,cos_theta2);

    k1=ARM_L1+ARM_L2*cos(theta2);
    k2=ARM_L2*sin(theta2);
    sin_theta1=(k1*y3-k2*z3)/(k1*k1+k2*k2);
    cos_theta1=(k2*y3+k1*z3)/(k1*k1+k2*k2);
    theta1=atan2(sin_theta1,cos_theta1);
    
    theta3=pitch+PI/2-theta2-theta1;

    if(IsInRange(theta1,arm->joints[1]->min_angle,arm->joints[1]->max_angle) &&
        IsInRange(theta2,arm->joints[2]->min_angle,arm->joints[2]->max_angle) &&
        IsInRange(theta3,arm->joints[3]->min_angle,arm->joints[3]->max_angle)){
            arm->joint_target[0]=polarTheta;
            arm->joint_target[1]=theta1;
            arm->joint_target[2]=theta2;
            arm->joint_target[3]=theta3;
    }
}

//TODO: 机械臂控制未完成
void MecArm_Control(Manipulator_Type *arm){
    //临时测试
    for (uint8_t i = 0; i < 4; i++)
    {
#if GIMBAL_TUNING_ENABLED
        if(i==0) continue; // joint0 belongs solely to the guarded tuning state machine
#endif
        Joint_Command_t *joint_command=&(arm->joints[i]->joint_command);
        if(i==3){
            joint_command->kd=0;
            joint_command->kp=0;
            joint_command->tau_ff=0;
            joint_command->vel_des=0;
            joint_command->pos_des=arm->joint_target[i];
            joint_command->pos_pid=&Motor6020_MecArm_3_PosPID;
            joint_command->speed_pid=&Motor6020_MecArm_3_SpeedPID;
        }else if(i==0){
            joint_command->kd=0;
            joint_command->kp=0;
            joint_command->tau_ff=0;
            joint_command->vel_des=0;
            joint_command->pos_des=arm->joint_target[i];
            joint_command->pos_pid=&Motor6020_MecArm_Group_0_PosPID;
            joint_command->speed_pid=&Motor6020_MecArm_Group_0_SpeedPID;
        }else{
			joint_command->kd=0;
            joint_command->tau_ff=0;
            joint_command->vel_des=0;
            joint_command->pos_pid=NULL;
            joint_command->speed_pid=NULL;
            joint_command->pos_des=arm->joint_target[i];
            joint_command->kp=0.2f;
        }
        arm->joints[i]->Set_Target(arm->joints[i],joint_command);
    }
}