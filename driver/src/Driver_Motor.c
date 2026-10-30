#include "Driver_Motor.h"
#include "handle.h"
#include "macro.h"

#define PI 3.1415926

void Motor_Init(Motor_Type *motor, MotorProtocol_Type protocol, int8_t direction, Motor_RMDevice_Type type, float reductionRate, int8_t angleEnabled, int8_t inputEnabled) {
    motor->positionBias  = -1; // -1为未赋值状态
    motor->angleBias     = -1; // -1为未赋值状态
    motor->lastPosition  = -1; // -1为未赋值状态
    motor->protocol      = protocol;
    motor->direction     = direction;
    motor->type          = type;
    motor->reductionRate = reductionRate;
    motor->angleEnabled  = angleEnabled;
    motor->inputEnabled  = inputEnabled;
}

void Motor_Update(Motor_Type *motor, uint8_t data[8], uint8_t type) {
    int16_t position;
    int16_t speed;
    int16_t actualCurrent;
    int16_t temperature;
    float angle;
    switch (type)
    {
    //TODO:3508没有适配direction,2006电机解析还没写
    //DJI_3508
    case 1:
        position      = data[0] << 8 | data[1];
        speed         = data[2] << 8 | data[3];
        actualCurrent = data[4] << 8 | data[5];
        temperature   = data[6];
        angle         = 0;
        motor->updatedAt      = xTaskGetTickCount();
        motor->online         = 1;

        // 更新转子信息
        motor->position      = position;
        motor->speed         = speed;
        motor->actualCurrent = (actualCurrent / 16384.0) * 20;
        motor->torque        = MAX(24 * motor->actualCurrent, 0) / (speed * PI / 30.0 / motor->reductionRate + 2);
        motor->temperature   = temperature;

        //如果启用了连续角度计算
        if (motor->angleEnabled) {
            if (motor->lastPosition != -1) {
                //两次编码器的反馈值差别太大,表示圈数发生了改变
                motor->positionDiff = motor->position - motor->lastPosition;
                if (motor->positionDiff < -4200) {
                    motor->round++;
                } else if (motor->positionDiff > 4200) {
                    motor->round--;
                }
            }
            //计算得到连续角度值,范围正负无穷大
            angle = (motor->position / 8192.0f * 360 + motor->round * 360) / motor->reductionRate;

            // 更新连续角度初始位置
            if (motor->angleBiasInit == 0) {
                if (motor->positionBias != -1) {
                    motor->angleBias = motor->positionBias / 8192.0f * 360; // 向下兼容
                } else {
                    motor->angleBias = angle; // 将当前位置设为
                }
                motor->angleBiasInit = 1;
            }
            motor->angle = angle - motor->angleBias;

            // 更新前转子位置
            motor->lastPosition = motor->position;
        }
        break;

    //LK
    case 2:
        temperature = data[1];
        actualCurrent = data[2] | data[3] << 8;
        speed = data[4] | data[5] << 8;
        position = data[6] | data[7] << 8;
        angle         = 0;
        motor->updatedAt      = xTaskGetTickCount();
        motor->online         = 1;

        // 更新转子信息
        motor->position      = position;
        motor->speed         = speed / 60.0; //LK是dps输出，统一到rpm；
        motor->actualCurrent = actualCurrent; //MS系列没有电流反馈，以输出功率替代；
        motor->torque        = 0; //无法计算
        motor->temperature   = temperature;

        if (motor->angleEnabled) {
            if (motor->lastPosition != -1) {
                //两次编码器的反馈值差别太大,表示圈数发生了改变
                motor->positionDiff = motor->position - motor->lastPosition;
                if (motor->positionDiff < -32768) {
                    motor->round++;
                } else if (motor->positionDiff > 32768) {
                    motor->round--;
                }
            }
            //计算得到连续角度值,范围正负无穷大
            angle = (motor->position / 65536.0f * 360 + motor->round * 360) / motor->reductionRate;

            // 更新连续角度初始位置
            if (motor->angleBiasInit == 0) {
                if (motor->positionBias != -1) {
                    motor->angleBias = motor->positionBias / 65536.0f * 360; // 向下兼容
                } else {
                    motor->angleBias = angle; // 将当前位置设为
                }
                motor->angleBiasInit = 1;
            }
            motor->angle = angle - motor->angleBias;

            // 更新前转子位置
            motor->lastPosition = motor->position;
        }
    break;

    //DJI_6020_Current
    case 3:
        position      = data[0] << 8 | data[1];
        speed         = data[2] << 8 | data[3];
        actualCurrent = data[4] << 8 | data[5];
        temperature   = data[6];
        angle         = 0;
        motor->updatedAt      = xTaskGetTickCount();
        motor->online         = 1;

        // 更新转子信息
        motor->position      = position;
        motor->speed         = speed * motor->direction / motor->reductionRate;
        motor->actualCurrent = (actualCurrent / 16384.0) * 3.0 * motor->direction;
        // 基于官方 I-T 曲线连续性的 GM6020 转矩计算
        float calc_current = motor->actualCurrent; // 已经带符号 (+/ -)

        if (calc_current > 0.325f) { // 超过正向克服摩擦电流门槛 (0.241 / 0.741 ≈ 0.325A)
            motor->torque = (0.741f * calc_current - 0.241f) * motor->reductionRate;
        } else if (calc_current < -0.325f) { // 超过反向克服摩擦电流门槛
            motor->torque = (0.741f * calc_current + 0.241f) * motor->reductionRate;
        } else {
            // 在静摩擦死区范围内，输出力矩归零（平滑过原点，不发生突变）
            motor->torque = 0.0f;
        }
        motor->temperature   = temperature;

        //如果启用了连续角度计算
        if (motor->angleEnabled) {
            if (motor->lastPosition != -1) {
                //两次编码器的反馈值差别太大,表示圈数发生了改变
                motor->positionDiff = motor->position - motor->lastPosition;
                if (motor->positionDiff < -4096) {
                    motor->round++;
                } else if (motor->positionDiff > 4096) {
                    motor->round--;
                }
            }
            //计算得到连续角度值,范围正负无穷大
            angle = motor->direction * (motor->position/8192.0f*360.0f + motor->round*360.0f) / motor->reductionRate;

            // 更新连续角度初始位置
            if (motor->angleBiasInit == 0) {
                if (motor->positionBias != -1) {
                    motor->angleBias = motor->positionBias / 8192.0f * 360; // 向下兼容
                } else {
                    motor->angleBias = angle; // 将当前位置设为
                }
                motor->angleBiasInit = 1;
            }
            motor->angle = angle - motor->angleBias;

            // 更新前转子位置
            motor->lastPosition = motor->position;
        }
        break;
    default:
    break;
}
}

void Motor_Set_Angle_Bias(Motor_Type *motor, float angleBias) {
    motor->angleBias     = angleBias;
    motor->angleBiasInit = 1;
    motor->lastPosition  = -1;
}

/**
 * @brief 6020设置目标角度
 * @param *self Arm_Joint_Type句柄
 * @param *joint_command 控制指令集
 * @return void
 */
void Motor6020_Set_Target(Arm_Joint_Type *self, Joint_Command_t *joint_command){
    Motor_Type* motor=(Motor_Type*)self->motor_handle;
    float angle_feedback=self->Get_Angle(self);
    float target_speed=PID_Calculate(joint_command->pos_pid,joint_command->pos_des,angle_feedback);
    float speed_feedback=self->Get_Speed(self);
    float target_current=PID_Calculate(joint_command->speed_pid,target_speed,speed_feedback);
    motor->input=(int16_t)(target_current*motor->direction);
}

/**
 * @brief 读取6020连续角度
 * @param *self Arm_Joint_Type句柄
 * @return  6020连续角度
 */
float Motor6020_Get_Angle(Arm_Joint_Type *self){
    Motor_Type* motor=(Motor_Type*)self->motor_handle;
    float angle_rad=(motor->angle)/180.0f*PI;
    return angle_rad;
}

/**
 * @brief 读取6020速度
 * @param *self Arm_Joint_Type句柄
 * @return  6020当前速度
 */
float Motor6020_Get_Speed(Arm_Joint_Type *self){
    Motor_Type* motor=(Motor_Type*)self->motor_handle;
    float speed_rad=((float)(motor->speed))*2.0f*PI/60.0f;
    return speed_rad;
}

/**
 * @brief 读取6020在线状态
 * @param *self Arm_Joint_Type句柄
 * @return  6020当前在线状态
 */
uint8_t Motor6020_Is_Online(Arm_Joint_Type *self){
    Motor_Type* motor=(Motor_Type*)self->motor_handle;
    return motor->online;
}

/**
 * @brief 6020电机组设置目标角度，机械臂关节0使用了两个电机
 * @param Arm_Joint_Type* 电机关节结构体句柄
 * @param Joint_Command_t * 电机控制结构体
 */
void Motor6020_Group_Set_Target(Arm_Joint_Type *self, Joint_Command_t *joint_command){
    Motor_Type *motor0,*motor1;
    PID_Type *pid_pos;
    PID_Type *pid_speed;
    pid_pos=joint_command->pos_pid;
    pid_speed=joint_command->speed_pid;
    motor0=(Motor_Type*)(((Motor_Group_Type*)(self->motor_handle))->motor0);
    motor1=(Motor_Type*)(((Motor_Group_Type*)(self->motor_handle))->motor1);
    float angle_feedback=Motor6020_Group_Get_Angle(self);
    float target_speed=PID_Calculate(pid_pos,joint_command->pos_des,angle_feedback);
    float speed_feedback=Motor6020_Group_Get_Speed(self);
    float target_current=PID_Calculate(pid_speed,target_speed,speed_feedback);
    motor0->input=(int16_t)(target_current*motor0->direction/2.0f);
    motor1->input=(int16_t)(target_current*motor1->direction/2.0f);
}

/**
 * @brief 读取6020电机组连续角度
 * @param *self Arm_Joint_Type句柄
 * @return  6020电机组连续角度
 */
float Motor6020_Group_Get_Angle(Arm_Joint_Type *self){
    Motor_Type* motor0=(Motor_Type*)(((Motor_Group_Type*)(self->motor_handle))->motor0);
    float angle_rad=((motor0->angle)/180.0f*PI);
    return angle_rad;
}

/**
 * @brief 读取6020电机组速度
 * @param *self Arm_Joint_Type句柄
 * @return  6020电机组当前速度
 */
float Motor6020_Group_Get_Speed(Arm_Joint_Type *self){
    Motor_Type* motor0=(Motor_Type*)(((Motor_Group_Type*)(self->motor_handle))->motor0);
    float speed_rad=(((float)(motor0->speed))*2.0f*PI/60.0f);
    return speed_rad;
}

/**
 * @brief 读取6020电机组在线状态
 * @param *self Arm_Joint_Type句柄
 * @return  6020电机组当前在线状态
 */
uint8_t Motor6020_Group_Is_Online(Arm_Joint_Type *self){
    Motor_Type* motor0=(Motor_Type*)(((Motor_Group_Type*)(self->motor_handle))->motor0);
    Motor_Type* motor1=(Motor_Type*)(((Motor_Group_Type*)(self->motor_handle))->motor1);
    return (motor0->online & motor1->online);
}