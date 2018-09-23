#include "Driver_PanController.h"

#include "macro.h"

#include "handle.h"

int LastMode = 0;

void Chassis_Init_Yaw_Angle(void) {
    targetYawAngle = 0;
}

void Chassis_Set_Wheel_Speed(int XSpeed, int YSpeed, int WSpeed) {
    ChassisParam.TargetVX = (float) XSpeed / 660 * 5;
    ChassisParam.TargetVY = (float) YSpeed / 660 * 5;
    ChassisParam.TargetWR = (float) WSpeed / 660 * 15;
}

void Chassis_Update_Mecanum_Data(int buffer[4]) {
    float K   = 0.946; //测量值,等于机器人中心点到XY边缘的距离之和
    buffer[0] = 13.16 * ((ChassisParam.TargetVX) - (ChassisParam.TargetVY) + ChassisParam.TargetWR * (-K)) * 19.2; //麦克解算遥控器来的数值
    buffer[1] = 13.16 * ((ChassisParam.TargetVX) + (ChassisParam.TargetVY) + ChassisParam.TargetWR * (-K)) * 19.2; //转子的角速度/19=轮子角速度 (rad/s)
    buffer[2] = -13.16 * (ChassisParam.TargetVX - (ChassisParam.TargetVY) + ChassisParam.TargetWR * K) * 19.2;
    buffer[3] = -13.16 * ((ChassisParam.TargetVX) + (ChassisParam.TargetVY) + ChassisParam.TargetWR * K) * 19.2;
}

void Chassis_Get_XYW_Speed(int dir, int Mode) {
    yawSpeedFeed = mpu6500_data.gz / 16.4;

    // if(ABS(remote.ch1)<5)
    if (Mode == 2) {
        if (LastMode != 2) {
            YawAnglePID.output_I  = 0;
            YawSpeedPID1.output_I = 0;
            YawSpeedPID2.output_I = 0;
            targetYawAngle        = yawAngleFeed;
        }
        PID_Calculate(&YawAnglePID, targetYawAngle, yawAngleFeed);
        PID_Calculate(&YawSpeedPID1, YawAnglePID.output, yawSpeedFeed);

        Chassis_Set_Wheel_Speed(-remote.ch4 * dir, remote.ch3 * dir, YawSpeedPID1.output);
        LastMode = 2;
    } else if (Mode == 1) {
        if (LastMode != 1) {
            YawAnglePID.output_I  = 0;
            YawSpeedPID1.output_I = 0;
            YawSpeedPID2.output_I = 0;
        }

        PID_Calculate(&YawSpeedPID2, -remote.ch1 / 6, yawSpeedFeed);

        Chassis_Set_Wheel_Speed(-remote.ch4 * dir, remote.ch3 * dir, YawSpeedPID2.output);

        LastMode = 1;
    } else if (Mode == 0) {
        Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
    }
}

void Chassis_Limit_Wheel_Speed(int WheelSpeedOrigin[4], int WheelSpeedRes[4], int MaxWheelSpeed) {
    float MaxSpeed = 0;
    float Param    = 0;
    int   index    = 0;

    for (; index < 4; index++) {
        if (ABS(WheelSpeedOrigin[index]) > MaxSpeed) {
            MaxSpeed = ABS(WheelSpeedOrigin[index]);
        }
    }

    if (MaxWheelSpeed < MaxSpeed) {
        Param            = (float) MaxWheelSpeed / MaxSpeed;
        WheelSpeedRes[0] = WheelSpeedOrigin[0] * Param;
        WheelSpeedRes[1] = WheelSpeedOrigin[1] * Param;
        WheelSpeedRes[2] = WheelSpeedOrigin[2] * Param;
        WheelSpeedRes[3] = WheelSpeedOrigin[3] * Param;
    } else {
        WheelSpeedRes[0] = WheelSpeedOrigin[0];
        WheelSpeedRes[1] = WheelSpeedOrigin[1];
        WheelSpeedRes[2] = WheelSpeedOrigin[2];
        WheelSpeedRes[3] = WheelSpeedOrigin[3];
    }
}
