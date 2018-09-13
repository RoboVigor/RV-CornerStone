/**
 * 临时存放原main()里的loop程序
 * TODO: 在 #16 合并前移走这些代码
 */

int ArmatureRotateSpeed[4], Buffer[4];
int isDoublePID = 0; // 0为单速度pid

float lastYawAngleFeed                = 0;
float yawAngleFeedOffset              = 0;
float yawAngleFeedThreshold           = 0.003;
float yawAngleFeedDiff                = 0;
float yawAngleFeedOffsetSample        = 0;
float yawAngleFeedOffsetSampleCounter = 0;
int   panPIDMode                      = 1;

void mainTask(void) {

    if (DBusData.switchRight == 2) {
        Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
        return;
    }
    Gyroscope_Update_Angle_Data();
    if (ABS(DBusData.ch1) < 5) {
        panPIDMode = 2;
    } else {
        panPIDMode = 1;
    }

    if (EulerAngle.Yaw - yawAngleFeedOffset - lastYawAngleFeed > 300) {
        yawAngleFeedOffset += 360;
    } else if (EulerAngle.Yaw - yawAngleFeedOffset - lastYawAngleFeed < -300) {
        yawAngleFeedOffset -= 360;
    }

    yawAngleFeed = EulerAngle.Yaw - yawAngleFeedOffset;

    yawAngleFeedDiff = yawAngleFeed - lastYawAngleFeed;

    if (ABS(DBusData.ch1) < 10 && ABS(DBusData.ch3) < 10 && ABS(DBusData.ch4) < 10)
    // if(1)
    {
        if (ABS(yawAngleFeedDiff) < yawAngleFeedThreshold) {
            yawAngleFeedOffset += yawAngleFeedDiff;
            yawAngleFeed = lastYawAngleFeed;
            panPIDMode   = 0;
            if (yawAngleFeedOffsetSampleCounter < 100) {
                yawAngleFeedOffsetSample += yawAngleFeedDiff;
                yawAngleFeedOffsetSampleCounter += 1;
            }
        } else {
            lastYawAngleFeed = yawAngleFeed;
        }
    } else {
        if (DBusData.switchRight == 1) {
            yawAngleFeedOffset += yawAngleFeedOffsetSample / yawAngleFeedOffsetSampleCounter;
        }
        yawAngleFeed     = EulerAngle.Yaw - yawAngleFeedOffset;
        yawAngleFeedDiff = yawAngleFeed - lastYawAngleFeed;
        lastYawAngleFeed = yawAngleFeed;
    }

    if (DBusData.switchLeft == 1) //摄像头朝向丝杆 功能:移动
    {

        TIM_SetCompare1(TIM4, 23);
        //			-------------------------------------------------------------------------------------------------------------------------------

        CAN_Update_Encoder_Data(&Hook_Encoder, Motor_Feedback.motor205Angle);

        HookSpeedPID(&Hook_SpeedPID, 0, Motor_Feedback.motor205Speed);

        CAN_Set_HookArmour_Speed(CAN2, Hook_SpeedPID.output, 0, 0, 0);

        //			-------------------------------------------------------------------------------------------------------------------------------

        Chassis_Get_XYW_Speed(FORWARD, panPIDMode);

        Chassis_Update_Mecanum_Data(Buffer);

        Chassis_Limit_Wheel_Speed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

        //速度pid,都是rad/s 反馈转子转速
        PID_Set_Pan_Speed(&CM1PID, ArmatureRotateSpeed[0], Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
        PID_Set_Pan_Speed(&CM2PID, ArmatureRotateSpeed[1], Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
        PID_Set_Pan_Speed(&CM3PID, ArmatureRotateSpeed[2], Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
        PID_Set_Pan_Speed(&CM4PID, ArmatureRotateSpeed[3], Motor_Feedback.Motor_204_Speed * 2 * 3.14 / 60);

        Can_Set_CM_Current(CAN1, CM1PID.output, CM2PID.output, CM3PID.output,
                           CM4PID.output); //得到电流发送给电调
    }
    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (DBusData.switchLeft == 3) {
        TIM_SetCompare1(TIM4, 5);

        CAN_Update_Encoder_Data(&Hook_Encoder, Motor_Feedback.motor205Angle);

        HookFeedAngle = Hook_Encoder.ecdAngle;

        HookSpeedPID(&Hook_SpeedPID, DBusData.ch2, Motor_Feedback.motor205Speed);

        CAN_Set_HookArmour_Speed(CAN2, Hook_SpeedPID.output, 0, 0, 0);

        //=======移动=============================================================================

        Chassis_Get_XYW_Speed(BACKWARD, panPIDMode);

        Chassis_Update_Mecanum_Data(Buffer);

        Chassis_Limit_Wheel_Speed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

        //速度pid
        PID_Set_Pan_Speed(&CM1PID, ArmatureRotateSpeed[0], Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
        PID_Set_Pan_Speed(&CM2PID, ArmatureRotateSpeed[1], Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
        PID_Set_Pan_Speed(&CM3PID, ArmatureRotateSpeed[2], Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
        PID_Set_Pan_Speed(&CM4PID, ArmatureRotateSpeed[3],
                          Motor_Feedback.Motor_204_Speed * 2 * 3.14 / 60); //都是rad/s 反馈转子转速

        Can_Set_CM_Current(CAN1, CM1PID.output, CM2PID.output, CM3PID.output,
                           CM4PID.output); //得到电流发送给电调
    }
}

// // 下面还有一些初始化的...
// TIM4_PWM_Init();
// MPU6500_IntConfiguration();

// delay_ms(3000);
// Hook_Encoder.ecdBias = Motor_Feedback.motor205Angle;
// CAN_Get_Encoder_Bias(&Hook_Encoder);
// Armour1_Encoder.ecdBias = Motor_Feedback.motor206Angle;
// Armour2_Encoder.ecdBias = Motor_Feedback.motor207Angle;
// CAN_Get_Encoder_Bias(&Armour1_Encoder);
// CAN_Get_Encoder_Bias(&Armour2_Encoder);

// MPU6500_Initialize();

// delay_ms(3000);
// MPU6500_EnableInt();

// //  mpu6500 offset
// mpu6500_data.gx_offset = (short)0.406409323;
// mpu6500_data.gy_offset = (short)-2.91589163;
// mpu6500_data.gz_offset = (short)15.75639464;

// // 目标值初始化
// HookTargetAngleInit();
// Chassis_Init_Yaw_Angle();
// ArmourTargetAngleInit();

// PanYawSpeedPIDInit(&YawSpeedPID1, 2, 0, 0);
// PanYawSpeedPIDInit(&YawSpeedPID2, 2, 0, 0);
// // 底盘角度pid初始化
// PANAnglePIDInit(&YawAnglePID, 15, 0, 0);

// // 底盘速度pid初始化
// PANSpeedPIDInit(&CM1PID, 12, 0, 0);
// PANSpeedPIDInit(&CM2PID, 12, 0, 0);
// PANSpeedPIDInit(&CM3PID, 12, 0, 0);
// PANSpeedPIDInit(&CM4PID, 12, 0, 0);

// // 钩子角度速度pid初始化
// HookAnglePIDInit(&Hook_AnglePID, 0, 0, 0);
// HookSpeedPIDInit(&Hook_SpeedPID, 2.1, 0.2, 0);

// // 装甲板两个电机角度速度pid初始化
// ArmourAnglePIDInit(&Armour1_AnglePID, 0, 0, 0);
// ArmourAnglePIDInit(&Armour2_AnglePID, 0, 0, 0);
// ArmourSpeedPIDInit(&Armour1_SpeedPID, 0, 0, 0);
// ArmourSpeedPIDInit(&Armour2_SpeedPID, 0, 0, 0);

// // 装甲板单速度pid
// ArmourSpeedPIDInit(&Armour1_SinglePID_SpeedPID, 1.1, 0, 0);
// ArmourSpeedPIDInit(&Armour2_SinglePID_SpeedPID, 1, 0, 0);