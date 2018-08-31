/**
 * 临时存放原main()里的loop程序
 * TODO: 在 #16 合并前移走这些代码
 */

int ArmatureRotateSpeed[4], Buffer[4];
int isDoublePID = 0; // 0为单速度pid

float LastYawAngleFeed = 0;
float YawAngleFeedOffset = 0;
float YawAngleFeedThreshold = 0.003;
float YawAngleFeedDiff = 0;
float YawAngleFeedOffsetSample = 0;
float YawAngleFeedOffsetSampleCounter = 0;
int PanPIDMode = 1;

void mainTask(void) {

  if (DbusData.switchRight == 2) {
    Can_Set_CM_Speed(CAN1, 0, 0, 0, 0);
    return;
  }
  Motion_Update();
  if (ABS(DbusData.ch1) < 5) {
    PanPIDMode = 2;
  } else {
    PanPIDMode = 1;
  }

  if (Euler_Angle.Yaw - YawAngleFeedOffset - LastYawAngleFeed > 300) {
    YawAngleFeedOffset += 360;
  } else if (Euler_Angle.Yaw - YawAngleFeedOffset - LastYawAngleFeed < -300) {
    YawAngleFeedOffset -= 360;
  }

  YawAngleFeed = Euler_Angle.Yaw - YawAngleFeedOffset;

  YawAngleFeedDiff = YawAngleFeed - LastYawAngleFeed;

  if (ABS(DbusData.ch1) < 10 && ABS(DbusData.ch3) < 10 &&
      ABS(DbusData.ch4) < 10)
  // if(1)
  {
    if (ABS(YawAngleFeedDiff) < YawAngleFeedThreshold) {
      YawAngleFeedOffset += YawAngleFeedDiff;
      YawAngleFeed = LastYawAngleFeed;
      PanPIDMode = 0;
      if (YawAngleFeedOffsetSampleCounter < 100) {
        YawAngleFeedOffsetSample += YawAngleFeedDiff;
        YawAngleFeedOffsetSampleCounter += 1;
      }
    } else {
      LastYawAngleFeed = YawAngleFeed;
    }
  } else {
    if (DbusData.switchRight == 1) {
      YawAngleFeedOffset +=
          YawAngleFeedOffsetSample / YawAngleFeedOffsetSampleCounter;
    }
    YawAngleFeed = Euler_Angle.Yaw - YawAngleFeedOffset;
    YawAngleFeedDiff = YawAngleFeed - LastYawAngleFeed;
    LastYawAngleFeed = YawAngleFeed;
  }

  if (DbusData.switchLeft == 1) //摄像头朝向丝杆 功能:移动
  {

    TIM_SetCompare1(TIM4, 23);
    //			-------------------------------------------------------------------------------------------------------------------------------

    Can_Update_Encoder_Data(&Hook_Encoder, Motor_Feedback.motor205Angle);

    HookSpeedPID(&Hook_SpeedPID, 0, Motor_Feedback.motor205Speed);

    Can_Set_HookArmour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

    //			-------------------------------------------------------------------------------------------------------------------------------

    GetXYWSpeed(FORWARD, PanPIDMode);

    MecanumCalculation(Buffer);

    LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

    //速度pid
    PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0],
                Motor_Feedback.motor201Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1],
                Motor_Feedback.motor202Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2],
                Motor_Feedback.motor203Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3],
                Motor_Feedback.motor204Speed * 2 * 3.14 /
                    60); //都是rad/s 反馈转子转速

    Can_Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout,
                 CM4PID.PIDout); //得到电流发送给电调
  }
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if (DbusData.switchLeft == 3) {
    TIM_SetCompare1(TIM4, 5);

    Can_Update_Encoder_Data(&Hook_Encoder, Motor_Feedback.motor205Angle);

    HookFeedAngle = Hook_Encoder.ecdAngle;

    HookSpeedPID(&Hook_SpeedPID, DbusData.ch2,
                 Motor_Feedback.motor205Speed);

    Can_Set_HookArmour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

    //=======移动=============================================================================

    GetXYWSpeed(BACKWARD, PanPIDMode);

    MecanumCalculation(Buffer);

    LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

    //速度pid
    PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0],
                Motor_Feedback.motor201Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1],
                Motor_Feedback.motor202Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2],
                Motor_Feedback.motor203Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3],
                Motor_Feedback.motor204Speed * 2 * 3.14 /
                    60); //都是rad/s 反馈转子转速

    Can_Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout,
                 CM4PID.PIDout); //得到电流发送给电调
  }
}


  // // 下面还有一些初始化的...
  // TIM4_PWM_Init();
  // MPU6500_IntConfiguration();

  // delay_ms(3000);
  // Hook_Encoder.ecdBias = Motor_Feedback.motor205Angle;
  // Can_Get_Encoder_Bias(&Hook_Encoder);
  // Armour1_Encoder.ecdBias = Motor_Feedback.motor206Angle;
  // Armour2_Encoder.ecdBias = Motor_Feedback.motor207Angle;
  // Can_Get_Encoder_Bias(&Armour1_Encoder);
  // Can_Get_Encoder_Bias(&Armour2_Encoder);

  // MPU6500_Initialize();

  // delay_ms(3000);
  // MPU6500_EnableInt();

  // //  mpu6500 offset
  // mpu6500_data.gx_offset = (short)0.406409323;
  // mpu6500_data.gy_offset = (short)-2.91589163;
  // mpu6500_data.gz_offset = (short)15.75639464;

  // // 目标值初始化
  // HookTargetAngleInit();
  // PanTargetAngleInit();
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