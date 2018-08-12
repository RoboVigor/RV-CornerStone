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

  if (DBUS_ReceiveData.switch_right == 2) {
    Set_CM_Speed(CAN1, 0, 0, 0, 0);
    return;
  }
  Motion_Update();
  if (ABS(DBUS_ReceiveData.ch1) < 5) {
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

  if (ABS(DBUS_ReceiveData.ch1) < 10 && ABS(DBUS_ReceiveData.ch3) < 10 &&
      ABS(DBUS_ReceiveData.ch4) < 10)
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
    if (DBUS_ReceiveData.switch_right == 1) {
      YawAngleFeedOffset +=
          YawAngleFeedOffsetSample / YawAngleFeedOffsetSampleCounter;
    }
    YawAngleFeed = Euler_Angle.Yaw - YawAngleFeedOffset;
    YawAngleFeedDiff = YawAngleFeed - LastYawAngleFeed;
    LastYawAngleFeed = YawAngleFeed;
  }

  if (DBUS_ReceiveData.switch_left == 1) //摄像头朝向丝杆 功能:移动
  {

    TIM_SetCompare1(TIM4, 23);
    //			-------------------------------------------------------------------------------------------------------------------------------

    EncoderProcess(&Hook_Encoder, Motor_Feedback.Motor_205_Agree);

    HookSpeedPID(&Hook_SpeedPID, 0, Motor_Feedback.Motor_205_Speed);

    Set_Hook_Armour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

    //			-------------------------------------------------------------------------------------------------------------------------------

    GetXYWSpeed(FORWARD, PanPIDMode);

    MecanumCalculation(Buffer);

    LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

    //速度pid
    PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0],
                Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1],
                Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2],
                Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3],
                Motor_Feedback.Motor_204_Speed * 2 * 3.14 /
                    60); //都是rad/s 反馈转子转速

    Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout,
                 CM4PID.PIDout); //得到电流发送给电调
  }
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if (DBUS_ReceiveData.switch_left == 3) {
    TIM_SetCompare1(TIM4, 5);

    EncoderProcess(&Hook_Encoder, Motor_Feedback.Motor_205_Agree);

    HookFeedAngle = Hook_Encoder.ecd_angle;

    HookSpeedPID(&Hook_SpeedPID, DBUS_ReceiveData.ch2,
                 Motor_Feedback.Motor_205_Speed);

    Set_Hook_Armour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

    //=======移动=============================================================================

    GetXYWSpeed(BACKWARD, PanPIDMode);

    MecanumCalculation(Buffer);

    LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

    //速度pid
    PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0],
                Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1],
                Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2],
                Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3],
                Motor_Feedback.Motor_204_Speed * 2 * 3.14 /
                    60); //都是rad/s 反馈转子转速

    Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout,
                 CM4PID.PIDout); //得到电流发送给电调
  }
}


  // // 下面还有一些初始化的...
  // TIM4_PWM_Init();
  // MPU6500_IntConfiguration();

  // delay_ms(3000);
  // Hook_Encoder.ecd_bias = Motor_Feedback.Motor_205_Agree;
  // GetEncoderBias(&Hook_Encoder);
  // Armour1_Encoder.ecd_bias = Motor_Feedback.Motor_206_Agree;
  // Armour2_Encoder.ecd_bias = Motor_Feedback.Motor_207_Agree;
  // GetEncoderBias(&Armour1_Encoder);
  // GetEncoderBias(&Armour2_Encoder);

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