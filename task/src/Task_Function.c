/**
 * @brief 功能任务
 */

#include "main.h"

void Task_Blink(void *Parameters) {
  TickType_t LastWakeTime = xTaskGetTickCount();
  while (1) {
    GREEN_LIGHT_TOGGLE;
    vTaskDelayUntil(&LastWakeTime, 250);
  }

  vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
  TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
  int rotorSpeed[4];                             // 轮子转速
  float rpm2rps =
      3.14 / 60; // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
  // int mode = 2;                     // 底盘运动模式,1直线,2转弯
  // int lastMode = 2;                 // 上一次的运动模式
  // float yawAngleTarget = 0;         // 目标值
  // float yawAngleFeed, yawSpeedFeed; // 反馈值
  int FollowOutput = 0; // 输出底盘跟随
  //初始化麦轮速度PID（步兵）
  PID_Init(&PID_LFCM, 14, 0.12, 0, 4400, 2200); // 18,0.18
  PID_Init(&PID_LBCM, 14, 0.12, 0, 4400, 2200);
  PID_Init(&PID_RBCM, 14, 0.12, 0, 4400, 2200);
  PID_Init(&PID_RFCM, 14, 0.12, 0, 4400, 2200);
  // 初始化麦轮角速度PID
  /*PID_Init(&PID_LFCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_LBCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_RBCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_RFCM, 15, 0.3, 0, 4000, 2000);*/
  // 初始化底盘跟随PID(步兵)
  PID_Init(&PID_Follow_Angle, 2, 0, 0, 500, 0);   // 0.6
  PID_Init(&PID_Follow_Speed, 0.4, 0, 0, 660, 0); // 0.3
  // 初始化航向角角度PID和角速度PID
  /*PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 2, 0, 0, 4000, 1000);*/
  while (1) {
    if (cloud_counter > COUNT_QUATERNIONABSTRACTION) {
      // 更新运动模式
      // mode = ABS(remoteData.rx) < 5 ? 1 : 2;

      // 设置反馈值
      // yawAngleFeed = EulerAngle.Yaw;         // 航向角角度反馈
      // yawSpeedFeed = mpu6500_data.gz / 16.4; // 航向角角速度反馈

      // 切换运动模式
      // if (mode != lastMode) {
      //    PID_YawAngle.output_I = 0;            // 清空角度PID积分
      //    PID_YawSpeed.output_I = 0;            // 清空角速度PID积分
      //    yawAngleTarget        = yawAngleFeed; // 更新角度PID目标值
      //    lastMode              = mode;         // 更新lastMode
      //}

      // 根据运动模式计算PID
      // if (mode == 1) {
      //    PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngleFeed);      //
      //    计算航向角角度PID PID_Calculate(&PID_YawSpeed, PID_YawAngle.output,
      //    yawSpeedFeed); // 计算航向角角速度PID
      //} else {
      //    PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeedFeed); //
      //    计算航向角角速度PID
      //}

      if (Motor_Yaw.angle < 3 && Motor_Yaw.angle > -3) {
        FollowOutput = 0; //偏差小于3度认为跟上
      } else {
        //计算followpid
        PID_Calculate(&PID_Follow_Angle, 0, Motor_Yaw.angle);
        PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output,
                      Motor_Yaw.positionDiff);
        //输出followpid
        FollowOutput = PID_Follow_Speed.output;
      }

      // 设置底盘总体移动速度
      Chassis_Set_Speed((float)-remoteData.lx / 660.0,
                        (float)remoteData.ly / 660.0,
                        -FollowOutput / 660.0 * 25);

      // 麦轮解算&限幅,获得轮子转速
      Chassis_Get_Rotor_Speed(rotorSpeed);

      // 计算输出电流PID
      PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
      PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
      PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
      PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

      // 输出电流值到电调
      Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output,
               PID_RFCM.output);
    }

    // 底盘运动更新频率
    vTaskDelayUntil(&LastWakeTime, 10);
  }

  vTaskDelete(NULL);
}

void Task_Safe_Mode(void *Parameters) {

  while (1) {
    if (remoteData.switchRight == 2) {
#if CAN1_ENABLED
      Can_Send(CAN1, 0x200, 0, 0, 0, 0);
#endif // CAN1_ENABLED
#if CAN2_ENABLED
      Can_Send(CAN2, 0x200, 0, 0, 0, 0);
#endif // CAN1_ENABLED
      vTaskSuspendAll();
    }
    vTaskDelay(100);
  }

  vTaskDelete(NULL);
}

void Task_Cloud(void *Parameters) {
  TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
  //初始化offset
  mpu6500_data.gx_offset = 35;
  mpu6500_data.gy_offset = -9;
  mpu6500_data.gz_offset = -20;
  EulerAngle.Yaw_offset = 0;
  EulerAngle.Pitch_offset = 0;
  //初始化云台PID
  PID_Init(&PID_Cloud_YawAngle, 7, 0, 0, 1000, 0);
  PID_Init(&PID_Cloud_YawSpeed, 15, 0, 0, 3000, 0);
  PID_Init(&PID_Cloud_PitchAngle, 8, 0, 0, 1000, 0);
  PID_Init(&PID_Cloud_PitchSpeed, 14.7, 0, 0, 3000, 0);
  //给定positionbias，以保证yaw_motor.angle位于0
  Motor_Yaw.positionBias = 2720;
  //初始化云台结构体
  CloudPara_Init(&Pitch);
  CloudPara_Init(&Yaw);

  while (1) {
    if (cloud_counter > COUNT_QUATERNIONABSTRACTION) {

      Pitch.AngularSpeed = (float)(mpu6500_data.gx / GYRO_LSB); // UP- DOWN+
      Yaw.AngularSpeed = (float)(mpu6500_data.gz / GYRO_LSB);   // L- R+
      TargetAngleSet(remoteData.ry / (5 * 1100.0f),
                     remoteData.rx / (3 * 660.0f)); //设定输入target

      if (remoteData.switchLeft == 1) {

        PID_Calculate(&PID_Cloud_YawAngle, Yaw.TargetAngle,
                      Yaw.AnglePosition); // Yaw.TargetAngle
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output,
                      Yaw.AngularSpeed);
        PID_Calculate(&PID_Cloud_PitchAngle, Pitch.TargetAngle,
                      Pitch.AnglePosition); // Pitch.TargetAngle
        PID_Calculate(&PID_Cloud_PitchSpeed, -PID_Cloud_PitchAngle.output,
                      Pitch.AngularSpeed);

        Can_Send(CAN1, 0x1FF, PID_Cloud_YawSpeed.output,
                 PID_Cloud_PitchSpeed.output, 0, 0);
      }
    }

    vTaskDelayUntil(&LastWakeTime, 5);
  }
  vTaskDelete(NULL);
}
