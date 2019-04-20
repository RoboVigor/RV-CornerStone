/**
 * @brief 甩锅小车
 * @version 0.8.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    int        rotorSpeed[4];                      // 轮子转速
    float      rpm2rps      = 3.14 / 60;           // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    int        followOutput = 0;                   // 输出底盘跟随
    float      powerScale   = 1;

    // 遥控器输入限幅
    float lx     = 0;
    float ly     = 0;
    float lxLast = 0;
    float lyLast = 0;
    float lxDiff = 0;
    float lyDiff = 0;

    // 初始化麦轮速度PID（步兵）
    PID_Init(&PID_LFCM, 28, 0.5, 0, 3500, 1750); //  4400 5500 2750  0.18  25
    PID_Init(&PID_LBCM, 28, 0.5, 0, 3500, 1750);
    PID_Init(&PID_RBCM, 28, 0.5, 0, 3500, 1750);
    PID_Init(&PID_RFCM, 28, 0.5, 0, 3500, 1750);

    // 初始化底盘跟随PID(步兵)
    PID_Init(&PID_Follow_Angle, 8, 0, 0, 500, 0);   // 17
    PID_Init(&PID_Follow_Speed, 0.8, 0, 0, 660, 0); // 0.22

    //功率PID
    PID_Init(&PID_Power, 0.5, 0.02, 0, 1000, 500); // 0.5 0.12  //0.2  0.05

    while (1) {

        if (Motor_Pitch.angle < 2 && Motor_Pitch.angle > -2) {
            followOutput = 0; //偏差小于3度认为跟上
        } else {
            //计算followpid
            PID_Calculate(&PID_Follow_Angle, 0, Motor_Pitch.angle);
            PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output, Motor_Pitch.positionDiff);

            //输出followpid
            followOutput = PID_Follow_Speed.output;
        }

        lx = remoteData.lx;
        ly = remoteData.ly;

        if (ABS(lx) < 90) {
            lxDiff = 0;
        }
        if (ABS(ly) < 120) {
            lyDiff = 0;
        }

        if (lx >= 200 && lxLast < lx) { // 90
            lx = 200 + lxDiff;
            lxDiff++;
            lxLast = lx;
        } else if (lx <= -200 && lxLast > lx) {
            lx = -200 - lxDiff;
            lxDiff++;
            lxLast = lx;
        }
        if (ly > 120 && lyLast < ly) {
            ly = 120 + lyDiff;
            lyDiff += 0.5;
            lyLast = ly;
        } else if (ly < -120 && lyLast > ly) {
            ly = -120 - lyDiff;
            lyDiff += 0.5;
            lyLast = ly;
        }

        // 修正转向力
        lx           = -lx / 660.0f;
        ly           = ly / 660.0f * 3;
        followOutput = -followOutput / 660.0f * 12; // 12
        // 设置底盘总体移动速度
        Chassis_Update(&ChassisData, lx, ly, followOutput);
        // Chassis_Update(&ChassisData, lx, ly, 0);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(&ChassisData, rotorSpeed);

        // 根据功率限幅
        // powerScale    = Chassis_Power_Control(lx, ly);
        rotorSpeed[0] = rotorSpeed[0] * powerScale;
        rotorSpeed[1] = rotorSpeed[1] * powerScale;
        rotorSpeed[2] = rotorSpeed[2] * powerScale;
        rotorSpeed[3] = rotorSpeed[3] * powerScale;

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
        PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
        PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
        PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);
        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

float Chassis_Power_Control(float VX, float VY) {

    powerfeed = Judge.powerHeatData.chassisPower;
    if (powerfeed >= 60) {
        PID_Calculate(&PID_Power, 700, powerfeed * 10);

        return (float) (PID_Power.output + 1000) / 1000.0;
    } else {
        if (VX == 0 && VY == 0) {
            PID_Power.output_I = 0;
            PID_Power.output   = 0;
        }
        return 1.0f;
    }
}

void Task_Debug_Magic_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        // printf();
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 发射机构代码
 *
 * @param Parameters
 */

int turnNumber = 1;

void Task_Fire(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      rpm2rps      = 3.14 / 60;           // 转子的转速(round/min)换算成角速度(rad/s)
    float      r            = 0.0595;
    // uint8_t    startCounter = 0;                   // 启动模式计数器

    // 标志位
    uint8_t frictState = 0;
    uint8_t stirState  = 0;
    uint8_t stirFlag   = 0;
    int     lastSwitch = 0;

    // 常量
    // float frictSpeed = -24 / 0.0595 * 2 * 60 / 2 / 3.14;
    // 摩擦轮线速度(mps)转转速(rpm)
    float frictSpeed = 336; // 336 rad/s
    float stirSpeed  = -40;

    // PID 初始化
    PID_Init(&PID_LeftFrictSpeed, 25, 0.5, 0, 20000, 6000);  // 4 0.08 0    6000  1000
    PID_Init(&PID_RightFrictSpeed, 25, 0.5, 0, 20000, 6000); // 10 0.08           1000
    PID_Init(&PID_StirAnlge, 12, 0.01, 0, 4000, 2000);       // 12 0.01
    PID_Init(&PID_StirSpeed, 5, 0.2, 0, 4000, 2000);         // 5

    while (1) {
        // Debug Code
        // if (remoteData.switchRight == 1) {
        //     frictState = 0;
        //     stirState  = 1;
        // } else if (remoteData.switchRight == 3) {
        //     frictState = 0;
        //     stirState  = 0;
        // }
        if (remoteData.switchRight == 1) {
            frictState = 1;
        } else if (remoteData.switchRight == 3) {
            frictState = 0;
        }

        // 摩擦轮 PID 控制
        if (frictState == 0) {
            // LASER_OFF;                                                                          // 关闭激光
            PID_Increment_Calculate(&PID_LeftFrictSpeed, 0, Motor_LeftFrict.speed * rpm2rps * r);   // 左摩擦轮停止
            PID_Increment_Calculate(&PID_RightFrictSpeed, 0, Motor_RightFrict.speed * rpm2rps * r); // 右摩擦轮停止
            PID_LeftFrictSpeed.output  = PID_LeftFrictSpeed.output / 0.0595 * 60 / 3.14;
            PID_RightFrictSpeed.output = PID_RightFrictSpeed.output / 0.0595 * 60 / 3.14;
            Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, 0, 0);
        } else {
            LASER_ON; // 开启激光
            // PID_Calculate(&PID_LeftFrictSpeed, frictSpeed, Motor_LeftFrict.speed * rpm2rps * r);   // 左摩擦轮转动
            // PID_Calculate(&PID_RightFrictSpeed, frictSpeed, Motor_RightFrict.speed * rpm2rps * r); // 右摩擦轮转动
            // debugF                     = PID_LeftFrictSpeed.output;
            // debugG                     = PID_RightFrictSpeed.output;
            // PID_LeftFrictSpeed.output  = PID_LeftFrictSpeed.output / 0.0595 * 60 / 3.14;
            // PID_RightFrictSpeed.output = PID_RightFrictSpeed.output / 0.0595 * 60 / 3.14;
            // Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, 0, 0);
            PID_Calculate(&PID_LeftFrictSpeed, frictSpeed, Motor_LeftFrict.speed * rpm2rps);    // 左摩擦轮转动
            PID_Calculate(&PID_RightFrictSpeed, -frictSpeed, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮转动
            Can_Send(CAN2, 0x200, PID_RightFrictSpeed.output, PID_LeftFrictSpeed.output, 0, 0);
            // Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, 0, 0, 0);
            // Can_Send(CAN2, 0x200, -1000, 1000, 0, 0);
            // Can_Send(CAN2, 0x200, -500, 500, 0, 0);
        }
        if (remoteData.switchLeft == 1) {
            TIM_SetCompare1(TIM4, 15);
        }
        if (remoteData.switchLeft == 2) {
            stirState = 2;
        } else if (remoteData.switchLeft == 3) {
            // lastSwitch = 3;
            stirState = 0;
            TIM_SetCompare1(TIM4, 7);
        }
        // else if (remoteData.switchLeft == 2 && lastSwitch == 3) {
        //     stirState = 3;
        //     turnNumber++;
        //     lastSwitch = 2;
        // }

        //拨弹轮 PID 控制
        if (stirState == 0) { // 停止模式
            PID_Increment_Calculate(&PID_StirSpeed, 0, Motor_Stir.speed * rpm2rps);
            // }
            // else if (stirState == 1) { // 三连发模式
            //                              //     // Debug Code
            //                              //     // if (stirFlag < 3) {
            //                              //     //     stirFlag++;
            //                              //     //     PID_Increment_Calculate(&PID_StirAnlge, (Motor_Stir.angle - 36 * 60), Motor_Stir.angle);
            //                              //     //     PID_Increment_Calculate(&PID_StirSpeed, PID_StirAnlge.output, Motor_Stir.speed);
            //                              //     //     Can_Send(CAN1, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
            //                              //     // } else {
            //                              //     //     PID_Increment_Calculate(&PID_StirSpeed, 0, Motor_Stir.speed * rpm2rps);
            //                              //     //     Can_Send(CAN1, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
            //                              //     // }

            //     PID_Increment_Calculate(&PID_StirAnlge, (Motor_Stir.angle - 60), Motor_Stir.angle);
            //     PID_Increment_Calculate(&PID_StirSpeed, PID_StirAnlge.output, Motor_Stir.speed);
            //     Can_Send(CAN2, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
        } else if (stirState == 2) { // 连发模式
            PID_Increment_Calculate(&PID_StirSpeed, stirSpeed, Motor_Stir.speed * rpm2rps);
            //     Can_Send(CAN2, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
            // } else if (stirState == 3) { // 单点测试模式
            //     PID_Increment_Calculate(&PID_StirAnlge, -turnNumber * 60, Motor_Stir.angle);
            //     // PID_Increment_Calculate(&PID_StirAnlge, 0, Motor_Stir.angle);
            //     PID_Increment_Calculate(&PID_StirSpeed, PID_StirAnlge.output, Motor_Stir.speed * 2 * rpm2rps);
            //     // PID_Increment_Calculate(&PID_StirSpeed, 100, Motor_Stir.speed);
            //     Can_Send(CAN2, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
            // } else if (stirState == 4) { // 直接给电流
        }
        // Can_Send(CAN2, 0x1FF, 0, 0, 500, 0);
        Can_Send(CAN2, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
        // Decode_JudgeData();

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}

void Task_Cloud(void *Parameters) {

    // 时钟
    TickType_t LastWakeTime = xTaskGetTickCount();

    //初始化航向角target
    float yawTargetAngle   = 0;
    float pitchTargetAngle = 0;

    //初始化角速度反馈
    float yawFeedAngularSpeed   = 0;
    float pitchFeedAngularSpeed = 0;

    //初始化角度反馈(陀螺仪反馈角度)
    float yawFeedAngle   = 0;
    float pitchFeedAngle = 0;

    // yaw pitch 增量
    float yawDiff   = 0;
    float pitchDiff = 0;

    //初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 10, 0, 0, 2000, 0);   // 10
    PID_Init(&PID_Cloud_YawSpeed, 15, 0, 0, 2000, 0);   // 15
    PID_Init(&PID_Cloud_PitchAngle, 20, 0, 0, 2000, 0); // 20
    PID_Init(&PID_Cloud_PitchSpeed, 15, 0, 0, 5000, 0); // 15

    while (1) {
        //将陀螺仪值赋予使用变量
        yawFeedAngle          = -Gyroscope_EulerData.yaw;
        pitchFeedAngle        = Gyroscope_EulerData.pitch;
        yawFeedAngularSpeed   = -(float) (ImuData.gz / 16.4f);
        pitchFeedAngularSpeed = -(float) (ImuData.gx / 16.4f);

        //设定输入target
        if (remoteData.rx <= 10 && remoteData.rx >= -10) {
            yawDiff = 0;
        } else {
            yawDiff = remoteData.rx / 660.0f;
        }
        MIAO(yawDiff, -1, 1);
        yawTargetAngle = yawTargetAngle + yawDiff;

        if (remoteData.ry <= 30 && remoteData.ry >= -30) {
            pitchDiff = 0;
        } else {
            pitchDiff = remoteData.ry / 2200.0f;
        }

        MIAO(pitchDiff, -0.3, 0.3);
        pitchTargetAngle = pitchTargetAngle + pitchDiff;

        PID_Calculate(&PID_Cloud_YawAngle, yawTargetAngle, yawFeedAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawFeedAngularSpeed);
        PID_Calculate(&PID_Cloud_PitchAngle, pitchTargetAngle, pitchFeedAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchFeedAngularSpeed);

        Can_Send(CAN1, 0x1FF, PID_Cloud_PitchSpeed.output, PID_Cloud_YawSpeed.output, 0, 0);
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    // 初始化全局变量
    Handle_Init();

    // BSP们
    BSP_GPIO_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_DMA_Init();
    BSP_TIM_Init();
    BSP_NVIC_Init();
    BSP_USER_Init();

    // 初始化陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData);

    // 调试任务
#if DEBUG_ENABLED
    // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 高频任务
    xTaskCreate(Task_Cloud, "Task_Cloud", 1000, NULL, 5, NULL);

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    xTaskCreate(Task_Fire, "Task_Fire", 400, NULL, 3, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}