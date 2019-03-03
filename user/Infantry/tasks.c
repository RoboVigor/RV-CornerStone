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
    int        FollowOutput = 0;                   // 输出底盘跟随

    int   lastDataX = 0;
    int   lastDataY = 0;
    float n         = 0;
    int   m         = 0;
    //初始化麦轮速度PID（步兵）
    PID_Init(&PID_LFCM, 25, 0.5, 0, 3500, 1750); //  4400 5500 2750  0.18
    PID_Init(&PID_LBCM, 25, 0.5, 0, 3500, 1750);
    PID_Init(&PID_RBCM, 25, 0.5, 0, 3500, 1750);
    PID_Init(&PID_RFCM, 25, 0.5, 0, 3500, 1750);

    // 初始化底盘跟随PID(步兵)
    PID_Init(&PID_Follow_Angle, 17, 0, 0, 500, 0);   // 20
    PID_Init(&PID_Follow_Speed, 0.22, 0, 0, 660, 0); // 0.024  0.12
    PID_Init(&PID_Power, 0.5, 0.02, 0, 1000, 500);   // 0.5 0.12  //0.2  0.05

    // powercurrent = 0;
    // powerlast    = 0;

    Ramp_Lowpass_Init(&Ramp_Power);
    Ramp_Lowpass_Init(&Lowpass_Power);
    while (1) {
        powerfeed = Judge_PowerHeatData.chassisPower;
        // powercurrent = powerfeed * 0.08 + powerlast * (1 - 0.08);
        // powerlast = powercurrent;

        // Power_Ramp(&Ramp_Power, 0, 100);

        // ChassisStatus.Channel1 = powercurrent;

        if (Motor_Pitch.angle < 3 && Motor_Pitch.angle > -3) {
            FollowOutput = 0; //偏差小于3度认为跟上
        } else {
            //计算followpid
            PID_Calculate(&PID_Follow_Angle, 0, Motor_Pitch.angle);
            PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output, Motor_Pitch.positionDiff);

            //输出followpid
            FollowOutput = PID_Follow_Speed.output;
        }

        if (abs(remoteData.lx) < 90) {
            m = 0;
        }
        if (abs(remoteData.ly) < 120) {
            n = 0;
        }

        if (remoteData.lx >= 90 && lastDataX < remoteData.lx) {
            remoteData.lx = 90 + m;
            m++;
            lastDataX = remoteData.lx;
        } else if (remoteData.lx <= -90 && lastDataX > remoteData.lx) {
            remoteData.lx = -90 - m;
            m++;
            lastDataX = remoteData.lx;
        }
        if (remoteData.ly > 120 && lastDataY < remoteData.ly) {
            remoteData.ly = 120 + n;
            n += 0.3;
            lastDataY = remoteData.ly;
        } else if (remoteData.ly < -120 && lastDataY > remoteData.ly) {
            remoteData.ly = -120 - n;
            n += 0.3;
            lastDataY = remoteData.ly;
        }

        // 设置底盘总体移动速度
        Chassis_Set_Speed((float) -remoteData.lx / 660.0 * 1, (float) remoteData.ly / 660.0 * 3,
                          -FollowOutput / 660.0 * 12); // 5 15 60  1  4  12
        // debugH = remoteData.lx;
        // debugG = remoteData.ly;
        Power_Control();

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(rotorSpeed);

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

    // 初始化陀螺仪
    MPU6500_Initialize();
    MPU6500_EnableInt();

    // 遥控器数据初始化
    DBUS_Init();

    while (1) {
        if (g_stabilizerCounter == COUNT_QUATERNIONABSTRACTION) {
            break;
        }
    }

    // 调试任务
#if DEBUG_ENABLED
    Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
    xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6,
    // NULL); xTaskCreate(Task_Debug_Gyroscope_Sampling,
    // "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);

#endif

    // 高频任务
    // xTaskCreate(Task_Gyroscope, "Task_Gyroscope", 400, NULL, 5, NULL);

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    xTaskCreate(Task_Cloud, "Task_Cloud", 800, NULL, 4, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}

void Task_Gyroscope(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    g_stabilizerCounter     = 0;
    /*while (1) {
        if (GYROSCOPE_YAW_QUATERNION_ABSTRACTION == 1) {
            if (g_stabilizerCounter <= COUNT_QUATERNIONABSTRACTION) {
                g_stabilizerCounter = g_stabilizerCounter + 1;
                debugA       = g_stabilizerCounter;
            }
        }
        Gyroscope_Update_Angle_Data();

        vTaskDelayUntil(&LastWakeTime, 3);
    }*/
    vTaskDelete(NULL);
}

void Task_Cloud(void *Parameters) {

    // 时钟
    TickType_t LastWakeTime = xTaskGetTickCount();

    //初始化offset
    mpu6500_data.gx_offset = 35;
    mpu6500_data.gy_offset = -9;
    mpu6500_data.gz_offset = -20;

    //初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 2000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 17, 0, 0, 2000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 10, 0, 0, 2000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 13, 0, 0, 5000, 0);

    //初始化云台结构体
    Stabilizer_Init_Course_Angle(&Pitch_Angle);
    Stabilizer_Init_Course_Angle(&Yaw_Angle);

    while (1) {
        //设定输入target
        Stabilizer_Set_TargetAngle(remoteData.ry / (4 * 1100.0f), remoteData.rx / (1 * 660.0f));

        PID_Calculate(&PID_Cloud_YawAngle, Yaw_Angle.targetAngle, Yaw_Angle.anglePosition);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, -Yaw_Angle.angularSpeed);
        PID_Calculate(&PID_Cloud_PitchAngle, Pitch_Angle.targetAngle, Pitch_Angle.anglePosition);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, -Pitch_Angle.angularSpeed);
        Can_Send(CAN1, 0x1FF, PID_Cloud_PitchSpeed.output, PID_Cloud_YawSpeed.output, 0, 0);

        // Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
        vTaskDelayUntil(&LastWakeTime, 5);
    }

    vTaskDelete(NULL);
}