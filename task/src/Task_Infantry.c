/**
 * @brief 步兵
 * @version 1.0.0
 */
#include "main.h"

#if ROBOT_TYPE == 1

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

    //初始化麦轮速度PID（步兵）
    PID_Init(&PID_LFCM, 14, 0.12, 0, 4400, 2200); // 18,0.18
    PID_Init(&PID_LBCM, 14, 0.12, 0, 4400, 2200);
    PID_Init(&PID_RBCM, 14, 0.12, 0, 4400, 2200);
    PID_Init(&PID_RFCM, 14, 0.12, 0, 4400, 2200);

    // 初始化底盘跟随PID(步兵)
    PID_Init(&PID_Follow_Angle, 2, 0, 0, 500, 0);   // 0.6
    PID_Init(&PID_Follow_Speed, 0.4, 0, 0, 660, 0); // 0.3
    while (1) {
        if (cloudcounter > COUNT_QUATERNIONABSTRACTION) {

            if (Motor_Yaw.angle < 3 && Motor_Yaw.angle > -3) {
                FollowOutput = 0; //偏差小于3度认为跟上
            } else {
                //计算followpid
                PID_Calculate(&PID_Follow_Angle, 0, Motor_Yaw.angle);
                PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output, Motor_Yaw.positionDiff);
                //输出followpid
                FollowOutput = PID_Follow_Speed.output;
            }

            // 设置底盘总体移动速度
            Chassis_Set_Speed((float) -remoteData.lx / 660.0, (float) remoteData.ly / 660.0, -FollowOutput / 660.0 * 25);

            // 麦轮解算&限幅,获得轮子转速
            Chassis_Get_Rotor_Speed(rotorSpeed);

            // 计算输出电流PID
            PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
            PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
            PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
            PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

            // 输出电流值到电调
            Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);
        }

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Debug_Magic_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        printf("Yaw: %f \r\n", EulerAngle.Yaw);
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {
    // 进入临界区
    taskENTER_CRITICAL();

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
    MPU6500_IntConfiguration();
    MPU6500_Initialize();
    MPU6500_EnableInt();

    // 调试任务
#if DEBUG_ENABLED
    Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
    xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 高频任务
    xTaskCreate(Task_Gyroscope, "Task_Gyroscope", 400, NULL, 5, NULL);

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Event_Group, "Task_Event_Group", 400, NULL, 3, NULL);
    xTaskCreate(Task_Cloud, "Task_Cloud", 800, NULL, 4, NULL);

    // 完成使命
    vTaskDelete(NULL);

    // 退出临界区
    taskEXIT_CRITICAL();
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
    cloudcounter            = 0;
    while (1) {
        if (GYROSCOPE_YAW_QUATERNION_ABSTRACTION == 1) {
            if (cloudcounter <= COUNT_QUATERNIONABSTRACTION) {
                cloudcounter = cloudcounter + 1;
            }
        }
        Gyroscope_Update_Angle_Data();

        vTaskDelayUntil(&LastWakeTime, 3);
    }
    vTaskDelete(NULL);
}

void Task_Cloud(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    //初始化offset
    mpu6500_data.gx_offset  = 35;
    mpu6500_data.gy_offset  = -9;
    mpu6500_data.gz_offset  = -20;
    EulerAngle.Yaw_offset   = 0;
    EulerAngle.Pitch_offset = 0;
    //初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 7, 0, 0, 1000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 15, 0, 0, 3000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 8, 0, 0, 1000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 14.7, 0, 0, 3000, 0);
    //给定positionbias，以保证yaw_motor.angle位于0

    //初始化云台结构体
    CloudPara_Init(&Pitch);
    CloudPara_Init(&Yaw);

    while (1) {
        if (cloudcounter > COUNT_QUATERNIONABSTRACTION) {

            Pitch.AngularSpeed = (float) (mpu6500_data.gx / GYRO_LSB); // UP- DOWN+
            Yaw.AngularSpeed   = (float) (mpu6500_data.gz / GYRO_LSB); // L- R+
            TargetAngleSet(remoteData.ry / (5 * 1100.0f),
                           remoteData.rx / (3 * 660.0f)); //设定输入target

            if (remoteData.switchLeft == 1) {

                PID_Calculate(&PID_Cloud_YawAngle, Yaw.TargetAngle,
                              Yaw.AnglePosition); // Yaw.TargetAngle
                PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, Yaw.AngularSpeed);
                PID_Calculate(&PID_Cloud_PitchAngle, Pitch.TargetAngle,
                              Pitch.AnglePosition); // Pitch.TargetAngle
                PID_Calculate(&PID_Cloud_PitchSpeed, -PID_Cloud_PitchAngle.output, Pitch.AngularSpeed);

                Can_Send(CAN1, 0x1FF, PID_Cloud_YawSpeed.output, PID_Cloud_PitchSpeed.output, 0, 0);
            }
        }

        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

#endif