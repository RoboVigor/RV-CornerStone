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
    float      powerScale;

    // 遥控器输入限幅
    float lx     = 0;
    float ly     = 0;
    float lxLast = 0;
    float lyLast = 0;
    float lxDiff = 0;
    float lyDiff = 0;

    // 初始化麦轮速度PID（步兵）
    PID_Init(&PID_LFCM, 25, 0.5, 0, 3500, 1750); //  4400 5500 2750  0.18
    PID_Init(&PID_LBCM, 25, 0.5, 0, 3500, 1750);
    PID_Init(&PID_RBCM, 25, 0.5, 0, 3500, 1750);
    PID_Init(&PID_RFCM, 25, 0.5, 0, 3500, 1750);

    // 初始化底盘跟随PID(步兵)
    PID_Init(&PID_Follow_Angle, 17, 0, 0, 500, 0);   // 20
    PID_Init(&PID_Follow_Speed, 0.22, 0, 0, 660, 0); // 0.024  0.12

    //功率PID
    PID_Init(&PID_Power, 0.5, 0.02, 0, 1000, 500); // 0.5 0.12  //0.2  0.05

    while (1) {

        if (Motor_Pitch.angle < 3 && Motor_Pitch.angle > -3) {
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

        if (abs(lx) < 90) {
            lxDiff = 0;
        }
        if (abs(ly) < 120) {
            lyDiff = 0;
        }

        if (lx >= 90 && lxLast < lx) {
            lx = 90 + lxDiff;
            lxDiff++;
            lxLast = lx;
        } else if (lx <= -90 && lxLast > lx) {
            lx = -90 - lxDiff;
            lxDiff++;
            lxLast = lx;
        }
        if (ly > 120 && lyLast < ly) {
            ly = 120 + lyDiff;
            lyDiff += 0.3;
            lyLast = ly;
        } else if (ly < -120 && lyLast > ly) {
            ly = -120 - lyDiff;
            lyDiff += 0.3;
            lyLast = ly;
        }

        // 修正转向力
        lx           = -lx / 660.0f;
        ly           = ly / 660.0f * 3;
        followOutput = -followOutput / 660.0f * 12;

        // 设置底盘总体移动速度
        Chassis_Set_Speed(lx, ly, followOutput);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(rotorSpeed);

        // 根据功率限幅
        powerScale    = Chassis_Power_Control(lx, ly);
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

    float powerfeed = Judge_PowerHeatData.chassisPower;
    if (powerfeed >= 60) {
        PID_Power.output_I = 0;
        return (float) (PID_Power.output + 1000) / 1000.0;
    } else {
        if (VX == 0 && VY == 0) {
            PID_Power.output_I = 0;
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
