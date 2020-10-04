/**
 * @brief 甩锅小车
 * @version 1.2.0
 */
#include "main.h"

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      interval     = 0.01;            // 任务运行间隔 s
    int        intervalms   = interval * 1000; // 任务运行间隔 ms
    while (1) {
        SafetyMode = RIGHT_SWITCH_BOTTOM && LEFT_SWITCH_BOTTOM;
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    float pitchAngle     = 0;
    float lastPitchAngle = 0;

    float speedTargetLeft  = 0;
    float speedTargetRight = 0;

    float linearSpeed  = 0;
    float angularSpeed = 0;

    Filter_Chassis_Motor_Speed_Left.windowSize  = 1;
    Filter_Chassis_Motor_Speed_Right.windowSize = 1;

    // PID初始化
    PID_Init(&PID_Chassis_PitchAngle, 4, 0, 15, 16000, 0);
    PID_Init(&PID_Chassis_LeftSpeed, 60, 0, 0, 16000, 100);
    PID_Init(&PID_Chassis_RightSpeed, 60, 0, 0, 16000, 100);

    while (1) {
        pitchAngle = Gyroscope_EulerData.pitch; // 俯仰角角度反馈

        linearSpeed  = 0;
        angularSpeed = 0;
        if (ABS(remoteData.ry) > 30) {
            linearSpeed += remoteData.ry / 660.0f * 360 * interval;
        }

        // speedTargetLeft  = remoteData.lx / 5;
        // speedTargetRight = -remoteData.rx / 5;
        // PID_Chassis_LeftSpeed.p  = CHOOSEL(45, 60, 75);
        // PID_Chassis_RightSpeed.p = CHOOSEL(45, 60, 75);
        PID_Chassis_PitchAngle.p = CHOOSEL(4, 5.5, 7);

        //计算PID
        PID_Calculate(&PID_Chassis_PitchAngle, 0, pitchAngle);

        speedTargetLeft  = PID_Chassis_PitchAngle.output + linearSpeed;
        speedTargetRight = -1 * (PID_Chassis_PitchAngle.output + linearSpeed);

        Filter_Update(&Filter_Chassis_Motor_Speed_Left, Motor_Left.speed / 19.2f);
        Filter_Update_Moving_Average(&Filter_Chassis_Motor_Speed_Left);
        Filter_Update(&Filter_Chassis_Motor_Speed_Right, Motor_Right.speed / 19.2f);
        Filter_Update_Moving_Average(&Filter_Chassis_Motor_Speed_Right);

        PID_Calculate(&PID_Chassis_LeftSpeed, speedTargetLeft, Filter_Chassis_Motor_Speed_Left.movingAverage);
        PID_Calculate(&PID_Chassis_RightSpeed, speedTargetRight, Filter_Chassis_Motor_Speed_Right.movingAverage);

        if (pitchAngle >= -20 && pitchAngle <= 20) {
            Motor_Left.input  = PID_Chassis_LeftSpeed.output;
            Motor_Right.input = PID_Chassis_RightSpeed.output;
        } else {
            Motor_Left.input  = 0;
            Motor_Right.input = 0;
        }

        // 调试信息
        DebugData.debug1 = PID_Chassis_PitchAngle.target;
        DebugData.debug2 = PID_Chassis_PitchAngle.feedback;
        DebugData.debug3 = PID_Chassis_PitchAngle.output_I;
        DebugData.debug4 = PID_Chassis_RightSpeed.target;
        DebugData.debug5 = PID_Chassis_RightSpeed.feedback;
        DebugData.debug6 = PID_Chassis_RightSpeed.output_I;
        DebugData.debug7 = pitchAngle;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Board_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    uint16_t id;         // 通讯ID
    uint16_t dataLength; // 数据长度

    while (1) {

        // 板间通信
        if (Board_Id == 1) {
            id                                 = 0x501;
            ProtocolData.user.boardAlpha.data1 = 1.11;
            ProtocolData.user.boardAlpha.data2 = 2.22;
            ProtocolData.user.boardAlpha.data3 = 3.33;
            ProtocolData.user.boardAlpha.data4 = 4.44;
        }

        if (Board_Id == 2) {
            id                                = 0x502;
            ProtocolData.user.boardBeta.data1 = 0;
            ProtocolData.user.boardBeta.data2 = 0;
            ProtocolData.user.boardBeta.data3 = 0;
            ProtocolData.user.boardBeta.data4 = 1.11;
        }

        // USART发送
        DMA_Disable(UART7_Tx);
        dataLength = Protocol_Pack(&UserChannel, id);
        DMA_Enable(UART7_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // Can发送
        dataLength = Protocol_Pack(&UserChannel, id);
        Can_Send_Msg(CAN1, id, UserChannel.sendBuf, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ProtocolData.user.boardAlpha.data1 * 1000;
        // DebugData.debug2 = ProtocolData.user.boardBeta.data4 * 1000;
    }
    vTaskDelete(NULL);
}

void Task_Vision_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    uint16_t id = 0x0401; // 通讯ID
    uint16_t dataLength;  // 数据长度

    while (1) {

        // 视觉通信
        ProtocolData.host.autoaimData.yaw_angle_diff   = 1.23;
        ProtocolData.host.autoaimData.pitch_angle_diff = 4.56;
        ProtocolData.host.autoaimData.biu_biu_state    = 7;

        // DMA重启
        DMA_Disable(UART8_Tx);
        dataLength = Protocol_Pack(&HostChannel, id);
        DMA_Enable(UART8_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Can_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    CAN_TypeDef *Canx[2]          = {CAN1, CAN2};
    Motor_Type **Canx_Device[2]   = {Can1_Device, Can2_Device};
    uint16_t     Can_Send_Id[3]   = {0x200, 0x1ff, 0x2ff};
    uint16_t     Can_ESC_Id[3][4] = {{0x201, 0x202, 0x203, 0x204}, {0x205, 0x206, 0x207, 0x208}, {0x209, 0x020a, 0x20b, 0x20c}};

    int         i, j, k;        // CAN序号 发送ID序号 电调ID序号
    int         isNotEmpty = 0; // 同一发送ID下是否有电机
    Motor_Type *motor;          // 根据i,j,k锁定电机
    int16_t     currents[4];    // CAN发送电流

    while (1) {
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 3; j++) {
                isNotEmpty = 0;
                for (k = 0; k < 4; k++) {
                    motor       = *(Canx_Device[i] + ESC_ID(Can_ESC_Id[j][k]));
                    currents[k] = (motor && motor->inputEnabled) ? motor->input : 0;
                    isNotEmpty  = isNotEmpty || (motor && motor->inputEnabled);
                }
                if (isNotEmpty && !SafetyMode) {
                    Can_Send(Canx[i], Can_Send_Id[j], currents[0], currents[1], currents[2], currents[3]);
                } else if (isNotEmpty && SafetyMode) {
                    Can_Send(Canx[i], Can_Send_Id[j], 0, 0, 0, 0);
                }
            }
        }
        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Debug_Magic_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        printf("Yaw: %f \r\n", Gyroscope_EulerData.yaw);
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        LED_Run_Horse_XP(); // XP开机动画,建议延时200ms
        // LED_Run_Horse(); // 跑马灯,建议延时20ms
        vTaskDelayUntil(&LastWakeTime, 200);
    }

    vTaskDelete(NULL);
}

void Task_Startup_Music(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        if (KTV_Play(Music_Soul)) break;
        vTaskDelayUntil(&LastWakeTime, 60);
    }
    vTaskDelete(NULL);
}

void Task_OLED(void *Parameters) {
    uint16_t   JoystickValue = -1;
    TickType_t LastWakeTime  = xTaskGetTickCount();
    oled_init();
    while (1) {
        JoystickValue = ADC_GetConversionValue(ADC1);
        oled_clear(Pen_Clear);
        oled_menu(JoystickValue);
        oled_refresh_gram();
        vTaskDelayUntil(&LastWakeTime, 125);
    }
    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    //获得 Stone ID
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);

    // 初始化全局变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData);

    if (Board_Id == 1) {
    }

    // 调试任务
#if DEBUG_ENABLED
    // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_OLED, "Task_OLED", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);

    // DMA发送任务
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Vision_Communication, "Task_Vision_Communication", 500, NULL, 6, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 5, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
