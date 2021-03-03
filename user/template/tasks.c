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
        SafetyMode = (RIGHT_SWITCH_BOTTOM && LEFT_SWITCH_BOTTOM);
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 运动模式
    int   mode           = 1; // 底盘运动模式,1直线,2转弯
    int   lastMode       = 2; // 上一次的运动模式
    float yawAngleTarget = 0; // 目标值
    float yawAngle, yawSpeed; // 反馈值

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_LBCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_RBCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_RFCM, 15, 0.3, 0, 4000, 2000);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 2, 0, 0, 4000, 1000);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    while (1) {

        // 更新运动模式
        mode = ABS(remoteData.rx) < 5 ? 1 : 2;

        // 设置反馈值
        yawAngle = Gyroscope_EulerData.yaw;         // 航向角角度反馈
        yawSpeed = -1 * ImuData.gz / GYROSCOPE_LSB; // 逆时针为正

        // 切换运动模式
        if (mode != lastMode) {
            PID_YawAngle.output_I = 0;        // 清空角度PID积分
            PID_YawSpeed.output_I = 0;        // 清空角速度PID积分
            yawAngleTarget        = yawAngle; // 更新角度PID目标值
            lastMode              = mode;     // 更新lastMode
        }

        // 根据运动模式计算PID
        if (mode == 1) {
            PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);      // 计算航向角角度PID
            PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
        } else {
            PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeed); // 计算航向角角速度PID
        }

        // 设置底盘总体移动速度
        Chassis_Update(&ChassisData, (float) -remoteData.lx / 660.0f, (float) remoteData.ly / 660.0f, (float) PID_YawSpeed.output / 660.0f);

        // 麦轮解算
        Chassis_Calculate_Rotor_Speed(&ChassisData);

        // 设置转子速度上限 (rad/s)
        Chassis_Limit_Rotor_Speed(&ChassisData, 300);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调(安全起见默认注释此行)
        Motor_LF.input = PID_LFCM.output;
        Motor_LB.input = PID_LBCM.output;
        Motor_RB.input = PID_RBCM.output;
        Motor_RF.input = PID_RFCM.output;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

// void Task_Client_Communication(void *Parameters) {
//     TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
//     float      interval     = 0.1;                 // 任务运行间隔 s
//     int        intervalms   = interval * 1000;     // 任务运行间隔 ms

//         int      index = 0;
//         uint16_t id;
//         uint16_t dataLength;
//         uint16_t length;

//     while (1) {

//         switch (Judge.mode) {
//         case MODE_CLIENT_DATA: {
//             // 客户端自定义数据
//             Judge.clientCustomData.data_cmd_id = Protocol_Interact_Id_Client_Data;
//             Judge.clientCustomData.send_id     = Judge.robotState.robot_id;
//             Judge.clientCustomData.receiver_id = (Judge.clientCustomData.send_id % 10) | (Judge.clientCustomData.send_id / 10) << 4 | (0x01 << 8);

//             Judge.clientCustomData.data1 = 1;
//             Judge.clientCustomData.data2 = 1.1;
//             Judge.clientCustomData.data3 = 1.11;
//             Judge.clientCustomData.masks = 0x3c;

//             id         = Protocol_Interact_Id_Client_Data;
//             dataLength = PROTOCOL_PACK_0301_HEADER + PROTOCOL_PACK_LENGTH_0301_Client_Data;

//             Judge.mode = MODE_ROBOT_INTERACT;
//         } break;

//         case MODE_ROBOT_INTERACT: {
//             // 机器人间通信
//             Judge.robotInteractiveData[0].data_cmd_id = 0x0200;
//             Judge.robotInteractiveData[0].send_id     = Judge.robotState.robot_id;
//             Judge.robotInteractiveData[0].receiver_id = 1;

//             Judge.robotInteractiveData[0].transformer[index++].F = 1;
//             Judge.robotInteractiveData[0].transformer[index++].F = 1.1;
//             Judge.robotInteractiveData[0].transformer[index++].F = 1.11;
//             Judge.robotInteractiveData[0].transformer[index++].F = 1.111;

//             id         = 0x200;
//             dataLength = PROTOCOL_PACK_0301_HEADER + index * sizeof(float);

//             Judge.mode = MODE_CLIENT_GRAPH;
//         } break;

//         case MODE_CLIENT_GRAPH: {
//             // 客户端自定义图形
//             Judge.clientGraphicDraw.data_cmd_id = Protocol_Interact_Id_Client_Graph;
//             Judge.clientGraphicDraw.send_id     = Judge.robotState.robot_id;
//             Judge.clientGraphicDraw.receiver_id = (Judge.clientCustomData.send_id % 10) | (Judge.clientCustomData.send_id / 10) << 4 | (0x01 << 8);

//             Judge.clientGraphicDraw.operate_tpye = 1; // 0:空操作 1:增加图形 2:修改图形 3:删除单个图形 5:删除一个图层 6:删除所有图形
//             Judge.clientGraphicDraw.graphic_tpye = 3; // 0:空形 1:直线 2:矩形 3:正圆 4:椭圆 5:弧形 6:文本（ASCII 字码）
//             Judge.clientGraphicDraw.layer        = 5; // 图层0-9
//             Judge.clientGraphicDraw.width        = 4; // 线宽
//             Judge.clientGraphicDraw.color        = 4; // 官方 0:红/蓝 1:黄 2:绿 3:橙 4:紫 5:粉 6:青 7:黑 8:白
//                                                       // 自测 0:红 1:橙 2:黄 3:绿 4:青 5:蓝 6:紫 7:粉 8:黑

//             Judge.clientGraphicDraw.graphic_name[0] = 0;
//             Judge.clientGraphicDraw.graphic_name[1] = 0;
//             Judge.clientGraphicDraw.graphic_name[2] = 0;
//             Judge.clientGraphicDraw.graphic_name[3] = 0;
//             Judge.clientGraphicDraw.graphic_name[4] = 1;

//             Judge.clientGraphicDraw.start_x = 960;
//             Judge.clientGraphicDraw.start_y = 540;
//             Judge.clientGraphicDraw.radius  = 100;

//             id         = Protocol_Interact_Id_Client_Graph;
//             dataLength = PROTOCOL_PACK_0301_HEADER + PROTOCOL_PACK_LENGTH_0301_Client_Graph;

//             Judge.mode = MODE_CLIENT_DATA;
//         } break;

//         default:
//             break;
//         }

//         length = PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength;

//         // DMA重启
//         DMA_Disable(USART6_Tx);
//         Protocol_Pack(&Node_Judge, dataLength, id);
//         DMA_Enable(USART6_Tx, length);

//         // 发送频率
//         vTaskDelayUntil(&LastWakeTime, intervalms);

//     }
//     vTaskDelete(NULL);
// }

void Task_Board_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.004;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    uint16_t commandID; // 通讯ID
    uint16_t length;

    while (1) {

        // 修改数据
        commandID                     = 0x501;
        ProtocolData.boardAlpha.data1 = remoteData.lx;
        ProtocolData.boardAlpha.data2 = remoteData.ly;
        ProtocolData.boardAlpha.data3 = remoteData.rx;
        ProtocolData.boardAlpha.data4 = remoteData.ry;

        // 发送数据
        Bridge_Send_Protocol(&Node_Host, commandID);
        // DMA_Disable(USART6_Tx);
        // length = Protocol_Pack(&Node_Judge, id);
        // DMA_Enable(USART6_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN+length);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ProtocolData.boardAlpha.data1 * 1000;
        // DebugData.debug2 = ProtocolData.boardBeta.data4 * 1000;
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
        ProtocolData.autoaimData.yaw_angle_diff   = 1.23;
        ProtocolData.autoaimData.pitch_angle_diff = 4.56;
        ProtocolData.autoaimData.biu_biu_state    = 7;

        // DMA重启
        DMA_Disable(UART8_Tx);
        dataLength = Protocol_Pack(&Node_Host, id);
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

    while (1) {
        Bridge_Send_Motor(&BridgeData, SafetyMode);
        vTaskDelayUntil(&LastWakeTime, intervalms); // 发送频率
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
    // while (!remoteData.state) {
    // }

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);

    // DMA发送任务
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);
    xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Vision_Communication, "Task_Vision_Communication", 500, NULL, 6, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
