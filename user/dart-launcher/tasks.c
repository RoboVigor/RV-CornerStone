/**
 * @brief 甩锅小车
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            vTaskSuspendAll();
            while (1) {
                Can_Send(CAN1, 0x200, 0, 0, 0, 0);
                vTaskDelay(2);
            }
        }
        vTaskDelay(2);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 运动模式
    int   mode           = 2; // 底盘运动模式,1直线,2转弯
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
        Chassis_Limit_Rotor_Speed(&ChassisData, 700);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调(安全起见默认注释此行)
        // Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_DMA_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        int      i;
        int      index = 0;
        uint8_t *send_p;

        while (DMA_GetFlagStatus(DMA1_Stream0, DMA_IT_TCIF0) != SET) {
        }
        DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
        DMA_Cmd(DMA1_Stream0, DISABLE);

        DMA_Send_Buffer[index++] = 1;
        DMA_Send_Buffer[index++] = 2;
        DMA_Send_Buffer[index++] = 3;

        send_p = DMA_Send_Buffer;
        for (i = 0; i < index; i++) {
            *send_p++ = DMA_Send_Buffer[i];
        }

        DMA_SetCurrDataCounter(DMA1_Stream0, index);
        DMA_Cmd(DMA1_Stream0, ENABLE);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = DMA_Receive_Buffer[0];
        // DebugData.debug2 = DMA_Receive_Buffer[1];
        // DebugData.debug3 = DMA_Receive_Buffer[2];
    }
    vTaskDelete(NULL);
}

void Task_Client_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        int      index = 0;
        uint16_t dataLength;

        while (DMA_GetFlagStatus(DMA2_Stream6, DMA_IT_TCIF6) != SET) {
        }
        DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
        DMA_Cmd(DMA2_Stream6, DISABLE);

        switch (Judge.mode) {
        case MODE_CLIENT_DATA: {
            // 客户端自定义数据
            Judge.clientCustomData.data_cmd_id = Protocol_Interact_Id_Client_Data;
            Judge.clientCustomData.send_id     = Judge.robotState.robot_id;
            Judge.clientCustomData.receiver_id = (Judge.clientCustomData.send_id % 10) | (Judge.clientCustomData.send_id / 10) << 4 | (0x01 << 8);

            Judge.clientCustomData.data1 = 1;
            Judge.clientCustomData.data2 = 1.1;
            Judge.clientCustomData.data3 = 1.11;
            Judge.clientCustomData.masks = 0x3c;

            dataLength = Protocol_Pack_Length_0301_Header + Protocol_Pack_Length_0301_Client_Data;

            Protocol_Pack(&Judge, dataLength, Protocol_Interact_Id_Client_Data);

            Judge.mode = MODE_ROBOT_INTERACT;
        } break;

        case MODE_ROBOT_INTERACT: {
            // 机器人间通信
            Judge.robotInteractiveData[0].data_cmd_id = 0x0200;
            Judge.robotInteractiveData[0].send_id     = Judge.robotState.robot_id;
            Judge.robotInteractiveData[0].receiver_id = 1;

            Judge.robotInteractiveData[0].transformer[index++].F = 1;
            Judge.robotInteractiveData[0].transformer[index++].F = 1.1;
            Judge.robotInteractiveData[0].transformer[index++].F = 1.11;
            Judge.robotInteractiveData[0].transformer[index++].F = 1.111;

            dataLength = Protocol_Pack_Length_0301_Header + index * sizeof(float);

            Protocol_Pack(&Judge, dataLength, 0x0200);

            Judge.mode = MODE_CLIENT_GRAPH;
        } break;

        case MODE_CLIENT_GRAPH: {
            // 客户端自定义图形
            Judge.clientGraphicDraw.data_cmd_id = Protocol_Interact_Id_Client_Graph;
            Judge.clientGraphicDraw.send_id     = Judge.robotState.robot_id;
            Judge.clientGraphicDraw.receiver_id = (Judge.clientCustomData.send_id % 10) | (Judge.clientCustomData.send_id / 10) << 4 | (0x01 << 8);

            Judge.clientGraphicDraw.operate_tpye = 1; // 0:空操作 1:增加图形 2:修改图形 3:删除单个图形 5:删除一个图层 6:删除所有图形
            Judge.clientGraphicDraw.graphic_tpye = 3; // 0:空形 1:直线 2:矩形 3:正圆 4:椭圆 5:弧形 6:文本（ASCII 字码）
            Judge.clientGraphicDraw.layer        = 5; // 图层0-9
            Judge.clientGraphicDraw.width        = 4; // 线宽
            Judge.clientGraphicDraw.color        = 4; // 官方 0:红/蓝 1:黄 2:绿 3:橙 4:紫 5:粉 6:青 7:黑 8:白
                                                      // 自测 0:红 1:橙 2:黄 3:绿 4:青 5:蓝 6:紫 7:粉 8:黑

            Judge.clientGraphicDraw.graphic_name[0] = 0;
            Judge.clientGraphicDraw.graphic_name[1] = 0;
            Judge.clientGraphicDraw.graphic_name[2] = 0;
            Judge.clientGraphicDraw.graphic_name[3] = 0;
            Judge.clientGraphicDraw.graphic_name[4] = 1;

            Judge.clientGraphicDraw.start_x = 960;
            Judge.clientGraphicDraw.start_y = 540;
            Judge.clientGraphicDraw.radius  = 100;

            dataLength = Protocol_Pack_Length_0301_Header + Protocol_Pack_Length_0301_Client_Graph;

            Protocol_Pack(&Judge, dataLength, Protocol_Interact_Id_Client_Graph);

            Judge.mode = MODE_CLIENT_DATA;
        } break;

        default:
            break;
        }

        DMA_SetCurrDataCounter(DMA2_Stream6, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
        DMA_Cmd(DMA2_Stream6, ENABLE);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = Judge.robotInteractiveData[1].transformer[0].F * 1000;
        // DebugData.debug2 = Judge.robotInteractiveData[1].transformer[1].F * 1000;
        // DebugData.debug3 = Judge.robotInteractiveData[1].transformer[2].F * 1000;
        // DebugData.debug4 = Judge.robotInteractiveData[1].transformer[3].F * 1000;
    }
    vTaskDelete(NULL);
}

void Task_Board_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        uint16_t dataLength;

        // disable DMA
        while (DMA_GetFlagStatus(DMA1_Stream1, DMA_IT_TCIF1) != SET) {
        }
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
        DMA_Cmd(DMA1_Stream1, DISABLE);

        // 板间通信
        Board.boardInteractiveData[0].data1 = 1.11;
        Board.boardInteractiveData[0].data2 = 2.22;
        Board.boardInteractiveData[0].data3 = 3.33;
        Board.boardInteractiveData[0].data4 = 4.44;
        Board.boardInteractiveData[0].data5 = 5.55;

        dataLength = Protocol_Pack_Length_0302;

        Protocol_Pack(&Board, dataLength, Protocol_Interact_Id_Board);

        // enable DMA
        DMA_SetCurrDataCounter(DMA1_Stream1, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
        DMA_Cmd(DMA1_Stream1, ENABLE);

        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = Board.boardInteractiveData[1].data1 * 1000;
        // DebugData.debug2 = Board.boardInteractiveData[1].data2 * 1000;
        // DebugData.debug3 = Board.boardInteractiveData[1].data3 * 1000;
        // DebugData.debug4 = Board.boardInteractiveData[1].data4 * 1000;
        // DebugData.debug5 = Board.boardInteractiveData[1].data5 * 1000;
    }
    vTaskDelete(NULL);
}

void Task_Vision_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        int      index = 0;
        uint16_t dataLength;

        // disable DMA
        while (DMA_GetFlagStatus(DMA1_Stream3, DMA_IT_TCIF3) != SET) {
        }
        DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
        DMA_Cmd(DMA1_Stream3, DISABLE);

        // 视觉通信
        Ps.visionInteractiveData.transformer[index].U16[1]   = 0x6666;
        Ps.visionInteractiveData.transformer[index++].U16[2] = 0x6666;

        dataLength = index * sizeof(float);

        Protocol_Pack(&Ps, dataLength, Protocol_Interact_Id_Vision);

        // enable DMA
        DMA_SetCurrDataCounter(DMA1_Stream3, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
        DMA_Cmd(DMA1_Stream3, ENABLE);

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

void Task_Sys_Init(void *Parameters) {

    // 初始化全局变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData);

    // 调试任务
#if DEBUG_ENABLED
    // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    //xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);

    // DMA发送任务
    // xTaskCreate(Task_DMA_Send, "Task_DMA_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Vision_Communication, "Task_Vision_Communication", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
