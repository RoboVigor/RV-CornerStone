/**
 * @brief 任务
 */
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"

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

void Task_Communication(void *Parameters) {
    TickType_t         LastWakeTime = xTaskGetTickCount(); // 时钟
    float              interval     = 0.001;               // 任务运行间隔 s
    int                intervalms   = interval * 1000;     // 任务运行间隔 ms
    ProtocolInfo_Type *protocolInfo = Protocol_Get_Info_Handle(0x501);
    extern DMA_Type    DMA_Table[10];
    while (1) {

        // 修改数据
        ProtocolData.boardAlpha.data1 = 1;
        ProtocolData.boardAlpha.data2 = 2;
        ProtocolData.boardAlpha.data3 = 3;
        ProtocolData.boardAlpha.data4 = 4;

        // 建议在main.c中使用Bridge_Send_Protocol
        // Bridge_Send_Protocol_Once(&Node_Host, 0x501);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ProtocolData.boardAlpha.data1 * 1000;
        // DebugData.debug2 = ProtocolData.boardAlpha.data2 * 1000;
        // DebugData.debug3 = Node_Host.sendSeq;
        // DebugData.debug3 = Node_Host.sendLock;
        // DebugData.debug4 = protocolInfo->receiveCount;
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
