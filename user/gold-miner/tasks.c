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
        // SafetyMode = (RIGHT_SWITCH_BOTTOM && LEFT_SWITCH_BOTTOM);
        SafetyMode = SafetyMode ? 1 : (ABS(remoteData.rx) > 440);
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
    float              interval     = 0.002;               // 任务运行间隔 s
    int                intervalms   = interval * 1000;     // 任务运行间隔 ms
    ProtocolInfo_Type *protocolInfo = Protocol_Get_Info_Handle(0x501);
    extern DMA_Type    DMA_Table[10];

    int8_t i = 0;
    while (1) {
        i++;
        // 修改数据
        ProtocolData.jointState.base_joint_position     = Motor_BaseJoint.angle;
        ProtocolData.jointState.base_joint_speed        = Motor_BaseJoint.speed;
        ProtocolData.jointState.shoulder_joint_position = Motor_ShoulderJoint.angle;
        ProtocolData.jointState.shoulder_joint_speed    = Motor_ShoulderJoint.speed;
        ProtocolData.jointState.elbow_joint_position    = Motor_ElbowJoint.angle;
        ProtocolData.jointState.elbow_joint_speed       = Motor_ElbowJoint.speed;
        ProtocolData.jointState.wrist_joint_1_position  = Motor_WristJoint1.angle;
        ProtocolData.jointState.wrist_joint_1_speed     = Motor_WristJoint1.speed;
        ProtocolData.jointState.wrist_joint_2_position  = Motor_WristJoint2.angle;
        ProtocolData.jointState.wrist_joint_2_speed     = Motor_WristJoint2.speed;

        // PID_WristJoint1Speed.i = CHOOSEL(1, 2, 3) * CHOOSER(100, 1000, 0);
        // PID_WristJoint1Speed.p = CHOOSEL(1, 4, 7) * CHOOSER(0.01, 0.1, 1);
        // PID_WristJoint2Angle.p = CHOOSEL(1, 3, 10) * CHOOSER(0.1, 1, 10);
        // PID_WristJoint1Speed.p = CHOOSER(4000, 5500, 7000);

        // 建议在main.c中使用Bridge_Send_Protocol
        // Bridge_Send_Protocol_Once(&Node_Host, 0x501);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        DebugData.debug1 = PID_WristJoint1Angle.target * 1000;
        DebugData.debug2 = PID_WristJoint1Angle.feedback * 1000;
        DebugData.debug3 = PID_WristJoint1Angle.output;
        // DebugData.debug2 = ProtocolData.boardAlpha.data2 * 1000;
        // DebugData.debug3 = Node_Host.sendSeq;
        // DebugData.debug3 = Node_Host.sendLock;
        // DebugData.debug4 = protocolInfo->receiveCount;
    }
    vTaskDelete(NULL);
}

void Task_Manipulator(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float base_Jointangle, base_Jointspeed, shoulder_Jointangle, shoulder_Jointspeed, elbow_Jointangle, elbow_Jointspeed;
    float wrist_Joint_1angle, wrist_Joint_1speed, wrist_Joint_2angle, wrist_Joint_2speed;

    // PID_Init(&PID_BaseJointAngle, 40, 3, 0, 16384, 16384);
    // PID_Init(&PID_BaseJointSpeed, 0, 0, 0, 16384, 16384);
    // PID_Init(&PID_ShoulderJointAngle, 40, 3, 0, 16384, 16384);
    // PID_Init(&PID_ShoulderJointSpeed, 0, 0, 0, 16384, 16384);
    // PID_Init(&PID_ElbowAngle, 40, 3, 0, 16384, 16384);
    // PID_Init(&PID_ElbowSpeed, 0, 0, 0, 16384, 16384);
    PID_Init(&PID_WristJoint1Angle, 0.3, 0, 0, 16384, 16384);
    PID_Init(&PID_WristJoint1Speed, 4000, 100, 0, 16384, 4000);
    PID_Init(&PID_WristJoint2Angle, 80, 0, 0, 400, 0);
    PID_Init(&PID_WristJoint2Speed, 30, 0.75, 0, 15000, 3000);

    while (1) {

        // PID_Calculate(&PID_BaseJointSpeed, PID_BaseJointSpeed.target, Motor_BaseJoint.speed * RPM2RPS);
        // PID_Calculate(&PID_ShoulderJointSpeed, PID_ShoulderJointSpeed.target, Motor_ShoulderJoint.speed * RPM2RPS);
        // PID_Calculate(&PID_ElbowSpeed, PID_ElbowSpeed.target, Motor_ElbowJoint.speed * RPM2RPS);
        // PID_Calculate(&PID_WristJoint1Speed, PID_WristJoint1Speed.target, Motor_WristJoint1.speed * RPM2RPS);
        // PID_Calculate(&PID_WristJoint2Speed, PID_WristJoint2Speed.target, Motor_WristJoint2.speed * RPM2RPS);

        PID_WristJoint1Angle.target = ABS(remoteData.lx) > 100 ? remoteData.lx / 300 * 45 : 0;
        PID_WristJoint2Angle.target = ABS(remoteData.ly) > 100 ? remoteData.ly / 300 * 15 : 0;

        // PID_Calculate(&PID_BaseJointAngle, PID_BaseJointAngle.target, Motor_BaseJoint.angle);
        // PID_Calculate(&PID_BaseJointSpeed, PID_BaseJointAngle.output, Motor_BaseJoint.speed * RPM2RPS);
        // PID_Calculate(&PID_ShoulderJointAngle, PID_ShoulderJointAngle.target, Motor_ShoulderJoint.angle);
        // PID_Calculate(&PID_ShoulderJointSpeed, PID_ShoulderJointAngle.output, Motor_ShoulderJoint.speed * RPM2RPS);
        // PID_Calculate(&PID_ElbowAngle, PID_ElbowAngle.target, Motor_ElbowJoint.angle);
        // PID_Calculate(&PID_ElbowSpeed, PID_ElbowAngle.output, Motor_ElbowJoint.speed * RPM2RPS);
        PID_Calculate(&PID_WristJoint1Angle, PID_WristJoint1Angle.target, Motor_WristJoint1.angle);
        PID_Calculate(&PID_WristJoint1Speed, PID_WristJoint1Angle.output, Motor_WristJoint1.speed * RPM2RPS);
        PID_Calculate(&PID_WristJoint2Angle, PID_WristJoint2Angle.target, Motor_WristJoint2.angle);
        if (ABS(remoteData.ry) > 100) {
            PID_WristJoint2Angle.output    = remoteData.ry / 330 * 5 * 30;
            Motor_WristJoint2.positionBias = Motor_WristJoint2.position;
            Motor_WristJoint2.round        = 0;
        }
        PID_Calculate(&PID_WristJoint2Speed, PID_WristJoint2Angle.output, Motor_WristJoint2.speed * RPM2RPS);

        //输出电流到电调
        Motor_BaseJoint.input     = PID_BaseJointSpeed.output;
        Motor_ShoulderJoint.input = PID_ShoulderJointSpeed.output;
        Motor_ElbowJoint.input    = PID_ElbowSpeed.output;
        Motor_WristJoint1.input   = PID_WristJoint1Speed.output;
        Motor_WristJoint2.input   = PID_WristJoint2Speed.output;

        vTaskDelayUntil(&LastWakeTime, intervalms); // 发送频率
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
