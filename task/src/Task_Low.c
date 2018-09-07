#include "main.h"

/**
 * @brief  LED闪烁任务 确认存活
 * @param  void *Parameters
 * @return void
 */

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}

/**
 * @brief  底盘@甩锅小车
 * @param  void *Parameters
 * @return void
 */

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    int        WheelSpeedRes[4], Buffer[4];
    int        kFeedback = 2 * 3.14 / 60;
    int        debugValue=0;

    while (1) {

        if (USART_RX_STA==0x8000){
           printf("%d \r\n", USART_RX_STA & 0x8000);
            // for(int i=0)
        }
        vTaskDelayUntil(&LastWakeTime, 50);
        continue;


        yawSpeedFeed = mpu6500_data.gz / 16.4;
        yawAngleFeed = EulerAngle.Yaw;
        PANAnglePIDInit(&YawAnglePID, 15, 0, 0);
        PanYawSpeedPIDInit(&YawSpeedPID, 2, 0, 0);
        PANSpeedPIDInit(&CM1PID, 12, 0, 0);
        PANSpeedPIDInit(&CM2PID, 12, 0, 0);
        PANSpeedPIDInit(&CM3PID, 12, 0, 0);
        PANSpeedPIDInit(&CM4PID, 12, 0, 0);

        PanAnglePID(&YawAnglePID, DBusData.ch1, yawAngleFeed);
        PanYawSpeedPID(&YawSpeedPID, YawAnglePID.PIDout, yawSpeedFeed);
        Chassis_Set_Wheel_Speed(DBusData.ch4, DBusData.ch3, YawSpeedPID.PIDout); //设定XYZ三个轴的速度

        Chassis_Update_Mecanum_Data(Buffer); //麦轮的解算

        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED); //限幅

        PID_Set_Pan_Speed(&CM1PID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback);
        PID_Set_Pan_Speed(&CM2PID, WheelSpeedRes[1], Motor_Feedback.motor202Speed * kFeedback);
        PID_Set_Pan_Speed(&CM3PID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        PID_Set_Pan_Speed(&CM4PID, WheelSpeedRes[3], Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, CM1PID.PIDout, -CM2PID.PIDout, -CM3PID.PIDout, CM4PID.PIDout);

        vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}
