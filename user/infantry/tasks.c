/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"
#include "gimbal_tuning.h"

void Task_MecArm(void *Parameters){
    TickType_t xLastWakeTime=xTaskGetTickCount();
    const TickType_t xFrequency=pdMS_TO_TICKS(1);

    while (1)
    {
        // Unitree_Motor_Safety_Test(&Unitree_Bridge, 0, 20);
		// Unitree_Motor_Safety_Test(&Unitree_Bridge, 1, 20);

        // Joint_Command_t test;
        // test.pos_des=PI/2;
        // test.pos_pid=&Motor6020_MecArm_3_PosPID;
        // test.speed_pid=&Motor6020_MecArm_3_SpeedPID;
        // Arm_Joint_3.Set_Target(&Arm_Joint_3,&test);
        
#if !GIMBAL_TUNING_ENABLED
        // Legacy VOFA owns USART3 only outside the tuning build.
        vf_debug.data[0]=PI/2.0f;
        vf_debug.data[1]=(Motor6020_MecArm_0_0.angle)/180.0f*PI;
        vf_debug.data[2]=(Motor6020_MecArm_0_1.angle)/180.0f*PI;
        Vofa_Debug_Send(3);
#endif

        MecArm_Calculate(&MecArm,PI/2.0f,500.0f,0,PI/2.0f,0);
        MecArm_Control(&MecArm);
        GimbalTuning_Service();
        //电机发送
		Unitree_Circular_Send(&Unitree_Bridge);
        Bridge_Send_Motor(&BridgeData, SafetyMode);

        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}