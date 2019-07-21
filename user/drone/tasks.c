/**
 * @brief 无人机代码
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2 && remoteData.switchLeft==2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(2);
    }
    vTaskDelete(NULL);
}


void Task_Snail(void *Parameters) {
    // snail摩擦轮任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    float dutyCycleStart  = 0.376; // 起始占空比为37.6
    float dutyCycleMiddle = 0.446; // 启动需要到44.6
    float dutyCycleEnd    = 0.600; // 加速到你想要的占空比

    float dutyCycleSnailTarget    = dutyCycleStart; //目标占空比
    float dutyCycleSnailProgress1 = 0;              //存储需要的两个过程（初始到启动，启动到你想要的速度）
    float dutyCycleSnailProgress2 = 0;

    int snailState = 0; //标志启动完后需要的延时
    
    
    int lastMouseDataRight=0;

    snailStart = 0; 

    while (1) {
        LASER_ON; //打开激光
        if(controlMode == 1){
            if (remoteData.switchLeft == 1) {
                snailStart=0;
            }else{
                snailStart=1;
            }
        }else if(controlMode == 2){
            if (mouseData.pressRight == 1 && snailStart == 0 && lastMouseDataRight == 0) {
                snailStart=1;
                lastMouseDataRight = mouseData.pressRight;
            }else if(mouseData.pressRight == 1 && snailStart == 1&& lastMouseDataRight == 0){
                snailStart=0;
                lastMouseDataRight = mouseData.pressRight;
            }else{
					lastMouseDataRight = mouseData.pressRight;
				}

        }

        if (snailStart == 0) {
            // 摩擦轮不转
            dutyCycleSnailTarget    = dutyCycleStart;
            dutyCycleSnailProgress1 = 0;
            dutyCycleSnailProgress2 = 0;
        } else {
            // 启动摩擦轮
            if (dutyCycleSnailProgress1 <= 1) {
                //初始状态
                dutyCycleSnailTarget = RAMP(dutyCycleStart, dutyCycleMiddle,
                                            dutyCycleSnailProgress1); //斜坡上升
                dutyCycleSnailProgress1 += 0.01f;
            } else {
                if (!snailState) { //初始状态停留100ms
                    vTaskDelay(200);
                    snailState = 1;
                } else {
                    if (dutyCycleSnailProgress2 <= 1) {
                        //启动状态
                        dutyCycleSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd,
                                                    dutyCycleSnailProgress2); //斜坡上升
                        dutyCycleSnailProgress2 += 0.005f;
                    }
                }
            }
        }
        // 设置占空比
        PWM_Set_Compare(&PWM_Snail1, dutyCycleSnailTarget * 1250);
        PWM_Set_Compare(&PWM_Snail2, dutyCycleSnailTarget * 1250);
        // 任务延时
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

void Task_Gimbal(void *Parameters) {
    //云台
    TickType_t LastWakeTime = xTaskGetTickCount(); //时钟
    //初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 25, 0, 0, 4000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 40, 0, 0, 5000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 30, 0, 0, 5000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 8, 0.1, 0, 4000, 1000);

    int yawMode       = 1; // yaw,pitch切换转动和停止状态
    int pitchMode     = 1;
    int yawLastMode   = 1;
    int pitchLastMode = 1;

    float yawTargetAngle   = 0;
    float pitchTargetAngle = 0;

    while (1) {
        if(controlMode == 1){
        yawMode   = ABS(remoteData.rx) < 30 ? 1 : 2;
        pitchMode = ABS(remoteData.ry) < 30 ? 1 : 2;
        }else{

        }
        
        //设定输入target
        if (yawMode != yawLastMode) {
            yawTargetAngle = Motor_Yaw.angle; // 更新角度PID目标值
            yawLastMode    = yawMode;         // 更新lastMode
        }

        if (yawMode == 1) {
            PID_Calculate(&PID_Cloud_YawAngle, yawTargetAngle,
                          Motor_Yaw.angle); // 计算航向角角度PID
            PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output,
                          Motor_Yaw.speed); // 计算航向角角速度PID
        } else if (Motor_Yaw.angle > 33 && remoteData.rx > -30) {
            PID_Calculate(&PID_Cloud_YawAngle, 34,
                          Motor_Yaw.angle); // 计算航向角角度PID
            PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output,
                          Motor_Yaw.speed); // 计算航向角角速度PID

        } else if (Motor_Yaw.angle < -33 && remoteData.rx < 30) {
            PID_Calculate(&PID_Cloud_YawAngle, -34,
                          Motor_Yaw.angle); // 计算航向角角度PID
            PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output,
                          Motor_Yaw.speed); // 计算航向角角速度PID

        } else {
            PID_Calculate(&PID_Cloud_YawSpeed, remoteData.rx * 0.3,
                          Motor_Yaw.speed); // 计算航向角角速度PID
        }

        if (pitchMode != pitchLastMode) {
            pitchTargetAngle = Motor_Pitch.angle; // 更新角度PID目标值
            pitchLastMode    = pitchMode;         // 更新lastMode
        }
        if (pitchMode == 1) {
            PID_Calculate(&PID_Cloud_PitchAngle, pitchTargetAngle,
                          Motor_Pitch.angle); // 计算航向角角度PID
            PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output,
                          Motor_Pitch.speed); // 计算航向角角速度PID
        } else {
            PID_Calculate(&PID_Cloud_PitchSpeed, remoteData.ry,
                          Motor_Pitch.speed); // 计算航向角角速度PID
        }

        Can_Send(CAN1, 0x1FF, PID_Cloud_YawSpeed.output, PID_Cloud_PitchSpeed.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 5);
    }
}

void Task_Fire(void *Parameters) {       //拨弹轮
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float rpm2rps      = 3.14 / 60;           // 转子的转速(round/min)换算成角速度(rad/s)
    float r            = 0.0595;
    // uint8_t    startCounter = 0;                   // 启动模式计数器

#define LASER_ON GPIO_SetBits(GPIOG, GPIO_Pin_13) // 激光开启
    // #define LASER_OFF GPIO_ResetBits(GPIOG, GPIO_Pin_13) // 激光关闭

    // 标志位
    uint8_t frictState = 0;
    uint8_t stirState  = 0;
    uint8_t stirFlag   = 0;

    int turnNumber = 1;
    int lastSwitch = 2; //
    // 常量
    // float frictSpeed = -24 / 0.0595 * 2 * 60 / 2 / 3.14;
    // 摩擦轮线速度(mps)转转速(rpm)
    float frictSpeed = 336; // 336 rad/s
    float stirSpeed  = 36 * 36;
    float stirAmpre  = 0;

    // PID 初始化
    
    PID_Init(&PID_StirSpeed, 0.2, 0.003, 0, 8000, 6000); // 1.8
    
    int stirstate = 0; //播弹轮开启为1 关闭为0
    u32 stircount = 0; //堵转计数器 保证反转
    int stopstate = 0; //堵转模式 1为堵转 0为正常
    float currentTarget=0;
   

    while (1) {
        if(controlMode == 1){
            if (remoteData.lx < -60) {
                stirstate=1; //正转
            }else if (remoteData.lx > 60){
                stirstate=-1;
            }else if (remoteData.lx < 60 && remoteData.lx > -60){
                stirstate=0;
            }
        }else{
            if(mouseData.pressLeft == 1 && snailStart == 1){
                stirstate=1;
            }else if(mouseData.pressLeft == 0){
                stirstate=0;
            }
        }

        //拨弹轮 PID 控制
       
        // if (stirstate == 1) { //正转
        //     PID_Calculate(&PID_StirSpeed, -6500.0, Motor_Stir.speed);
        //     Can_Send(CAN1, 0x200, 0, 0, PID_StirSpeed.output, 0);
        // } else if (stirstate == -1) { //高速倒转，清弹
        //     PID_Calculate(&PID_StirSpeed, remoteData.lx * 15, Motor_Stir.speed);
        //     Can_Send(CAN1, 0x200, 0, 0, PID_StirSpeed.output, 0);
        // } else if (stirstate == 0) {
        //     PID_Calculate(&PID_StirSpeed, 0, Motor_Stir.speed);
        //     Can_Send(CAN1, 0x200, 0, 0, PID_StirSpeed.output, 0);
        // }
        if(stirstate == 1){
            if(currentTarget <= -2300 ){
                stopstate=1;
            }
            if(stopstate == 1 && stircount < 20){
                currentTarget =2000;
                Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);
                stircount += 1;
            }else if(stopstate == 1 && stircount >= 20 && stircount <= 30 ){
                //stircount = 0;
                //stopstate = 0;
                PID_Calculate(&PID_StirSpeed, -1000, Motor_Stir.speed);
                currentTarget=PID_StirSpeed.output;
                stircount += 1;
                Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);
            }else if(stopstate == 1 && stircount>= 30 ){
                currentTarget=0;
                stircount = 0;
                stopstate = 0;
                Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);
            }else if(stopstate == 0){
            PID_Calculate(&PID_StirSpeed, -5500.0, Motor_Stir.speed);
            currentTarget=PID_StirSpeed.output;
            
            Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);
            }
        }else if(stirstate == 0){
            currentTarget=0;
            
            Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);

        }
        else if(stirstate == -1){
            currentTarget = 2000;
            Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);
            
        }
        debug1 = currentTarget;
        debug2 = stircount;
        
        
        

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    // 初始化全局变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    // Gyroscope_Init(&Gyroscope_EulerData);

    TIM5CH1_CAPTURE_STA = 0;
    while (!remoteData.state) {
        // debug1=Motor_Pitch.position;
        // debug2=Motor_Yaw.position;
    }

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 800, NULL, 5, NULL);
    xTaskCreate(Task_Snail, "Task_Snail", 500, NULL, 6, NULL);
    xTaskCreate(Task_Fire, "Task_Fire", 500, NULL, 7, NULL);
    xTaskCreate(Task_Control, "Task_Control", 500, NULL, 4,NULL);
    // 完成使命
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

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        // Keyboard_Check(&CTRL, &remoteData, KEY_CTRL);

        if (remoteData.switchRight == 1) {
            controlMode = 1; //遥控器模式
        } else if (remoteData.switchRight != 1) {
            controlMode = 2; //键鼠模式
        }
        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}