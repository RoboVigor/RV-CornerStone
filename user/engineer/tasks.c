/**
 * @brief 甩锅小车
 * @version 0.8.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2 && remoteData.switchLeft == 2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(2);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    // 运动模式
    int   mode                    = 2; // 底盘运动模式,1直线,2转弯
    int   lastMode                = 2; // 上一次的运动模式
    int   Chassis_Parallel_Finish = 0;
    float yawAngleTarget          = 0; // 目标值
    float yawAngle, yawSpeed;          // 反馈值
    Chassis_Detect_Parallel = 0;
    Chassis_State           = -1;

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, 28, 1.2, 0, 10000, 4000);
    PID_Init(&PID_LBCM, 28, 1.2, 0, 10000, 4000);
    PID_Init(&PID_RBCM, 28, 1.2, 0, 10000, 4000);
    PID_Init(&PID_RFCM, 28, 1.2, 0, 10000, 4000);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 5, 0, 0, 4000, 1000);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    while (1) {
        // 设置反馈值
        yawAngle = Gyroscope_EulerData.yaw;    // 航向角角度反馈
        yawSpeed = ImuData.gz / GYROSCOPE_LSB; // 航向角角速度反馈

        if (remoteData.switchRight == 1 || remoteData.switchRight == 3) {
            // 更新运动模式
            mode = ABS(remoteData.rx) < 5 ? 1 : 2;

            // 对平行
            if (remoteData.switchLeft == 3) {
                if (Distance1 > 3000 && Distance2 > 3000) {
                    PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID
                    // 设置底盘总体移动速度
                    Chassis_Update(&ChassisData, 0.4, 0, (float) PID_YawSpeed.output / 660.0f);
                } else if (Chassis_Parallel_Finish == 1) {
                    PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID
                    Chassis_Update(&ChassisData, 0.4, 0, (float) PID_YawSpeed.output / 660.0f);
                    if (Distance1 < 150 && Distance2 < 150) {
                        PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID
                        Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                    }
                } else {
                    if (Distance1 - Distance2 > 20) {
                        PID_Calculate(&PID_YawSpeed, 20, yawSpeed); // 计算航向角角速度PID
                        Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                    } else if (Distance1 - Distance2 < -20) {
                        PID_Calculate(&PID_YawSpeed, -20, yawSpeed); // 计算航向角角速度PID
                        Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                    } else {
                        Chassis_Parallel_Finish = 1;
                    }
                }
                // 设置左右
            } else if (Chassis_Detect == 1 && remoteData.switchRight == 1) {
                Chassis_State           = CHASSIS_DETECT_RIGHT;
                Chassis_Parallel_Finish = 0;
            } else if (Chassis_Detect == 2 && remoteData.switchRight == 1) {
                Chassis_State           = CHASSIS_DETECT_LEFT;
                Chassis_Parallel_Finish = 0;
                // 正常运动
            } else if (remoteData.switchRight != 1 && Chassis_Delanding_State == 1) {
                Chassis_State = CHASSIS_DELANDING;
            } else {
                Chassis_State           = CHASSIS_NORMAL;
                Chassis_Parallel_Finish = 0;
            }

            if (Chassis_State == CHASSIS_NORMAL) {
                // 切换运动模式
                if (mode != lastMode) {
                    PID_YawAngle.output_I = 0;        // 清空角度PID积分
                    PID_YawSpeed.output_I = 0;        // 清空角速度PID积分
                    yawAngleTarget        = yawAngle; // 更新角度PID目标值
                    lastMode              = mode;     // 更新lastMode
                }

                // 根据运动模式计算PID
                if (mode == 1) {
                    PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID
                } else {
                    PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeed); // 计算航向角角速度PID
                }

                // 设置底盘总体移动速度
                Chassis_Update(&ChassisData, (float) -remoteData.lx / 660.0f, (float) remoteData.ly / 660.0f, (float) PID_YawSpeed.output / 1320.0f);
                // 重新赋初值
                Chassis_State = -1;
            } else if (Chassis_State == CHASSIS_DETECT_LEFT) {
                if (((T_State2 == 0 && T_State1 == 1) || (T_State3 == 0 && T_State4 == 1)) && TV_Ready == 1) {
                    Chassis_Detect = 0;
                    Detected_State = 1;
                    PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID

                    Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                } else {
                    PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID

                    Chassis_Update(&ChassisData, 0, -0.15, (float) PID_YawSpeed.output / 660.0f);
                }
            } else if (Chassis_State == CHASSIS_DETECT_RIGHT) {
                // 读单边提高响应速度
                if ((T_State2 == 0 && T_State1 == 1) || (T_State3 == 0 && T_State4 == 1) && TV_Ready == 1) {
                    Chassis_Detect = 0;
                    Detected_State = 1;
                    // 赋返回速度
                    Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                } else {
                    Chassis_Update(&ChassisData, 0, 0.15, 0);
                }
            } else if (Chassis_State == CHASSIS_DELANDING) {
                if (Distance_Delanding_Parallel1 < 2500 && Distance_Delanding_Parallel2 < 2500) {
                    PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID
                    Chassis_Update(&ChassisData, 0.1, 0, (float) PID_YawSpeed.output / 660.0f);
                } else if (Distance_Delanding_Parallel1 - Distance_Delanding_Parallel2 > 50) {
                    PID_Calculate(&PID_YawSpeed, 20, yawSpeed); // 计算航向角角速度PID
                    Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                } else if (Distance_Delanding_Parallel1 - Distance_Delanding_Parallel2 < -50) {
                    PID_Calculate(&PID_YawSpeed, 20, yawSpeed); // 计算航向角角速度PID
                    Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                } else {
                    PID_Calculate(&PID_YawSpeed, 0, yawSpeed); // 计算航向角角速度PID
                    Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
                    Chassis_Delanding_Parallel_Over = 1;
                }
            }
        }

        Chassis_Calculate_Rotor_Speed(&ChassisData); // 麦轮解算
        // 麦轮解算&限幅,获得轮子转速
        Chassis_Limit_Rotor_Speed(&ChassisData, 300);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调(安全起见默认注释此行)
        Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Take_Fsm(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int Fsm_State      = 0;
    int six_box_cnt    = 0;
    int a              = 2;
    TV_Ready           = 0;
    Detected_Direction = 0;
    Fsm_TIM14_Cnt      = 0;
    Fsm_TIM14_State    = 0;

    while (1) {
        if (remoteData.switchRight == 1) {
            switch (Fsm_State) {
            case 0:
                Take_Reset();
                Fsm_TIM14_State = 0;
                six_box_cnt     = 0;

                if (remoteData.switchLeft == 1 && remoteData.switchRight != 3) {
                    Fsm_State = 1;
                }
                break;
            case 1:

                if (six_box_cnt < 3) {
                    Take_TV_1();
                } else if (six_box_cnt < 6) {
                    Take_TV_2();
                }

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    Fsm_TIM14_State = 1;
                    if (Fsm_TIM14_Cnt >= 800 * a) {
                        TV_Ready        = 1;
                        Fsm_State       = 2;
                        Fsm_TIM14_State = 0;
                        if (Detected_State == 1) {
                            Fsm_State = 3;
                        } else {
                            Fsm_State = 2;
                        }
                    }
                }
                break;
            case 2:
                if (six_box_cnt < 2) {
                    Take_Chassis_Detect_Left();
                } else if (six_box_cnt < 3) {
                    Take_Horizontal_Left();
                } else if (six_box_cnt < 5) {
                    Take_Chassis_Detect_Right();
                } else if (six_box_cnt < 6) {
                    Take_Horizontal_Right();
                }

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Detected_State == 1) {
                        Fsm_State = 3;
                    }
                }
                break;
            case 3:
                Take_Start_Get();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 2000 * a) {
                        Fsm_State       = 4;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 4:
                Take_ON();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 700 * a) {
                        Fsm_State       = 5;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 5:
                Take_Up();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 2000 * a) {
                        Fsm_State       = 6;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 6:
                Take_TV_Progress();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 1000 * a) {
                        Fsm_State       = 7;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 7:
                Take_Rotate_OFF();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 1000 * a) {
                        Fsm_State       = 8;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 8:
                Take_OFF();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 1500 * a) {
                        Fsm_State       = 9;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 9:
                Take_Catapult_On();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 500 * a) {
                        Fsm_State       = 10;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 10:
                CATAPULT_OFF;
                six_box_cnt++;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    Fsm_State = 11;
                }
                break;
            case 11:
                Take_Down();

                Fsm_TIM14_State = 1;

                if (remoteData.switchLeft == 2 && remoteData.switchRight != 3) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 1000 * a) {
                        Fsm_State       = 1;
                        Fsm_TIM14_State = 0;
                    }
                }

                Detected_State = 0;
                break;
            // case 12:
            // case 13:
            default:
                break;
            }
        }
        DebugA = six_box_cnt;

        vTaskDelayUntil(&LastWakeTime, 8);
    }
    vTaskDelete(NULL);
}

void Task_Take_Vertical(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // TV_Out 0: 初始位置 1：近排位置 2：远程位置 3：撤回补弹位置
    int   targetAngle    = 0;
    int   last_TV_Out    = 0;
    float TV_Progress    = 0;
    float TV_AngleTarget = 0;
    float TV_Ramp_Start  = 0;
    TV_Out               = 0; // 近排:1 远排:2

    PID_Init(&PID_TV_Angle, 7, 0, 0, 4500, 1000);
    PID_Init(&PID_TV_Speed, 1.5, 0.05, 0, 4500, 1000);

    while (1) {
        // // 手动调试伸出
        // if (remoteData.switchLeft == 1) {
        //     TV_Out = 0;
        // } else if (remoteData.switchLeft == 3) {
        //     TV_Out = 1;
        // } else if (remoteData.switchLeft == 2) {
        //     TV_Out = 2;
        // }

        if (TV_Out == 2) {
            targetAngle = -1190;
            if (last_TV_Out != 2) {
                TV_Progress   = 0;
                TV_Ramp_Start = Motor_TV.angle;
            }
            last_TV_Out = 2;
        } else if (TV_Out == 1) {
            targetAngle = -400;
            if (last_TV_Out != 1) {
                TV_Progress   = 0;
                TV_Ramp_Start = Motor_TV.angle;
            }
            last_TV_Out = 1;
        } else if (TV_Out == 0) {
            targetAngle = 0;
            if (last_TV_Out != 0) {
                TV_Progress   = 0;
                TV_Ramp_Start = Motor_TV.angle;
            }
            last_TV_Out = 0;
        } else if (TV_Out == 3) {
            targetAngle = -300;
            if (last_TV_Out != 3) {
                TV_Progress   = 0;
                TV_Ramp_Start = Motor_TV.angle;
            }
            last_TV_Out = 3;
        }

        TV_AngleTarget = RAMP(TV_Ramp_Start, targetAngle, TV_Progress);
        if (TV_Progress < 1) {
            TV_Progress += 0.05f;
        }

        PID_Calculate(&PID_TV_Angle, TV_AngleTarget, Motor_TV.angle);
        PID_Calculate(&PID_TV_Speed, PID_TV_Angle.output, Motor_TV.speed);

        Can_Send(CAN2, 0x200, 0, 0, PID_TV_Speed.output, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Landing_Fsm(void *Parameters) {
    TickType_t LastWakeTime      = xTaskGetTickCount();
    uint8_t    bangMode          = 0; // 上岛传感器切换， 0 前端， 1 后端
    uint8_t    powerMode         = 0; // 电源
    float      guide_wheel_speed = 0;
    int        Fsm_Landing_State = 0;

    // PID 初始化
    PID_Init(&PID_LGW, 5, 0, 0, 4000, 2000);
    PID_Init(&PID_RGW, 5, 0, 0, 4000, 2000);

    while (1) {

        if (remoteData.switchRight == 3) {
            Fsm_Landing_State = 0;
        }

        switch (Fsm_Landing_State) {
        case 0:
            guide_wheel_speed = 0;
            LANDING_OFF;

            if (remoteData.switchLeft == 1 && remoteData.switchRight != 1) {
                Fsm_Landing_State = 1;
            } else if (remoteData.switchLeft == 2 && remoteData.switchRight != 1) {
                Fsm_Landing_State = 3;
            }
            break;
        case 1:
            LANDING_ON;
            guide_wheel_speed = 0.2;
            Fsm_Landing_State = 2;
            break;
        case 2:
            if (Distance_Landing_Behind < 1500) {
                LANDING_OFF;
                guide_wheel_speed = 0;
                Fsm_Landing_State = 0;
            }
            break;
        case 3:
            Chassis_Delanding_State = 1;
            if (Chassis_Delanding_Parallel_Over == 1) {
                Chassis_Delanding_State         = 0;
                Chassis_Delanding_Parallel_Over = 0;
                Fsm_Landing_State               = 4;
            }
            break;
        case 4:
            guide_wheel_speed = -0.2;
            if (Distance_Landing_Front > 2500) {
                guide_wheel_speed = 0;
                LANDING_OFF;
                Fsm_Landing_State = 5;
            }
            break;

        default:
            break;
        }

        // PID 计算
        PID_Increment_Calculate(&PID_LGW, guide_wheel_speed, Motor_LGW.speed * RPM2RPS);
        PID_Increment_Calculate(&PID_RGW, -guide_wheel_speed, Motor_RGW.speed * RPM2RPS);

        // 发送数据至电调
        Can_Send(CAN2, 0x200, PID_LGW.output, PID_RGW.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Supply(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        // Controller
        if (remoteData.keyBoard.keyCode == KEY_R) {
            PWM_Set_Compare(&PWM_Supply1, 15);
            PWM_Set_Compare(&PWM_Supply2, 15);
        } else {
            PWM_Set_Compare(&PWM_Supply1, 5);
            PWM_Set_Compare(&PWM_Supply2, 25);
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

void Task_Rescue(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    uint8_t rescueMode = 0; // 0 救援爪抬起 1 救援爪放下

    while (1) {

        if (remoteData.switchRight == 1) {
            RESCUE_HOOK_UP;
        } else {
            RESCUE_HOOK_DOWN;
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}

void Task_Distance_Sensor(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // 通过PWM波读取距离信息
    uint16_t temp1         = 0;
    uint16_t temp2         = 0;
    uint16_t last_distance = 0;
    Distance1              = 0;
    Distance2              = 0;

    while (1) {
        temp1     = TIM2CH1_CAPTURE_VAL; //得到总的高电平时间
        Distance1 = temp1;               // cm us

        temp2     = TIM5CH1_CAPTURE_VAL; //得到总的高电平时间
        Distance2 = temp2;               // cm

        Distance_Delanding_Parallel1 = TIM3CH3_CAPTURE_VAL;

        Distance_Delanding_Parallel2 = TIM9CH1_CAPTURE_VAL;

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Upthrow_Horizontial(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // TH_Move 0: 初始位置 1：一段抬升位置 2：二段抬升位置
    float TH_TargetSpeed = 0;
    float TH_TargetAngle = 0;
    TH_Reset             = 0;
    Detected_State       = 0;
    TH_Move              = 0; // Right:1 Left:2

    float upthrowAngleTarget = 0;
    float upthrowProgress    = 0;
    float TH_Ramp_Start      = 0;
    int   last_TU_state      = 0;
    TU_Up                    = 0;

    PID_Init(&PID_TH_Speed, 35, 0.5, 0, 5000, 2000); // 25 0.1

    PID_Init(&PID_Upthrow1_Angle, 18, 0.015, 0, 500, 300); //
    PID_Init(&PID_Upthrow1_Speed, 30, 1, 0, 8000, 4000);   //
    PID_Init(&PID_Upthrow2_Angle, 8, 0.018, 0, 500, 300);  //
    PID_Init(&PID_Upthrow2_Speed, 35, 0.5, 0, 8000, 4000); //

    while (1) {
        // 状态机使用
        if (remoteData.switchRight == 1 && remoteData.switchLeft != 1) {
            TU_Up = 1;
        } else if (remoteData.switchRight == 3 && remoteData.switchLeft != 1) {
            TU_Up = 0;
        }

        // // 调PID使用
        // if (remoteData.switchRight == 2) {
        //     TU_Up = 0;
        // } else if (remoteData.switchRight == 3) {
        //     TU_Up = 1;
        // } else if (remoteData.switchRight == 1) {
        //     TU_Up = 2;
        // }

        if (TU_Up == 1) {
            if (last_TU_state != 1) {
                upthrowProgress = 0;
                TH_Ramp_Start   = Motor_Upthrow1.angle;
            }
            upthrowAngleTarget = RAMP(TH_Ramp_Start, 420, upthrowProgress);
            if (upthrowProgress < 1) {
                upthrowProgress += 0.05f;
            }
            last_TU_state = 1;
        } else if (TU_Up == 0) {
            if (last_TU_state != 0) {
                upthrowProgress = 0;
                TH_Ramp_Start   = Motor_Upthrow1.angle;
            }
            upthrowAngleTarget = RAMP(TH_Ramp_Start, 0, upthrowProgress);
            if (upthrowProgress < 1) {
                upthrowProgress += 0.01f;
                // upthrowProgress += 1.0f;
            }
            last_TU_state = 0;
        } else if (TU_Up == 2) {
            if (last_TU_state != 2) {
                upthrowProgress = 0;
                TH_Ramp_Start   = Motor_Upthrow1.angle;
            }
            upthrowAngleTarget = RAMP(TH_Ramp_Start, 670, upthrowProgress);
            if (upthrowProgress < 1) {
                upthrowProgress += 0.08f;
                // upthrowProgress += 1.0f;
            }
            last_TU_state = 2;
        }

        PID_Calculate(&PID_Upthrow1_Angle, upthrowAngleTarget, Motor_Upthrow1.angle);
        PID_Calculate(&PID_Upthrow1_Speed, PID_Upthrow1_Angle.output, Motor_Upthrow1.speed * RPM2RPS);
        PID_Calculate(&PID_Upthrow2_Angle, -Motor_Upthrow1.angle, Motor_Upthrow2.angle);
        PID_Calculate(&PID_Upthrow2_Speed, PID_Upthrow2_Angle.output, Motor_Upthrow2.speed * RPM2RPS);

        // TH

        if (((T_State2 == 0 && T_State1 == 1) || (T_State3 == 0 && T_State4 == 1)) && remoteData.switchLeft == 1 && TV_Ready == 1) {
            TH_Move        = 0;
            Detected_State = 1;
        }

        if ((LSR_State == 1 && TH_Move != 2) || (LSL_State == 1 && TH_Move != 1)) {
            TH_Move = 0;
        }

        if (TH_Move == 1) {
            TH_TargetSpeed = 250;
        } else if (TH_Move == 2) {
            TH_TargetSpeed = -250;
        } else if (TH_Move == 0) {
            TH_TargetSpeed = 0;
        }

        PID_Calculate(&PID_TH_Speed, TH_TargetSpeed, Motor_TH.speed * RPM2RPS);

        Can_Send(CAN1, 0x1FF, PID_Upthrow1_Speed.output, PID_Upthrow2_Speed.output, PID_TH_Speed.output, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Optoelectronic_Input_Take(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int last_take_state1 = 0;
    int last_take_state2 = 0;
    int last_take_state3 = 0;
    int last_take_state4 = 0;
    int take_state1      = 0;
    int take_state2      = 0;
    int take_state3      = 0;
    int take_state4      = 0;
    Find_Box             = 0;

    while (1) {
        // 取弹
        take_state1 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_7);
        if (last_take_state1 != take_state1) {
            vTaskDelay(10);
            if (take_state1 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_7)) {
                T_State1         = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_7);
                last_take_state1 = take_state1;
            }
        }
        take_state2 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_6);
        if (last_take_state2 != take_state2) {
            vTaskDelay(10);
            if (take_state2 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_6)) {
                T_State2         = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_6);
                last_take_state2 = take_state2;
            }
        }
        take_state3 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5);
        if (last_take_state3 != take_state3) {
            vTaskDelay(10);
            if (take_state3 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5)) {
                T_State3         = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5);
                last_take_state3 = take_state3;
            }
        }
        take_state4 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_2);
        if (last_take_state4 != take_state4) {
            vTaskDelay(10);
            if (take_state4 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_2)) {
                T_State4         = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_2);
                last_take_state4 = take_state4;
            }
        }

        vTaskDelayUntil(&LastWakeTime, 5);
    }

    vTaskDelete(NULL);
}

void Task_Optoelectronic_Input_Landing(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int last_landing_front_state1  = 0;
    int last_landing_front_state2  = 0;
    int last_landing_behind_state1 = 0;
    int last_landing_behind_state2 = 0;
    int landing_front_state1       = 0;
    int landing_front_state2       = 0;
    int landing_behind_state1      = 0;
    int landing_behind_state2      = 0;

    while (1) {
        // 登岛
        landing_front_state1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5);
        if (last_landing_front_state1 != landing_front_state1) {
            vTaskDelay(10);
            if (landing_front_state1 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) {
                LF_State1                 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5);
                last_landing_front_state1 = landing_front_state1;
            }
        }
        landing_front_state2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
        if (last_landing_front_state2 != landing_front_state2) {
            vTaskDelay(10);
            if (landing_front_state2 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) {
                LF_State2                 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
                last_landing_front_state2 = landing_front_state2;
            }
        }
        landing_behind_state1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
        if (last_landing_behind_state1 != landing_behind_state1) {
            vTaskDelay(10);
            if (landing_behind_state1 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)) {
                LB_State1                  = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
                last_landing_behind_state1 = landing_behind_state1;
            }
        }
        landing_behind_state2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12);
        if (last_landing_behind_state2 != landing_behind_state2) {
            vTaskDelay(10);
            if (landing_behind_state2 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)) {
                LB_State2                  = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12);
                last_landing_behind_state2 = landing_behind_state2;
            }
        }
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Limit_Switch(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int last_limit_switch_left  = 0;
    int limit_switch_left       = 0;
    int last_limit_switch_right = 0;
    int limit_switch_right      = 0;

    while (1) {
        limit_switch_left = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
        if (last_limit_switch_left != limit_switch_left) {
            vTaskDelay(10);
            if (limit_switch_left == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)) {
                LSL_State              = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                last_limit_switch_left = limit_switch_left;
            }
        }
        limit_switch_right = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
        if (last_limit_switch_right != limit_switch_right) {
            vTaskDelay(10);
            if (limit_switch_right == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)) {
                LSR_State               = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
                last_limit_switch_right = limit_switch_right;
            }
        }

        vTaskDelayUntil(&LastWakeTime, 10);
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
        if (KTV_Play(Music_Earth)) break;
        vTaskDelayUntil(&LastWakeTime, 120);
    }

    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    // 初始化局部变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData);

    // 调试任务
#if DEBUG_ENABLED
    xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // Structure
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    xTaskCreate(Task_Distance_Sensor, "Task_Distance_Sensor", 400, NULL, 3, NULL);
    xTaskCreate(Task_Take_Fsm, "Task_Take_Fsm", 400, NULL, 4, NULL);
    xTaskCreate(Task_Landing_Fsm, "Task_Landing_Fsm", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Supply, "Task_Supply", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Rescue, "Task_Rescue", 400, NULL, 3, NULL);
    xTaskCreate(Task_Take_Vertical, "Task_Take_Vertical", 400, NULL, 3, NULL); // 前后伸缩
    xTaskCreate(Task_Optoelectronic_Input_Take, "Task_Optoelectronic_Input_Take", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Optoelectronic_Input_Landing, "Task_Optoelectronic_Input_Landing", 400, NULL, 3, NULL);
    xTaskCreate(Task_Upthrow_Horizontial, "Task_Upthrow_Horizontial", 400, NULL, 3, NULL); // 抬升与平移
    xTaskCreate(Task_Limit_Switch, "Task_Limit_Switch", 400, NULL, 3, NULL);
    /* End */

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}