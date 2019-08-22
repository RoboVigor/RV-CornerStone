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
    int   last_Chassis_State      = 0;
    int   in_parallel             = 0;
    float vx                      = 0;
    float vy                      = 0;
    float vw                      = 0;
    float yawAngleTarget          = 0; // 目标值
    float yawAngle, yawSpeed;          // 反馈值
    Chassis_Detect_Parallel = 0;
    Chassis_Detect          = 0;
    Chassis_State           = -1;

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, 32, 1.1, 0, 12000, 6000);
    PID_Init(&PID_LBCM, 32, 1.1, 0, 12000, 6000);
    PID_Init(&PID_RBCM, 32, 1.1, 0, 12000, 6000);
    PID_Init(&PID_RFCM, 32, 1.1, 0, 12000, 6000);

    // 初始化航向角角度PID和角速度PID
    // PID_Init(&PID_YawAngle, 50, 0, 0, 1700, 850); // 50
    PID_Init(&PID_YawAngle, 5, 0.01, 0, 1700, 850);
    PID_Init(&PID_YawSpeed, 3, 0, 0, 4000, 2000); // 3  2.25 0.04

    // 初始化底盘
    Chassis_Init(&ChassisData);

    while (1) {
        // 设置反馈值
        yawAngle = Gyroscope_EulerData.yaw;    // 航向角角度反馈
        yawSpeed = ImuData.gz / GYROSCOPE_LSB; // 航向角角速度反馈

        // 更新运动模式
        mode = ABS(remoteData.rx) < 5 ? 1 : 2;

        if (Chassis_Detect == 1 && Fsm_Ready == 1) {
            Chassis_State           = CHASSIS_DETECT_RIGHT;
            last_Chassis_State      = CHASSIS_DETECT_RIGHT;
            Chassis_Parallel_Finish = 0;
            in_parallel             = 0;
        } else if (Chassis_Detect == 2 && Fsm_Ready == 1) {
            Chassis_State           = CHASSIS_DETECT_LEFT;
            last_Chassis_State      = CHASSIS_DETECT_LEFT;
            Chassis_Parallel_Finish = 0;
            in_parallel             = 0;
            // 登岛
            // } else if (Fsm_Ready != 1 && Chassis_Delanding_State == 1) {
            //     Chassis_State      = CHASSIS_DELANDING;
            //     last_Chassis_State = CHASSIS_DELANDING;
            //     in_parallel        = 0;
        } else {
            Chassis_State           = CHASSIS_NORMAL;
            last_Chassis_State      = CHASSIS_NORMAL;
            Chassis_Parallel_Finish = 0;
            in_parallel             = 0;
        }

        if (last_Chassis_State != Chassis_State) {
            yawAngleTarget = yawAngle;
        }

        if (Chassis_State == CHASSIS_NORMAL) {
            // 上坡用(虽然实战中没用过)
            if (mouseData.pressLeft == 1) {
                PID_Init(&PID_YawAngle, 50, 0, 0, 1700, 850);
            } else {
                PID_Init(&PID_YawAngle, 5.15, 0.01, 0, 1700, 850);
            }

            // 通过遥控器切换运动模式
            if (remoteData.switchRight == 1) {
                // 正走
                vx = -remoteData.lx / 660.0f * 4;
                vy = remoteData.ly / 660.0f * 2;
                vw = PID_YawSpeed.output / 4000.0f * 6;
            } else if (remoteData.switchRight == 3) {
                // 侧走
                vx = remoteData.ly / 660.0f * 4;
                vy = remoteData.lx / 660.0f * 2;
                vw = PID_YawSpeed.output / 4000.0f * 6;
            } else if (remoteData.switchRight == 2) {
                // 倒走
                vx = remoteData.lx / 660.0f * 4;
                vy = -remoteData.ly / 660.0f * 2;
                vw = PID_YawSpeed.output / 4000.0f * 6;
            }
            // 切换运动模式
            if (mode != lastMode) {
                yawAngleTarget = yawAngle; // 更新角度PID目标值
                lastMode       = mode;     // 更新lastMode
            }

            // 根据运动模式计算PID
            if (mode == 1) {
                PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);
                PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
            } else {
                PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeed); // 计算航向角角速度PID
            }

            // 慢速模式
            if (remoteData.switchRight == 3) {
                vx = vx / 6;
                vy = vy / 6;
                vw = vw / 4;
            }

            Chassis_Update(&ChassisData, vx, vy, vw);
        } else if (Chassis_State == CHASSIS_DETECT_LEFT) {
            if (T_State2 == 0 && T_State1 == 1 && T_State3 == 0 && T_State4 == 1 && TV_Ready == 1) {
                Chassis_Detect = 0;
                Detected_State = 1;
                // PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);
                // PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
                PID_Calculate(&PID_YawSpeed, 0, yawSpeed);

                Chassis_Update(&ChassisData, 0, 0, PID_YawSpeed.output / 660.0f);
            } else {
                // PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);
                // PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
                PID_Calculate(&PID_YawSpeed, 0, yawSpeed);

                Chassis_Update(&ChassisData, 0, -0.25, PID_YawSpeed.output / 660.0f);
            }
        } else if (Chassis_State == CHASSIS_DETECT_RIGHT) {
            // 读单边提高响应速度
            if (T_State2 == 0 && T_State1 == 1 && T_State3 == 0 && T_State4 == 1 && TV_Ready == 1) {
                Chassis_Detect = 0;
                Detected_State = 1;
                // 赋返回速度
                Chassis_Update(&ChassisData, 0, 0, PID_YawSpeed.output / 660.0f);
            } else {
                Chassis_Update(&ChassisData, 0, 0.25, 0);
            }
            // 登岛
            // } else if (Chassis_State == CHASSIS_DELANDING) {
            //     if (Distance_Delanding_Parallel1 < 2500 && Distance_Delanding_Parallel2 < 2500) {
            //         PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);
            //         PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
            //         Chassis_Update(&ChassisData, 0.1, 0, (float) PID_YawSpeed.output / 660.0f);
            //     } else if (Distance_Delanding_Parallel1 - Distance_Delanding_Parallel2 > 50) {
            //         PID_Calculate(&PID_YawSpeed, 20, yawSpeed); // 计算航向角角速度PID
            //         Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
            //     } else if (Distance_Delanding_Parallel1 - Distance_Delanding_Parallel2 < -50) {
            //         PID_Calculate(&PID_YawSpeed, 20, yawSpeed); // 计算航向角角速度PID
            //         Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
            //     } else {
            //         PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);
            //         PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
            //         Chassis_Update(&ChassisData, 0, 0, (float) PID_YawSpeed.output / 660.0f);
            //         Chassis_Delanding_Parallel_Over = 1;
            //     }
        }

        Chassis_Calculate_Rotor_Speed(&ChassisData); // 麦轮解算
        // 麦轮解算&限幅,获得轮子转速
        if (Chassis_State == CHASSIS_NORMAL) {
            Chassis_Limit_Rotor_Speed(&ChassisData, 800);
        } else {
            Chassis_Limit_Rotor_Speed(&ChassisData, 300);
        }

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

    int Fsm_State              = 0;
    int take_mode              = 0; // 0自动 1手动
    int three_box_state        = 0; // 三箱计数
    int quick_three_box_enable = 0; // 快速三箱模式(没调过)
    int detect_distance        = 0; // 检测距离(近:1, 远:2)
    TV_Ready                   = 0; // 伸出运动完成时为1
    Detected_Direction         = 0; // 运动方向
    Fsm_TIM14_Cnt              = 0; // 计时器的数值(单位:ms)
    Fsm_TIM14_State            = 0; // 开启或关闭计时器
    Fsm_Reset                  = 1; // 状态机为初始状态

    while (1) {
        // 当条件允许取弹机构启动(抬升机构抬起到取弹位置)
        if (Fsm_Ready == 1) {
            switch (Fsm_State) {
            case 0:
                Take_Reset();
                Fsm_TIM14_State = 0;
                three_box_state = 0;
                Fsm_Reset       = 1;

                // 外加按键控制弹射机构
                if (keyboardData.Z == 1) {
                    CATAPULT_ON;
                } else {
                    CATAPULT_OFF;
                }

                // 自动
                if (keyboardData.E == 1 && keyboardData.Ctrl != 1) {
                    Fsm_State = 1;
                    take_mode = 0;
                    // 手动
                } else if (keyboardData.W == 1 && keyboardData.Ctrl != 1) {
                    Fsm_State = 1;
                    take_mode = 1;
                }
                break;
            case 1:
                Fsm_Reset = 0;
                CATAPULT_OFF;
                // 确定取弹距离
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Shift != 1 && keyboardData.Ctrl != 1) {
                    Take_TV_1();
                    detect_distance = 1;
                } else if ((((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Shift == 1 && keyboardData.Ctrl != 1) || detect_distance == 2) {
                    Take_TV_2();
                    detect_distance = 2;
                }

                if (TV_Ready == 1 && take_mode == 0) {
                    if (Detected_State == 1) {
                        Fsm_State = 3;
                    } else {
                        Fsm_State = 2;
                    }
                } else if (TV_Ready == 1) {
                    if (keyboardData.A == 1) {
                        Fsm_State = 3;
                    }
                }

                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                }

                break;
            case 2:
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    if (Detected_State == 1) {
                        Fsm_State = 3;
                    } else {
                        Take_Chassis_Detect_Right();
                        if (keyboardData.A == 1) {
                            Fsm_State = 3;
                        }
                    }
                }
                break;
            case 3:
                Take_Start_Get();

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    // 转动完成
                    if (TR_Ready == 1) {
                        Fsm_State = 4;
                    }
                }
                break;
            case 4:
                Take_ON();

                // 开启计时器
                Fsm_TIM14_State = 1;

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    // 计时完成
                    if (Fsm_TIM14_Cnt >= 1000) {
                        Fsm_State       = 5;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 5:
                Take_Up();

                Fsm_TIM14_State = 1;

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 1000) {
                        Fsm_State = 6;
                    }
                }
                break;
            case 6:
                Take_TV_Progress();

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    Fsm_State = 7;
                }
                break;
            case 7:
                Take_Rotate_OFF();

                Fsm_TIM14_State = 1;

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    if (TR_Ready == 0) {
                        Fsm_State       = 8;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 8:
                Take_OFF();

                Fsm_TIM14_State = 1;

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 1500) {
                        Fsm_State       = 9;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 9:
                Take_Catapult_On();

                Fsm_TIM14_State = 1;

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    if (Fsm_TIM14_Cnt >= 500) {
                        Fsm_State       = 13;
                        Fsm_TIM14_State = 0;
                    }
                }
                break;
            case 13:
                Take_Down();

                // 重置状态机
                if (((keyboardData.E == 1) || (keyboardData.W == 1)) && keyboardData.Ctrl == 1) {
                    Fsm_State = 0;
                } else {
                    // 快速取弹模式(没调过)
                    if (quick_three_box_enable != 1) {
                        Fsm_State = 1;
                    } else {
                        three_box_state++;
                        if (three_box_state == 1) {
                            TH_Angle_State = 1;
                            TV_Out         = 2;
                            if (TH_Ready == 1 && TV_Ready == 1) {
                                Fsm_State = 3;
                            }
                        } else if (three_box_state == 2) {
                            TH_Angle_State = 1;
                        } else {
                            Fsm_State              = 0;
                            three_box_state        = 0;
                            quick_three_box_enable = 1;
                        }
                    }
                }

                Detected_State = 0;
                break;
            default:
                Fsm_State = 0;
                break;
            }
        }

        vTaskDelayUntil(&LastWakeTime, 8);
    }
    vTaskDelete(NULL);
}

void Task_Take_Vertical_GuideWheel(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // TV_Out 0: 初始位置 1：近排位置 2：远程位置 3：撤回补弹位置
    int   targetAngle       = 0;
    int   last_TV_Out       = 0; // 上一次伸出的状态
    float TV_Progress       = 0; // 斜坡函数增量
    float TV_AngleTarget    = 0;
    float TV_Ramp_Start     = 0;
    float guide_wheel_speed = 0; // 引导轮目标速度
    TV_Out                  = 0; // 近排:1 远排:2

    PID_Init(&PID_TV_Angle, 8, 0, 0, 2000, 1000);
    PID_Init(&PID_TV_Speed, 4, 0.1, 0, 12000, 5000);

    PID_Init(&PID_LGW, 5, 0.001, 0, 5000, 2500);
    PID_Init(&PID_RGW, 5, 0.001, 0, 5000, 2500);

    while (1) {
        // 放弃斜坡函数
        if (TV_Out == 2) {
            targetAngle = -1102;
            // if (last_TV_Out != 2) {
            //     TV_Progress   = 0;
            //     TV_Ramp_Start = Motor_TV.angle;
            // }
            // last_TV_Out = 2;
        } else if (TV_Out == 1) {
            targetAngle = -360;
            // if (last_TV_Out != 1) {
            //     TV_Progress   = 0;
            //     TV_Ramp_Start = Motor_TV.angle;
            // }
            // last_TV_Out = 1;
        } else if (TV_Out == 0) {
            targetAngle = 0;
            // if (last_TV_Out != 0) {
            //     TV_Progress   = 0;
            //     TV_Ramp_Start = Motor_TV.angle;
            // }
            // last_TV_Out = 0;
        } else if (TV_Out == 3) {
            targetAngle = -300;
            // if (last_TV_Out != 3) {
            //     TV_Progress   = 0;
            //     TV_Ramp_Start = Motor_TV.angle;
            // }
            // last_TV_Out = 3;
        }

        if ((ABS(Motor_TV.angle - targetAngle) < 4) && TV_Out != 0) {
            TV_Ready = 1;
        } else {
            TV_Ready = 0;
        }

        // TV_AngleTarget = RAMP(TV_Ramp_Start, targetAngle, TV_Progress);
        // if (TV_Progress < 1) {
        //     TV_Progress += 0.05f;
        // }

        // PID_Calculate(&PID_TV_Angle, TV_AngleTarget, Motor_TV.angle);
        PID_Calculate(&PID_TV_Angle, targetAngle, Motor_TV.angle);
        PID_Calculate(&PID_TV_Speed, PID_TV_Angle.output, Motor_TV.speed);

        if (Landing_State == 1) {
            guide_wheel_speed = remoteData.ly / 2.2;
        } else {
            guide_wheel_speed = 0;
        }

        PID_Calculate(&PID_LGW, guide_wheel_speed, Motor_LGW.speed);
        PID_Calculate(&PID_RGW, -guide_wheel_speed, Motor_RGW.speed);

        Can_Send(CAN2, 0x200, PID_LGW.output, PID_RGW.output, PID_TV_Speed.output, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Take_Rotate(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    float targetspeed = 0;
    int   last_tr_get = 0;
    TR_Get            = 0; // 爪子目标位置(2:转出)
    TR_Ready          = 0; // 旋转电机到位

    PID_Init(&PID_Rotate_Left_Angle, 15, 0.05, 0, 2400, 1200);  // 6.5 0.01  10.2 0.0035 1200
    PID_Init(&PID_Rotate_Left_Speed, 13, 0.53, 0, 16000, 8000); // 10.5 0.48  8.5 0.25  12000
    PID_Init(&PID_Rotate_Right_Angle, 15, 0.05, 0, 2400, 1200);
    PID_Init(&PID_Rotate_Right_Speed, 13, 0.53, 0, 16000, 8000);

    while (1) {

        if (TR_Get == 2) {
            if (Motor_Rotate_Left.angle > -170) {
                targetspeed = -1000;
            } else if (Motor_Rotate_Left.angle < -176) {
                targetspeed = 800;
            } else {
                targetspeed = 0;
                TR_Ready    = 1;
            }
        } else {
            if (Motor_Rotate_Left.angle > 0) {
                targetspeed = -800;
            } else if (Motor_Rotate_Left.angle < -8) {
                targetspeed = 1000;
            } else {
                targetspeed = 0;
                TR_Ready    = 0;
            }
        }

        PID_Calculate(&PID_Rotate_Left_Speed, targetspeed, Motor_Rotate_Left.speed);

        Can_Send(CAN2, 0x1FF, PID_Rotate_Left_Speed.output, -PID_Rotate_Left_Speed.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Landing_Fsm(void *Parameters) {
    TickType_t LastWakeTime      = xTaskGetTickCount();
    uint8_t    bangMode          = 0; // 上岛传感器切换， 0 前端， 1 后端
    uint8_t    powerMode         = 0; // 电源
    int        Fsm_Landing_State = 0;
    int        landing_position  = 0;

    while (1) {
        if ((keyboardData.D == 1 || keyboardData.C == 1) && keyboardData.Ctrl == 1) {
            LANDING_OFF;
            Fsm_Landing_State = 0;
            Landing_State     = 0;
        }
        switch (Fsm_Landing_State) {
        case 0:
            if (keyboardData.D == 1 && keyboardData.Ctrl != 1 && keyboardData.Shift == 1 && TU_Up == 0) {
                LANDING_ON;
                Fsm_Landing_State = 0;
                Landing_State     = 1;

            } else if (keyboardData.C == 1 && keyboardData.Ctrl != 1 && keyboardData.Shift == 1 && TU_Up == 0) {
                LANDING_ON;
                Fsm_Landing_State = 3;
                Landing_State     = 1;
            }
            break;
        case 1:
            if (Distance_Landing_Behind > 2500 || Distance_Landing_Front > 2500) {
                Fsm_Landing_State = 2;
            }
            break;
        case 2:
            if (Distance_Landing_Behind < 1000) {
                LANDING_OFF;
                Fsm_Landing_State = 0;
            }
        case 3:
            if (Distance_Landing_Behind > 2500 || Distance_Landing_Front > 2500) {
                Fsm_Landing_State = 4;
            }
            break;
        case 4:
            if (Distance_Landing_Front > 2500) {
                LANDING_OFF;
                Landing_State     = 0;
                Fsm_Landing_State = 0;
            }
            break;

        default:
            break;
        }

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Supply(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        // 补给舵机(抬升到最高时启动)
        if (TU_Up == 2) {
            if (keyboardData.R == 1 && keyboardData.Ctrl != 1) {
                TIM_SetCompare3(TIM4, 5);
            } else if (keyboardData.R == 1 && keyboardData.Ctrl == 1) {
                TIM_SetCompare3(TIM4, 14);
            }
        } else {
            // 抬升机构不在最高时缩回
            TIM_SetCompare3(TIM4, 14);
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

void Task_Rescue(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {

        // 救援爪
        if (keyboardData.S == 1 && keyboardData.Ctrl != 1) {
            RESCUE_HOOK_DOWN;
        } else if (keyboardData.S == 1 && keyboardData.Ctrl == 1) {
            RESCUE_HOOK_UP;
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}

void Task_Visual(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    Move_Mode = 0; // 图传对准位置

    while (1) {

        // 图传位置
        if (remoteData.switchRight == 1) {
            PWM_Set_Compare(&PWM_Image_Yaw, 5);
            Move_Mode = 0;
        } else {
            PWM_Set_Compare(&PWM_Image_Yaw, 18);
            Move_Mode = 1;
        }
        // 后方摄像头角度(图传向前时自动抬起)
        if (Move_Mode == 1) {
            if (remoteData.switchLeft == 2) {
                PWM_Set_Compare(&PWM_Visual_Rescue, 15);
            } else if (remoteData.switchLeft == 3) {
                PWM_Set_Compare(&PWM_Visual_Rescue, 10);
            } else if (remoteData.switchLeft == 1) {
                PWM_Set_Compare(&PWM_Visual_Rescue, 5);
            }
        } else {
            PWM_Set_Compare(&PWM_Visual_Rescue, 5);
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}

void Task_Distance_Sensor(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // 通过PWM波读取距离信息
    uint16_t temp1          = 0;
    uint16_t temp2          = 0;
    uint16_t last_distance  = 0;
    Distance_Landing_Front  = 0;
    Distance_Landing_Behind = 0;

    while (1) {
        // 获得激光传感器距离
        Distance_Landing_Front = TIM2CH1_CAPTURE_VAL;

        Distance_Landing_Behind = TIM9CH1_CAPTURE_VAL;

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Board_Communication_Send(void *Parameters) {
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
        Board.boardInteractiveData[0].data1 = Distance_Delanding_Parallel1;
        Board.boardInteractiveData[0].data2 = Distance_Delanding_Parallel2;
        Board.boardInteractiveData[0].data3 = Distance_Delanding_Parallel3;
        Board.boardInteractiveData[0].data4 = Distance_Delanding_Parallel4;
        Board.boardInteractiveData[0].data5 = Distance_Delanding_Parallel5;

        dataLength = Protocol_Pack_Length_0302;

        Protocol_Pack(&Board, dataLength, Protocol_Interact_Id_Board);

        // enable DMA
        DMA_SetCurrDataCounter(DMA1_Stream1, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
        DMA_Cmd(DMA1_Stream1, ENABLE);

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Board_Communication_Recive(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        Distance_Delanding_Parallel1 = Board.boardInteractiveData[1].data1;
        Distance_Delanding_Parallel2 = Board.boardInteractiveData[1].data2;
        Distance_Delanding_Parallel3 = Board.boardInteractiveData[1].data3;
        Distance_Delanding_Parallel4 = Board.boardInteractiveData[1].data4;
        Distance_Delanding_Parallel5 = Board.boardInteractiveData[1].data5;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Upthrow_Horizontial(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // 取弹机构平移
    float TH_TargetSpeed = 0;
    float TH_TargetAngle = 0;
    TH_Reset             = 0;
    Detected_State       = 0;
    TH_Move              = 0; // Right:1 Left:2
    TH_Angle_State       = 0; // 0: 初始位置(第一箱) 1：近排第二箱 2：远排中间

    // TU_Up 0: 初始位置 1：一段抬升位置 2：二段抬升位置
    float upthrowAngleTarget = 0;
    float upthrowProgress    = 0;
    float TH_Ramp_Start      = 0;
    int   last_TU_state      = 0; // 上一次抬升机构位置
    TU_Up                    = 0;
    Fsm_Ready                = 0; // 确保在一段抬升中开启取弹状态机

    // PID_Init(&PID_TH_Angle, 1, 0, 0, 144, 72);
    PID_Init(&PID_TH_Speed, 35, 0.5, 0, 5000, 2000); // 25 0.1

    PID_Init(&PID_Upthrow1_Angle, 18, 0.015, 0, 340, 170);   // 18 0.015
    PID_Init(&PID_Upthrow1_Speed, 30, 1, 0, 10000, 5000);    // 30 1
    PID_Init(&PID_Upthrow2_Angle, 24, 0.02, 0, 290, 145);    // 24 0.018
    PID_Init(&PID_Upthrow2_Speed, 40, 0.85, 0, 10000, 5000); // 35 0.5

    while (1) {

        if (keyboardData.Q == 1 && keyboardData.Ctrl != 1 && keyboardData.Shift != 1) {
            TU_Up     = 1;
            Fsm_Ready = 1;
        } else if (keyboardData.Q == 1 && keyboardData.Ctrl != 1 && keyboardData.Shift == 1) {
            TU_Up     = 2;
            Fsm_Ready = 0;
        } else if (keyboardData.Q == 1 && keyboardData.Ctrl == 1 && Fsm_Reset == 1 && keyboardData.Shift != 1) {
            TU_Up     = 0;
            Fsm_Ready = 0;
        }

        if (TU_Up == 1) {
            if (last_TU_state != 1) {
                upthrowProgress = 0;
                TH_Ramp_Start   = Motor_Upthrow1.angle;
            }
            upthrowAngleTarget = RAMP(TH_Ramp_Start, 475, upthrowProgress);
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
                upthrowProgress += 0.05f;
                // upthrowProgress += 1.0f;
            }
            last_TU_state = 0;
        } else if (TU_Up == 2) {
            if (last_TU_state != 2) {
                upthrowProgress = 0;
                TH_Ramp_Start   = Motor_Upthrow1.angle;
            }
            upthrowAngleTarget = RAMP(TH_Ramp_Start, 640, upthrowProgress);
            if (upthrowProgress < 1) {
                upthrowProgress += 0.08f;
                // upthrowProgress += 1.0f;
            }
            last_TU_state = 2;
        }

        // 主从控制
        PID_Calculate(&PID_Upthrow1_Angle, upthrowAngleTarget, Motor_Upthrow1.angle);
        PID_Calculate(&PID_Upthrow1_Speed, PID_Upthrow1_Angle.output, Motor_Upthrow1.speed * RPM2RPS);
        PID_Calculate(&PID_Upthrow2_Angle, -Motor_Upthrow1.angle, Motor_Upthrow2.angle);
        PID_Calculate(&PID_Upthrow2_Speed, PID_Upthrow2_Angle.output, Motor_Upthrow2.speed * RPM2RPS);

        // 取弹机构平移(运动时C610电调报警,停用了)

        // 单速度环控制
        // if (((T_State2 == 0 && T_State1 == 1) || (T_State3 == 0 && T_State4 == 1)) && TV_Ready == 1) {
        //     TH_Move        = 0;
        //     Detected_State = 1;
        // }

        // if ((LSR_State == 1 && TH_Move != 2) || (LSL_State == 1 && TH_Move != 1)) {
        //     TH_Move = 0;
        // }

        // if (TH_Move == 1) {
        //     TH_TargetSpeed = 250;
        // } else if (TH_Move == 2) {
        //     TH_TargetSpeed = -250;
        // } else if (TH_Move == 0) {
        TH_TargetSpeed = 0;
        // }

        // if (TH_Angle_State == 1) {
        //     TH_TargetAngle = 300;
        // } else if (TH_Angle_State == 2) {
        //     TH_TargetAngle = 150;
        // } else {
        //     TH_TargetAngle = 0;
        // }

        // if (ABS(Motor_TH.angle - TH_TargetAngle) < 4) {
        //     TH_Ready = 1;
        // } else {
        //     TH_Ready = 0;
        // }

        // PID_Calculate(&PID_TH_Angle, TH_TargetAngle, Motor_TH.angle);
        // PID_Calculate(&PID_TH_Speed, PID_TH_Angle.output, Motor_TH.speed * RPM2RPS);
        PID_Calculate(&PID_TH_Speed, TH_TargetSpeed, Motor_TH.speed * RPM2RPS);

        Can_Send(CAN1, 0x1FF, PID_Upthrow1_Speed.output, PID_Upthrow2_Speed.output, PID_TH_Speed.output, 0);

        DebugA = Motor_Upthrow1.angle;
        DebugB = Motor_Upthrow2.angle;

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

    while (1) {
        // 取弹光电开关读取(消抖)
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

void Task_Limit_Switch(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int last_limit_switch_left  = 0;
    int limit_switch_left       = 0;
    int last_limit_switch_right = 0;
    int limit_switch_right      = 0;

    while (1) {
        // 限位开关
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
        if (KTV_Play(Music_Sky)) break;
        vTaskDelayUntil(&LastWakeTime, 350);
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
    // xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // Structure
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    xTaskCreate(Task_Distance_Sensor, "Task_Distance_Sensor", 400, NULL, 3, NULL);
    xTaskCreate(Task_Take_Fsm, "Task_Take_Fsm", 400, NULL, 4, NULL);
    xTaskCreate(Task_Take_Rotate, "Task_Take_Rotate", 400, NULL, 4, NULL);
    xTaskCreate(Task_Landing_Fsm, "Task_Landing_Fsm", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Board_Communication_Send, "Task_Board_Communication_Send", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Board_Communication_Recive, "Task_Board_Communication_Recive", 400, NULL, 3, NULL);
    xTaskCreate(Task_Supply, "Task_Supply", 400, NULL, 3, NULL);
    xTaskCreate(Task_Rescue, "Task_Rescue", 400, NULL, 3, NULL);
    xTaskCreate(Task_Visual, "Task_Visual", 400, NULL, 3, NULL);
    xTaskCreate(Task_Take_Vertical_GuideWheel, "Task_Take_Vertical_GuideWheel", 400, NULL, 3, NULL); // 前后伸缩
    xTaskCreate(Task_Optoelectronic_Input_Take, "Task_Optoelectronic_Input_Take", 400, NULL, 3, NULL);
    xTaskCreate(Task_Upthrow_Horizontial, "Task_Upthrow_Horizontial", 400, NULL, 3, NULL); // 抬升与平移
    // xTaskCreate(Task_Limit_Switch, "Task_Limit_Switch", 400, NULL, 3, NULL);
    /* End */

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}

// 补弹：R 取弹：自动：近：E 远：Shift E 手动：近：W 远：Shift W 抬升：Q 救援：S 登岛：Shift+D 下岛：Shift+C