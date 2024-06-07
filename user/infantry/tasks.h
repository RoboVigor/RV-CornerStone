#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

void Task_Sys_Init(void *Parameters);      // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);         // 跑马灯
void Task_Startup_Music(void *Parameters); // 开机音乐
void Task_Chassis(void *Parameters);       // 底盘运动
void Task_Safe_Mode(void *Parameters);     // 安全模式
void Task_Gimbal(void *Parameters);        // 云台任务
void Task_Fire_Stir(void *Parameters);     // 发射机构 (拨弹轮)
void Task_Fire_Frict(void *Parameters);    // 发射机构 (摩擦轮)
void Task_Control(void *Parameters);
void Task_Can_Send(void *Parameters);
void Task_Host(void *Parameters);
void Task_Wait(void *Parameters);

#endif
