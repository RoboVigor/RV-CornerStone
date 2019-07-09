#ifndef __DRIVER_FSM_H
#define __DRIVER_FSM_H

#include "stm32f4xx_conf.h"

// 状态表
typedef struct FsmTable_s {
    int event;             //事件
    int currentState;      //当前状态
    void (*eventActFun)(); //函数指针
    int nextState;         //下一个状态
} FsmTable_t;

// 状态机
typedef struct Fsm_s {
    int         curState;  //当前状态
    FsmTable_t *pFsmTable; //状态表
    int         size;      //表的项数
} Fsm_t;

// 示例状态
// enum year_state { SPRING = 1, SUMMER, AUTUMN, WINTER };

// 示例事件
// enum year_event {
//     EVENT1 = 1,
//     EVENT2,
//     EVENT3,
//     EVENT4,
// };

// 示例状态表
// FsmTable_t year_table[] = {
// {到来的事件，当前的状态，将要要执行的函数，下一个状态}
// {EVENT1, SPRING, summer_thing, SUMMER},
// {EVENT2, SUMMER, autumn_thing, AUTUMN},
// {EVENT3, AUTUMN, winter_thing, WINTER},
// {EVENT4, WINTER, spring_thing, SPRING},
// };

// 函数
void Fsm_Init(Fsm_t *pFsm, FsmTable_t *pTable);
void Fsm_Update(Fsm_t *pFsm, int event);

#endif