#include "Driver_Fsm.h"

void Fsm_Init(Fsm_t *pFsm, FsmTable_t *pTable) {
    pFsm->pFsmTable = pTable;
}

void Fsm_Update(Fsm_t *pFsm, int event) {
    FsmTable_t *pActTable = pFsm->pFsmTable;
    void (*eventActFun)() = 0; //函数指针初始化为空
    int nextState;
    int currentState = pFsm->curState;
    int tableSize    = pFsm->size;
    int flag         = 0; // 标识是否满足条件
    int i;

    /*查找符合当前事件及状态的状态表项*/
    for (i = 0; i < tableSize; i++) {
        // 判断该项是否和当前状态和事件匹配
        if (event == pActTable[i].event && currentState == pActTable[i].currentState) {
            flag        = 1;
            eventActFun = pActTable[i].eventActFun;
            nextState   = pActTable[i].nextState;
            break;
        }
    }

    if (flag) //如果满足条件了
    {
        /*动作执行*/
        if (eventActFun) {
            eventActFun();
        }

        //跳转到下一个状态
        pFsm->curState = nextState;
    } else {
        printf("there is no match\n");
    }
}