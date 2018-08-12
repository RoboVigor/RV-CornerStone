#include "main.h"

int main( void ) {

    //创建系统初始化任务
    xTaskCreate( Task_SysInit, "Task_SysInit", 400, NULL, 1, NULL );

    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while ( 1 ) {
    }
}
