#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "BSP_GPIO.h"
#include "BSP_NVIC.h"
#include "BSP_UART.h"
#include "BSP_DMA.h"
#include "BSP_CAN.h"
#include "BSP_TIM.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_PanController.h"
#include "Driver_HookController.h"
#include "Driver_Armour.h"
#include "Driver_Angular.h"
#include "mpu6500_interrupt.h"
#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define FORWARD 1
#define BACKWARD -1
#define MAXWHEELSPEED 1150