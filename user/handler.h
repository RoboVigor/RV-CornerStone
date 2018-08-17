#ifndef __HANDLER_H
#define __HANDLER_H

#ifdef  __HANDLER_GLOBALS
#define __HANDLER_EXT
#else
#define __HANDLER_EXT   extern
#endif

__HANDLER_EXT TaskHandle_t taskHandler_DBUS;
__HANDLER_EXT TaskHandle_t taskHandler_Debug;
__HANDLER_EXT TaskHandle_t taskHandler_Blink;

__HANDLER_EXT QueueHandle_t queue_test;

#endif
