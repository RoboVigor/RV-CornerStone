#ifndef __HANDLE_H
#define __HANDLE_H

#ifdef  __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT   extern
#endif

__HANDLE_EXT TaskHandle_t TaskHandler_Dbus;
__HANDLE_EXT TaskHandle_t TaskHandler_Debug;
__HANDLE_EXT TaskHandle_t TaskHandler_Blink;

__HANDLE_EXT QueueHandle_t QueueTest;

#endif
