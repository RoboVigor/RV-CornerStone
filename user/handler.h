#ifndef __HANDLER_H
#define __HANDLER_H

#ifdef  __HANDLER_GLOBALS
#define __HANDLER_EXT
#else
#define __HANDLER_EXT   extern
#endif

__HANDLER_EXT TaskHandle_t TaskHandler_DBUS;
__HANDLER_EXT TaskHandle_t TaskHandler_USART3;
__HANDLER_EXT TaskHandle_t TaskHandler_Blink;

#endif
