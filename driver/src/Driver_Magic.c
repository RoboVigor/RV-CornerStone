#include "Driver_Magic.h"
#include "math.h"
#include "string.h"
#include "sys.h"
#include "config.h"

#define Magic_Init_Handle Magic_Init

//标准库需要的支持函数
struct __FILE {
    int handle;
};

FILE __stdout;

//重定义fputc函数
#ifdef SERIAL_DEBUG_PORT
int fputc(int ch, FILE *f) {
    //循环发送,直到发送完毕
    while ((SERIAL_DEBUG_PORT->SR) & 0X40 == 0) {
    }
    SERIAL_DEBUG_PORT->DR = (u8) ch;
    return ch;
}
#else
int fputc(int ch, FILE *f) {
    return ITM_SendChar(ch);
}

//重定义fgetc函数
volatile int32_t ITM_RxBuffer;
int              fgetc(FILE *f) {
    while (ITM_CheckChar() != 1)
        __NOP();
    return ITM_ReceiveChar();
}
#endif

void Magic_Init(MagicHandle_Type *magic, int defaultValue) {
    magic->defaultValue = defaultValue;
}

int lastResult   = -1; // 记录接收到的调试数据
int receivedSign = 0;

void Magic_Get_Debug_Value(MagicHandle_Type *magic) {
    int result = 0;
    int length = 0;

    if ((magic->sta & 0x8000) > 0) { // 串口接收到调试数据
        uint8_t i = 0;

        length = magic->sta - 0xC000; // 调试数据的长度
        result = 0;                   // 调试数据

        for (i = 0; i < length; i++) {
            result += pow(10.0, length - i - 1) * (magic->buf[i] - 48); // 将串口数据转码
            magic->buf[i] = 0;                                          // 清空数据缓存
        }
        magic->sta   = 0;      // 清空串口的接收缓存
        lastResult   = result; // 记录接收到的调试数据
        receivedSign = 1;
    } else {                              // 串口 没有 接收到调试数据
        if (receivedSign == 1) {          // 之前 已经 接收过数据
            result = lastResult;          // 返回上一次接收的调试数据
        } else {                          // 之前 没有 接收过数据
            result = magic->defaultValue; // 返回设定的默认值
        }
    }
    magic->value = result;
}
