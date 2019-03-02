#include "Driver_Magic.h"
#include "string.h"
#include "sys.h"
#include "math.h"
#include "handle.h"

//标准库需要的支持函数
struct __FILE {
    int handle;
};

FILE __stdout;

//重定义fputc函数
int fputc(int ch, FILE *f) {
    while ((USART6->SR & 0X40) == 0)
        ; //循环发送,直到发送完毕
    USART6->DR = (u8) ch;
    return ch;
}

void Magic_Init_Handle(MagicHandle_Type *magic, int defaultValue) {
    magic->defaultValue = defaultValue;
}

int lastResult   = -1; // 记录接收到的调试数据
int receivedSign = 0;

void Magic_Get_Debug_Value(MagicHandle_Type *magic) {
    int result = 0;
    int length = 0;

    if ((USART_RX_STA & 0x8000) > 0) { // 串口接收到调试数据
        uint8_t i = 0;

        length = USART_RX_STA - 0xC000; // 调试数据的长度
        result = 0;                     // 调试数据

        for (i = 0; i < length; i++) {
            result += pow(10.0, length - i - 1) * (USART_RX_BUF[i] - 48); // 将串口数据转码
            USART_RX_BUF[i] = 0;                                          // 清空数据缓存
        }
        USART_RX_STA = 0;      // 清空串口的接收缓存
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
