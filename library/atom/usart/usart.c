#include "usart.h"
#include "string.h"
#include "sys.h"
#include "math.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB

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
