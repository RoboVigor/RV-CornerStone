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

/**
 * @brief 串口 USART6 初始化 (IO3)
 *
 * @param bound - 波特率
 */
void Magic_Init_Config(u32 bound) {
    // GPIO端口设置
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);  //使能GPIOG时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //使能USART6时钟

    //串口3对应引脚复用映射
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9,
                     GPIO_AF_USART6); // GPIOG8复用为USART6
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource14,
                     GPIO_AF_USART6); // GPIOG9复用为USART6

    // USART6端口配置
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_14; // GPIOG9与GPIOG14
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;             //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;             //上拉
    GPIO_Init(GPIOG, &GPIO_InitStructure);                    //初始化

    // USART6 初始化设置
    USART_InitStructure.USART_BaudRate            = bound;                          //波特率设置
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;            //字长为8位数据格式
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;               //一个停止位
    USART_InitStructure.USART_Parity              = USART_Parity_No;                //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
    USART_Init(USART6, &USART_InitStructure);                                       //初始化串口3

    USART_Cmd(USART6, ENABLE); //使能串口3

    // USART_ClearFlag(USART6, USART_FLAG_TC);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); //开启相关中断

    // USART6 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel                   = USART6_IRQn; //串口3中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;           //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;           //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;      // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                     //根据指定的参数初始化VIC寄存器
}

/**
 * @brief 初始化调试数据的默认值
 *
 * @param magic
 * @param defaultValue
 */
void Magic_Init_Handle(MagicHandle_Type *magic, int defaultValue) {
    magic->defaultValue = defaultValue;
}

/**
 * @brief 调试数据接收函数
 *
 * @return int - 返回给系统的调试数据
 */

int lastResult = -1; // 记录接收到的调试数据

int Magic_Get_Debug_Value(MagicHandle_Type *magic) {
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
        return result;
    } else {                            // 串口 没有 接收到调试数据
        if (lastResult != -1) {         // 之前 已经 接收过数据
            return lastResult;          // 返回上一次接收的调试数据
        } else {                        // 之前 没有 接收过数据
            return magic->defaultValue; // 返回设定的默认值
        }
    }
}
