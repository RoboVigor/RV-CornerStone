#include "usart.h"
#include "string.h"
#include "sys.h"
#include "math.h"

#include "handle.h"
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1

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
#endif

#if EN_USART6_RX //如果使能了接收
//串口3中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误

//初始化IO 串口3
// bound:波特率
void uart_init(u32 bound) {
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

#endif

void USART6_Send_Package(uint8_t *data, uint8_t count) {
    uint8_t i;

    for (i = 0; i < count; i++) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
            ;
        USART6->DR = data[i];
    }
    memset(data, 0, count);
}

