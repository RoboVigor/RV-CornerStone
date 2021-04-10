# 更新记录

## v1.8.0
1. Driver_Bridge: 负责干脏活的，负责建立板子上的通讯接口与抽象的软件接口之间的连接，用例:
    - 串口或CAN中断后数据通过Driver_Bridge传递给Driver_Protocol
    - 通过串口或CAN发送电机或Protocol的数据
    - 按一定频率发送某Protocol
2. Driver_Protocol:
    - node: channel全部重命名为node，可以看作和主控板通讯的对象（比如裁判系统或上位机）
    - ProtocolInfo: 每个协议都有对应的ProtocolInfo结构体，保存了协议编号、长度，、偏移、是否接收及接收计数、发送频率及发送任务
3. Driver_Gyroscope: 删除config.h中开机解算延迟相关配置，在Gyroscope_Init()参数中配置开机解算延迟
4. Driver_Magic: 重写，用作调试。
    - 输出报错信息或调试信息
5. User:
    - 结构简化: main.c: 原bsp.c + handle.c + Task_Sys_Init(), 删除无用文件
    - protocol.h: 全部协议合并进一个文件，通过代码生成；每个用户独立
    - main.c:
