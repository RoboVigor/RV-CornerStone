# RoboMaster A 型：常用硬件连接

来源代号见 [资料目录](README.md)；核心参数与未解决问题见 [硬件事实](board-facts.md)。以下为**文档核对后的连接关系，不是 CubeMX 初始化方案或实物通断测试**。TX/RX 均以 MCU 为视角；AF 编号、GPIO 模式、上拉、波特率与时钟须按具体需求另查。插座朝向用原件丝印/位号图确认，不凭表格猜测。

## LED、按键、调试与控制信号

| 板上功能 / 位号 | MCU 引脚 | 电气关系 / 备注 | 出处 |
|---|---|---|---|
| 8 颗绿色 LED，AD1–AD8 | 依次 PG1–PG8 | 低电平亮；阳极接 VCC_3V3_S，经限流电阻连接 MCU | M p10 §1.6；S p3 / Sheet 4，AD1–AD8/U1C（图像核对） |
| 单独绿色 LED，D9 | PF14 | 低电平亮 | M p10 §1.5；S p3，D9/R15 |
| 单独红色 LED，D10 | PE11 | 低电平亮 | M p10 §1.5；S p3，D10/R16 |
| 用户按键 S1 | PB2 | 按下为高，同时为 BOOT1 | M p10 §1.7–1.8；S p2 U1A、p5 S1 |
| 复位按键 SW1 | NRST | 按下接地，不是普通用户 GPIO | S p4 / Sheet 5，SW1 |
| SWDIO / SWCLK，J1 | PA13 / PA14 | J1 pin1 SWDIO、pin2 SWCLK、pin3 GND、pin4 3.3 V；各信号串 100 Ω。按图中 pin 编号，不按插头观察方向排序 | M p9 §1.3；S p2 / Sheet 3，J1/R3/R4/U1A（图像核对） |
| 蜂鸣器 LS1 | PH6 | 板图标 TIM12_CH1，经 Q4 驱动 | M p12 §1.10；S p6 / Sheet 7，LS1/Q4 |
| 激光/可控 5 V 接口 J15 | PG13 | 经 Q3 控制；手册给此接口最大 400 mA，不等同主电源输出 | M p12 §1.11；S p5 / Sheet 6，J15/Q3 |
| 可控电源 J20 / VCC_OUT1 | PH2 | POWER1_CTRL，经 Q9/Q6 控制 | M p18；S p7 / Sheet 8 |
| 可控电源 J22 / VCC_OUT2 | PH3 | POWER2_CTRL，经 Q13/Q10 控制 | M p18；S p7 / Sheet 8 |
| 可控电源 J19 / VCC_OUT3 | PH4 | POWER3_CTRL，经 Q8/Q7 控制 | M p18；S p7 / Sheet 8 |
| 可控电源 J21 / VCC_OUT4 | PH5 | POWER4_CTRL，经 Q12/Q11 控制 | M p18；S p7 / Sheet 8 |

电源控制这里只记录网名与连接，不规定上电默认输出或开启策略；电流限制见 board-facts。

## CAN 与串口

| 功能 | MCU TX | MCU RX | 板上连接 / 备注 | 出处 |
|---|---|---|---|---|
| CAN1 | PD1 | PD0 | U5 MCP2562；外接的是 CANH/CANL，不是 MCU 逻辑 TX/RX | S p7 / Sheet 8，U5；M p32 丝印图 |
| CAN2 | PB13 | PB12 | U6 MCP2562；SDK 专用 J3 与普通 J4/J8/J11 **线序不同** | S p7，U6；M p13 §1.12；S p5 |
| SDK USART3，J2 | PD8 | PD9 | SDK 口线序不同，不套用 USER 串口接线 | M p9 §1.4；S p3 U1B、p5 J2 |
| USER USART6，J36 | PG14 | PG9 | 端口编号以 MCU 为准 | M p9；S p9 / Sheet 10，J36 |
| USER UART8，J37 | PE1 | PE0 | M 正文称 USART8；图纸网名 UART8 | M p9；S p9，J37 |
| USER UART7，J38 | PE8 | PE7 | M 正文称 USART7；图纸网名 UART7 | M p9；S p9，J38 |
| 预留蓝牙 USART2，J18 | PD5 | PD6 | 仅接口预留，不表示板上带蓝牙模块 | M p19 §1.21；S p5 J18 |
| DBUS，J43 | — | PB7 / USART1_RX | J43 输入先经 Q14 **反相**再到 PB7；不是直接接普通 UART RX 的同相接口 | M p8 §1.2；S p8 / Sheet 9，J43/Q14 |

- S p7 的 R79/R80 标 120 Ω，但附 `XXX` 注记；本摘要**不保证实物已安装 CAN 终端电阻**。
- USER 插座 J36/J37/J38 的原理图 pin1=5 V、pin2=GND、pin3=TX、pin4=RX（S p9，图像核对）。SDK USART3 不套用此顺序；接线回看 M p9、p32。
- M p32 丝印图把 USER 串口电源写为 `5V_S`，S p9 的网名则是 `VCC_5V_U`；两者都标 5 V，但**电源域命名不一致**。本摘要不把这两个电源域合并，不据丝印判断其供电来源。

## IMU 连接（型号与地址有冲突）

**依赖具体器件型号/地址的驱动配置必须先解决 [冲突](board-facts.md#必须保留的来源冲突)。** 下面仅描述 S p6 / Sheet 7 的拓扑，不能以连线一致推断器件标签正确。

| 功能 | MCU 引脚 / 连接 | 出处 |
|---|---|---|
| U3 SPI5 NSS | PF6 | S p6，U3 CSn（图像核对）；M p14 |
| U3 SPI5 SCK | PF7 | S p6，U3 SCL/SCLK；M p14 |
| U3 SPI5 MISO | PF8 | S p6，U3 AD0/SDO；M p14 |
| U3 SPI5 MOSI | PF9 | S p6，U3 SDA/SDI；M p14 |
| U3 中断 | PB8 | S p6，U3 INT；M p14 |
| IMU 加热控制 | PB5 | S p6，Heat_PWM/Q5/R51–R60；M p13–14 |
| IST8310（U4）复位 | PE2 | S p6，Set/Reset → U4 RSTN；M p14 |
| IST8310（U4）数据就绪 | PE3 | S p6，U4 DRDY；网名写作 `5833_INT`，不要将此网名误作器件型号 |
| IST8310（U4）总线 | AUX_CL / AUX_DA 连接 U3 的辅助总线 | S p6，U3/U4；**不是直接连接 MCU 的 I2C2/I2C3** |

## PWM 输出

以下按 S p8 / Sheet 9 的**连接器 pin 编号**整理并已看图核对；定时器/通道是图纸网名，不代表本固件已启用。

| pin | J26 信号 | 图纸网名 | J31 信号 | 图纸网名 |
|---|---|---|---|---|
| 1 | PD12 | TIM4_CH1 | PA0 | TIM2_CH1 |
| 2 | PD13 | TIM4_CH2 | PA1 | TIM2_CH2 |
| 3 | PD14 | TIM4_CH3 | PA2 | TIM2_CH3 |
| 4 | PD15 | TIM4_CH4 | PA3 | TIM2_CH4 |
| 5 | PH10 | TIM5_CH1 | PI5 | TIM8_CH1 |
| 6 | PH11 | TIM5_CH2 | PI6 | TIM8_CH2 |
| 7 | PH12 | TIM5_CH3 | PI7 | TIM8_CH3 |
| 8 | PI0 | TIM5_CH4 | PI2 | TIM8_CH4 |

J26 配套电源排 J24、地排 J25；J31 配套电源排 J29、地排 J30。电源为可调 `VCC_5V_ADJ`，不是 MCU 的 GPIO 电平；M p16、S p8–9。另有 J27/J28/J32/J33：pin1 分别为 PA8/PA9/PE13/PE14（TIM1_CH1–4），pin2 图示未连接，pin3 PGND（S p8）。

## 其他常用接口

| 功能 | 连接 | 出处 |
|---|---|---|
| USB OTG FS | DM PA11、DP PA12、ID PA10 | M p11–12 §1.9；S p2 U1A、p5 USB 接口；仅板上连接，不证明当前时钟/固件支持 USB |
| TF 卡 J6 | D0 PC8、D1 PC9、D2 PC10、D3 PC11、CK PC12、CMD PD2、卡检测 PE15 | M p15 §1.14；S p2–3 MCU 网名、p5 J6；M p32 |
| OLED 接口 J17 | MOSI PA7、SCLK PB3、DC PB9、RST PB10；按键模拟量 PA6 | M p17 §1.17；S p2 U1A、p5 J17；M p32 |
| 同步接口 J42 | PB14/PB15；图纸包含 3.3 V/5 V 域与 Q15 | M p17 §1.18；S p9。不能直接按两个普通同电压 GPIO 使用 |
| 扩展口 J34 | PF0/PF1（I2C2 网名），PE4/PE5/PE6/PE12（SPI4 网名），PB0/PB1、PC0–PC5、PA4/PA5、PI9/PF10 | M p15 §1.15；S p8 / Sheet 9。完整 pin1–20 朝向和接线仍查原图，本摘要不逐一转录 |

未记录的端子极性、传感器寄存器和接口供电路径不视为已确认；发现代码与连接表不符时报告差异，不为了匹配代码改写硬件事实。
