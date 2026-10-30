# 事实与设计依据

## 已有证据
- 代码定位文档：docs/gimbal-pid-auto-tuning.md，基于此前静态检查，未验证实机。
- 关节 0：两台 GM6020 共用位置 PD/速度 PI，取 motor0 角度与速度反馈，输出 50/50 分配。
- 用户现象：拨离目标后返回过程中回弹、震荡、稳定慢。第二轮已确认拨动时始终使能并抵抗手的外力；按住时长未知，积分积累仍仅为待验证假设。
- 现有 VOFA 遥测含目标常数与两台电机角度，不能直接满足实验评分与故障诊断。
- 候选优化库：https://github.com/bayesian-optimization/BayesianOptimization 。不是 PID 专用成品，需实验接口与实时保护。

## 第二轮确认
- 单工况自动整定；先限制回弹再优化稳定时间；性能阈值先测基线再确认。
- 分级准入、监护下自动跑批；当前仍只规划。

## 接口与保护静态证据（实施前）
- user/infantry/interrupt.c:43：USART3_IRQHandler 调用 Bridge_Receive_USART，现有串口不能假定为无占用的命令入口。
- driver/src/Driver_Bridge.c:80：SafetyMode 路径发送零输出，不是主动位置保持；且是公共电机发送函数，不适合不加区分地停止所有关节。
- Bridge_Send_Motor 按每台电机在线标志门控；未体现关节 0 双机失联时的成组安全处理。

## 第三至五轮决策
- 用户确认关节0有支撑/安全行程，可承受零输出和滑行；其他关节固定姿态保留控制。
- Python独占串口转发VOFA；允许分阶段改善保护/抗饱和/切换，必须保留基线对照。
- 故障锁定人工恢复，普通停止和单轮结束也双机零输出；仅已批准批次内允许重新预检并用基线自动准备起点。
- 上电/复位关节0默认零输出，明确启动才使能。
- Windows用户烧录验收；CLI+配置+VOFA；PC保存数据和候选，RAM试用，不做Flash。
- 所有实机安全配置、搜索范围和预算缺失时拒绝对应阶段运行；性能阈值先基线后用户确认。
- Keil工程mdk/infantry.uvprojx：STM32F427II，uAC6=0，未编译。

## 实施核对与契约
- 用户最新“Implement the plan”已授权软件实施；不连接/烧录/操作硬件。
- 实施起点源码是原始位置 PD + 速度 PI；当时Task_MecArm每tick重新启动VOFA DMA、USART3无接收中断配置、关节0在线检查重复motor0。现已在调参模式中隔离旧串口路径、采用有界队列并修复双机检查。
- `Motor_Type.updatedAt` 仅16位；调参适配层需独立32位关节0反馈时钟/计数，避免以截断时间作为保护证据。
- 其他关节保持原 MecArm 路径，调参仅跳过关节0并在最终 CAN 槽位覆盖零值。
- 共享契约已写入 docs/gimbal-tuning-protocol.md：版本1，最大256字节payload，device分配session、STOP撤销session、CRC16、140字节安全配置、24字节参数、124字节遥测。
- 主机工具可用 Python3、cc、arm-none-eabi-gcc；后者不等于项目 Keil ARMCC5 编译器。`pi --list-models gpt-6.1-sol`确认 openai-codex/gpt-6.1-sol 可用，implementation角色无模型pin。

- 接入检查发现 `driver/src/Driver_CAN.c:Can_Send` 无限等待 CAN ACK；若直接强制发送零帧而总线断开，会卡住控制任务。调参关节0所在 0x1fe 帧必须改走有界非阻塞发送/邮箱监督，保留同帧其他槽位；不修改全局 Can_Send 语义。上电初始化即请求零帧，不能等1.5s任务启动后才首次请求。
- ARM GCC 对真实工程头图检查被原有 ARMCC/Keil 代码阻挡；已用真实 STM32/CMSIS 头 + 仅替代旧 handle 图的边界stub验证新适配层语法，不能作为真实 Keil build 通过。

- 新RNG依赖检查：`user/common/system_stm32f4xx.c` 的 F427分支 PLL_M=12、N=360、P=2、Q=7；`user/common/stm32f4xx.h` HSE_VALUE=12MHz，按源码PLLQ域约51.43MHz而非48MHz。ST AN3988检索摘要将USB/RNG/SDIO描述为48MHz域（https://www.st.com/resource/en/application_note/an3988-clock-configuration-tool-for-stm32f40xx41xx427x437x-microcontrollers-stmicroelectronics.pdf）。只读了检索摘要，未完整读取PDF，未确认具体芯片RNG允许范围/实测错误状态；不能认定已经出现故障，也不能将非零DRDY当作时钟合规证明。
- 不擅自修改全局PLL/其他关节时序；RNG/PLL域和TIM5实际时钟列为Gate B前用户必须核对的硬件准入。RNG取不到有效启动标识拒绝运动，不能固定boot ID绕过。

## 最终软件证据与边界
- 主代理最终完整命令 `/tmp/gimbal-host-cde-venv/bin/python tests/run_gimbal_checks.py --sanitize --arm-syntax`：68项宿主+5项真实C跨端Python测试全通过，无跳过；C核心/适配mock的ASan/UBSan、ARM核心/适配边界两模式语法、Keil XML静态通过。
- 完整QA日志：`/var/folders/lx/q08bgxxn4qscb6nz1t4cf3wh0000gn/T/gimbal-final-qa-k77qbk1p.log`。这不是完整Keil编译/链接，也不是硬件验证。
- 两阶段真实BO库模拟各8完整试验/2观测，原始重放/未验收导出通过；样例最佳均性能不可行。默认位置例的60s预算中止确认零状态，部分第9轮不注册。
- 无数值实机配置、基线/性能验收或可发布PID结果；无自动参数发布/Flash写入。无关.DS_Store/.pi保留，未暂存/推送。完整Keil构建未验证，因此未自动做Git checkpoint。

## 待确认
- 具体安全数值、实机基线和性能阈值属于明确的后续准入任务，不是当前默认值或已验证事实。
