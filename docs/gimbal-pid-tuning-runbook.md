# 关节 0 自动整定：交付与现场准入手册

软件范围：单工况、双 GM6020（CAN1 0x205/0x206）、motor0 闭环、50/50 指令分配；先速度 PI，再位置 PD。**软件/模拟测试不等于固件编译成功、闭环稳定或实机验收。**

- 设计：[实施计划](gimbal-pid-implementation-plan.md)
- 字节级接口：[协议 v1](gimbal-tuning-protocol.md)
- Python 安装、CLI、日志、评分和 BO：[工具手册](../tools/gimbal_tuning/README.md)
- 历史代码定位：[静态分析](gimbal-pid-auto-tuning.md)（原提交分析，不是当前验收状态）

## 1. 停止语义与不可绕过的边界

1. 上电/复位不授权关节 0 运动；初始化即请求双机零输出，正常控制阶段也在最终 CAN 槽位覆盖。
2. STOP、正常单轮结束、故障均为双机零指令，**不是主动保持、机械制动或确认已经静止**。必须有支撑，实测惯性滑行停距并留足余量。
3. 不修改其他关节的目标/控制律，不设置全局 SafetyMode，不因关节 0 故障让整臂主动断力。其他关节反馈、实际姿态/速度或既定目标不符合固定工况时，关节 0 实验中止。
4. 掉线、保护、准备失败等取消批次并锁定故障。重新连接不续跑；清故障与重新授权启动是独立动作。故障不自动换基线继续驱动。
5. 软件只能**请求**发送零指令。CAN 物理断开、CPU/调度器锁死、供电/电调异常时，不能由普通 MCU 控制任务保证指令已送达。独立急停必须不依赖 PC、这条串口或控制任务。没有增加会使其他关节失去控制的全局看门狗复位策略。
6. 有力输出时不可用调试断点暂停 CPU；恢复执行后检测到超时不代表暂停期间已停机。
7. PC 保存记录和候选，设备只存 RAM，不写 Flash、不自动发布/启用最佳参数。原始默认 PID 增益没有被搜索结果替换。

## 2. 文件与固件开关

| 文件 | 职责 |
|---|---|
| `driver/{inc,src}/Driver_GimbalTuning.*` | 可移植协议、状态机、参数事务、关节0控制/保护/遥测 |
| `user/infantry/gimbal_tuning.*` | USART3 ISR 搬运、TIM5 时钟、STM32快照、DMA队列、CAN邮箱监督 |
| `user/infantry/config.h` | `GIMBAL_TUNING_ENABLED`，默认 **1**：专用调参模式 |
| `user/infantry/tasks.c` | 保留其他关节控制；调参模式禁用原串口 VOFA 发送 |
| `driver/src/Driver_Bridge.c` | 最终 CAN 槽位覆盖，0x1fe 非阻塞发送，双机反馈时间戳 |
| `mdk/infantry.uvprojx` | 原 Keil ARMCC5 / STM32F427II 工程，已加入新文件 |
| `tests/run_gimbal_checks.py` | 不打开设备的离线检查入口 |

`GIMBAL_TUNING_ENABLED=0` 是显式选择原控制程序，**不具有上述调参模式的上电零输出/实验保护语义**，不能拿它连接自动整定器。不要把该开关当作在线切换/故障回退按钮。

USART3（PD8/PD9、500000、8N1）由 Python 独占，RXNE 中断只保存字节与实际到达时间；控制任务每轮最多处理 128 字节。旧 Bridge USART3 解析器和旧 VOFA DMA 不会同时运行。VOFA 只接收 PC 的本机 UDP JustFloat。

调参适配占用 TIM5，依据 APB1 时钟配置为名义 1 MHz 单调计数；实际单位、时钟及控制周期仍需现场验证。启动标识使用硬件 RNG，取不到有效非零值时拒绝运动，不能用固定种子绕过。

**上机前新增依赖检查：**当前源码 HSE_VALUE=12MHz、PLL M=12/N=360/P=2/Q=7，PLLQ域按源码约51.43MHz，不可仅凭“PLL48”名称认定RNG时钟已验证。须依据具体F427规格/时钟寄存器确认RNG时钟合法并实测启动标识和时序，再确认运动准入；DRDY非零不等于证明时钟合规。本次没有擅自更改全局PLL/其他关节时序。INFO 固件协议版本为 1，软件版本为 `0x00010000`；现场须同时记录 Git/烧录产物标识，修改控制/评分语义时不能沿用已验收结论。

CAN1 0x1fe 仍携带 0x205、0x206、0x207、0x208 四个槽位：只覆盖前两个，后两个保留现有控制值。当前目标未绑定 0x207，0x208 是关节 3。单个在途邮箱受监督，不等 CAN ACK 卡住任务；正常使能期间在途超过名义 1 ms/发送失败会锁定 TRANSPORT 故障。STOP 会取消尚未完成的本组非零帧，再请求零帧。**这不是在断线总线上成功停机的证明。**

## 3. 不接设备的检查

仓库根目录（macOS/Linux，有 `cc` 时）：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/run_gimbal_checks.py --sanitize --arm-syntax
```

- C 核心及实际旧 `Driver_PID.c` 对照；ASan/UBSan。
- 真实适配层配寄存器 mock：初始零输出、邮箱超时/取消、其他槽位保留、DMA不可重启在途帧、ISR到达时刻、RX溢出。
- Python 单元测试与实际 C 核心的双向帧交叉测试。
- Keil XML 文件条目/设备/C99 静态检查。
- 可选 ARM GCC：核心和新适配层的 **vendor/CMSIS头边界语法检查**，旧 handle 图被测试 stub 替代。不是原工程链接/Keil build。

Windows 可先运行 Python 测试（在 `tools\gimbal_tuning`，已安装解释器和依赖）：

```powershell
$env:PYTHONDONTWRITEBYTECODE = "1"
python -m unittest discover -s tests -v
python -m gimbal_tuning simulate
python -m gimbal_tuning simulate --mode speed
python -m gimbal_tuning validate examples\hardware-template.json --gate A --hardware
```

模拟配置必须保持 `environment: simulation_only`。模板的 Gate A 可读不意味着 Gate B/C 已准入；原始仓库 PID 参数也不是已知安全种子。

安装工具手册所列可选依赖后，可先验证完整记录/BO/重放（目录名须尚不存在）：

```powershell
python -m gimbal_tuning baseline --simulate --config examples\simulation.json --run-dir runs\baseline-sim-001
python -m gimbal_tuning optimize --simulate --config examples\simulation-speed.json --run-dir runs\speed-sim-001 --export-best runs\speed-best-unverified.json
python -m gimbal_tuning replay runs\speed-sim-001 --propose --output runs\speed-replay.json
```

派生报告/导出必须放在证据目录外，不能写进 `runs\speed-sim-001` 或刷新旧 manifest。记录使用 SHA256 清单作一致性检查，不是对恶意改写者的签名认证。直接依赖固定版本，数值依赖的实际安装版本也会记录；不是完整跨平台环境锁或位级复现承诺。

## 4. Keil 构建、烧录前检查（用户执行）

1. 打开 `mdk/infantry.uvprojx`，目标 STM32F427II，`uAC6=0`（ARMCC5，项目配置 5.06）。不要用 ARM GCC 通过冒充 Keil 通过。
2. 核对两对新 `.c/.h` 文件被工程包含，保持目标原有 include/device/FreeRTOS 配置；检查调参开关为 1。
3. 完整 Rebuild、记录日志、错误/告警与烧录产物标识，再由用户操作烧录。当前环境没有执行 Keil、链接或烧录。
4. 已知 ARM GCC 直接检查原工程会遇到 RVDS/ARMCC 内联汇编、既有 `protocol.h` 匿名成员和 Gyroscope 声明问题。没有将这些无关平台代码大范围改写；它们在真实 Keil 下是否构成阻塞以完整编译结果为准。
5. 烧录前确认其他关节启动行为、支撑与急停；首次上电只验证零输出，不启动实验。
6. 核对实际 CAN 帧中前两槽是零、其他关节行为未被改变；仅看 `motor->input=0` 或只看在线标志不够。
7. 电调掉线/总线无 ACK、任意一台关节0反馈掉线、串口中断/PC退出，按分级现场保护清单验证；这类试验须用户现场执行，不由软件交付代替。

## 5. Gate A：仅连接与读取

```powershell
python -m gimbal_tuning status --port COM7
```

`COM7` 只是示例，必须由用户核实。连接发 HELLO，会撤销旧批次；若设备正在实验，将请求停止，**不会接管/续跑或使能**。旧固件/错误版本/无有效启动标识应拒绝进入实验，不能用忽略版本/关闭校验解决。

连接不代表机械安全确认。串口不能同时由 VOFA、串口助手或另一个 Python 实例持有。VOFA 的 UDP 问题与设备串口独立处理，不能通过让 VOFA 再打开同一串口来“修复”。

## 6. Gate B：受保护的小幅原始基线

复制 `examples/hardware-template.json`，保留 `environment: hardware`；逐项填写真实值。缺失值、非有限数、非法范围和未确认项会拒绝对应运动入口。

**必须现场确认并留下记录：**

- 固件/协议/烧录产物；系统、PLLQ/RNG和TIM5时钟的合法性与实测值；0x205/0x206 实际映射、方向、零位、减速比；rad/rad/s/A/°C 与 ESC 指令量。输入命令量不能当作 A。
- `pos_min/pos_max/margin`：同时约束两台电机，边界余量包含最坏停止/滑行距离；启动点和目标在内侧区间。
- 最大速度、总输出、速度积分项上限、两机反馈电流/温度、两机角度差；不是随搜索任意放大的自由参数。
- 每台反馈时效、PC心跳期限、准备/稳定/单轮/批次超时、最大控制间隔；实际控制和反馈更新率须测量。
- 其他三关节固定姿态、允许位置/速度偏差与有效反馈；保留其原控制。
- 限速/限加速度准备轨迹、起点/目标/单次时长、稳定误差/速度容差和保持时长。
- 人工监护、支撑、独立急停、零输出可承受且停距已核实。
- 原始基线增益、限幅、算法版本、初始化策略。Gate B 不会把仓库参数自动升级成已验收基线。

受限速度测试采用 `mode=1`、绕过位置外环，但两机位置边界/速度/其他保护始终保留。位置测试为 `mode=0`。准备/稳定期间使用基线，不计入 RUN 评分；候选只在安静起点的周期边界整体生效。每轮结束真实零输出，下一轮重读实际位置。

用工具手册中**明确授权单轮**命令执行最小幅度试验；先检查日志、时间戳/序号、掉帧、CAN/串口时序和版本，再判断能否继续。没有合格记录不得进入自动搜索。

## 7. 原始/修正基线纪律（D阶段现场工作）

- 算法 0 保留原离散位置 PD + 速度 PI；I 每样本累加，D 每样本差分，不能把连续 Ki/Kd 原样填入。
- I、目标与微分历史的固定策略为 `reset_at_rest_v1`：准备与候选切换时清积分，按切换前的测量误差播种历史。用于可重复性，不宣称物理上绝对无扰。
- 算法 1 是**仅关节0**的条件积分版本；通用 `Driver_PID.c` 语义未改变。必须先测算法0基线，依据波形逐项比较，再测并明确接受算法1的新基线，不能在同一 BO 批次偷偷改变算法。硬件 Gate B 的首次修正基线采集要求 `conditional_trial_authorized=true`，但不要求先接受尚未测量的基线；只有 Gate C 还要求 `conditional_baseline_accepted=true` 与通用基线验收。
- 观察位置/速度误差、积分、输出及饱和、反馈年龄/更新计数、控制间隔。积分积累仍是待实测的假设，不能因为有抗饱和代码就认定根因/效果。
- Ki=0 可作单独版本的诊断对照，不预先认定最佳；D噪声、微分冲击、延迟或机械因素占主导时，暂停增益搜索。
- 保存原始基线、每项改动和修正后基线各自的配置/日志/算法版本。基线不能安全完成时停下诊断，不强行补齐数据。

## 8. Gate C：监护下自动批次

Gate B 之外还需独立确认：已验收基线与准备策略、性能阈值、有界搜索盒、候选/时间/单轮/重复预算、同一场景顺序、随机种子、评分版本和固定工况。

- 先 `speed_pi`：只变速度 P/I，受限速度场景，不让位置外环掩盖结果。
- `position_pd`：固定**另行验收**的速度 P/I，仅搜索位置 P/D，完整双环位置场景。速度搜索导出结果不自动成为已验收内环。
- 位置性能阈值用 rad，速度性能阈值用 rad/s；不能共用同一个数值却省略单位。
- 稳定时间必须连续保持误差带；超调、最终误差和持续振荡约束优先于速度。完整但性能差的实验可按明确惩罚注册；部分、保护、准备、通信/遥测无效数据不注册为正常 BO 观测。
- RUN 起止样本、实际参数版本、MCU启动/会话/试验身份必须吻合；最终 RUN 零指令、下一 COMPLETE_ZERO 和清洁状态回读均不可省。
- 故障、失联或准备失败停批并锁定；普通 STOP 取消授权。只有仍有效的既定批次可自动准备下一轮。
- 中断/超时若没有收到 STOP 零输出状态确认，记录为**停止未确认**，靠 MCU 的活性期限和现场独立急停处理，不自动重连假定成功。

VOFA 用本机 UDP JustFloat 查看波形；完整串口原始记录才是分析依据。VOFA 断开/未打开不影响设备保护与日志；遥测不足仍会使实验无效。

### 人工清故障（不等于启动）

故障原因消除并重新核对 Gate B 后，可由操作者通过现有 `ProtocolClient` 明确调用 CLEAR_FAULT。以下是 PowerShell/已安装工具环境的手工 API 用法说明，**本次未对设备执行**；必须先退出实验进程释放串口。它不调用 START，不验收/发布基线，也不会恢复旧批次。

```powershell
@'
from gimbal_tuning.config import Configuration
from gimbal_tuning.protocol import Message as M, Status
from gimbal_tuning.transport import ProtocolClient, SerialTransport
cfg = Configuration.load("my-hardware.json").validate("B", hardware=True)
port = SerialTransport("COM7")  # 示例端口，必须核实
try:
    client = ProtocolClient(port, config=cfg, hardware=True)
    client.connect()  # 撤销旧授权，故障锁定仍保留
    client.command(M.SET_SAFETY_CONFIG, cfg.safety.pack())
    client.command(M.HEARTBEAT)
    client.command(M.CLEAR_FAULT)
    status = Status.unpack(client.command(M.GET_STATUS).payload)
    assert status.state == 0 and status.fault == 0 and not status.flags & 24
    print("IDLE_ZERO；新实验仍须重新明确授权")
finally:
    port.close()
'@ | & $python -
```

条件未恢复时设备会拒绝；不能通过改内存锁定位或自动重试绕过。若启动标识/时钟异常导致连接无法成立，先处理相应构建/硬件依赖，不能用固定 boot ID 或忽略版本校验恢复运动。

## 9. 独立验收、参数发布

1. 对最佳候选与修正后基线做独立重复、双向响应与全部约束核对；搜索中表现好不等于最终验收。
2. 报告保留置信不足/重复不足说明、全部原始记录、有效/无效观测、配置、参数版本与评分版本。
3. 可在已批准条件下做手拨/按住后释放体验；其初速度、积分和微分历史不同，不直接由自动阶跃分数推断手感或回摆根因。
4. 导出文件只是待验收候选，不会写 Flash 或更改默认增益。是否发布由用户决定。
5. 算法1的增益不能只复制进原 `PID_Init` 并声称行为相同；发布须保留匹配的关节0控制算法/状态策略与限幅版本。把调参算法移入普通运行模式属于单独人工审核的发布动作。

## 10. 当前验收边界

本轮离线验收通过：68项宿主 + 5项真实C核心跨端测试（共73，无跳过）、C核心/寄存器mock ASan/UBSan、ARM核心/新适配层vendor头边界语法、Keil XML静态、文档与任务diff检查。真实BO库的速度/位置模拟各8完整试验和2观测、原始重放通过；样例结果均性能不可行，仍不可发布。默认位置例的60s预算停止也已验证，部分轮不注册。当前离线失败为0。

完整Keil编译不是上述边界语法检查；原工程直接GCC检查曾被既有ARMCC/RVDS代码及头文件问题阻挡。没有把该平台限制称为Keil成功，也未改无关平台代码来刷绿。

**仍未执行：**Windows Keil完整编译/链接/烧录、真实 CAN/串口/控制周期与抖动、反馈单位/零位/两机一致性、实际停止距离、VOFA界面/UDP支持、原始/修正基线对照、监护下实机BO、最佳参数独立/手拨验收。全部是继续实机阶段的真实门槛，不因软件实现完成而自动打勾。
