# 进度

## 规划启动
- 用户要求 plan and grill-me。
- 已创建可见任务和独立计划目录；当前不实施、不连接电机、不启动子代理。
- 第一轮围绕目标、范围、性能取舍与实机试验门槛提问；等待用户决定后再展开依赖问题。
- 尚未完成设计，不创建完成性提交。

## 第一轮结果
- 用户选择单工况自动整定，先限制回弹、再缩短稳定时间。
- 用户认为手拨至起点和程序改变目标可统一为位置恢复任务，询问能否自动改变 target 完成搜索。
- 已回答：可以自动化；需区分积分、速度及微分历史等起始状态。使能时按住再松手不能直接等同目标阶跃。
- 第二轮结果：始终使能；分级准入后监护下自动跑批；先测基线再确认性能阈值。

## 第三轮结果
- 支撑与安全行程可保障关节0双机零输出及惯性滑行；详细物理边界留作准入检查。
- 其他关节固定姿态保持、保留原控制。
- Python独占串口并转发VOFA。
- 允许分阶段改进保护、抗饱和、参数切换；必须记录前后基线。
- 已只读核对USART3中断路径与Bridge发送/看门狗代码，未修改固件或连接硬件。

## 第四、五轮完成
- 已确认停止/结束零输出、故障锁定人工恢复、PC保存/RAM试用、Windows用户烧录验收。
- 已确认批次内自动准备、上电零输出、CLI+配置+VOFA、实机必填数值准入。
- 输出 docs/gimbal-pid-implementation-plan.md，涵盖协议事务、状态机、数据流、基线修正纪律、BO评分和A–F阶段验收。
- 当前仅规划文档；尚未获最终实施确认，未改固件、安装依赖、编译或操作硬件。
- 文档检查通过：四份计划文档非空、代码围栏配对、无行尾空白、背景文档与Keil入口存在。此检查不代表固件或实机验证。
- 阶段1和2完成；阶段3等待用户确认。

## 软件实施启动
- 用户“Implement the plan”完成最终软件实施授权；保留全部现场准入边界。
- 已读取所选计划/发现/进度、设计与必要控制链，建立 A–F 细粒度可见 todo；现场验收单列 Phase G，不能当作当前已完成。
- Git 初始只有无关未跟踪 `.DS_Store` 与 `.pi/`；保持不动。
- 已定义 docs/gimbal-tuning-protocol.md，冻结帧、会话/重放、配置、事务、状态和遥测；将据此实现双端并做交叉验证。
- 全局implementation角色无model pin，已确认指定SOL模型可用；已将Python工具责任有界委派给 `gimbal-host`（请求模型 openai-codex/gpt-6.1-sol，实际模型元数据尚待最终结果确认），主代理负责固件与集成，不派reviewer。
- 已实现可移植 C 核心（显式协议序列化、严格准入、会话/重放屏障、状态机、原子参数、保护及遥测），尚待STM32适配接入。
- C 主机测试已通过 ASan/UBSan、CRC/坏帧/截断/重放/重复START/STOP/单机掉线/超时/其他关节/参数生效/正常结束零输出。
- 增加与实际 Driver_PID.c 的 2000 样本离散算法对照，发现并修正表达式求和顺序对浮点舍入的差别；原始速度PI和位置PD对照现在逐样本相等。最初的条件积分测试案例期望也已校正，不代表实机控制效果已验证。

## Phase A 软件完成
- 独立运行worker实现的31项Python测试、位置/速度模拟CLI、硬件空模板拒绝测试，均通过。
- 根目录首次测试遗漏PYTHONPATH导致3个ImportError，按目录/包入口修正测试调用后通过（不是实现故障）。
- 主代理新增4项真实C核心↔Python编解码交叉测试：配置字节一致、位置/速度完整窗口、版本实际生效、最终零输出、STOP屏障和故障回读，全部通过。
- 已核对worker实际源码与报告；工具结果未暴露实际模型元数据，保留该验证边界。相同worker已resume继续管理器/日志/VOFA/评分/BO，未创建新责任或更换模型。
- 下一步固件STM32适配和最终CAN发送保护；Keil真编译、现场参数和实机全部未运行。
- 注意新发现：旧Can_Send无限等ACK，调参0x1fe改为非阻塞邮箱监督，初始就请求双机零输出；不会更改其他帧/关节控制语义。

## Phase B 软件完成 / Phase C 集成
- USART3调参接收与旧Bridge/VOFA隔离，ISR只搬运字节和到达时间；PC活性不以排队后解析时刻延长。最终CAN强制零帧且保留0x207/0x208槽位，修复motor1在线遗漏，其他关节原控制保留。
- 新寄存器mock测试通过：上电立即请求零帧、CAN等待/超时不阻塞、取消在途非零、邮箱故障锁定、同帧其他槽位不变、DMA未完成不重启、接收溢出和延迟解析心跳。
- `python3 tests/run_gimbal_checks.py --sanitize --arm-syntax --no-python`通过：ASan/UBSan C测试、Keil XML静态、核心ARM严格语法、新适配层启用/禁用两种模式真实vendor头边界语法。旧handle头图由stub替代，绝不是完整Keil build。
- 严格ARM GCC检查发现CRC表达式符号性告警，已明确uint32移位后通过。控制生成器新增finite/可行准备时限检查，避免极端合法数值使轨迹运算失效后仍使能。
- 参考轨迹速度/加速度、COMPLETE_ZERO观测屏障、运行中拒绝改参数/放宽保护、NaN/越界事务、原始PID2000样本对照均通过。
- 软件的物理边界已写入 docs/gimbal-pid-tuning-runbook.md：CPU锁死/总线断开不能由控制任务证明零帧送达，必须有独立急停；未引入让其他关节失控的全局复位。
- Phase C的MCU状态机与窗口已通过，PC日志/活性管理/BO仍在同一worker实现；已提醒真实设备新会话版本必须重分配、SerialTransport不能因遗漏hardware标记绕过配置准入。

## 主机实现返回与集成回归
- worker返回管理器/独立串口心跳所有者/异步持久日志/UDP/评分/BO/离线重放，报告源树59通过2可选跳过，外部固定依赖venv及最终wheel61/61通过；尚不据此宣称整体交付通过。
- 主代理已读取实际manager/io_owner/scoring/optimization/config/recording/analysis/cli实现，新增真实C核心→评分和同设备两次明确新会话的管理器测试。发现3处失败：评分要求每个RUN样本motion=true，但真实C最终RUN端点已零输出并motion=false；原纯Python测试未覆盖此契约差异。
- C END_BATCH现在明确清空batch_id，与管理器取消确认一致；线协议布局不变。软件阻塞todo #24已建立，C2保留in_progress，不误报完成。
- 同时复核条件积分Gate B首次基线采集的循环准入问题：可授权受保护的新算法单轮，不应要求先验收同一未测基线；Gate C仍须已验收基线。将通过同worker有界rework修正并测真实C回归。
- 时钟依赖已列为Gate B显式clock_rng_verified；未擅改PLL，全局时序保持原样。源码PLLQ约51.43MHz的具体F427合法性仍未确认，不是已观测故障根因。

## Phase C 软件验收通过
- 主代理独立运行真实C跨端测试5/5通过（含真实管理器两次新会话），管理器测试15/15通过（慢BO/fsync不阻断心跳、预算监督、ACK丢失/重启/断线、部分数据拒绝、UDP失效不影响记录等）。
- 收尾标志只对RESULT精确端点允许disabled/zero，非终端RUN仍须使能；END_BATCH清当前batch_id。条件积分Gate B首次诊断只需明确trial_authorized，Gate C才要求独立baseline_accepted。数值溢出和相反电机符号输出变化已有回归。
- 另修复CLI派生文件写入已封存证据目录会使manifest失效的问题：导出/分析路径在任何运动或分析前检查，拒绝覆盖输入证据。未弱化清单/刷新旧哈希。
- worker最终报告源树固定依赖测试68/68、根C跨端5/5及最终wheel烟测通过。主代理继续D/E和全量交付复核，不据worker报告单独宣称完成。
- 其实际模型仍无工具元数据，按请求模型/原会话保留路由记录，没有自动换模型或审查代理。

## Phase D 离线支持完成
- 原始/条件积分和周期状态策略的最终C native+ASan/UBSan以及ARM边界检查通过；全局PID代码/默认增益未改，实测效果仍未知。
- 主代理以CLI实际跑通模拟baseline、trial、conditional baseline；analyze原始记录/清单通过，compare确认配置可比且causal_improvement_established=false。
- 本轮CLI验证目录：`/var/folders/lx/q08bgxxn4qscb6nz1t4cf3wh0000gn/T/gimbal-final-cli-vf7imgtf`（临时离线证据，不是硬件数据）。
- 下一步验证约束评分、固定版本真实BO库、双向场景和预算/重放；真实基线采集与是否接受控制修正仍在Phase G。

## Phase E 主代理验收通过
- 固定依赖外部venv下：scoring 10/10、真实BO/阶段/UDP集成7/7、CLI封存证据输出保护4/4，全通过。
- 实际CLI：speed_pi与position_pd（后者明确simulation-only两候选配置）各8完整试验/2有效观测；真实BayesianOptimization3.1.0提议/注册、未验收导出、manifest原始重放/离线建议通过。两例最佳仍性能不可行，未冒充可发布结果。
- 默认position例是5候选、60s总预算，首轮验收脚本误假设退出0；实际退出3为正确wall-budget路径：STOP/zero确认、2完整观测保留，第9个部分试验不注册。已将其作为预算负例验收，并另验证两候选计数正常结束。
- 安装后的最终wheel在/tmp清除PYTHONPATH运行simulate通过，资源读取正常。
- CLI临时marker已删除，证据仍在上述临时目录；无设备连接、烧录、实测PID或VOFA进程验证。
- 进入F最终全量测试、文档及任务diff核对；外部硬件/Keil/时钟待用户。

## Phase F 软件交付完成 / Phase G 待用户
- 最终完整命令：`PYTHONDONTWRITEBYTECODE=1 /tmp/gimbal-host-cde-venv/bin/python tests/run_gimbal_checks.py --sanitize --arm-syntax`。
- 主代理最终完整日志（退出0）：`/var/folders/lx/q08bgxxn4qscb6nz1t4cf3wh0000gn/T/gimbal-final-qa-k77qbk1p.log`。宿主68/68、实际C跨端5/5，无跳过；C core/adapter mock ASan/UBSan、ARM core与adapter启用/禁用边界语法、Keil XML静态通过。显示截断已通过落盘日志回收完整摘要，不凭半截输出宣称成功。
- 文档UTF-8、围栏、控制字符、空白/本地链接、git diff --check通过；46个新任务文件无构建/缓存/二进制产物。当前分支feat_joint0，任务diff已核对；原始.DS_Store/.pi未触碰。
- 入口README与docs/gimbal-pid-tuning-runbook.md；完整CLI/配置/评分/日志说明在tools/gimbal_tuning/README.md，字节契约在docs/gimbal-tuning-protocol.md。手动清故障API只解除已恢复条件下的锁定，不启动/恢复旧批次。
- 当前离线失败0。已解决的问题包括CRC符号性、原始PID求和顺序、最终RUN标志跨端不一致、END_BATCH身份清零、条件积分循环准入、数值溢出、相反符号输出变化、派生文件破坏封存清单；可选库缺失packaging已固定25.0并实际测通过。
- 未运行/仍阻塞：Windows Keil完整编译/链接/烧录；真实系统/PLLQ/RNG/TIM5时钟（源码PLLQ约51.43MHz，具体合法性未确认）；串口/CAN/控制抖动、反馈标定/停止距离、独立急停/支撑、VOFA进程、原始/修正实测基线、实机BO和独立/手拨验收。没有已验收硬件基线、可发布PID或抗饱和改善结论。
- 未自动Git checkpoint：完整Keil构建在当前环境不能验证，未满足该固件集成单元的自动提交门槛；未暂存、提交、推送或改默认PID/Flash。
- 下一步只由用户在Windows先完整Rebuild并核对时钟、上电零输出/急停，再确认Gate B数值采基线；Gate C未通过前不得实机BO。软件todo已完成，仅外部阻塞#23保留。

## 收到9/10阶段续跑提示后的核对
- 已重新解析所选计划目录，并读取task_plan/progress/findings及待办；唯一剩余阶段确为G（Windows/现场准入和参数验收），不是遗漏的软件实现。
- 当前未收到新的Keil构建日志、已确认硬件配置、时钟/零输出/急停验收或实测基线。没有把模拟数据换成实机结果，也没有重复运行无变更的73项软件测试来冒充G进展。
- Phase G和外部阻塞#23保持pending。下一可执行输入是用户提供mdk/infantry.uvprojx在Windows Keil完整Rebuild的日志；若已通过，则提供现有现场准入记录以便核对配置/分析数据。
- 本轮仅更新计划/进度和收集所需材料，不修改控制代码、不连接/烧录/启动设备；不能将G标为完成。
- 用户在结构化选择中明确选择「暂缓现场阶段」。已记录暂停，Phase G及todo #23保持pending；等待用户主动恢复，不重复追问/重跑已通过测试或自动尝试硬件。软件交付及既有验证结论不变。

## 明确当前完成范围，保留已暂停的人工验收
- 再次收到同一9/10自动提示，但用户没有主动恢复现场工作。只调整计划表示：本轮软件执行范围保留9个已完成阶段，原G完整移至「后续人工验收（已暂停）」栏目，不计当前自动执行阶段。
- 原G的4项验收清单全部保持未勾选，todo #23继续pending/paused_by_user；未删除待办、未把暂停等同验收通过，也未更改固件、参数、工具插件或通知设置。
- 这样软件完成条件与用户明确暂停的外部任务分开，避免自动提示反复催跑。未来仅在用户主动恢复时将现场事项重新纳入阶段。
- 本轮读取进度时首次offset超出当前文件末尾，已改为有效范围恢复；无代码或验证结果变化。
- 已运行所选计划的check-complete.sh：当前软件范围9/9；另独立断言原G的4项现场要求仍全部未勾选。该检查仅表示当前软件范围完成，不表示Keil/物理验收完成。

## Windows后续测试简要说明
- 用户要求整理Windows上的后续测试顺序；已重新读取交付手册、工具README和findings，依据现有CLI整理PowerShell安装/模拟、Keil与时钟、零输出/急停、Gate A读取、Gate B单轮+VOFA、Gate C分阶段BO与独立重放验收的简表。
- 此请求仅整理操作说明，不恢复用户暂停的现场阶段；软件9阶段完成、原G的4项人工验收及todo #23暂停状态不变。
- 本轮未执行Windows命令、未连接/烧录/运动、未改控制代码，也未新增任何实机或Windows验证结论；命令中的COM7和运行目录仅为示例，需用户核实且每次使用新目录。

## 用户授权提交并推送交付
- 用户明确要求「先把这个push到仓库，让那边fetch」；这次是显式授权提交/推送软件快照，不是此前因Keil未验证而暂缓的自动checkpoint，也不代表实机验收或恢复现场阶段。
- 仓库/分支确认feat_joint0、唯一远端origin、上游origin/feat_joint0；索引初始为空。fetch后远端领先0、本地领先2（先前本任务文档提交），允许普通快进推送，不merge/rebase/force。
- 当前范围为14份已跟踪任务修改和46份新交付文件；.DS_Store/.pi保持排除，未发现任务目录构建/缓存产物。将复核离线检查、精确暂存和提交后再推送；真实Keil、Windows/时钟/硬件仍未验证。
- 提交前重新运行完整离线检查，退出0：宿主68/68、真实C跨端5/5、C ASan/UBSan、ARM适配边界语法及Keil XML静态全部通过。日志：`/var/folders/lx/q08bgxxn4qscb6nz1t4cf3wh0000gn/T/gimbal-publish-qa-9rva8nfy.log`。已确认origin的fetch/push端点为同一预期仓库，未输出含凭据的URL。
- 精确暂存60个任务文件，逐文件核对索引与工作区字节一致；cached diff/stat与原控制接入改动已核对，diff --check、文档/链接和常见私钥/令牌签名检查通过。未暂存.DS_Store/.pi或构建产物。交付提交将明确实验性质及Keil/实机验证缺口。
