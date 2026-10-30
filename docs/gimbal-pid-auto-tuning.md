# 云台旋转双电机 PID 定位与自动调参方案

> 本文保留原提交 `273617f` 的历史静态定位与建议。当前实现、操作入口和未完成的现场验收见 [交付手册](gimbal-pid-tuning-runbook.md)，不要把下文历史“尚待处理”清单当作当前代码状态，也不要将已修复代码当作实机验证。

## 1. 范围与结论

本次主要研究机械臂云台旋转（关节 0）的 PID 自动调参，不是整条机械臂所有关节的联合调参。

源码核对基线：Git 提交 `273617f`。以下为静态分析结论，未编译、未连接实机，参数有效性与闭环稳定性尚未验证。行号用于定位，后续修改可能导致偏移。

新版代码已经实现：**两台 GM6020 共用一套位置—速度双环 PID，只使用第一台电机的位置和速度作为闭环反馈，将控制输出各分配一半给两台电机。**

用户描述第二台用于卸力、不独立做 PID；代码具体实现是跟随共同闭环输出，并非固定卸力电流。指令均分不代表实际机械力矩必然完全均分。

## 2. 电机与控制入口

### 2.1 双电机组

初始化位于 `user/infantry/main.c:38–48`：

| 对象 | CAN 反馈 ID | 减速比配置 | 作用 |
|---|---|---|---|
| `Motor6020_MecArm_0_0` | `0x205` | 2.0 | 提供位置、速度反馈，承担一半输出指令 |
| `Motor6020_MecArm_0_1` | `0x206` | 2.0 | 不独立做 PID，承担另一半输出指令 |

两个电机绑定到 `Motor6020_MecArm_Group_0`，再通过 `Arm_Joint_0` 绑定电机组控制回调。

### 2.2 调用链

```text
Task_MecArm()                         user/infantry/tasks.c
  ├─ MecArm_Calculate()               driver/src/Driver_MecArm.c
  │    └─ joint_target[0] = polarTheta
  ├─ MecArm_Control()
  │    └─ 关节 0 绑定目标和电机组 PID 指针
  │         └─ Motor6020_Group_Set_Target()
  │              ├─ 位置 PID ← motor0 角度
  │              ├─ 速度 PID ← motor0 速度
  │              ├─ 50% 输出 → motor0
  │              └─ 50% 输出 → motor1
  └─ Bridge_Send_Motor() → CAN 下发
```

关键位置：

| 文件 / 函数 | 含义 |
|---|---|
| `user/infantry/main.c:43–44` | 电机组 PID 初始化 |
| `user/infantry/tasks.c:8` / `Task_MecArm()` | 周期任务，配置周期 1 ms |
| `driver/src/Driver_MecArm.c:90` / `MecArm_Calculate()` | 旋转角目标来自 `polarTheta` |
| `driver/src/Driver_MecArm.c` / `MecArm_Control()` | `i == 0` 分支绑定电机组 PID |
| `driver/src/Driver_Motor.c:240` / `Motor6020_Group_Set_Target()` | 双环计算与双电机输出分配 |
| `driver/src/Driver_Motor.c:261` / `Motor6020_Group_Get_Angle()` | 只读取 motor0 角度，转换为 rad |
| `driver/src/Driver_Motor.c:272` / `Motor6020_Group_Get_Speed()` | 只读取 motor0 速度，转换为 rad/s |
| `driver/src/Driver_PID.c` / `PID_Calculate()` | 通用 PID 计算入口 |

当前任务调用 `MecArm_Calculate(&MecArm, PI/2.0f, 500.0f, 0, PI/2.0f, 0)`。需要注意，关节 0 目标赋值也在关节 1～3 的角度范围判断内部；若该判断不通过，旋转目标不会更新。

### 2.3 输出分配代码

```c
float angle_feedback = Motor6020_Group_Get_Angle(self);
float target_speed = PID_Calculate(pid_pos, joint_command->pos_des, angle_feedback);
float speed_feedback = Motor6020_Group_Get_Speed(self);
float target_current = PID_Calculate(pid_speed, target_speed, speed_feedback);
motor0->input = (int16_t)(target_current * motor0->direction / 2.0f);
motor1->input = (int16_t)(target_current * motor1->direction / 2.0f);
```

`target_current` 是代码中的电流控制指令量，不应直接视为单位为 A 的物理电流。当前两台电机初始化方向均为 1。

## 3. 本次要调的参数

```c
PID_Init(&Motor6020_MecArm_Group_0_PosPID,
         1.8f, 0.0f, 25.0f, 2.0f, 1.0f);
PID_Init(&Motor6020_MecArm_Group_0_SpeedPID,
         800.0f, 16.0f, 0.0f, 16384.0f, 10000.0f);
```

`PID_Init()` 参数依次是 Kp、Ki、Kd、输出限幅、积分项限幅。

| 环路 | Kp | Ki | Kd | 输出限幅 | 积分项限幅 |
|---|---:|---:|---:|---:|---:|
| 位置环 | 1.8 | 0 | 25 | 2 | 1 |
| 速度环 | 800 | 16 | 0 | 16384 | 10000 |

当前为位置 PD + 速度 PI。第一版可先保持这一结构和 50/50 输出分配，只优化四个有效增益；先整定速度内环，再整定位置外环，最后联合验证。限幅应作为经过确认的保护约束，不与增益一起任意放大。

以下对象不应混淆：

- `Motor6020_MecArm_3_PosPID/SpeedPID` 属于关节 3、CAN ID `0x208`，不是本次主要目标。
- 宇树关节 1、2 下发 Kp/Kd 与前馈力矩，不使用同一套 MCU 双环 PID。
- `Motor_Yaw/Motor_Pitch` 和 `PID_Cloud_*` 在已检查的 `user/driver` 源码中只有声明，不能凭名称认定为实际旋转控制入口。

## 4. 现有观测与需要补充的数据

新版 `user/infantry/tasks.c` 已开启 VOFA 三通道发送：

1. 固定参考值 `PI/2`；
2. motor0 角度，单位 rad；
3. motor1 角度，单位 rad。

`main.c` 配置调试串口为 500000 波特率。第一通道是固定参考常数，不是动态读取实际应用的目标角度，自动实验时应改为记录真实目标。

建议自动调参数据至少包括：时间戳、实际目标角度、两电机角度和速度、PID 总输出、两电机输出指令与反馈电流、饱和状态、在线状态、温度。根据带宽选择降采样或缓冲发送，避免采集影响控制周期。

第二台反馈用于一致性与故障监测，不必增加第二套位置 PID。比较两台角度前应核实零位、方向、减速比和机械映射一致。

## 5. 自动调参前置事项

### 5.1 已确认的在线检查错误

`driver/src/Driver_Motor.c:283` 的 `Motor6020_Group_Is_Online()` 返回：

```c
return (motor0->online & motor0->online);
```

重复检查 motor0，漏掉 motor1。需要修正为检查两台电机，并确保控制路径实际调用与执行故障处理。单独修复这个函数不等于已经完成双电机保护。

### 5.2 采样周期与 PID 参数语义

此前读取的 `PID_Calculate()` 使用离散累加与误差差分，没有显式 dt：

```c
output_I += i * error;
output_D = d * (error - lastError);
```

若仍使用该实现，连续时间增益需按固定采样周期换算：

```text
i_code = Ki_continuous × Ts
d_code = Kd_continuous ÷ Ts
```

实施前重新核实 PID 实现及实际采样周期。1 ms 是任务配置，不等于已经验证调度周期、反馈新鲜度和端到端延迟。

### 5.3 保护、参数更新与测试边界

- 关节 0 初始化的角度、速度和力矩限制目前均为 0，不能当作已生效的机械安全范围。
- 建立双电机在线检查、反馈超时检查、角度/速度/电流/温度限制、异常中止与安全基线回退。
- 更新参数应在控制周期边界统一应用，避免读取到混合参数。
- 不要用反复调用 `PID_Init()` 的方式进行在线增益更新；该函数会清空积分和历史状态，需要专门的无扰切换策略。
- 对助力比例先保持固定；改变分配比例会改变被控对象，不能混入同一批实验而不记录。
- 新版 `MecArm_Control()` 已恢复目标/PID 指针赋值，并将之前的 `&joint_command` 改为正确的 `joint_command`，旧版这两项问题不再沿用。

## 6. 推荐算法与落地路线

### 6.1 区分自动整定和在线自适应

- **自动整定**：运行一段测试轨迹，评价响应，再生成下一组参数。
- **在线自适应**：正常运行中根据姿态、负载等变化更新参数。

推荐组合是：**上位机贝叶斯优化进行自动整定，STM32 使用经过验证的工况参数表进行增益调度。** 不建议第一版每个控制周期都进行自由参数搜索。

| 算法 | 建议定位 |
|---|---|
| 贝叶斯优化 BO | 优先用于低维、实验代价较高的实机自动整定 |
| 增益调度 | 根据机械臂姿态、负载等查表/插值，适合运行时调整 |
| RLS 在线辨识 + 自整定 | 后续阶段；需要有效激励、模型可信度检查与参数约束 |
| 模糊 PID | 轻量备选，但规则仍需设计，不能天然保证更优 |
| 继电反馈 / Z-N | 不作为整臂首选，振荡实验需考虑机械限位与耦合 |
| PSO / 遗传算法 / 强化学习 | 优先用于仿真，不作为第一版实机探索方案 |

### 6.2 第一阶段：固定分配下的双电机组自动整定

```text
Python 优化器
  → 下发候选增益
  → STM32 验证范围并在周期边界应用
  → 执行限幅、可重复的测试轨迹
  → 采集与回传数据
  → 计算评分
  → 保留更优参数 / 异常中止并回退
```

评分可由归一化后的跟踪误差、超调、控制输出变化、饱和程度构成。角度、速度、电流、温度及通信超时必须另设硬约束，不能只在评分中扣分。BO 的模型约束不替代实时保护。

先固定其余关节姿态和负载条件，完成速度内环、位置外环的整定，再验证完整运动任务。

### 6.3 第二阶段：工况增益调度

对不同机械臂伸展姿态、末端负载建立已验证参数表，运行时限制增益变化速度并平滑切换。各工况分别稳定不代表插值与切换过程一定稳定，必须额外验证过渡区域。

### 6.4 第三阶段：可选在线辨识

若参数表不能覆盖负载变化，再加入 RLS 等方法估计局部动力学，仅在可信范围内调整增益。激励不足、反馈过期、饱和或模型不可信时冻结更新并回退。

参考资料：机器人控制器贝叶斯自动整定案例：
https://alonsomarco.me/project/auto_tuning_bayesian

## 7. 验证状态与下一步

- 已完成：新版双电机初始化、PID 绑定、反馈来源、输出分配和 VOFA 调试路径的静态核对。
- 未执行：编译、实机运行、自动整定实验、稳定性验证。
- 尚待处理：双电机在线检查错误、实际保护路径、采样与反馈时序、参数更新接口和实验采集协议。
- 建议下一步：围绕关节 0 电机组打通保护与数据采集，再开展固定 50/50 输出分配下的自动整定。
