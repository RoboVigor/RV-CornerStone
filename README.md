# RV-CornerStone

RoboVigor 电控基石。

## 规范

- 永远采用最佳实践
- 使用配置文件调整参数
- 使用utf-8
- 采用代码规范
- 采用FreeRTOS
- 采用[Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)作为版本管理

## 关节 0 PID 自动整定（实验版）

- [交付与实机准入手册](docs/gimbal-pid-tuning-runbook.md)
- [Python CLI、模拟、记录与 BO](tools/gimbal_tuning/README.md)
- [协议与字段](docs/gimbal-tuning-protocol.md)

专用调参固件默认关节 0 零输出；停止不是主动保持。Keil 编译、现场安全数值、基线和实机验收仍须分别完成，不能将模拟结果当作可发布参数。

仓库根目录离线检查（不打开串口）：`python tests/run_gimbal_checks.py`。

---

♫
新的风暴已经出现
怎么能够停止不前
鼓气勇气，坚定向前
奇迹一定会出现
