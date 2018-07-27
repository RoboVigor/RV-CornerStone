# RV-CornerStone

RoboVigor 电控基石。

## 规范

- 裸机（无系统）
- 使用配置文件调整功能开关及参数
- 使用utf-8
- 采用代码规范
- 采用[Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)作为版本管理

## 规划

- [x] 将工程车仓库中将所有代码文件（.c, .h）格式从ansi转为utf8以解决中文注释乱码的问题
- [ ] 给while(1)的所有脚本全部安排上计时器 
- [ ] 整理工程车项目目录
- [ ] 将工程车代码并入本仓库
- [ ] 测试无线串口调试
- [ ] 参考电科代码重新封装模块、隔离变量、重写不规范的代码
- [ ] 在工程车上调试底盘运动等基础功能
- [ ] 以英雄车为模板重写功能代码（脱离Free RTOS）
- [ ] 各车种兼容性调试
- [ ] 分发各车种版本
- [ ] 实现各车种特有功能

