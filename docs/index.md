---
layout: home

hero:
  name: "RoboCup Demo 源码全解"
  text: "人形机器人足球系统 · 中文逐行讲解"
  tagline: 从入口 ./scripts/start.sh 出发，读懂每一行代码在做什么、为什么这么做
  actions:
    - theme: brand
      text: 开始阅读 →
      link: /01-启动与架构/
    - theme: alt
      text: 大脑决策（核心）
      link: /07-行为树与决策/

features:
  - title: 01 · 启动与架构
    details: start.sh 逐行、脚本家族、ROS2 launch、main 多线程、编译与依赖
    link: /01-启动与架构/
  - title: 02 · 接口与消息
    details: 各节点之间的数据契约——vision / game_controller / booster ROS2 消息定义
    link: /02-接口与消息/
  - title: 03 · 视觉模块
    details: YOLOv8 检测+分割，以及像素反推三维坐标的核心几何
    link: /03-视觉模块/
  - title: 04 · 裁判机与通信
    details: GameController UDP 协议解析、队内 UDP 发现与协作选主
    link: /04-裁判机与通信/
  - title: 05 · 数据与坐标系
    details: Brain / BrainConfig / BrainData，以及相机/机器人/场地三套坐标系换算
    link: /05-大脑数据与坐标系/
  - title: 06 · 定位与球预测
    details: 粒子滤波自定位 Locator + IMM 卡尔曼球预测，决策的两大输入引擎
    link: /06-定位与球预测/
  - title: 07 · 行为树与决策
    details: 项目灵魂——BehaviorTree.CPP 主树、前锋/守门员决策、追球/调整/踢球动作
    link: /07-行为树与决策/
  - title: 08 · 控制与底层
    details: RobotClient 指令翻译、状态回调、头部主动控制、避障、可视化与训练记录
    link: /08-机器人控制与底层/
---

## 这是个什么项目？

一句话：**让一台人形机器人自己踢一场完整的 RoboCup 足球赛**。

机器人要能：用摄像头**看见**球、球门、对手、场地线 → **算出**它们在场地上的位置 → 知道**自己站在哪**（自定位）→ 听**裁判机**的开球/罚球/暂停指令 → 跟**队友**通信协作 → 然后**决策**是追球、调整、还是射门 → 最后**控制双足**走过去并踢球。

整个系统由三个独立运行的程序（ROS2 节点）组成，外加一堆消息接口包：

| 程序 | 角色 | 通俗比喻 |
|------|------|----------|
| **vision** | 视觉识别 | 机器人的**眼睛** |
| **brain** | 决策大脑 | 机器人的**大脑** |
| **game_controller** | 裁判机通信 | 机器人的**耳朵**（听裁判口令）|

它们之间通过 **ROS2 话题（topic）** 这种"发布-订阅"机制传递消息，互不阻塞、各跑各的。

## 系统全景图

```mermaid
flowchart LR
    REF([裁判机 UDP 3838]) -->|UDP| GC[game_controller<br/>耳朵·裁判机通信]
    CAM([摄像头]) -->|RGB-D 图像| VIS[vision<br/>眼睛·视觉识别]
    VIS -->|/detection · /line_segments| BRAIN[brain<br/>大脑·决策]
    GC -->|/game_controller| BRAIN
    TM([队友机器人]) <-->|UDP 组播/单播·队内通信| BRAIN
    BRAIN -->|RPC 指令| SDK[Robot SDK / 双足<br/>走路·踢球·转头]
```

## 怎么读这套文档

- 建议**按 01 → 08 顺序阅读**建立全局观，也可用左侧侧边栏按需跳读。
- 文中所有形如 `src/brain/src/brain.cpp:866` 的引用都是**真实文件:行号**，可在编辑器里点开定位。
- 涉及"为什么这么设计"的地方用 `💡` 标注，涉及"比赛规则约束"的地方用 `🏆` 标注。
- 关键术语第一次出现会加粗解释，后文复用。
- 每个模块是一个文件夹，内含**模块索引 + 细粒度子篇**，篇与篇之间用底部导航条互相跳转。

> 本套文档是仓库根目录官方 `README.md` 的**代码级深度补充**。
