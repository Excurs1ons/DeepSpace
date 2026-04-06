# DeepSpace 项目设计文档 (Game Design Document)

本设计文档基于《坎巴拉太空计划》(Kerbal Space Program, KSP) 的完整玩法循环，结合我们“微软飞行模拟级别的完全拟真物理”的要求，规划 `DeepSpace` 的长远开发路线。

---

## 1. 核心玩法循环 (Core Gameplay Loop)

KSP 的魅力在于“设计 -> 试错 -> 飞行 -> 探索”的闭环。DeepSpace 将完全继承并深化这一循环。

### 1.1 航天器设计 (VAB / SPH)
- **玩法**：玩家在一个类似于垂直装配大楼 (VAB) 的 UI 界面中，通过模块化组件（指令舱、燃料箱、引擎、气动控制面、分离环）拼装火箭。
- **机制**：
  - **基于节点 (Node-based) 捕捉**：部件之间通过预设的附着点连接。
  - **核心指标计算**：在车间内实时计算并显示：推重比 (TWR)、总 $\Delta v$ (Delta-V)、质心 (CoM)、推力中心 (CoT) 和气动中心 (CoL)。
- **DeepSpace 特色**：提供真实世界的组件库 (如 SpaceX, NASA 历史组件)，并考虑真实的材料应力与燃料晃动 (Slosh) 效应。

### 1.2 任务规划与发射 (Mission Planning & Launch)
- **玩法**：将设计好的火箭推上发射台。玩家需要手动或编写脚本控制节流阀 (Throttle) 和姿态控制 (Pitch/Yaw/Roll)。
- **机制**：
  - **重力转弯 (Gravity Turn)**：在致密大气中起飞，并在高空逐渐倾斜，以最小的气动损失和重力损失进入轨道。
  - **分级 (Staging)**：在最佳时机抛弃死重（空油箱、用过的助推器）。
- **DeepSpace 特色**：引入高拟真的天气系统（风切变层）、真实的空气动力学（跨音速激波、Max-Q 结构过载限制）。

### 1.3 轨道机动与领航 (Orbital Execution & Piloting)
- **玩法**：在太空中，飞行不再是“一直踩油门”，而是“在特定时间点（节点）进行精确变轨”。
- **机制**：
  - **机动节点 (Maneuver Nodes)**：玩家在地图视图中规划未来的加速计划（顺行、逆行、法向、径向），系统预测燃烧后的新轨道。
  - **交会对接 (Rendezvous & Docking)**：霍曼转移轨道、调整相对速度，并利用 RCS (反推控制系统) 进行毫米级微调对接。

### 1.4 探索与经营 (Exploration & Career)
- **玩法**：到达其他天体（如月球、火星），进行着陆、科研或建立基地。
- **机制**：
  - **科技树 (Tech Tree)**：通过收集科学数据解锁更先进的部件（如核推进、离子引擎）。
  - **合同与资金 (Contracts & Funds)**：接受商业发射任务获取资金以维持航天局运转。

---

## 2. 引擎架构映射 (Engine Architecture Mapping)

为了实现上述宏大的玩法，我们的代码架构将围绕以下模块展开：

### 2.1 物理与环境层 (Physics & Environment)
- **`PhysicsBody` & `OrbitalMechanics`**：处理双精度的牛顿运动定律、开普勒轨道根数解算、N体引力场（未来扩展）。
- **`Planet` & `Atmosphere`**：实现基于真实大气数据（如 US Standard Atmosphere 1976）的温度、压力、密度梯度。
- **`Aerodynamics`**：处理基于马赫数和攻角的升力、阻力、气动加热。

### 2.2 载具系统 (Vessel Systems)
- **`Part` & `PartLibrary`**：部件的基础类，包含干质量、资源储备等。
- **`StagingSystem`**：管理部件的生命周期和逻辑树，处理分离产生的拓扑变化（一个 Vessel 分裂为多个 Vessel）。
- **资源管理 (Resource System)**：管理液氧 (LOX)、液氢 (RP-1)、单组元推进剂 (Monopropellant)、电力 (Electric Charge) 的流动与消耗。

### 2.3 交互与表现层 (UI & Rendering)
- **`FlightUI` (HUD)**：提供姿态仪 (Navball)、高度计、速度计、G力表、燃料余量监控。
- **`MapUI` (Map View)**：渲染行星系、显示轨道线、机动节点规划器。
- **`VAB Layer`**：负责组装逻辑，UI 拖拽、对称放置 (Symmetry) 以及 $\Delta v$ 计算器。

---

## 3. 开发里程碑 (Development Roadmap)

- [x] **Milestone 1: 亚轨道与大气物理**（已完成）
  - 核心力学、变比冲引擎、真实气动模型、重力转弯雏形。
- [x] **Milestone 2: 多级入轨**（已完成）
  - 完善的 Staging 逻辑，成功模拟 Falcon 9 两级入轨。
- [ ] **Milestone 3: 轨道机动与地图视图**（Next）
  - 实现 RCS 姿态控制、圆轨逻辑 (Circularization)。
  - 建立基础的轨道预判模型（推算未来的远/近地点）。
- [ ] **Milestone 4: VAB 载具组装大楼**
  - 实现基于数据的部件树 (JSON/YAML 驱动)，允许玩家自由拼接。
- [ ] **Milestone 5: 进阶天体力学**
  - 添加“月球”，实现跨 SOI (Sphere of Influence, 引力作用球) 的轨迹计算。

---
*Last Updated: 2026-04-06*
