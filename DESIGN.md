# DeepSpace 项目设计文档 (Game Design Document)

本设计文档基于《坎巴拉太空计划》(Kerbal Space Program, KSP) 的完整玩法循环，结合“微软飞行模拟级别”的高拟真物理目标，规划 `DeepSpace` 的开发路线。

---

## 1. 核心玩法循环 (Core Gameplay Loop)

KSP 的魅力在于“设计 -> 试错 -> 飞行 -> 探索”的闭环。DeepSpace 继承该循环，并强化真实任务流程。

### 1.1 航天器设计 (VAB / SPH)
- **玩法**：在类似 VAB 的装配界面中，使用模块化部件拼装火箭与上面级。
- **机制**：
  - **基于节点捕捉**：部件通过附着点连接。
  - **实时指标**：TWR、总 Delta-V、CoM、CoT、CoL。
- **DeepSpace 特色**：支持真实推进剂组合（如 RP-1/LOX、LH2/LOX）与任务模板。

### 1.2 任务规划与发射 (Mission Planning & Launch)
- **玩法**：将载具推上发射台，执行自动/手动姿态与分级控制。
- **机制**：
  - **重力转弯**：在大气层内按高度进行俯仰程序。
  - **分级**：在推进剂耗尽后分离级间结构并点火上面级。
- **当前任务模板**：新增 **Artemis II 飞行计划**（近地轨道入轨 + 上面级圆轨流程）。

### 1.3 轨道机动与领航 (Orbital Execution & Piloting)
- **机制**：
  - 近拱点圆轨自动点火（可切换）。
  - 轨道预测：通过短时真空积分预估 Ap/Pe。
  - RCS 平移与姿态稳定用于精调。

### 1.4 探索与经营 (Exploration & Career)
- 后续扩展：月球转移、交会对接、科研与合同体系。

---

## 2. 引擎架构映射 (Engine Architecture Mapping)

### 2.1 物理与环境层 (Physics & Environment)
- `PhysicsBody`：双精度积分与姿态更新。
- `OrbitalMechanics`：轨道根数解算与轨道预判。
- `Planet`/`Atmosphere`：重力、大气压与密度模型。

### 2.2 载具系统 (Vessel Systems)
- `Part` / `PartLibrary`：部件模型与任务组件库。
- `EnginePart`：支持燃料+氧化剂双组元质量比（O/F）。
- `FuelTankPart`：按推进剂类型分仓。
- `Vessel::Update`：按 `Stage + PropellantType` 路由推进剂消耗。

### 2.3 交互与表现层 (UI & Rendering)
- `SimulationLayer`：任务编排、输入、自动驾驶与遥测输出。
- 遥测新增：总流量、燃料流量、氧化剂流量。

---

## 3. 开发里程碑 (Development Roadmap)

- [x] **Milestone 1: 亚轨道与大气物理**
- [x] **Milestone 2: 多级入轨**
- [x] **Milestone 3: 轨道机动与地图视图基础**
  - RCS 姿态控制与圆轨逻辑已接入。
  - 基础轨道预判（Ap/Pe 采样）已实现。
- [ ] **Milestone 4: VAB 载具组装大楼**
  - 计划转为 JSON/YAML 数据驱动。
- [ ] **Milestone 5: 进阶天体力学**
  - 添加月球与跨 SOI 轨迹。

---
*Last Updated: 2026-04-07*
