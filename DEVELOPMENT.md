# DeepSpace 模拟器开发技术文档 & 知识积累

本文件记录基于 PrismaEngine 的高拟真火箭发射模拟器 `DeepSpace` 的关键实现、踩坑记录与工程约束。

---

## 1. 核心架构设计 (Architecture)

### 1.1 PrismaEngine SDK 集成模式
DeepSpace 使用 **PrismaEngine 预编译 SDK** 构建。
- 通过 `find_package(PrismaEngine REQUIRED)` + `target_link_libraries(DeepSpace PRIVATE PrismaEngine::Engine)` 集成。
- 目前自动下载仅覆盖 Linux ARM64；Windows x64 SDK 尚未发布，CMake 已显式给出提示并要求手动指定 `PRISMA_SDK_DIR`。

### 1.2 双精度物理为底线
- 所有轨道与动力学计算统一 `double` 与 `Vec3d`。
- `PhysicsBody` 增加了 dt/惯量保护，避免异常步长导致姿态发散。

---

## 2. 核心算法与模型更新 (Aerospace Physics)

### 2.1 变比冲推力模型
- `EnginePart` 维持海平面/真空比冲插值。
- 实际推力按 `mdot * g0 * Isp(p) * throttle` 计算。

### 2.2 双组元推进剂系统（新）
- 新增 `PropellantType`：`RP1`、`LOX`、`LH2`、`MMH`、`NTO`。
- `EnginePart` 新增：
  - `fuelType` / `oxidizerType`
  - `mixtureRatio`（O/F 质量比）
  - 燃料/氧化剂质量分数计算。
- `FuelTankPart` 新增推进剂类型字段。
- `Vessel::Update` 改为按 **Stage + PropellantType** 消耗推进剂：
  - 先计算所需燃料与氧化剂质量。
  - 分仓拉取推进剂并按最短缺项计算 `burnRatio`。
  - 遥测新增总流量、燃料流量、氧化剂流量。

### 2.3 轨道机动与预判（Milestone 3 完成）
- `OrbitalMechanics::CalculateElements` 支持更稳健的束缚轨道判定。
- 新增 `PredictVacuumExtrema`：短时真空积分预测未来 Ap/Pe。
- 圆轨逻辑支持近拱点自动点火与油门切换。

---

## 3. Artemis II 飞行计划（新）

`SimulationLayer` 已切换为 `Artemis II Mission` 模板：
- **Stage 1**：上升级（当前以可运行的 RP-1/LOX 引擎组代理主推进段）。
- **Stage 0**：ICPS（RL10B-2，LH2/LOX 双组元）。
- 自动流程：
  - T-0 点火主推进段。
  - 主推进段 LOX 耗尽后自动分级。
  - ICPS 接管并执行高空圆轨引导。

> 注：当前为“可运行任务模板”，非 1:1 结构复刻，重点在任务流程与推进系统验证。

---

## 4. 工程经验总结 (Key Takeaways)

1. 分层是关键：任务逻辑在 `SimulationLayer`，推进与资源路由在 `Vessel`。
2. 双组元必须按质量比建模，不能再用“单油箱总量”近似。
3. 轨道预判与实时遥测同等重要，是调参与回归验证的基础。

---
*Last Updated: 2026-04-07*
