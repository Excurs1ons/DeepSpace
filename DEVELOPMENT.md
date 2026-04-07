# DeepSpace 模拟器开发技术文档 & 知识积累

本文件记录基于 PrismaEngine 的高拟真火箭发射模拟器 `DeepSpace` 的关键实现、踩坑记录与工程约束。

---

## 1. 核心架构设计 (Architecture)

### 1.1 PrismaEngine SDK 集成模式
DeepSpace 使用 **PrismaEngine 预编译 SDK** 构建。
- 通过 `find_package(PrismaEngine REQUIRED)` + `target_link_libraries(DeepSpace PRIVATE PrismaEngine::Engine)` 集成。
- 自动下载仅覆盖 Linux ARM64；Windows x64 SDK 尚未发布，CMake 会提示并要求手动指定 `PRISMA_SDK_DIR`。

### 1.2 双精度物理为底线
- 所有轨道与动力学计算统一 `double` 与 `Vec3d`。
- `PhysicsBody` 提供步长与惯量保护，避免异常姿态发散。

---

## 2. 核心算法与模型更新 (Aerospace Physics)

### 2.1 变比冲推力模型
- `EnginePart` 保持海平面/真空比冲插值。
- 推力按 `mdot * g0 * Isp(p) * throttle` 计算。

### 2.2 双组元推进剂系统
- 新增 `PropellantType`：`RP1`、`LOX`、`LH2`、`MMH`、`NTO`。
- `EnginePart` 新增：
  - `fuelType` / `oxidizerType`
  - `mixtureRatio`（O/F 质量比）
  - 燃料/氧化剂质量分数。
- `FuelTankPart` 支持推进剂类型分仓。
- `Vessel::Update` 按 **Stage + PropellantType** 消耗推进剂：
  - 对每个发动机分别计算燃料与氧化剂需求。
  - 按最短缺项形成 `burnRatio`。
  - 输出 `totalFuelFlow`、`totalOxidizerFlow`。

### 2.3 轨道机动与预判
- `OrbitalMechanics::CalculateElements`：稳健束缚轨道判定。
- `PredictVacuumExtrema`：短时真空积分预估 Ap/Pe。
- 圆轨逻辑支持近拱点自动点火与节流。

---

## 3. Artemis II 飞行计划（持续完善）

`SimulationLayer` 采用 `Artemis II Mission` 模板并加入任务事件机：
- **Stage 2（上升段）**：主推进段（可运行代理）+ Max-Q 自动降油门。
- **Stage 1（ICPS）**：RL10B-2 + LH2/LOX 双组元圆轨段。
- **Stage 0（Orion）**：AJ10-190 + MMH/NTO 服务舱推进段。

已实现事件：
1. Max-Q 观测与通过提示。
2. 主级耗尽自动分级。
3. ICPS 稳定窗口提示。
4. TEI 准备窗口提示。
5. Orion 服务舱推进接管。

> 当前仍是“可运行任务流程优先”的工程模板，不是 1:1 飞行力学复刻。

---

## 4. 遥测与验证

遥测输出包含：
- `Alt/Vel/Mach/q`
- `Ap/Pe` 与预测 `PredAp/PredPe`
- `Thrust`、`ThrPct`、`mdot`
- `fuel/ox` 实时流量
- ICPS 与 Orion 关键推进剂余量

建议每次调参对比日志中的 Max-Q 时间、分级时刻与圆轨收敛趋势。

---

## 5. 工程经验总结

1. 分层是关键：任务逻辑在 `SimulationLayer`，推进路由在 `Vessel`。
2. 双组元必须按质量比建模，不能回退到“总油量”近似。
3. 任务事件机 + 可观测遥测是保证复杂流程可维护的核心。

---
*Last Updated: 2026-04-07*
