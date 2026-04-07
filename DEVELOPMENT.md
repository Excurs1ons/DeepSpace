# DeepSpace 模拟器开发技术文档 & 知识积累

本文件记录基于 Mock 引擎接口的高拟真火箭发射模拟器 `DeepSpace` 的关键实现、踩坑记录与工程约束。

---

## 1. 核心架构设计 (Architecture)

### 1.1 Mock-First 开发策略
DeepSpace 采用 **Mock-First** 策略：先用 Mock 引擎实现功能，等项目基本完成后再切换真实 SDK。

**Mock 引擎接口层 (`src/engine/`)：**
- `MockEngine.h` - 引擎主类，模拟游戏循环与 Layer 系统
- `MockInputManager.h` - 键盘/手柄输入模拟
- `MockWindow.h` - 文本模式窗口（无 SDL3/OpenGL 依赖）
- `MockLogger.h` - 日志输出到控制台
- `MockSceneManager.h` - 简化场景管理
- `MockAudioManager.h` - 音频占位（输出日志而非播放）
- `MockRenderer.h` - 渲染占位（用于未来扩展）

**逆推引擎需求：**
- 通过 Mock 接口的使用方式，编写 `ENGINE_REQUIREMENTS.md`
- 明确 PrismaEngine SDK 需要提供的底层接口
- 确保切换时有完整的需求文档

### 1.2 双精度物理为底线
- 所有轨道与动力学计算统一 `double` 与 `Vec3d`。
- `PhysicsBody` 提供步长与惯量保护，避免异常姿态发散。

### 1.3 CMake 构建模式
```cmake
# Mock 模式（默认）
cmake -S . -B build -G Ninja
# -> 自动启用 USE_MOCK_ENGINE=ON

# 真实 SDK 模式
cmake -S . -B build -G Ninja -DUSE_MOCK_ENGINE=OFF
# -> 需要 PrismaEngine SDK，可下载或手动指定 PRISMA_SDK_DIR
```

**当前状态：优先使用 Mock 模式开发，SDK 模式仅用于最终验证。**

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
2. 双组元必须按质量比建模，不能回退到"总油量"近似。
3. 任务事件机 + 可观测遥测是保证复杂流程可维护的核心。

---

## 6. 并行开发规范

### 6.1 关联性判断标准
| 关联等级 | 定义 | 并行可行性 |
|---------|------|-----------|
| **高关联** | 修改同一文件、同一模块、共享数据结构 | ❌ 禁止并行 |
| **中关联** | 不同文件但同一模块、有接口依赖 | ⚠️ 需协商接口 |
| **低关联** | 不同模块、无共享状态 | ✅ 可并行 |

### 6.2 并行开发许可矩阵
```
可并行开发:
  ✅ physics/ ↔ vessel/           (独立物理域)
  ✅ telemetry/ ↔ environment/     (无共享数据)
  ✅ engine/mock/ ↔ physics/      (单向依赖：physics不依赖engine)
  ✅ ui/ ↔ environment/          (无共享数据)

禁止并行:
  ❌ ui/ ↔ DeepSpaceApp.h       (共享SimulationLayer)
  ❌ engine/mock/ ↔ simulation/  (接口依赖)
  ❌ 不同文件修改同一模块          (需串行)
```

### 6.3 接口先行原则
在并行开发前，必须先定义好接口：
1. 接口类定义放在 `src/core/` 或 `src/engine/`
2. 实现类放在对应模块
3. 通过头文件依赖确定并行可行性

### 6.4 当前开发阶段并行建议
| 当前阶段 | 可并行任务 |
|---------|-----------|
| Mock引擎开发 | telemetry/, environment/ (无关联) |
| 物理系统开发 | UI组件/, 音频系统/ (无关联) |
| 任务系统开发 | 串行开发（涉及SimulationLayer） |
| VAB界面开发 | 独立模块，可与其他模块并行 |

---
*Last Updated: 2026-04-07*
