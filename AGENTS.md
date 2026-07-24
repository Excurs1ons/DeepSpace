# AGENTS.md

This file provides guidance to Lingma (lingma.aliyun.com) when working with code in this repository.

## 项目定位

`DeepSpace` — Rust 航天模拟引擎与宇宙沙盘。Workspace 包含两个 crate：
- `deepspace/` — 物理库（**零外部依赖**，纯 std），提供 N 体引力、辛积分器、轨道力学、6DOF 航天器动力学、制导、火箭部件等。
- `demo/` — 应用层（依赖 `macroquad` 渲染 + `deepspace`），提供 CLI 仿真循环和 3D 可视化。

## 构建与测试命令

```bash
# 构建
cargo build

# 全量测试（物理库，179 个测试）
cargo test -p deepspace --lib

# 按模块过滤测试
cargo test -p deepspace --lib physics    # 物理引擎（33）
cargo test -p deepspace --lib scene      # 场景系统（13）
cargo test -p deepspace --lib vessel     # 飞船系统
cargo test -p deepspace --lib guidance   # 制导系统

# 运行单个测试
cargo test -p deepspace --lib physics::tests::test_name_here -- --exact

# 运行二进制
cargo run --bin rocket-sim                          # 3D 可视化
cargo run --bin rocket-sim -- --headless             # 控制台仿真
cargo run --bin rocket-sim -- --headless --csv out.csv
cargo run --bin nbody-sim                           # 3D（默认 solar_system）
cargo run --bin nbody-sim -- --scene scenes/figure8.scene
cargo run --bin nbody-sim -- --headless --scene scenes/three_body.scene --csv output.csv

# 场景热切换（headless 模式）
echo "/path/to/new_scene.scene" > /tmp/switch
cargo run --bin nbody-sim -- --headless --scene scenes/solar_system.scene --switch-file /tmp/switch
```

## 架构概览

### 两条独立仿真管线

本项目有两条**互不依赖**的仿真管线，共享 `deepspace` 物理库底层：

1. **N 体宇宙沙盘管线** (`nbody-sim`)：
   `SceneConfig`(解析 .scene) → `SceneRuntime`(持有 `GravitationalSystem`) → 辛积分器步进 → 3D/CSV 输出

2. **火箭任务管线** (`rocket-sim`)：
   `MissionConfig`(解析 .conf) → `SimulationApp`(持有 `Vessel` + `Planet` + `MissionControl` + `FlightComputer`) → 逐步物理仿真 → 3D/CSV 输出

### 物理库模块依赖关系

```
lib.rs (Vec3, G, G0, 常量)
  ├── core.rs (Quaternion, Mat3x3) ← 被 physics/space_physics/guidance 使用
  ├── environment.rs (Planet, Atmosphere ISA-1976, ThermalSimulation)
  ├── physics.rs
  │     ├── PhysicsBody — 通用可积分刚体（力/扭矩累积 → 欧拉步进）
  │     ├── Integrators — RK4 + RKF45 自适应（通用 ODE）
  │     ├── GravBody + GravitationalSystem — N 体引力核心
  │     │     step_leapfrog() / step_symplectic4() / step_adaptive()
  │     └── OrbitalMechanics / OrbitalElements — 轨道要素转换
  ├── space_physics.rs（构建在 physics.rs 之上）
  │     ├── SpacecraftBody — 6DOF 刚体（偏轴推力 → 扭矩）
  │     ├── SoiTree — 引力影响球层次查找
  │     ├── SpacePhysicsWorld — 统一入口（N体 + 航天器 + WarpMode 时间加速）
  │     └── FlightAssist — SAS 姿态控制
  ├── guidance.rs
  │     ├── GuidanceAlgorithm trait — 可插拔制导接口
  │     ├── FlightComputer — 飞控盒（持有算法 + 状态）
  │     └── CosineGuidance — 余弦重力转弯（当前默认算法）
  ├── vessel.rs (Part/PartKind 枚举组合, Vessel, 推进剂消耗, 级分离)
  ├── simulation.rs (MissionPhase 状态机, MissionControl, MissionScript, 事件系统)
  ├── scene.rs (SceneConfig 解析, SceneRuntime 运行时, 热切换)
  └── frame_graph.rs (层次化坐标参考系, LengthUnit 自动选择, LCA 跨帧距离)
```

### 关键设计决策

- **`deepspace` 零依赖**：所有数学（Vec3/Quaternion/Mat3x3）手写实现，不引入 nalgebra/glam。修改时不要添加外部依赖。
- **辛积分器是核心**：`GravitationalSystem` 的 `step_leapfrog()`（2 阶）和 `step_symplectic4()`（Yoshida 4 阶）保证长期能量守恒，适合数亿年模拟。`step_adaptive()` 根据最近天体距离自动调步长。
- **场景系统运行时可变**：`SceneRuntime` 支持 `load_scene()` 热切换（保留时钟）、`add_body()`/`remove_body()` 动态增删。
- **制导算法可插拔**：实现 `GuidanceAlgorithm` trait 即可添加新制导律，通过 `GuidanceConfig.algorithm` 字符串选择。
- **火箭部件用枚举组合**：`Part { kind: PartKind }` 替代继承，`PartKind::Engine/FuelTank/Decoupler`。
- **demo 层双模式**：每个 bin 无参数启动 3D 窗口（macroquad async main），`--headless` 进入纯控制台仿真。

### demo 层结构

- `app.rs` — `CliArgs` 参数解析 + `SimulationApp` 火箭仿真主循环（重力/阻力/发动机/制导/任务控制）
- `render.rs` — `OrbitalCamera`（轨道相机）+ 绘制原语（`draw_planet`, `draw_path` 等），封装 macroquad
- `bin/rocket-sim.rs` — 火箭任务入口（headless 调用 `SimulationApp::run()`，3D 用帧间隔 × 时间倍率步进）
- `bin/nbody-sim.rs` — N 体沙盘入口（headless 调用 `SceneRuntime::run_loop()`，3D 每帧 `runtime.step()`）

## 代码风格

- Rust 2021 edition，4 空格缩进
- 物理计算统一 `f64` + `Vec3`（`deepspace::Vec3`，非 macroquad 的）
- demo 层 3D 坐标用 `macroquad::math::Vec3`（f32），通过 `to_mvec3()` 转换
- 场景文件是自定义 INI 格式（`[scene]` + `[body.Name]`），解析在 `scene.rs`
- 任务配置也是 INI 格式（`[mission]` + 发动机/油箱 section），解析在 `simulation.rs`

## 提交规范

`type(scope): summary` — `feat:`、`fix:`、`refactor:`、`docs:`。
涉及物理参数/轨道要素变更时，保持测试覆盖。

## 仓库卫生

- 不提交：`target/`、`*.csv`
- 场景文件放 `scenes/`，任务配置放 `missions/`，均不参与编译（但 scene.rs 通过 `include_str!` 内嵌了三个内置场景用于测试）
# Repository Guidelines

## 项目定位
`DeepSpace` — Rust 航天模拟引擎与宇宙沙盘。核心引擎在 `deepspace/` 目录下（零外部依赖），
提供 N 体引力辛积分器（Leapfrog/Yoshida4）、场景化沙箱仿真、轨道力学、6DOF 航天器动力学等。
支持热切换场景文件，适合数亿年尺度的混沌三体模拟。3D 可视化在 `demo/` 目录下（macroquad 渲染）。

## 目录结构

```
DeepSpace/
├── Cargo.toml              ← workspace root
├── deepspace/              ← 物理库（零外部依赖）
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs          (Vec3, 常量, 模块导出)
│       ├── core.rs         (Quaternion, Mat3x3)
│       ├── physics.rs      (N体引力, 辛积分器, 轨道力学)
│       ├── environment.rs  (行星、大气、热模拟)
│       ├── space_physics.rs (SpacecraftBody, SoI, Warp, SAS)
│       ├── guidance.rs     (飞控, 制导律)
│       ├── vessel.rs       (火箭部件, 推进剂, 级分离)
│       ├── scene.rs        (场景配置解析, 运行时)
│       ├── simulation.rs   (任务控制, 脚本, 事件系统)
│       └── frame_graph.rs  (参考系图, 距离转换)
├── demo/                   ← 应用层（macroquad 可视化）
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs          (模块导出)
│       ├── app.rs          (CLI 参数解析, SimulationApp 仿真循环)
│       ├── render.rs       (3D 渲染原语: OrbitalCamera, 绘制函数)
│       └── bin/
│           ├── rocket-sim.rs  (火箭模拟: 无参数→3D, --headless→控制台)
│           └── nbody-sim.rs   (N体沙盘: 无参数→3D, --headless→控制台)
├── scenes/                 ← 场景文件 (.scene)
├── missions/               ← 任务配置文件 (.conf)
└── AGENTS.md               ← AI 辅助开发指南
```

## 构建与运行

```bash
# 全量测试（物理库）
cargo test -p deepspace --lib

# 火箭任务模拟：无参数→3D窗口，--headless→控制台
cargo run --bin rocket-sim                          # 3D 可视化
cargo run --bin rocket-sim -- --headless             # 控制台仿真
cargo run --bin rocket-sim -- --headless --csv out.csv

# N体宇宙沙盘：无参数→3D窗口，--headless→控制台
cargo run --bin nbody-sim                           # 3D（默认 solar_system）
cargo run --bin nbody-sim -- --scene scenes/figure8.scene   # 3D
cargo run --bin nbody-sim -- --headless --scene scenes/three_body.scene --csv output.csv

# 场景热切换（headless 模式）
echo "/path/to/new_scene.scene" > /tmp/switch
cargo run --bin nbody-sim -- --headless --scene scenes/solar_system.scene --switch-file /tmp/switch
```

## 二进制目标

| Target | 无参数 | --headless | 用途 |
|--------|--------|-----------|------|
| `rocket-sim` | 3D 可视化 | 控制台仿真 | 火箭任务模拟（SLS Block 1 / Artemis II） |
| `nbody-sim` | 3D 可视化 | 控制台仿真 | N体宇宙沙盘模拟器 |

## 3D 可视化操作

所有可视化程序共享：
- **鼠标左键拖拽** — 旋转视角
- **滚轮** — 缩放
- **ESC** — 退出
- **T** — 切换跟踪火箭/自由视角（仅 rocket-viz）

## 场景文件

场景描述文件 (`.scene`) 位于 `scenes/` 目录：
- `solar_system.scene` — 太阳 + 水金地火
- `three_body.scene` — 恒星 + 2 行星（层次三体）
- `figure8.scene` — Chenciner-Montgomery 图-8 三体稳定轨道

## 场景文件格式

```ini
[scene]
name = My Scene
dt = 1000.0
integrator = symplectic4    # 或 leapfrog
duration = 3.15576e9
adaptive = true
softening = 1e6

[body.Star]
mass = 1.989e30; radius = 6.96e8
pos.x = 0; pos.y = 0; pos.z = 0
vel.x = 0; vel.y = 0; vel.z = 0

[body.Planet]
mass = 5.972e24; radius = 6.371e6
pos.x = 1.496e11; pos.y = 0; pos.z = 0
vel.x = 0; vel.y = 29780; vel.z = 0
```

## 代码风格与规范
- Rust 2021 edition, 4空格缩进。
- 物理计算统一 `f64` 与 `Vec3`。
- 辛积分器在 `physics.rs` 的 `GravitationalSystem` 中实现。
- 场景配置解析在 `scene.rs` 的 `SceneConfig` / `SceneRuntime` 中。
- 3D 渲染原语在 `demo/src/render.rs` 中，依赖 macroquad。

## 测试规范
```bash
cargo test -p deepspace --lib    # 全部测试（179）
cargo test -p deepspace --lib physics  # 物理引擎测试（33）
cargo test -p deepspace --lib scene    # 场景系统测试（13）
```

## 提交规范
推荐 `type(scope): summary`：`feat:`、`fix:`、`refactor:`、`docs:`。
涉及物理参数/轨道要素变更时，保持测试覆盖。

## 配置与仓库卫生
- 不提交构建产物：`target/`、`*.csv`。
- 场景文件放 `scenes/`，不被编译。
- 此 `AGENTS.md` 为 AI 辅助开发指南，与 `deepspace/AGENTS.md` 互补。
