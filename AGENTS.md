# AGENTS.md

`DeepSpace` — Rust 航天模拟引擎与宇宙沙盘。

## 工作区结构

| Crate | 路径 | 角色 | 外部依赖 |
|-------|------|------|----------|
| `deepspace` | `deepspace/` | 物理库（**零外部依赖**，纯 std） | 无（serde optional） |
| `demo` | `demo/` | 应用层（macroquad 渲染 + CLI） | macroquad |

## 构建与测试

```bash
# 构建
cargo build

# 全量测试（物理库，~183 tests）
cargo test -p deepspace --lib

# 按模块过滤
cargo test -p deepspace --lib physics    # 物理引擎（~33）
cargo test -p deepspace --lib scene      # 场景系统（~13）
cargo test -p deepspace --lib vessel     # 飞船系统
cargo test -p deepspace --lib guidance   # 制导系统

# 单个测试
cargo test -p deepspace --lib physics::tests::test_name_here -- --exact

# Lint / format（无本地配置，用默认）
cargo clippy -p deepspace --lib
cargo fmt -p deepspace --check

# 生成 docs
cargo doc -p deepspace --no-deps --document-private-items
```

## 运行二进制

```bash
cargo run --bin rocket-sim                    # 3D 可视化
cargo run --bin rocket-sim -- --headless       # 控制台仿真
cargo run --bin rocket-sim -- --headless --csv out.csv

cargo run --bin nbody-sim                     # 3D（默认 solar_system）
cargo run --bin nbody-sim -- --scene scenes/figure8.scene
cargo run --bin nbody-sim -- --headless --scene scenes/three_body.scene --csv output.csv

# 场景热切换（headless）
echo "/path/to/new_scene.scene" > /tmp/switch
cargo run --bin nbody-sim -- --headless --scene scenes/solar_system.scene --switch-file /tmp/switch
```

## 两条独立仿真管线

1. **N 体宇宙沙盘** (`nbody-sim`)：`SceneConfig` → `SceneRuntime`(`GravitationalSystem`) → 辛积分器步进 → 3D/CSV
2. **火箭任务** (`rocket-sim`)：`MissionConfig` → `SimulationApp`(`Vessel`+`Planet`+`MissionControl`+`FlightComputer`) → 物理仿真 → 3D/CSV

## 物理库模块图

```
lib.rs (Vec3, G, G0, 常量)
 ├── core.rs (Quaternion, Mat3x3)
 ├── environment.rs (Planet, Atmosphere ISA-1976, Thermal)
 ├── physics.rs (PhysicsBody, Integrators, GravBody/GravitationalSystem, OrbitalMechanics)
 ├── space_physics.rs (SpacecraftBody, SoiTree, SpacePhysicsWorld, FlightAssist/SAS)
 ├── guidance.rs (GuidanceAlgorithm trait, FlightComputer, CosineGuidance)
 ├── vessel.rs (Part/PartKind, Vessel, 推进剂, 级分离)
 ├── simulation.rs (MissionPhase 状态机, MissionControl, MissionScript, 事件系统)
 ├── scene.rs (SceneConfig 解析, SceneRuntime 运行时, 热切换)
 └── frame_graph.rs (参考系图, LengthUnit, 跨帧距离)
```

## 关键设计规则

- **`deepspace` 零依赖**：所有数学手写（Vec3/Quaternion/Mat3x3），不引入 nalgebra/glam。**修改时不要添加外部依赖。**
- **辛积分器是核心**：`step_leapfrog()`（2 阶）和 `step_symplectic4()`（Yoshida 4 阶）保证长期能量守恒。`step_adaptive()` 根据最近天体距离自动调步长。
- **制导算法可插拔**：实现 `GuidanceAlgorithm` trait 即可，通过 `GuidanceConfig.algorithm` 字符串选择。
- **火箭部件用枚举组合**：`Part { kind: PartKind }` 替代继承。
- **demo 层双模式**：每个 bin 无参数启动 3D 窗口（macroquad async main），`--headless` 进入纯控制台仿真。
- **坐标系隔离**：物理计算统一 `f64` + `deepspace::Vec3`，demo 层 3D 坐标用 `macroquad::math::Vec3`（f32），通过 `to_mvec3()` 转换。

## 3D 可视化操作

- **鼠标左键拖拽** — 旋转视角
- **鼠标右键拖拽** — 平移视角（pan）
- **滚轮** — 缩放
- **点击天体** — 相机平滑跟随（nbody-sim / interop-sim）
- **Home** — 回到总览视角（取消跟随）
- **ESC** — 退出
- **T** — 切换跟踪火箭/自由视角（仅 rocket-sim）

## 编码约定

- Rust 2021 edition，4 空格缩进
- `type(scope): summary` 提交格式 — `feat:`、`fix:`、`refactor:`、`docs:`
- 不提交：`target/`、`*.csv`
- 场景文件（`.scene`）放 `scenes/`，任务配置（`.conf`）放 `missions/`，均不参与编译
- scene.rs 通过 `include_str!` 内嵌了三个内置场景用于测试
