# DeepSpace 🚀

**Rust 航天模拟引擎与宇宙沙盘** — 零外部依赖的物理库 + 基于 macroquad 的 3D 可视化。

| Crate | 路径 | 角色 | 外部依赖 |
|-------|------|------|----------|
| `deepspace` | `deepspace/` | 物理引擎 + 任务系统（纯 std） | 无（serde 可选） |
| `demo` | `demo/` | 应用层（macroquad 渲染 + CLI） | `macroquad 0.4` |

---

## 功能特性

### 物理引擎 (`deepspace`)
- **N 体引力系统** — 辛积分器（Leapfrog 2 阶 / Yoshida 4 阶），支持自适应步长
- **轨道力学** — Kepler 要素转换（半长轴、偏心率、倾角、RAAN、近地点幅角），圆/逃逸速度计算
- **6DOF 航天器** — `SpacecraftBody` 满自由度刚体，偏轴推力 → 扭矩，四元数姿态积分
- **引力影响球 (SoI)** — `SoiTree` 层次化查找 O(log n)，支持 SoI 过渡事件
- **轨道预测** — `project_orbit()` Kepler 解析 / RK4 数值两种模式
- **SAS 姿态控制** — PD 控制器，支持 Prograde / Retrograde / Stabilize / Normal 模式
- **大气模型** — US Standard 1976（7 层 + 外大气层扩展），马赫数、动压计算
- **再入加热** — Sutton-Graves 驻点加热模型，TPS 烧蚀
- **损伤传播** — 气动载荷驱动的结构损伤增长，飞行器解体判定
- **旋转参考系** — Coriolis / 离心力 / Euler 加速度，地面速度
- **坐标参考系图** — `FrameGraph` 层次化参考系，自动单位选择，LCA 跨帧距离
- **统一模拟世界** — `World` 将 N 体 / 6DOF 航天器 / 弹道目标 / 拦截导弹 / 雷达 / 事件流收敛进同一实体表，拦截闭环开箱即用
- **C ABI 接入层** — `ffi.rs` 导出稳定 C 接口（`ds_world_*`），可接入 **Unreal / Unity / 自研引擎**，见 `docs/ffi-integration.md` 与 `include/deepspace.h`

### 任务系统 (`simulation.rs`)
- **数据驱动阶段机** — INI 配置定义阶段转换条件 + 事件 + 命令
- **可插拔制导** — `GuidanceAlgorithm` trait，内置余弦重力转弯 + PEG 制导
- **双向推进剂** — 支持 time rewind（负 dt 消耗）
- **级分离** — 多级火箭自动分离 + 发动机激活
- **任务遥测** — 全状态记录，CSV 导出
- **损伤/再入配置** — config-driven TPS 阈值、结构失效、加热参数

### 可视化 (`demo`)
- **3D / 控制台双模式** — 无参数 → 3D 窗口，`--headless` → 控制台
- **轨道相机** — 鼠标拖拽旋转，滚轮缩放，T 键跟踪火箭
- **时间倍率** — 0.001× 到 1000× 正反向（左右方向键）
- **HUD 面板** — 遥测数据、阶段进度、任务里程碑
- **2D 投影渲染** — 避免 f32 精度问题，支持天文尺度

### N 体宇宙沙盘 (`nbody-sim`)
- **场景文件驱动** — INI 格式定义天体、积分器参数
- **热切换** — 运行时通过文件更换场景（保留时钟）
- **CSV 导出** — 位置/速度/能量数据
- **内置场景** — 太阳系（5 体）、图-8 三体、层次三体

---

## 快速开始

```bash
# 构建
cargo build

# 全量测试（~183 个）
cargo test -p deepspace --lib

# 按模块测试
cargo test -p deepspace --lib physics   # 物理引擎
cargo test -p deepspace --lib scene     # 场景系统
cargo test -p deepspace --lib vessel    # 飞船系统
cargo test -p deepspace --lib guidance  # 制导系统
```

### 火箭任务模拟

```bash
# 3D 可视化（默认 Artemis II）
cargo run --bin rocket-sim

# 控制台模式
cargo run --bin rocket-sim -- --headless

# 自定义任务 + CSV 遥测
cargo run --bin rocket-sim -- --headless --mission missions/columbia.conf --csv out.csv

# 指定时间步长和最大仿真时长
cargo run --bin rocket-sim -- --headless --dt 0.05 --duration 5000
```

### N 体宇宙沙盘

```bash
# 3D 可视化（默认太阳系）
cargo run --bin nbody-sim

# 指定场景 + 控制台
cargo run --bin nbody-sim -- --headless --scene scenes/figure8.scene --csv output.csv

# 运行时热切换场景
echo "scenes/three_body.scene" > /tmp/switch
cargo run --bin nbody-sim -- --headless --scene scenes/solar_system.scene --switch-file /tmp/switch
```

### Lint / Docs

```bash
cargo clippy -p deepspace --lib
cargo fmt -p deepspace --check
cargo doc -p deepspace --no-deps --document-private-items
```

---

## 项目结构

```
DeepSpace/
├── Cargo.toml                    # workspace 根
├── README.md
├── DESIGN.md
├── AGENTS.md                     # AI 辅助开发指南
├── deepspace/                    # ← 物理库（零外部依赖）
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs                # Vec3, 物理常量, 模块导出
│       ├── core.rs               # Quaternion, Mat3x3
│       ├── physics.rs            # PhysicsBody, Integrators, GravBody, GravitationalSystem,
│       │                         # OrbitalMechanics, OrbitalElements, Aerodynamics,
│       │                         # RotatingFrame, ArtificialGravity
│       ├── environment.rs        # Planet, Atmosphere (US Std 1976),
│       │                         # DamageSystem, ThermalSimulation
│       ├── simulation.rs         # MissionConfig, MissionControl, MissionScript,
│       │                         # PhaseTransition, EventTriggerSystem, Command
│       ├── guidance.rs           # GuidanceAlgorithm trait, FlightComputer,
│       │                         # CosineGuidance, PEGGuidance
│       ├── vessel.rs             # Part/PartKind, Vessel, StagingSystem,
│       │                         # Rcs, DockingPort, EnduranceStation, PartLibrary
│       ├── scene.rs              # SceneConfig, SceneRuntime
│       ├── space_physics.rs      # SpacecraftBody, SoiTree, SoiTransition,
│       │                         # WarpMode, FlightAssist/SAS, SpacePhysicsWorld
│       └── frame_graph.rs        # FrameGraph, FrameId, EntityPosition,
│                                 # LengthUnit, Scalar
├── demo/                         # ← 应用层
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs
│       ├── app.rs                # CliArgs, SimulationApp, build_sls_stack
│       ├── render.rs             # OrbitalCamera, 2D 绘制原语
│       └── bin/
│           ├── rocket-sim.rs     # 火箭任务模拟器入口
│           └── nbody-sim.rs      # N 体沙盘入口
├── missions/                     # 任务配置文件 (.conf)
│   ├── artemis2.conf             # Artemis II 月球飞越（基准任务）
│   └── columbia.conf             # 哥伦比亚号 STS-107 泡沫撞击情景
└── scenes/                       # N 体场景文件 (.scene)
    ├── solar_system.scene        # 太阳 + 水金地火
    ├── three_body.scene          # 恒星 + 2 行星
    └── figure8.scene             # 图-8 三体稳定轨道
```

---

## 配置格式

### 任务配置 (`.conf`) — 自定义 INI 格式

```ini
[mission]
name = Artemis II
targetAp_km = 185.0
targetPe_km = 180.0

[rs25]
thrustSeaLevel_N = 1860000
thrustVacuum_N = 2279000
engineCount = 4

[guidance]
algorithm = peg
pitchStartAlt_m = 2000.0
pitchEndAlt_m = 220000.0

[damage]
initial_tps = 0.0
initial_structural = 0.0

[thermal]
sutton_graves_k = 0.000183
nose_radius_m = 1.0

[structural]
onset_dynamic_pressure = 1000.0
rate_coefficient = 0.02

# 阶段转换 — 数据驱动状态机
[transition.0]
from = Launch
to = Ascent
require_all = true
condition_0_type = AltitudeAbove
condition_0_value = 100.0

# 事件系统
[event.0]
time = 81.0
name = Foam Impact
trigger_0_type = TimeElapsed
trigger_0_value = 81.0
command_0_type = ApplyDamage
command_0_value = 0.35
command_0_parameter = Foam strike on left wing RCC panel
```

### 场景配置 (`.scene`)

```ini
[scene]
name = Solar System
dt = 3600.0
integrator = symplectic4
duration = 3.15576e9
adaptive = true
softening = 1e8

[body.Sun]
mass = 1.989e30; radius = 6.96e8
pos.x = 0; pos.y = 0; pos.z = 0
vel.x = 0; vel.y = 0; vel.z = 0

[body.Earth]
mass = 5.972e24; radius = 6.371e6
pos.x = 1.496e11; pos.y = 0; pos.z = 0
vel.x = 0; vel.y = 29780; vel.z = 0
```

---

## 3D 可视化操作

| 按键 | 功能 |
|------|------|
| 鼠标左键拖拽 | 旋转视角 |
| 滚轮 | 缩放 |
| ← / → | 减小 / 增加时间倍率 |
| T | 切换跟踪火箭 / 自由视角（rocket-sim） |
| ESC | 退出 |

---

## 测试

```bash
# 模块测试
cargo test -p deepspace --lib physics        # 物理引擎（~33）
cargo test -p deepspace --lib scene          # 场景系统（~13）
cargo test -p deepspace --lib vessel         # 飞船系统
cargo test -p deepspace --lib guidance       # 制导系统

# 单个测试
cargo test -p deepspace --lib physics::tests::test_grav_long_term_conservation -- --exact
```

---

## 提交规范

```
type(scope): summary
```

- `feat:` — 新功能
- `fix:` — 修复
- `refactor:` — 重构（不改变行为）
- `docs:` — 文档
- `chore:` — 杂项（构建、CI、清理）

涉及物理参数 / 轨道要素变更时，保持测试覆盖。

---

## 许可证

MIT