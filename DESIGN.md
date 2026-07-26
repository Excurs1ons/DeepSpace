# DeepSpace 设计文档

> Rust 航天模拟引擎与宇宙沙盘 — 架构决策与设计详解

---

## 目录

1. [总体架构](#1-总体架构)
2. [双管线系统](#2-双管线系统)
3. [物理引擎核心](#3-物理引擎核心)
4. [任务系统与数据驱动状态机](#4-任务系统与数据驱动状态机)
5. [航天器与飞船系统](#5-航天器与飞船系统)
6. [环境与损伤模型](#6-环境与损伤模型)
7. [制导系统](#7-制导系统)
8. [坐标参考系图](#8-坐标参考系图)
9. [太空物理层](#9-太空物理层)
10. [可视化与渲染](#10-可视化与渲染)
11. [设计决策记录](#11-设计决策记录)

---

## 1. 总体架构

```
                    ┌─────────────────────────────────┐
                    │           demo crate             │
                    │   (macroquad 可视化 + CLI)        │
                    │                                  │
                    │  ┌─────────┐  ┌──────────────┐   │
                    │  │rocket-sim│  │  nbody-sim   │   │
                    │  │ 火箭任务  │  │ N 体宇宙沙盘  │   │
                    │  └────┬─────┘  └──────┬───────┘   │
                    │       │               │           │
                    │  ┌────▼────┐    ┌─────▼──────┐    │
                    │  │ app.rs  │    │ render.rs  │    │
                    │  │仿真循环  │    │ 轨道相机+绘制│    │
                    │  └─────────┘    └────────────┘    │
                    └──────────┬───────────────────────┘
                               │ 依赖
                    ┌──────────▼───────────────────────┐
                    │         deepspace crate           │
                    │    (零外部依赖，纯 std)             │
                    │                                   │
                    │  ┌─────────────────────────────┐   │
                    │  │   lib.rs — Vec3 + 物理常量    │   │
                    │  └─────────────┬───────────────┘   │
                    │                │                    │
                    │  ┌─────────────▼────────────────┐  │
                    │  │  core.rs (Quaternion, Mat3x3) │  │
                    │  └─────────────┬─────────────────┘  │
                    │                │                     │
                    │  ┌─────────────▼────────────────┐  │
                    │  │  physics.rs                  │  │
                    │  │  ├─ PhysicsBody (刚体)        │  │
                    │  │  ├─ Integrators (RK4, RKF45)  │  │
                    │  │  ├─ GravBody +                │  │
                    │  │  │  GravitationalSystem       │  │
                    │  │  ├─ OrbitalMechanics/          │  │
                    │  │  │  OrbitalElements           │  │
                    │  │  ├─ Aerodynamics              │  │
                    │  │  ├─ RotatingFrame             │  │
                    │  │  └─ ArtificialGravity         │  │
                    │  └──────┬──────┬─────────────────┘  │
                    │         │      │                     │
                    │  ┌──────▼──┐ ┌─▼───────────────┐   │
                    │  │space_   │ │environment.rs    │   │
                    │  │physics  │ │├─ Planet          │   │
                    │  │.rs      │ │├─ Atmosphere      │   │
                    │  │├─ Space │ ││  (US Std 1976)  │   │
                    │  ││  craft │ │├─ DamageSystem    │   │
                    │  ││  Body  │ │└─ ThermalSimul.  │   │
                    │  │├─ Soi   │ └────────┬─────────┘   │
                    │  ││  Tree  │          │              │
                    │  │├─ Warp  │    ┌─────▼─────────┐   │
                    │  ││  Mode  │    │ simulation.rs  │   │
                    │  │├─ Flight│    │├─ MissionPhase  │   │
                    │  ││  Assist│    │├─ MissionConfig │   │
                    │  │└─ Space │    │├─ MissionControl│   │
                    │  │  Physics│    │├─ PhaseTransit. │   │
                    │  │  World  │    │├─ EventTrigger  │   │
                    │  └─────────┘    │├─ Command       │   │
                    │                 │└─ Telemetry     │   │
                    │  ┌───────────┐  └────────┬────────┘  │
                    │  │ guidance   │          │            │
                    │  │ .rs        │    ┌─────▼─────────┐  │
                    │  │├─ Guidance │    │ vessel.rs      │  │
                    │  ││  Algor.   │    │├─ Part/PartKind │  │
                    │  ││  trait    │    │├─ Vessel       │  │
                    │  │├─ Flight   │    │├─ StagingSystem│  │
                    │  ││  Computer │    │├─ DockingPort  │  │
                    │  │├─ Cosine   │    │├─ EnduranceSta.│  │
                    │  ││  Guidance │    │└─ PartLibrary  │  │
                    │  │└─ PEGGuid. │    └────────────────┘  │
                    │  └───────────┘                          │
                    │                                         │
                    │  ┌─────────────┬──────────────┐        │
                    │  │ scene.rs    │frame_graph.rs │        │
                    │  │├─ SceneCfg │├─ FrameGraph   │        │
                    │  │└─ SceneRun │├─ LengthUnit   │        │
                    │  │  time     │└─ EntityPos    │        │
                    │  └─────────────└──────────────┘        │
                    └─────────────────────────────────────────┘
```

## 2. 双管线系统

项目有两条独立的仿真管线，共享 `deepspace` 物理库底层，但运行时互相不依赖。

### 2.1 N 体宇宙沙盘管线

```
.scene 文件 ──→ SceneConfig ──→ SceneRuntime ──→ GravitationalSystem
                        │                            │
                        │                         辛积分器步进
                        │                     (Leapfrog/Symplectic4)
                        │                            │
                        └── 热切换 ──────────→ 3D / CSV 输出
```

**关键组件：**
- `SceneConfig` — 解析 `.scene` 文件，构建天体列表与积分器参数
- `SceneRuntime` — 运行时，持有 `GravitationalSystem`，支持 `load_scene()` 热切换
- `GravitationalSystem` — N 体引力核心，提供三种积分模式

**用途：** 数亿年尺度的轨道演化、混沌三体模拟、引力辅助轨迹研究

### 2.2 火箭任务管线

```
.conf 文件 ──→ MissionConfig ──→ SimulationApp ──→ 逐步物理仿真
                        │              │
                        │         ┌────┴────────┐
                        │         │ Vessel       │
                        │         │ Planet       │
                        │         │ MissionCntrl │
                        │         │ FlightComp.  │
                        │         │ ThermalSim.  │
                        │         └─────────────┘
                        │              │
                        └── → 3D / CSV 输出
```

**关键组件：**
- `MissionConfig` — 解析 `.conf` 文件，定义发动机、燃料箱、制导、阶段转换等
- `SimulationApp` — 持有所有运行时对象，主循环每步调用 `step(dt)`
- `MissionControl` — 阶段状态机 + 事件触发器 + 任务结束判定
- `FlightComputer` — 封装制导算法，输出推力方向与油门

**用途：** 火箭发射入轨、月球飞越、再入与损伤情景模拟

---

## 3. 物理引擎核心

### 3.1 数学基础

所有数学实现手写，零外部依赖：

- **`Vec3`** — `lib.rs` 定义，f64 双精度，完整运算符重载
- **`Quaternion`** — `core.rs`，支持轴角 / 欧拉角构造、旋转、共轭
- **`Mat3x3`** — `core.rs`，列主序存储，支持 Rodrigues 旋转、逆矩阵

### 3.2 N 体引力系统 (`GravitationalSystem`)

```
GravBody:
    name: String
    mass: f64
    radius: f64
    position: Vec3
    velocity: Vec3

GravitationalSystem:
    bodies: Vec<GravBody>
    softening: f64       # 软化参数，避免奇点
```

#### 积分器方案

| 方法 | 阶数 | 特点 | 适用场景 |
|------|------|------|----------|
| `step_leapfrog(dt)` | 2 阶 | 速度 Verlet 形式，辛，时间可逆 | 快速预览、大时间步 |
| `step_symplectic4(dt)` | 4 阶 | Yoshida 1990 系数，辛 | 高精度长期积分 |
| `step_adaptive()` | 可变 | 根据 `min_distance()` 自动调整 dt | 近天体遭遇需精细步长 |

**为什么用辛积分器？** 在数亿年模拟中，非辛方法（如 RK4）会系统性漂移能量。辛积分器保持近似的 Hamilton 量守恒，轨道半长轴不会系统性衰减。

### 3.3 通用积分器

```rust
Integrators::rk4(f, t, y, dt) -> StateVector          // 经典 RK4
Integrators::adaptive_step(f, t, y, dt, tol, t_end)    // RKF45 自适应
Integrators::propagate_two_body(...)                    // 两体传播
```

### 3.4 轨道力学

```rust
orbital_elements(pos, vel, mu) -> OrbitalElements
    // 半长轴, 偏心率, 倾角, RAAN, 近地点幅角, 平近点角

OrbitalMechanics:
    circular_orbit_velocity(altitude, mu) -> f64
    delta_v_to_raise_apoapsis(...) -> f64
    time_to_apoapsis(...) -> f64
    is_escape_orbit(...) -> bool
    get_escape_velocity(...) -> f64
```

### 3.5 气动模型

```rust
Aerodynamics:
    apply(body, planet, altitude, damage_factor)    // 应用阻力
    dynamic_pressure(planet, altitude, speed) -> f64
    mach(planet, altitude, speed) -> f64
```

阻力系数为马赫数分段函数：
| 马赫范围 | Cd |
|----------|----|
| < 1.0 (亚音速) | 0.25 |
| 1.0–1.2 (跨音速峰) | 0.80 |
| 1.2–5.0 (超音速) | 0.45 |
| ≥ 5.0 (高超音速) | 0.30 |

### 3.6 旋转参考系

```rust
RotatingFrame:
    coriolis_accel(omega, vel) -> Vec3      // -2 ω × v
    centrifugal_accel(omega, pos) -> Vec3    // -ω × (ω × r)
    euler_accel(alpha, pos) -> Vec3         // -α × r
    ground_speed(body_vel, omega, pos) -> f64
```

---

## 4. 任务系统与数据驱动状态机

### 4.1 阶段转换系统

任务阶段转换现在**完全由配置驱动**。每个阶段转换定义在 `.conf` 文件中：

```ini
[transition.0]
from = Launch
to = Ascent
require_all = true                    # AND 语义（false = OR）
condition_0_type = AltitudeAbove
condition_0_value = 100.0
```

支持的触发条件类型：

| 类型 | 含义 |
|------|------|
| `TimeElapsed` | 任务时间超过 value（秒） |
| `AltitudeAbove` / `Below` | 海拔高于/低于 value（米） |
| `VelocityAbove` / `Below` | 速度高于/低于 value（m/s） |
| `VelocityRatioAbove` / `Below` | 速度与当前高度环绕速度之比 |
| `TimeSincePhaseAbove` | 在当前阶段停留时间超 value |
| `DynamicPressureAbove` | 动压超 value（Pa） |
| `MaxqPassed` | 最大动压已过（下降沿） |
| `PropellantDepleted` | 指定 stage 推进剂耗尽 |
| `ApoapsisAbove` / `Below` | 远地点高于/低于 value（m） |
| `PeriapsisAbove` / `Below` | 近地点高于/低于 value（m） |
| `OrbitCircularized` | 轨道已圆化 |
| `DamageExceeded` | 损伤超 value |
| `MachAbove` / `Below` | 马赫数超/低于 value |
| `EngineCutoff` | 发动机已关机 |
| `StageActivated` | 指定级已激活 |
| `FlagIsTrue` / `FlagIsFalse` | 运行时布尔标志 |

### 4.2 事件与命令系统

```ini
[event.0]
time = 81.0
name = Foam Impact
trigger_0_type = TimeElapsed
trigger_0_value = 81.0
command_0_type = ApplyDamage
command_0_value = 0.35
command_0_parameter = Foam strike on left wing RCC panel
```

命令类型：
- `StageSeparation{stage}` — 级分离
- `SetThrottle{stage,value}` — 设置油门
- `SetOrientation{pitch,yaw}` — 设置姿态
- `EnableRcs` — 启用 RCS
- `LogMessage{text}` — 记录日志
- `CircularizationBurn` — 圆化轨道点火
- `AbortMission` — 终止任务
- `ApplyDamage{amount,message}` — 施加损伤
- `Wait{duration}` — 等待

### 4.3 任务阶段划分

```rust
MissionPhase:
    PreLaunch → Launch → Ascent → MaxQ → Staging → Coast →
    Circularization → Orbit → Tei → Translunar →
    MissionEvents → Reentry → Success / Failure / Abort
```

---

## 5. 航天器与飞船系统

### 5.1 两组物理层

项目存在两层物理抽象：

| 层 | 类型 | 自由度 | 用途 |
|----|------|--------|------|
| 基础物理 | `PhysicsBody` | 单轴旋转 + 位置 | 火箭任务管线 |
| 太空物理 | `SpacecraftBody` | 6DOF (满张量惯性) | 太空物理世界 |

`PhysicsBody` 用于 `vessel.rs` 中的火箭，`SpacecraftBody` 用于 `space_physics.rs` 中的通用航天器。

### 5.2 部件系统 (枚举组合)

Rust 枚举替代继承层次：

```rust
enum PartKind {
    Engine(EnginePart),       // 引擎: 推力、比冲、混合比
    FuelTank(FuelTankPart),   // 油箱: 容量、当前燃料、推进剂类型
    Decoupler(DecouplerPart), // 分离器: marker struct
}
```

### 5.3 级分离与推进剂

- 每级多个油箱、多台发动机
- `consume_fuel(amount)` 支持**双向消耗**（正 = 消耗，负 = 加注），实现 time rewind
- 级分离自动解耦 + 激活下一级发动机
- 海/空比冲线性插值：`Isp(p) = lerp(Isp_sl, Isp_vac, p_ratio)`

### 5.4 部件库

`PartLibrary` 提供真实火箭引擎/油箱的工厂函数：

```rust
PartLibrary::create_rs25()      // ~2280 kN vac, 452s Isp
PartLibrary::create_sls_srb()   // 16.4 MN vac, 269s Isp
PartLibrary::create_rl10b2()    // 110 kN vac, 462s Isp
PartLibrary::create_merlin1d()  // 845 kN sl, 311s Isp
```

---

## 6. 环境与损伤模型

### 6.1 大气模型

US Standard 1976 的 Rust 实现：

- **7 层梯度模型** 从海平面到 84.852 km
- **外大气层扩展** 用于 84.852 km 以上（指数衰减）
- 输出：温度 (K)、压力 (Pa)、密度 (kg/m³)、声速 (m/s)

### 6.2 再入加热

Sutton-Graves 驻点加热模型：

```rust
q = k * sqrt(ρ / R_nose) * v³
```

其中：
- `k` — Sutton-Graves 常数（地球 = 1.83e-4）
- `ρ` — 大气密度
- `R_nose` — 鼻锥半径
- `v` — 飞行器速度

附加：对流加热 `q_conv = h * (T_r - T_w)`，损伤放大（损伤 > 0 时乘以 `damage_heat_multiplier`）。

### 6.3 TPS 烧蚀

当热流超过 `tps_ablation_threshold` 时，TPS 以线性速率烧蚀：

```rust
ablation_rate = heat_flux_above_threshold * ablation_rate_coefficient
```

### 6.4 结构损伤传播

气动载荷驱动的损伤增长模型：

```rust
rate = existing_damage * (q / q_ref) * rate_coefficient
```

- `onset_dynamic_pressure` — 损伤增长启动阈值
- 仅当 `q > onset` 且 `existing_damage > 0` 时增长
- 飞行器解体判定：结构损伤 ≥ `structural_failure_threshold`，且在高/时间范围内

### 6.5 损伤系统

```rust
DamageSystem::update(dt, damage, damage_factor)  // 自修复
DamageSystem::survival_probability(damage) -> f64 // 生存概率
```

损伤类型：`Tps | Structural | Propulsion | LifeSupport`

---

## 7. 制导系统

### 7.1 可插拔架构

```rust
trait GuidanceAlgorithm: Debug + Send {
    fn compute(&mut self, state: GuidanceState, config: &GuidanceConfig) -> SteeringCommand;
    fn reset(&mut self);
}
```

`FlightComputer` 通过策略字符串选择算法：

```rust
FlightComputer::from_config(&config)
// "cosine" → CosineGuidance
// "peg"    → PEGGuidance
```

### 7.2 余弦重力转弯

```rust
pitch(progress) = 90° × (1 - cos(progress × π/2))
progress = clamp((alt - pitch_start) / (pitch_end - pitch_start), 0, 1)
```

- 起始海拔以下：垂直上升
- 在 `pitch_start` 到 `pitch_end` 之间：余弦转弯
- 超过 `pitch_end`：保持最终角度

### 7.3 PEG 制导

两阶段制导：
- **Stage 0** — 同余弦重力转弯（火箭上升段）
- **Stage ≥ 1** — 沿速度矢量（Prograde），用于圆化轨道点火

---

## 8. 坐标参考系图

### 8.1 设计目标

在同一个系统中同时支持从地球表面（米）到本星系群（百万秒差距）的坐标，避免精度损失。

### 8.2 实现

```rust
FrameGraph:
    nodes: Vec<FrameNode>        // 帧节点树
    FrameId(u64)                 // 不透明 ID
    ROOT: FrameId(0)            // 宇宙根帧

LengthUnit: Mm | M | Km | Au | Ly | Kpc | Mpc
    best_for(meters) -> (value, unit)  // 自动选择合适单位
```

### 8.3 标准宇宙层次

`build_standard_universe()` 预定义参考系：

```
Root (Mpc)
  └── LocalGroup (Mpc)
       └── MilkyWay (Kpc)
            ├── Sol (AU)
            │    ├── Earth (km)
            │    ├── Mars (km)
            │    └── Luna (km)
            └── AlphaCentauri (AU)
                 └── ProximaB (AU)
```

### 8.4 跨帧距离计算

```rust
let dist = graph.distance_auto(entity_a, entity_b);
// 自动找到 LCA（最低公共祖先），
// 将两个位置转换到 LCA 帧，计算欧氏距离，
// 选择最合适的单位显示
```

---

## 9. 太空物理层

### 9.1 引力影响球 (SoI)

```rust
SoiTree:
    节点: body_idx, soi_radius, parent, children
    构建: 按质量排序，r_soi = r_body × (m_small / m_large)^0.4
    查找: O(log n) 递归树搜索
```

### 9.2 时间加速模式

```rust
WarpMode:
    RealTime                // 1:1
    Fast(factor)            // factor > 1 线性加速
    KeplerWarp(factor)      // 加速模式下用 Kepler 解析传播
```

- `thrust_allowed()` — `RealTime` 和 `Fast` 允许，`KeplerWarp` 不允许

### 9.3 SAS 姿态控制

```rust
SasMode: Disabled | Stabilize | Prograde | Retrograde | Normal | AntiNormal

FlightAssist:
    kp = 10, kd = 5, max_torque = 100 kN·m
    compute_torque(ship, vel, mu) -> Vec3  // PD 控制律
```

### 9.4 SpacePhysicsWorld

统一世界模拟入口，每步执行：

1. N 体步进（symplectic4）
2. 每个航天器：
   - Kepler warp（如启用）或
   - SoI 引力 + 推力器积分 + SAS + 刚体步进
   - SoI 过渡检测

---

## 10. 可视化与渲染

### 10.1 渲染架构

所有可视化程序使用 **2D 正交投影渲染**，而非真 3D：

```
OrbitalCamera (3D 轨道位置)
    ↓ world → screen 投影
2D 绘制原语 (macroquad shapes + text)
```

选择 2D 投影的原因：
- 避免 f32 精度问题（天文尺度坐标截断）
- 简化跨平台兼容性
- 轨道力学可视化在 2D 投影中更清晰

### 10.2 相机系统

```rust
OrbitalCamera:
    target: Vec3              // 目标点
    distance: f32             // 距离（带平滑）
    azimuth / elevation       // 球坐标角度
    sensitivity / zoom_sens   // 交互参数
```

- 鼠标拖拽 = 改变 azimuth / elevation
- 滚轮 = 改变 distance（指数缩放）
- `update()` 每帧调用，处理输入 + 平滑

### 10.3 绘制原语

```
draw_planet(pos, radius, color)       → 3D 球体
draw_gizmo(pos, orientation, scale)   → 姿态指示器
draw_path(points, color)              → 3D 轨迹线
draw_predicted_path(points, color)    → 虚线预测轨迹
draw_velocity_arrow(pos, vel, scale)  → 速度矢量
draw_attitude_indicator_2d(...)       → 2D HUD 姿态球
draw_phase_panel(state, x, y)         → 阶段进度条
draw_task_panel(state, x, y)          → 任务里程碑
draw_earth_2d(camera, radius, sw, sh) → 投影 2D 地球
draw_grid_2d(camera, ...)             → 投影 2D 网格
```

### 10.4 时间控制

rocket-sim 支持可变时间倍率：

- 左右方向键调整，范围 -1000× 到 1000×
- 每帧取 `get_frame_time() * time_warp` 作为步长
- 子步长上限 0.016s（约 60 FPS 帧间隔）

---

## 11. 设计决策记录

### ADR-1: 零外部依赖

**决策：** `deepspace` crate 不引入任何运行时依赖，serde 为可选。

**理由：** 航天模拟是长期项目，依赖过时可能导致无法编译。核心数学手写避免 nalgebra/glam 版本迁移风险。

### ADR-2: 枚举组合替代继承

**决策：** 使用 `PartKind` 枚举而非 trait object 或继承层次。

**理由：** 模式匹配穷尽性确保所有部件类型被处理，新的部件类型需要显式添加 match arm，避免"忘了处理新类型"的运行时错误。

```rust
// 而不是
trait Part { fn mass(&self) -> f64; }
struct EnginePart: Part { ... }
struct TankPart: Part { ... }

// 使用
enum PartKind {
    Engine(EnginePart),
    FuelTank(FuelTankPart),
    Decoupler(DecouplerPart),
}
```

### ADR-3: 辛积分器为 N 体核心

**决策：** N 体系统只使用辛积分器（Leapfrog / Yoshida4），不用 RK4。

**理由：** 非辛方法在长期积分中系统性漂移能量。DeepSpace 目标包括数亿年模拟（如太阳系稳定性），辛性质保证 Hamilton 量守恒。

### ADR-4: 数据驱动任务阶段

**决策：** 阶段转换条件完全由配置文件定义。

**理由：** 不同任务（Artemis II、Columbia 损伤情景）有不同的阶段逻辑。硬编码阶段机不可复用。配置驱动允许不修改代码就定义任意任务剖面。

### ADR-5: 2D 投影渲染

**决策：** 可视化使用 2D 正交投影而非真 3D 渲染管线。

**理由：** f32 坐标在 AU 尺度下精度不足。2D 投影 + 球面坐标系避免了深度缓冲精度问题，同时轨道轨迹在投影视图中更清晰。

### ADR-6: 双向推进剂

**决策：** `consume_fuel(amount)` 接受负数（即加注）。

**理由：** 支持 time rewind 功能。当用户回退时间时，推进剂应恢复，而非仅位置/速度回退。

### ADR-7: 物理量全 f64

**决策：** 物理引擎所有数量用 f64。

**理由：** f32 精度不足以表示天文尺度上的小差异。地球轨道半径 ~1.5e11 m，用 f32 只能到约 7 位有效数字（米级精度），f64 可到 15 位（亚毫米级）。

### ADR-8: 两组物理层

**决策：** 存在 `PhysicsBody`（简单）和 `SpacecraftBody`（6DOF）两层。

**理由：** 火箭任务管线不需要满自由度建模（发动机力矩相对姿态控制可忽略），但太空物理模拟需要偏轴推力、RCS 等。分层避免不必要的计算开销。

---

## 附录 A：关键文件引用

| 组件 | 文件 |
|------|------|
| Vec3, 物理常量 | `deepspace/src/lib.rs` |
| 四元数, 矩阵 | `deepspace/src/core.rs` |
| N 体引力, 积分器, 轨道力学 | `deepspace/src/physics.rs` |
| 行星, 大气, 加热, 损伤 | `deepspace/src/environment.rs` |
| 任务系统, 配置, 阶段机 | `deepspace/src/simulation.rs` |
| 制导律, 飞控 | `deepspace/src/guidance.rs` |
| 飞船部件, 级分离, 对接 | `deepspace/src/vessel.rs` |
| 场景配置与运行 | `deepspace/src/scene.rs` |
| 6DOF, SoI, 时间加速, SAS | `deepspace/src/space_physics.rs` |
| 坐标参考系图 | `deepspace/src/frame_graph.rs` |
| CLI, 仿真循环, 火箭构建 | `demo/src/app.rs` |
| 轨道相机, 绘制原语 | `demo/src/render.rs` |
| 火箭模拟入口 | `demo/src/bin/rocket-sim.rs` |
| N 体沙盘入口 | `demo/src/bin/nbody-sim.rs` |

## 附录 B：测试覆盖率

```bash
cargo test -p deepspace --lib                    # ~183 tests
cargo test -p deepspace --lib physics            # 物理引擎
cargo test -p deepspace --lib scene              # 场景系统
cargo test -p deepspace --lib vessel             # 飞船系统
cargo test -p deepspace --lib guidance           # 制导系统
```

涉及物理参数 / 轨道要素变更时保持测试覆盖。