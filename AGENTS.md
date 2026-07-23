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
