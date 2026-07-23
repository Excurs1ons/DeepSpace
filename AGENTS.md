# Repository Guidelines

## 项目定位
`DeepSpace` — Rust 航天模拟引擎与宇宙沙盘。核心引擎提供 N 体引力辛积分器（Leapfrog/Yoshida4）、场景化沙箱仿真、轨道力学等。
支持热切换场景文件，适合数亿年尺度的混沌三体模拟。

## 构建与运行

```bash
# 全量测试
cargo test --lib

# 火箭任务模拟
cargo run --bin rocket-sim -- --headless

# N体宇宙沙盘
cargo run --bin nbody-sim -- --scene scenes/three_body.scene --csv output.csv

# 场景热切换（运行时写入切换文件）
echo "/path/to/new_scene.scene" > /tmp/switch
cargo run --bin nbody-sim -- --scene scenes/solar_system.scene --switch-file /tmp/switch
```

## 二进制目标

| Target | 命令 | 用途 |
|--------|------|------|
| `rocket-sim` | `cargo run --bin rocket-sim` | 火箭任务模拟（SLS Block 1 / Artemis II） |
| `nbody-sim` | `cargo run --bin nbody-sim` | N体宇宙沙盘模拟器 |

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

## 测试规范
```bash
cargo test --lib    # 全部测试（153+）
cargo test --lib physics  # 物理引擎测试（33）
cargo test --lib scene    # 场景系统测试（13）
```

## 提交规范
推荐 `type(scope): summary`：`feat:`、`fix:`、`refactor:`、`docs:`。
涉及物理参数/轨道要素变更时，保持测试覆盖。

## 配置与仓库卫生
- 不提交构建产物：`target/`、`*.csv`。
- 场景文件放 `scenes/`，不被编译。
- 此 `AGENTS.md` 为 AI 辅助开发指南。
