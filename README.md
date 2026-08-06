# DeepSpace 🚀

**Rust 太空物理模拟引擎** — 统一模拟世界（N 体引力、6DOF 航天器、弹道导弹、
拦截制导、雷达、事件流收敛进同一实体表），通过稳定 **C ABI** 接入
Unreal / Unity / 自研引擎。

| Crate | 角色 | 外部依赖 |
|-------|------|----------|
| `deepspace` | 物理引擎 + 统一世界 + C ABI（纯 std） | 无（serde 可选） |
| `deepspace-demo` | 演示应用（macroquad 3D + CLI） | `macroquad 0.4` |

---

## 快速开始

```bash
cargo build                      # 构建引擎 + demo
cargo test --workspace           # 全量测试（228）
```

**30 秒接入你的引擎（C）**：

```c
#include "deepspace.h"           // 由 cbindgen 自动生成，构建后位于 include/

DSWorld *w = ds_world_create();               // 1. 创建统一世界
DSBallisticConfig tgt = {0};
snprintf(tgt.name, sizeof tgt.name, "TGT-1");
tgt.position = (DSVec3){0.0, 6.5e6, 0.0};     // 500 km 高空靶弹
tgt.velocity = (DSVec3){1500.0, 500.0, 0.0};
ds_world_add_ballistic(w, &tgt);              // 2. 注册实体

for (int f = 0; f < 2000; ++f) {
    ds_world_step(w);                         // 3. 每帧推进（N 体+导弹+拦截+探测+命中）
    // ds_world_entity_count / ds_world_entity_at → 渲染实体
    // ds_world_poll_events                    → 事件流（发射/命中/结局）
}
ds_world_destroy(w);
```

---

## 接入你的引擎（Setup）

完整接入文档（架构图、踩坑表、绑定源码）见 **[`docs/ffi-integration.md`](docs/ffi-integration.md)**。
头文件契约 `include/deepspace.h` 由 **cbindgen 构建时自动生成**，与实现永不漂移。

### 1. 构建引擎库

```bash
cargo build -p deepspace
# 产物: target/debug/libdeepspace.so (.dylib / .dll) + include/deepspace.h
```

### 2. C / C++（自研引擎）

```bash
cc -I include my_engine.c -L target/debug -ldeepspace -o my_engine
LD_LIBRARY_PATH=target/debug ./my_engine
```

完整流程见文档第 4 节：`ds_world_create → add_ballistic → step 循环 → entity_at/poll_events → destroy`。

### 3. Unity（C# P/Invoke）

1. `libdeepspace.so` → `Assets/Plugins/Android/arm64-v8a/`（或 `.dylib` → `Assets/Plugins/`）
2. 用文档第 5.1 节的 `DeepSpace.cs` 绑定类（`DllImport("deepspace")`）
3. `Update()` 里每帧 `ds_world_step(w)` + 遍历 `entity_at` 生成 GameObject

### 4. Unreal Engine（C++）

1. `include/deepspace.h` + 库 → `Source/ThirdParty/DeepSpace/`
2. 用文档第 6.1 节的 `DeepSpace.Build.cs` 注册模块
3. ActorComponent 里 `BeginPlay` 建世界、`Tick` 驱动、`EndPlay` 销毁
4. ⚠️ UE 坐标是 **cm**，渲染时位置除以 10000

### 5. Android（arm64 .so）

```bash
cargo ndk -t arm64-v8a -o android-libs build -p deepspace --release
# 产物: android-libs/arm64-v8a/libdeepspace.so → 替换进 APK + apksigner 重签
```

### 6. 新增 API

在 `deepspace/src/ffi.rs` 写 `#[no_mangle] pub extern "C" fn` → 重新 `cargo build`
→ 头文件自动更新。加一个 `#[cfg(test)]` FFI 测试（参考 `ffi_interceptor_closed_loop`）。

---

## 功能特性

| 领域 | 能力 |
|------|------|
| **统一世界** | `World` 收敛 N 体 / 6DOF 航天器 / 弹道目标 / 拦截导弹 / 雷达 / 事件流到同一实体表，拦截闭环开箱即用 |
| **C ABI** | `ds_world_*` 稳定接口（不透明句柄 + POD 值类型 + 错误码），头文件自动生成 |
| **N 体引力** | 辛积分器（Leapfrog 2 阶 / Yoshida 4 阶）、自适应步长、SoI 层次查找 |
| **轨道力学** | Kepler 要素转换、圆/逃逸速度、轨道预测（解析 / RK4） |
| **6DOF 航天器** | 满自由度刚体、偏轴推力扭矩、四元数姿态、SAS PD 控制 |
| **大气/再入** | US Std 1976 大气、Sutton-Graves 加热、损伤传播 |
| **任务系统** | 数据驱动阶段机、可插拔制导（重力转弯 / PEG）、双向推进剂、级分离 |
| **参考系** | `FrameGraph` 层次化参考系、旋转系 Coriolis/离心力、LCA 跨帧距离 |
| **3D 可视化** | NASA Eyes 风格：3D 多行星轨道（轨道平面法线 = pos×vel）、渐变轨迹、发光天体、跟随相机 |

---

## 运行演示

```bash
cargo run -p deepspace-demo --bin rocket-sim                # 3D 火箭任务（Artemis II）
cargo run -p deepspace-demo --bin rocket-sim -- --headless  # 控制台模式
cargo run -p deepspace-demo --bin nbody-sim                 # 3D N 体沙盘（NASA Eyes 风格）
cargo run -p deepspace-demo --bin nbody-sim -- --headless --scene scenes/figure8.scene
cargo run -p deepspace-demo --bin interop-sim -- --headless # 拦截闭环演示
```

### 可视化操作（NASA Eyes 镜头）

| 操作 | 效果 |
|------|------|
| 左键拖拽 | 环绕旋转 |
| 右键拖拽 | 平移视角（pan） |
| 滚轮 | 面向目标平滑缩放 |
| 点击天体 | 相机平滑跟随该天体 |
| `Home` | 回到总览视角（取消跟随） |
| `ESC` | 退出 |

---

## 文档

- **[`docs/ffi-integration.md`](docs/ffi-integration.md)** — 引擎接入完整指南（C/C++ / Unity / UE / Android）
- `docs/artemis2-simulation-fidelity.md` — Artemis II 仿真保真度分析
- `DESIGN.md` — 架构设计

## 测试

```bash
cargo test --workspace             # 全量（228 passed）
cargo clippy -p deepspace          # 引擎零警告
cargo fmt -p deepspace --check
```

## 项目结构

```
DeepSpace/
├── deepspace/            # ← 引擎核心（纯 std，零外部依赖）
│   ├── src/ffi.rs        #   C ABI 接入层（ds_world_*）
│   ├── build.rs          #   cbindgen 自动生成头文件
│   └── cbindgen.toml
├── demo/                 # 演示应用（macroquad 渲染 + CLI）
├── include/deepspace.h   # 生成的 C ABI 契约（勿手改）
├── docs/                 # 接入文档
├── missions/             # 任务配置 (.conf)
└── scenes/               # N 体场景 (.scene)
```

## 许可证

MIT
