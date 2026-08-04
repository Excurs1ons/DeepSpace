# DeepSpace 统一世界接入文档（C ABI / FFI）

> DeepSpace 是一个**太空物理模拟引擎**：N 体引力、6DOF 航天器、弹道导弹、
> 拦截导弹制导、雷达/传感器、事件流全部收敛进**同一个模拟世界**
> （`World`）。本文档说明如何把它接入你自己的引擎——包括 C / C++ 程序、
> **Unreal Engine** 与 **Unity**。

---

## 1. 架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                    你的引擎（UE / Unity / 自研）               │
│   渲染、游戏逻辑、输入、AI — 每帧调用 ds_world_step()          │
└──────────────────────────┬──────────────────────────────────┘
                           │  C ABI（稳定、零拷贝、跨语言）
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    DeepSpace FFI 层 (ffi.rs)                 │
│   不透明句柄 DSWorld · POD 值类型 · 错误码 · 线程局部错误消息    │
└──────────────────────────┬──────────────────────────────────┘
                           │  Rust API
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    统一模拟世界 (World)                       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────┐    │
│  │ N 体引力  │ │ 6DOF 航天 │ │ 弹道目标  │ │ 拦截导弹 APN  │    │
│  │ (SoI+Warp)│ │ 器(SoI)  │ │ (ICBM)   │ │ (ProNav)     │    │
│  └──────────┘ └──────────┘ └──────────┘ └──────────────┘    │
│  统一实体表 entities · 统一事件流 events · 雷达 · 大气          │
└─────────────────────────────────────────────────────────────┘
```

**为什么用 C ABI？**
- Rust 没有稳定的 ABI，但 C ABI 是所有语言的"通用语"。
- UE 原生 C++ 可直接 `extern "C"` 链接；Unity 用 C# `DllImport`。
- 结构体全部 `#[repr(C)]` POD，跨语言 `memcpy` 零拷贝。

---

## 2. 构建

```bash
# 调试库（Termux / Linux / macOS 输出 .so；Windows 输出 .dll）
cargo build -p deepspace

# 产物
#   target/debug/libdeepspace.so      (或 .dylib / .dll)
#   include/deepspace.h              (C ABI 头文件契约)

# Android 目标（NDK 交叉编译，供 UE Android / Unity Android 使用）
cargo ndk -t arm64-v8a -o ../android-libs build -p deepspace --release
#   产物: ../android-libs/arm64-v8a/libdeepspace.so
```

> **注意**：`ffi.rs` 全部函数带 `#[no_mangle]` + `extern "C"`，库内符号
> 稳定导出，无需额外链接选项（`crate-type` 已在 `Cargo.toml` 配置为
> `["lib", "cdylib"]` 时自动导出；若需静态链接给 C++，用 `"staticlib"`）。

---

## 3. 核心概念

| 概念 | 说明 |
|------|------|
| `DSWorld*` | 不透明世界句柄。一次模拟一个世界，由驱动引擎主线程持有并每帧 drive。 |
| `DSEntityState` | 统一实体快照。飞船/卫星/弹道目标/拦截弹全部在这个表里，**每帧读它渲染**。 |
| `DSEvent` | 统一事件流。发射/探测/命中/结局，HUD 滚动日志直接轮询。 |
| 实体 id | `uint64_t` 稳定句柄，用于发射拦截导弹时指定目标。 |
| 错误处理 | 函数返回码（0 = `DS_OK`，负值 = 错误），细节见 `ds_last_error_message`。 |

**线程模型**：世界不是线程安全的。与 PhysX 默认场景一致，由**单一线程**
（引擎主线程 / Unity Update / UE GameThread）驱动。多线程并行物理是后续版本
的能力（届时 FFI 增加 `ds_world_step_parallel` 变体，ABI 保持不变）。

**单位制**：米 (m) / 秒 (s) / 千克 (kg) / 牛顿 (N)。坐标以地球质心为原点
的惯性系（`World::default` 地球固定在原点）。

---

## 4. 典型接入流程（C）

```c
#include "deepspace.h"
#include <stdio.h>

int main(void) {
    /* 1. 创建世界 */
    DSWorld *w = ds_world_create();

    /* 2. 添加一个弹道目标（ICBM 靶弹） */
    DSBallisticConfig tgt = {0};
    snprintf(tgt.name, sizeof tgt.name, "TGT-1");
    tgt.position = (DSVec3){0.0, 6.5e6, 0.0};   /* 500 km 高空 */
    tgt.velocity = (DSVec3){1500.0, 500.0, 0.0};
    tgt.mass     = 1000.0;
    tgt.ref_area_m2 = 0.5;
    tgt.cd       = 0.2;
    int64_t target_id = ds_world_add_ballistic(w, &tgt);

    /* 3. 主循环：每帧 step + 读实体表渲染 + 轮询事件 */
    for (int frame = 0; frame < 2000; ++frame) {
        ds_world_step(w);

        /* 3a. 遍历统一实体表（飞船/卫星/导弹/目标全在这） */
        size_t n = ds_world_entity_count(w);
        for (size_t i = 0; i < n; ++i) {
            DSEntityState st;
            if (ds_world_entity_at(w, i, &st) == DS_OK) {
                printf("entity %llu kind=%d pos=(%.0f, %.0f, %.0f) alt=%.0f km\n",
                       (unsigned long long)st.id, (int)st.kind,
                       st.position.x, st.position.y, st.position.z,
                       st.altitude_m / 1000.0);
            }
        }

        /* 3b. 轮询事件（新的在前，不消费） */
        DSEvent evts[16];
        size_t ne = ds_world_poll_events(w, evts, 16);
        for (size_t i = 0; i < ne; ++i) {
            printf("T+%.1fs event[%u] %s\n", evts[i].time, evts[i].kind, evts[i].text);
        }

        /* 3c. 拦截：雷达锁定后发射 */
        if (ds_world_detected_target(w) == target_id && frame == 100) {
            DSVec3 launcher = {0.0, 6.446e6, 0.0};   /* 75 km 高 */
            DSVec3 v0       = {0.0, 2500.0, 0.0};
            ds_world_fire_interceptor(w, (uint64_t)target_id, launcher, v0);
        }
    }

    /* 4. 释放 */
    ds_world_destroy(w);
    return 0;
}
```

编译：

```bash
cc -I include demo.c -L target/debug -ldeepspace -o demo
LD_LIBRARY_PATH=target/debug ./demo
```

---

## 5. Unity C# 绑定

Unity 通过 **P/Invoke**（`DllImport`）调用原生库。把
`libdeepspace.so`（Android 用 arm64-v8a 版本）放进
`Assets/Plugins/Android/`，或 `libdeepspace.dylib` 进 `Assets/Plugins/`。

### 5.1 绑定封装 `DeepSpace.cs`

```csharp
using System;
using System.Runtime.InteropServices;

namespace DeepSpace
{
    public enum EntityKind : int
    {
        Rocket = 0, Spacecraft = 1, Missile = 2, Icbm = 3, Aircraft = 4, Body = 5,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Vec3
    {
        public double x, y, z;
        public Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct EntityState
    {
        public ulong id;
        public EntityKind kind;
        public Vec3 position;
        public Vec3 velocity;
        public Vec3 acceleration;
        public double altitudeM;
        [MarshalAs(UnmanagedType.I1)] public bool alive;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)] public string name;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)] public string status;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WorldEvent
    {
        public double time;
        public uint kind;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)] public string text;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BallisticConfig
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)] public string name;
        public Vec3 position;
        public Vec3 velocity;
        public double mass;
        public double refAreaM2;
        public double cd;
        public double thrustN;
        public double thrustDurationS;
    }

    public static class Native
    {
        const string LIB = "deepspace";

        [DllImport(LIB)] public static extern IntPtr ds_world_create();
        [DllImport(LIB)] public static extern void ds_world_destroy(IntPtr world);
        [DllImport(LIB)] public static extern int ds_world_step(IntPtr world);
        [DllImport(LIB)] public static extern double ds_world_time(IntPtr world);
        [DllImport(LIB)] public static extern long ds_world_add_ballistic(IntPtr world, ref BallisticConfig cfg);
        [DllImport(LIB)] public static extern long ds_world_fire_interceptor(
            IntPtr world, ulong targetId, Vec3 pos, Vec3 vel);
        [DllImport(LIB)] public static extern long ds_world_detected_target(IntPtr world);
        [DllImport(LIB)] public static extern UIntPtr ds_world_entity_count(IntPtr world);
        [DllImport(LIB)] public static extern int ds_world_entity_at(
            IntPtr world, UIntPtr index, out EntityState state);
        [DllImport(LIB)] public static extern UIntPtr ds_world_poll_events(
            IntPtr world, [Out] WorldEvent[] events, UIntPtr max);
        [DllImport(LIB)] public static extern UIntPtr ds_last_error_message(
            [Out] byte[] buf, UIntPtr len);
    }
}
```

### 5.2 驱动组件 `DeepSpaceDriver.cs`

```csharp
using UnityEngine;

public class DeepSpaceDriver : MonoBehaviour
{
    System.IntPtr world;
    DeepSpace.WorldEvent[] events = new DeepSpace.WorldEvent[32];

    void Start()
    {
        world = DeepSpace.Native.ds_world_create();
        var tgt = new DeepSpace.BallisticConfig
        {
            name = "TGT-1",
            position = new DeepSpace.Vec3(0, 6.5e6, 0),
            velocity = new DeepSpace.Vec3(1500, 500, 0),
            mass = 1000, refAreaM2 = 0.5f, cd = 0.2f,
        };
        DeepSpace.Native.ds_world_add_ballistic(world, ref tgt);
    }

    void Update()
    {
        // 每帧推进世界（固定 0.1s 步长由引擎内部管理）
        DeepSpace.Native.ds_world_step(world);

        // 读实体表 → 生成/更新 GameObject
        ulong n = DeepSpace.Native.ds_world_entity_count(world).ToUInt64();
        for (ulong i = 0; i < n; i++)
        {
            DeepSpace.Native.ds_world_entity_at(
                world, new System.UIntPtr(i), out var st);
            Debug.Log($"{st.name} @ ({st.position.x}, {st.position.y}, {st.position.z})");
        }

        // 轮询事件 → 滚动日志
        ulong ne = DeepSpace.Native.ds_world_poll_events(
            world, events, new System.UIntPtr((uint)events.Length)).ToUInt64();
        for (ulong i = 0; i < ne; i++)
            Debug.Log($"T+{events[i].time:F1}s [{events[i].kind}] {events[i].text}");
    }

    void OnDestroy() => DeepSpace.Native.ds_world_destroy(world);
}
```

### 5.3 Unity 踩坑

| 坑 | 现象 | 解决 |
|----|------|------|
| `bool` 布局 | 状态错乱 | C# 侧用 `[MarshalAs(UnmanagedType.I1)]`（C 的 `bool` 是 1 字节） |
| `ByValTStr` 长度 | 字符串乱码/越界 | 长度必须与 C 侧一致：name=64、status=64、text=256 |
| Android 库路径 | DllNotFoundException | 放到 `Assets/Plugins/Android/arm64-v8a/`，`LIB` 常量用 `"deepspace"`（不带 lib 前缀和 .so 后缀） |
| 回调线程 | 崩溃 | 只在主线程（`Update`）调用；不要在 `FixedUpdate` 之外别的线程调 |
| f64 vs f32 | 精度损失 | C# `double` ↔ C `double` 直通；**不要**用 `float` 收位置 |

---

## 6. Unreal Engine C++ 绑定

UE 是 C++ 引擎，直接 `#include "deepspace.h"` 链接静态库/动态库即可。

### 6.1 库放置

```
YourProject/Source/ThirdParty/DeepSpace/
├── include/deepspace.h
├── lib/
│   ├── Win64/deepspace.lib      (静态)
│   └── Android/arm64-v8a/libdeepspace.so
└── DeepSpace.Build.cs
```

`DeepSpace.Build.cs`：

```csharp
public class DeepSpace : ModuleRules
{
    public DeepSpace(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "Win64", "deepspace.lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Android)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "Android", "arm64-v8a", "libdeepspace.so"));
            PublicAdditionalLibraries.Add("log");
        }
    }
}
```

### 6.2 Actor 组件 `UDeepSpaceSimComponent`

```cpp
// DeepSpaceSimComponent.h
#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "deepspace.h"
#include "DeepSpaceSimComponent.generated.h"

UCLASS(ClassGroup = (Simulation), meta = (BlueprintSpawnableComponent))
class MYGAME_API UDeepSpaceSimComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /** 世界句柄（Blueprint 只读，供查询） */
    UPROPERTY(BlueprintReadOnly, Category = "DeepSpace")
    int64 WorldHandle = 0;

private:
    DSWorld* World = nullptr;
};
```

```cpp
// DeepSpaceSimComponent.cpp
#include "DeepSpaceSimComponent.h"

void UDeepSpaceSimComponent::BeginPlay()
{
    Super::BeginPlay();
    World = ds_world_create();
    WorldHandle = reinterpret_cast<int64>(World);

    // 添加靶弹
    DSBallisticConfig tgt = {};
    FCStringAnsi::Strncpy(tgt.name, "TGT-1", sizeof(tgt.name));
    tgt.position = DSVec3{0.0, 6.5e6, 0.0};
    tgt.velocity = DSVec3{1500.0, 500.0, 0.0};
    tgt.mass = 1000.0; tgt.ref_area_m2 = 0.5; tgt.cd = 0.2;
    ds_world_add_ballistic(World, &tgt);
}

void UDeepSpaceSimComponent::TickComponent(float DeltaTime, ELevelTick, FActorComponentTickFunction*)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!World) return;

    // 推进（内部固定步长 0.1s；需要亚步进时调多次）
    ds_world_step(World);

    // 遍历实体表 → 同步到场景 Actor
    const size_t n = ds_world_entity_count(World);
    for (size_t i = 0; i < n; ++i)
    {
        DSEntityState st;
        if (ds_world_entity_at(World, i, &st) != DS_OK) continue;
        FVector pos(st.position.x / 10000.0, st.position.y / 10000.0, st.position.z / 10000.0);
        // ... 移动对应的 AActor（UE 世界坐标 cm，除以缩放）
    }

    // 事件流 → UE 日志
    DSEvent evts[16];
    const size_t ne = ds_world_poll_events(World, evts, 16);
    for (size_t i = 0; i < ne; ++i)
        UE_LOG(LogTemp, Warning, TEXT("DS T+%.1fs [%u] %s"),
            evts[i].time, evts[i].kind, ANSI_TO_TCHAR(evts[i].text));
}

void UDeepSpaceSimComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (World) { ds_world_destroy(World); World = nullptr; }
    Super::EndPlay(EndPlayReason);
}
```

### 6.3 UE 踩坑

| 坑 | 现象 | 解决 |
|----|------|------|
| UE 坐标单位 cm | 位置/速度差 100 倍 | 渲染时除以 10000（1 个 UE 单位 = 1 cm）；或乘以 0.01 |
| Android 需 log 库 | 链接错误 | `PublicAdditionalLibraries.Add("log")`（C 库内部用不到，但 NDK 链接器要求） |
| `#include "deepspace.h"` 在 .Build.cs 里找不到 | 编译错误 | `PublicIncludePaths.Add` 指向 `include/` 目录 |
| 蓝图线程 | 崩溃 | 只在 GameThread 调；异步线程勿碰 `World` 句柄 |

---

## 7. 其他语言

| 语言 | 方法 | 参考 |
|------|------|------|
| Rust | 直接 `use deepspace::ffi::*` 或高层 `World` API | `deepspace/src/ffi.rs` |
| C++ | `extern "C"` 链接 `libdeepspace` | 第 6 节 |
| C# (Unity) | `DllImport` | 第 5 节 |
| Python | `ctypes.CDLL` + `struct.Struct` 手动布局 | 同第 4 节 C 流程 |
| Godot 4 (GDExtension) | `godot::ffi` 绑 C 函数 | 同第 4 节 C 流程 |
| Java/Kotlin (Android) | `System.loadLibrary("deepspace")` + JNI 桥 | 见第 8 节 Android |

---

## 8. Android（Termux 交叉编译 + 替换进 APK）

```bash
# 1. 安装 NDK 工具链（Termux）
pkg install rust cargo-ndk

# 2. 交叉编译 arm64-v8a
cargo ndk -t arm64-v8a -o android-libs build -p deepspace --release

# 3. 产物
ls android-libs/arm64-v8a/libdeepspace.so

# 4. 替换进既有 APK + 重签（无需独立 Gradle 工程）
unzip -o app.apk -d apk_unpacked
cp android-libs/arm64-v8a/libdeepspace.so apk_unpacked/lib/arm64-v8a/
cd apk_unpacked && zip -r ../app_resigned.zip .
apksigner sign --ks your.keystore ../app_resigned.zip
```

---

## 9. API 参考（完整清单）

| 函数 | 说明 | 返回 |
|------|------|------|
| `ds_world_create()` | 创建标准世界 | `DSWorld*` |
| `ds_world_destroy(w)` | 释放世界 | — |
| `ds_world_step(w)` | 推进一步（固定 dt） | `int32` 错误码 |
| `ds_world_time(w)` | 当前世界时间 | `double` (s) |
| `ds_world_add_spacecraft(w, cfg)` | 注册 6DOF 航天器 | `int64` 实体 id |
| `ds_world_add_satellite(w, name, alt, incl)` | 圆轨道卫星 | `int64` 实体 id |
| `ds_world_add_ballistic(w, cfg)` | 弹道目标（ICBM/靶弹） | `int64` 实体 id |
| `ds_world_fire_interceptor(w, tid, pos, vel)` | 发射拦截导弹 | `int64` 实体 id |
| `ds_world_detected_target(w)` | 雷达锁定的第一个目标 | `int64` id / -1 |
| `ds_world_entity_count(w)` | 统一实体表长度 | `size_t` |
| `ds_world_entity_at(w, idx, out)` | 取实体快照 | `int32` 错误码 |
| `ds_world_event_count(w)` | 事件流长度 | `size_t` |
| `ds_world_poll_events(w, out, max)` | 轮询最近事件 | `size_t` 写入数 |
| `ds_last_error_message(buf, len)` | 最后错误消息 | `size_t` |

**错误码**：`DS_OK=0`、`DS_ERR_NULL=-1`、`DS_ERR_NOT_FOUND=-2`、
`DS_ERR_INVALID_ARG=-3`、`DS_ERR_OOM=-4`、`DS_ERR_STATE=-5`。

---

## 10. 常见问题

**Q: 每帧 step 一次够吗？**
世界内部用固定步长 0.1s；需要更高精度时每帧调多次 `ds_world_step`（如
10 次 = 1ms 步长）。事件不会重复（命中判定用线段插值）。

**Q: 能同时跑多个世界吗？**
可以。`ds_world_create` 返回独立句柄，互不干扰（多场景 / 多人服务器分片）。

**Q: 世界里的坐标是什么参考系？**
`World::default` 以地球质心为原点、地球固定（不绕日公转）的惯性系。
高保真场景用 `World::new(GravitationalSystem, dt, earth_radius)` 注入
自定义星体系统（见 `deepspace/src/world.rs` / `physics.rs`）。

**Q: 如何扩展 FFI 新增能力？**
1. 在 `deepspace/src/ffi.rs` 加 `#[no_mangle] pub extern "C" fn`。
2. 同步更新 `include/deepspace.h`。
3. 加一个 `#[cfg(test)]` FFI 测试（参考 `ffi_interceptor_closed_loop`）。
4. 跑 `cargo test --workspace` 全绿后提交。
