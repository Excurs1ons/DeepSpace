//! ## C ABI — 统一世界接入层（PhysX 风格）
//!
//! 将 DeepSpace 的统一模拟世界（N 体 + 6DOF 航天器 + 弹道目标 + 拦截导弹 +
//! 雷达 + 事件流）通过稳定的 **C 接口** 暴露给外部引擎（Unreal / Unity /
//! Godot / 自研引擎）。
//!
//! 设计原则（对齐常见物理引擎接入模式）：
//! - **不透明句柄** [`DSWorld`] = `*mut World`，外部不感知内部类型
//! - 所有值类型为 **POD**（`#[repr(C)]`、可 `memcpy`），跨语言零拷贝
//! - 字符串统一固定缓冲，避免 CString 生命周期 / 所有权纠纷
//! - 错误用返回码 + 线程局部消息（`ds_last_error_message`）
//! - 单线程访问约定：世界由驱动引擎的主线程持有并 drive
//!   （与 PhysX 默认场景一致，多线程场景见文档）
//!
//! 完整接入文档见 `docs/ffi-integration.md`（含 C / C++ / Unity C# / UE C++
//! 绑定示例）。头文件 `include/deepspace.h` 由 cbindgen 在构建时自动生成
//! （见 `build.rs` / `cbindgen.toml`），无需手改。

use crate::entity::{EntityKind, EventKind, WorldEvent};
use crate::missile::AamConfig;
use crate::world::{BallisticConfig, World};
use crate::Vec3;
use std::ffi::c_char;

/// FFI 句柄：指向内部 [`World`] 的不透明指针。
///
/// 无 `#[repr(C)]` 且字段私有 → cbindgen 生成 `typedef struct DSWorld DSWorld;`，
/// C 侧只见不透明句柄，不感知内部 [`World`] 的内存布局。
pub struct DSWorld {
    _private: [u8; 0],
}

/// 从不透明句柄拿内部 `World` 的可变引用（FFI 层内部用，非导出）。
unsafe fn as_world(world: *mut DSWorld) -> &'static mut World {
    unsafe { &mut *(world as *mut World) }
}

/// 从不透明句柄拿内部 `World` 的共享引用（FFI 层内部用，非导出）。
unsafe fn as_world_ref(world: *mut DSWorld) -> &'static World {
    unsafe { &*(world as *const World) }
}

// =====================================================================
// POD 值类型（#[repr(C)]，跨语言可 memcpy）
// =====================================================================

/// 三维向量（f64）
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DSVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl DSVec3 {
    fn from_vec3(v: Vec3) -> Self {
        DSVec3 {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
    fn to_vec3(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
}

/// 统一实体类型标签（与 [`EntityKind`] 一一对应）
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DSEntityKind {
    Rocket = 0,
    Spacecraft = 1,
    Missile = 2,
    Icbm = 3,
    Aircraft = 4,
    Body = 5,
}

impl From<EntityKind> for DSEntityKind {
    fn from(k: EntityKind) -> Self {
        match k {
            EntityKind::Rocket => DSEntityKind::Rocket,
            EntityKind::Spacecraft => DSEntityKind::Spacecraft,
            EntityKind::Missile => DSEntityKind::Missile,
            EntityKind::Icbm => DSEntityKind::Icbm,
            EntityKind::Aircraft => DSEntityKind::Aircraft,
            EntityKind::Body => DSEntityKind::Body,
        }
    }
}

/// 统一实体状态快照（外部引擎每帧读这里渲染）
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DSEntityState {
    /// 实体 id（稳定句柄）
    pub id: u64,
    /// 实体类型
    pub kind: DSEntityKind,
    /// 世界坐标位置 (m)
    pub position: DSVec3,
    /// 世界坐标速度 (m/s)
    pub velocity: DSVec3,
    /// 加速度 (m/s²)
    pub acceleration: DSVec3,
    /// 离地表海拔 (m)
    pub altitude_m: f64,
    /// 是否存活
    pub alive: bool,
    /// 显示名（固定缓冲，截断安全）
    pub name: [c_char; 64],
    /// 状态文本（如拦截阶段 / G 值）
    pub status: [c_char; 64],
}

/// 世界事件（闭环反馈日志）
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DSEvent {
    /// 世界时间 (s)
    pub time: f64,
    /// 事件类型
    pub kind: u32,
    /// 事件文本
    pub text: [c_char; 256],
}

/// 弹道目标配置（新建弹道目标用）
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DSBallisticConfig {
    pub name: [c_char; 64],
    pub position: DSVec3,
    pub velocity: DSVec3,
    pub mass: f64,
    pub ref_area_m2: f64,
    pub cd: f64,
    pub thrust_n: f64,
    pub thrust_duration_s: f64,
}

/// 6DOF 航天器配置
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DSSpacecraftConfig {
    pub name: [c_char; 64],
    pub position: DSVec3,
    pub velocity: DSVec3,
    pub mass: f64,
    pub inertia_xx: f64,
    pub inertia_yy: f64,
    pub inertia_zz: f64,
}

/// 拦截导弹配置（ProNav / APN 制导）
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DSInterceptorConfig {
    pub name: [c_char; 64],
    pub mass_kg: f64,
    pub kill_radius_m: f64,
    pub max_mach: f64,
    pub max_g_load: f64,
    pub nav_constant: f64,
}

// =====================================================================
// 错误码 + 线程局部错误消息
// =====================================================================

/// 返回码：0 成功；负值对应错误
pub const DS_OK: i32 = 0;
pub const DS_ERR_NULL: i32 = -1;
pub const DS_ERR_NOT_FOUND: i32 = -2;
pub const DS_ERR_INVALID_ARG: i32 = -3;
pub const DS_ERR_OOM: i32 = -4;
pub const DS_ERR_STATE: i32 = -5;

thread_local! {
    static LAST_ERROR: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
}

fn set_last_error(msg: impl Into<String>) {
    LAST_ERROR.with(|e| *e.borrow_mut() = msg.into());
}

// =====================================================================
// 辅助：固定缓冲 <-> Rust 字符串
// =====================================================================

/// 写字符串到固定 `[c_char; N]`，自动截断并保证 NUL 结尾
fn write_str<const N: usize>(buf: &mut [c_char; N], s: &str) {
    let bytes = s.as_bytes();
    let n = bytes.len().min(N - 1);
    for (i, b) in bytes[..n].iter().enumerate() {
        buf[i] = *b as c_char;
    }
    buf[n] = 0;
}

/// 从固定缓冲读 Rust 字符串（到首个 NUL 为止）
fn read_str<const N: usize>(buf: &[c_char; N]) -> String {
    let mut end = 0;
    while end < N && buf[end] != 0 {
        end += 1;
    }
    let slice = &buf[..end];
    // 从 c_char 切片转 UTF-8（对非法字节做 lossy 处理）。
    // c_char 平台相关（arm64=u8 / x86=i8），显式转换保持跨平台语义
    let bytes: Vec<u8> = slice
        .iter()
        .map(|&c| u8::from_ne_bytes([c as i8 as u8]))
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

// =====================================================================
// 生命周期
// =====================================================================

/// 创建一个标准统一世界（地球固定在原点 + 默认大气 + APG-77 雷达）
///
/// 等价于 [`World::default`]。返回句柄，用 [`ds_world_destroy`] 释放。
///
/// # Safety
/// 返回的指针必须用 [`ds_world_destroy`] 释放，不得跨线程移动。
#[no_mangle]
pub extern "C" fn ds_world_create() -> *mut DSWorld {
    let w = World::default();
    Box::into_raw(Box::new(w)) as *mut DSWorld
}

/// 释放世界
///
/// # Safety
/// `world` 必须来自 [`ds_world_create`] 且未释放过。
#[no_mangle]
pub unsafe extern "C" fn ds_world_destroy(world: *mut DSWorld) {
    if world.is_null() {
        set_last_error("ds_world_destroy: null world");
        return;
    }
    unsafe {
        drop(Box::from_raw(world as *mut World));
    }
}

/// 推进一个世界步（N 体 + 航天器 + 弹道 + 拦截 + 探测 + 命中）
///
/// # Safety
/// `world` 必须有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_step(world: *mut DSWorld) -> i32 {
    if world.is_null() {
        set_last_error("ds_world_step: null world");
        return DS_ERR_NULL;
    }
    unsafe {
        as_world(world).step();
    }
    DS_OK
}

/// 当前世界时间 (s)
///
/// # Safety
/// `world` 必须有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_time(world: *mut DSWorld) -> f64 {
    if world.is_null() {
        // 返回 0 并记录错误，对接方应从 ds_last_error_message 读取
        set_last_error("ds_world_time: null world");
        return 0.0;
    }
    unsafe { as_world_ref(world).time() }
}

/// 最后一条错误消息（线程局部，UTF-8）
///
/// # Safety
/// `buf` 必须指向至少 `buf_len` 字节的可写内存（`buf` 为空时仅返回长度）。
#[no_mangle]
pub unsafe extern "C" fn ds_last_error_message(buf: *mut c_char, buf_len: usize) -> usize {
    if buf.is_null() {
        return LAST_ERROR.with(|e| e.borrow().len());
    }
    let msg = LAST_ERROR.with(|e| e.borrow().clone());
    let bytes = msg.as_bytes();
    let n = bytes.len().min(buf_len.saturating_sub(1));
    unsafe {
        for (i, b) in bytes[..n].iter().enumerate() {
            *buf.add(i) = *b as c_char;
        }
        *buf.add(n) = 0;
    }
    n
}

// =====================================================================
// 实体操作
// =====================================================================

/// 注册一个 6DOF 航天器，返回实体 id（-1 失败）
///
/// # Safety
/// `world` 有效；`cfg` 非空。
#[no_mangle]
pub unsafe extern "C" fn ds_world_add_spacecraft(
    world: *mut DSWorld,
    cfg: *const DSSpacecraftConfig,
) -> i64 {
    if world.is_null() || cfg.is_null() {
        set_last_error("ds_world_add_spacecraft: null arg");
        return -1;
    }
    let cfg = unsafe { &*cfg };
    let pos = cfg.position.to_vec3();
    let vel = cfg.velocity.to_vec3();
    let name = read_str(&cfg.name);
    let craft = crate::space_physics::SpacecraftBody::new(
        pos,
        vel,
        cfg.mass,
        (cfg.inertia_xx, cfg.inertia_yy, cfg.inertia_zz),
    );
    unsafe { as_world(world).add_spacecraft(craft, &name) as i64 }
}

/// 添加一颗圆轨道卫星（绕世界原点，给定海拔与倾角）
///
/// # Safety
/// `world` 有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_add_satellite(
    world: *mut DSWorld,
    name: *const c_char,
    altitude_m: f64,
    inclination_rad: f64,
) -> i64 {
    if world.is_null() || name.is_null() {
        set_last_error("ds_world_add_satellite: null arg");
        return -1;
    }
    let name = unsafe { std::ffi::CStr::from_ptr(name) }
        .to_string_lossy()
        .into_owned();
    unsafe { as_world(world).add_satellite(&name, altitude_m, inclination_rad) as i64 }
}

/// 添加弹道目标（ICBM / 靶弹），返回实体 id
///
/// # Safety
/// `world`、`cfg` 有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_add_ballistic(
    world: *mut DSWorld,
    cfg: *const DSBallisticConfig,
) -> i64 {
    if world.is_null() || cfg.is_null() {
        set_last_error("ds_world_add_ballistic: null arg");
        return -1;
    }
    let cfg = unsafe { &*cfg };
    let inner = BallisticConfig {
        name: read_str(&cfg.name),
        position: cfg.position.to_vec3(),
        velocity: cfg.velocity.to_vec3(),
        mass: cfg.mass,
        ref_area_m2: cfg.ref_area_m2,
        cd: cfg.cd,
        thrust_n: cfg.thrust_n,
        thrust_duration_s: cfg.thrust_duration_s,
    };
    unsafe { as_world(world).add_ballistic(inner) as i64 }
}

/// 发射一枚拦截导弹拦截指定目标，返回拦截弹实体 id（-1 失败）
///
/// # Safety
/// `world` 有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_fire_interceptor(
    world: *mut DSWorld,
    target_id: u64,
    pos: DSVec3,
    vel: DSVec3,
) -> i64 {
    if world.is_null() {
        set_last_error("ds_world_fire_interceptor: null world");
        return -1;
    }
    let config = AamConfig::interceptor();
    unsafe {
        match as_world(world).fire_interceptor(config, pos.to_vec3(), vel.to_vec3(), target_id) {
            Some(id) => id as i64,
            None => {
                set_last_error("ds_world_fire_interceptor: target not found");
                -1
            }
        }
    }
}

/// 雷达探测到第一个存活弹道目标 id（无则 -1）
///
/// # Safety
/// `world` 有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_detected_target(world: *mut DSWorld) -> i64 {
    if world.is_null() {
        set_last_error("ds_world_detected_target: null world");
        return -1;
    }
    unsafe {
        as_world_ref(world)
            .detected_target()
            .map(|id| id as i64)
            .unwrap_or(-1)
    }
}

/// 实体总数（HUD 遍历用）
///
/// # Safety
/// `world` 有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_entity_count(world: *mut DSWorld) -> usize {
    if world.is_null() {
        return 0;
    }
    unsafe { as_world_ref(world).entities.len() }
}

/// 按索引取实体状态（实体表是有序的，索引 ∈ [0, count)）。
///
/// 把状态拷贝到 `out`（非空）。不存在或越界返回负错误码。
///
/// # Safety
/// `world`、`out` 有效；`out` 需指向足够大的 `[DSEntityState]` 缓冲区（至少 1）。
#[no_mangle]
pub unsafe extern "C" fn ds_world_entity_at(
    world: *mut DSWorld,
    index: usize,
    out: *mut DSEntityState,
) -> i32 {
    if world.is_null() || out.is_null() {
        set_last_error("ds_world_entity_at: null arg");
        return DS_ERR_NULL;
    }
    let w = as_world_ref(world);
    let Some(e) = w.entities.get(index) else {
        set_last_error("ds_world_entity_at: index out of range");
        return DS_ERR_NOT_FOUND;
    };
    let mut state = DSEntityState {
        id: e.id,
        kind: DSEntityKind::from(e.kind),
        position: DSVec3::from_vec3(e.position),
        velocity: DSVec3::from_vec3(e.velocity),
        acceleration: DSVec3::from_vec3(e.acceleration),
        altitude_m: e.altitude_m,
        alive: e.alive,
        name: [0; 64],
        status: [0; 64],
    };
    write_str(&mut state.name, &e.name);
    write_str(&mut state.status, &e.status);
    unsafe { *out = state };
    DS_OK
}

/// 返回事件流里可用的最大事件数（无输出缓冲时的查询）
///
/// # Safety
/// `world` 有效。
#[no_mangle]
pub unsafe extern "C" fn ds_world_event_count(world: *mut DSWorld) -> usize {
    if world.is_null() {
        return 0;
    }
    unsafe { as_world_ref(world).events.len() }
}

/// 轮询最近的事件：把最多 `max_events` 条事件写入 `out`，返回写入条数。
///
/// 顺序为**新的在前**（与 HUD 一致）。不消费事件流（多次调用得到同一批）。
///
/// # Safety
/// `world`、`out` 有效；`out` 需指向至少 `max_events` 个 [`DSEvent`]。
#[no_mangle]
pub unsafe extern "C" fn ds_world_poll_events(
    world: *mut DSWorld,
    out: *mut DSEvent,
    max_events: usize,
) -> usize {
    if world.is_null() || out.is_null() || max_events == 0 {
        return 0;
    }
    let w = as_world_ref(world);
    let events: Vec<&WorldEvent> = w.recent_events(max_events);
    let n = events.len().min(max_events);
    for (i, evt) in events.iter().take(n).enumerate() {
        let mut dsev = DSEvent {
            time: evt.time,
            kind: evt_kind_to_u32(evt.kind),
            text: [0; 256],
        };
        write_str(&mut dsev.text, &evt.text);
        unsafe {
            *out.add(i) = dsev;
        }
    }
    n
}

fn evt_kind_to_u32(k: EventKind) -> u32 {
    match k {
        EventKind::Info => DSEV_INFO,
        EventKind::Detect => DSEV_DETECT,
        EventKind::Launch => DSEV_LAUNCH,
        EventKind::Hit => DSEV_HIT,
        EventKind::Phase => DSEV_PHASE,
        EventKind::Outcome => DSEV_OUTCOME,
    }
}

// =====================================================================
// 事件类型常量（cbindgen 导出为宏，供外部引擎按语义判断事件 kind）
// =====================================================================

pub const DSEV_INFO: u32 = 0;
pub const DSEV_DETECT: u32 = 1;
pub const DSEV_LAUNCH: u32 = 2;
pub const DSEV_HIT: u32 = 3;
pub const DSEV_PHASE: u32 = 4;
pub const DSEV_OUTCOME: u32 = 5;

// =====================================================================
// 测试：FFI 层端到端（直接调用 extern "C" 函数）
// =====================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::ptr;

    #[test]
    fn ffi_create_step_destroy() {
        let w = ds_world_create();
        assert!(!w.is_null());
        for _ in 0..10 {
            assert_eq!(unsafe { ds_world_step(w) }, DS_OK, "{}", unsafe {
                CStr_from_last_error()
            });
        }
        let t = unsafe { ds_world_time(w) };
        assert!(t > 0.0);
        unsafe { ds_world_destroy(w) };
    }

    // 临时：把 Rust String 读出来打印
    fn CStr_from_last_error() -> String {
        let mut buf = [0 as c_char; 256];
        unsafe { ds_last_error_message(buf.as_mut_ptr(), buf.len()) };
        read_str(&buf)
    }

    #[test]
    fn ffi_add_ballistic_and_query() {
        let w = ds_world_create();
        assert!(!w.is_null());
        let mut name = [0 as c_char; 64];
        write_str(&mut name, "TGT-FFI");
        let cfg = DSBallisticConfig {
            name,
            position: DSVec3 {
                x: 0.0,
                y: 6_500_000.0,
                z: 0.0,
            },
            velocity: DSVec3 {
                x: 1500.0,
                y: 500.0,
                z: 0.0,
            },
            mass: 1000.0,
            ref_area_m2: 0.5,
            cd: 0.2,
            thrust_n: 0.0,
            thrust_duration_s: 0.0,
        };
        let id = unsafe { ds_world_add_ballistic(w, &cfg) };
        assert!(id >= 0, "add_ballistic failed: {}", CStr_from_last_error());
        let count = unsafe { ds_world_entity_count(w) };
        assert_eq!(count, 1, "弹道目标应进入统一实体视图");
        // 读取实体状态验证 FFI 回填
        let mut st = DSEntityState {
            id: 0,
            kind: DSEntityKind::Icbm,
            position: DSVec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            velocity: DSVec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            acceleration: DSVec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            altitude_m: 0.0,
            alive: false,
            name: [0; 64],
            status: [0; 64],
        };
        let rc = unsafe { ds_world_entity_at(w, 0, &mut st) };
        assert_eq!(rc, DS_OK, "{}", CStr_from_last_error());
        assert_eq!(st.id, id as u64);
        assert_eq!(st.kind, DSEntityKind::Icbm);
        assert!(
            (st.altitude_m - 129_000.0).abs() < 1.0,
            "alt={}",
            st.altitude_m
        );
        assert_eq!(read_str(&st.name), "TGT-FFI");
        unsafe { ds_world_destroy(w) };
    }

    #[test]
    fn ffi_interceptor_closed_loop() {
        let w = ds_world_create();
        assert!(!w.is_null());

        // 静止高空目标
        let mut name = [0 as c_char; 64];
        write_str(&mut name, "TGT-FFI2");
        let cfg = DSBallisticConfig {
            name,
            position: DSVec3 {
                x: 0.0,
                y: 6_771_000.0,
                z: 0.0,
            },
            velocity: DSVec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            mass: 1000.0,
            ref_area_m2: 0.5,
            cd: 0.2,
            thrust_n: 0.0,
            thrust_duration_s: 0.0,
        };
        let tid = unsafe { ds_world_add_ballistic(w, &cfg) };
        assert!(tid >= 0);

        // 拦截
        let start = DSVec3 {
            x: 0.0,
            y: 6_446_000.0,
            z: 0.0,
        };
        let vel = DSVec3 {
            x: 0.0,
            y: 2500.0,
            z: 0.0,
        };
        let mid = unsafe { ds_world_fire_interceptor(w, tid as u64, start, vel) };
        assert!(mid >= 0, "fire failed: {}", CStr_from_last_error());

        let mut hit = false;
        for _ in 0..4000 {
            unsafe { ds_world_step(w) };
            if unsafe { ds_world_time(w) } > 180.0 {
                break;
            }
            // 检查结局事件
            let mut evts = [DSEvent {
                time: 0.0,
                kind: 0,
                text: [0; 256],
            }; 8];
            let n = unsafe { ds_world_poll_events(w, evts.as_mut_ptr(), evts.len()) };
            for e in evts[..n].iter() {
                if e.kind == 5 {
                    hit = true;
                }
            }
            if hit {
                break;
            }
        }
        assert!(hit, "拦截闭环应产生 Outcome 事件");
        unsafe { ds_world_destroy(w) };
    }

    #[test]
    fn ffi_null_guard() {
        assert_eq!(unsafe { ds_world_step(ptr::null_mut()) }, DS_ERR_NULL);
        unsafe { ds_world_destroy(ptr::null_mut()) }; // 不崩溃
        assert_eq!(
            unsafe { ds_world_add_satellite(ptr::null_mut(), ptr::null(), 0.0, 0.0) },
            -1
        );
    }
}
