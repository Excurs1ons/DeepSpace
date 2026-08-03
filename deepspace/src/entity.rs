//! 统一实体抽象 — 多类型物理模拟互通的载体
//!
//! 火箭 / 航天器 / 拦截导弹 / 弹道导弹 / 飞机 / 天体 全部收敛为
//! 统一的 [`Entity`] 状态结构，由 [`crate::world::World`] 统一推进、
//! 统一回填、统一显示。HUD 只依赖本模块的视图数据，不感知各类型
//! 物理实现细节。
//!
//! 分工：
//! - `ballistics.rs` — ECEF 高保真三级弹道（独立演示用）
//! - `missile.rs`    — ProNav 气动拦截导弹（BVR 演示用）
//! - 本模块 + `world.rs` — 上述各类型在**同一世界**互通时的统一入口

use crate::Vec3;

// =====================================================================
// 实体类型
// =====================================================================

/// 统一实体类型标签
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    /// 火箭（Vessel 管线：多级、推进剂、级分离）
    Rocket,
    /// 6DOF 航天器（SoI 引力 + 推力器 + SAS）
    Spacecraft,
    /// 拦截导弹（ProNav 制导，气动 + 雷达）
    Missile,
    /// 弹道目标（地心引力 + 大气阻力 + 程序推力剖面）
    Icbm,
    /// 飞机（气动面 + 自动驾驶）
    Aircraft,
    /// 天体（N 体引力场成员）
    Body,
}

impl EntityKind {
    /// HUD 用短名
    pub fn name(&self) -> &'static str {
        match self {
            EntityKind::Rocket => "ROCKET",
            EntityKind::Spacecraft => "SPACECRAFT",
            EntityKind::Missile => "MISSILE",
            EntityKind::Icbm => "ICBM",
            EntityKind::Aircraft => "AIRCRAFT",
            EntityKind::Body => "BODY",
        }
    }
}

// =====================================================================
// 世界事件（闭环反馈日志）
// =====================================================================

/// 事件类型 — 决定 HUD 显示颜色
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    /// 常规信息
    Info,
    /// 探测到目标
    Detect,
    /// 发射
    Launch,
    /// 命中 / 拦截
    Hit,
    /// 阶段转换
    Phase,
    /// 任务结局（成功 / 失败）
    Outcome,
}

/// 世界统一事件 — 所有类型的模拟共用一条事件流
#[derive(Debug, Clone)]
pub struct WorldEvent {
    /// 世界时间 (s)
    pub time: f64,
    /// 事件类型
    pub kind: EventKind,
    /// 事件文本
    pub text: String,
}

impl WorldEvent {
    pub fn new(time: f64, kind: EventKind, text: impl Into<String>) -> Self {
        WorldEvent {
            time,
            kind,
            text: text.into(),
        }
    }
}

// =====================================================================
// 统一实体
// =====================================================================

/// 统一实体 — 所有可动对象的统一状态视图
///
/// 由 `World` 每步推进物理对象后**回填**，HUD / 探测 / 拦截判定
/// 只读本结构，不直接接触各物理实现。
#[derive(Debug, Clone)]
pub struct Entity {
    /// 世界内唯一 ID
    pub id: u64,
    /// 类型标签
    pub kind: EntityKind,
    /// 显示名
    pub name: String,
    /// 世界坐标位置 (m)
    pub position: Vec3,
    /// 世界坐标速度 (m/s)
    pub velocity: Vec3,
    /// 质量 (kg)
    pub mass: f64,
    /// 存活（被拦截后置 false）
    pub alive: bool,
    /// 海拔 (m) — 相对地球表面
    pub altitude_m: f64,
    /// 速度标量 (m/s)
    pub speed_mps: f64,
    /// 当前加速度 (m/s²) — 供 APN 制导补偿使用
    pub acceleration: Vec3,
    /// 状态文本（阶段 / 制导状态，HUD 显示）
    pub status: String,
}

impl Entity {
    /// 创建实体（自动计算海拔 / 速度）
    pub fn new(
        id: u64,
        kind: EntityKind,
        name: &str,
        position: Vec3,
        velocity: Vec3,
        mass: f64,
        earth_radius: f64,
    ) -> Self {
        let speed = velocity.length();
        let alt = (position.length() - earth_radius).max(0.0);
        Entity {
            id,
            kind,
            name: name.to_string(),
            position,
            velocity,
            mass,
            alive: true,
            acceleration: Vec3::zero(),
            altitude_m: alt,
            speed_mps: speed,
            status: "IDLE".to_string(),
        }
    }

    /// 从物理状态回填视图字段
    pub fn sync(&mut self, position: Vec3, velocity: Vec3, earth_radius: f64) {
        self.position = position;
        self.velocity = velocity;
        self.speed_mps = velocity.length();
        self.altitude_m = (position.length() - earth_radius).max(0.0);
    }

    /// 到另一个实体的距离 (m)
    pub fn distance_to(&self, other: &Entity) -> f64 {
        (self.position - other.position).length()
    }
}
