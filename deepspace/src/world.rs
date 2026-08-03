//! 统一世界 — 多类型物理模拟互通 + 闭环
//!
//! 在 [`crate::space_physics::SpacePhysicsWorld`]（N 体 + 6DOF 航天器 +
//! SoI + TimeWarp）之上叠加统一实体层：
//!
//! - **实体注册表** `entities: Vec<Entity>` — 所有对象的统一状态视图，
//!   每步物理推进后回填，HUD 只读这里
//! - **拦截导弹** `missiles: Vec<MissileState>` — ProNav 制导，
//!   雷达探测目标后发射，命中判定写入事件流
//! - **弹道目标** `ballistic: Vec<Entity>` — 简化地心引力 + 大气阻力弹道
//!   （作为 ICBM 目标实体，与 `ballistics.rs` 的 ECEF 高保真版分工）
//! - **统一事件流** `events: Vec<WorldEvent>` — 探测/发射/命中/结局
//!   全部进入同一条日志，HUD 滚动显示 → 闭环反馈
//!
//! 闭环：世界 state → HUD 显示 → 用户输入（发射/布防）→ 写回世界 → 事件。

use crate::entity::{Entity, EntityKind, EventKind, WorldEvent};
use crate::environment::Atmosphere;
use crate::missile::{AamConfig, MissileState};
use crate::physics::GravitationalSystem;
use crate::sensors::{Radar, RadarConfig, RadarMode};
use crate::space_physics::{SpacecraftBody, SpacePhysicsWorld};
use crate::{Vec3, G};

/// 简化弹道目标参数（ICBM 目标实体）
#[derive(Debug, Clone)]
pub struct BallisticConfig {
    pub name: String,
    /// 初始位置 (m, 世界坐标)
    pub position: Vec3,
    /// 初始速度 (m/s)
    pub velocity: Vec3,
    /// 质量 (kg)
    pub mass: f64,
    /// 弹道系数参考面积 (m²)
    pub ref_area_m2: f64,
    /// 阻力系数
    pub cd: f64,
    /// 发射后推力 (N)，0 = 纯弹道
    pub thrust_n: f64,
    /// 推力时长 (s)
    pub thrust_duration_s: f64,
}

/// 统一世界
pub struct World {
    /// 底层太空物理世界（N 体 + 6DOF 航天器 + SoI + Warp）
    pub space: SpacePhysicsWorld,
    /// 统一实体注册表（含回填后的视图）
    pub entities: Vec<Entity>,
    /// 拦截导弹（ProNav 制导），与 `missile_entity_ids` 平行
    pub missiles: Vec<MissileState>,
    /// 每个拦截导弹对应的实体 id（平行数组）
    pub missile_entity_ids: Vec<u64>,
    /// 弹道目标（简化 ICBM 实体）
    pub ballistic: Vec<(Entity, BallisticConfig)>,
    /// 拦截方雷达
    pub radar: Radar,
    /// 大气模型（弹道目标阻力用）
    pub atmosphere: Atmosphere,
    /// 统一事件流（闭环反馈）
    pub events: Vec<WorldEvent>,
    /// 地球半径
    pub earth_radius: f64,
    next_id: u64,
    /// 已发射拦截导弹数
    launched: usize,
}

impl World {
    /// 创建统一世界
    pub fn new(star_system: GravitationalSystem, dt: f64, earth_radius: f64) -> Self {
        let space = SpacePhysicsWorld::new(star_system, dt);
        let mut radar = Radar::new(RadarConfig::apg77());
        radar.set_mode(RadarMode::Stt);
        World {
            space,
            entities: Vec::new(),
            missiles: Vec::new(),
            missile_entity_ids: Vec::new(),
            ballistic: Vec::new(),
            radar,
            atmosphere: Atmosphere::new(101325.0, 8500.0),
            events: Vec::new(),
            earth_radius,
            next_id: 1,
            launched: 0,
        }
    }

    /// 注册一个 6DOF 航天器（自动加入实体注册表）
    pub fn add_spacecraft(&mut self, craft: SpacecraftBody, name: &str) -> u64 {
        let id = self.alloc_id();
        self.space.add_spacecraft(craft);
        self.entities.push(Entity::new(
            id,
            EntityKind::Spacecraft,
            name,
            Vec3::zero(),
            Vec3::zero(),
            0.0,
            self.earth_radius,
        ));
        id
    }

    /// 添加一颗圆轨道观测卫星（统一实体表，验证多类型互通）
    pub fn add_satellite(&mut self, name: &str, altitude_m: f64, inclination_rad: f64) -> u64 {
        let r = self.earth_radius + altitude_m;
        let mu = crate::G * 5.9722e24;
        let v = (mu / r).sqrt();
        let pos = Vec3::new(r, 0.0, 0.0);
        // 绕 Z 轴圆轨道，按倾角倾斜
        let vel = Vec3::new(
            0.0,
            v * inclination_rad.cos(),
            v * inclination_rad.sin(),
        );
        let craft = SpacecraftBody::new(pos, vel, 2000.0, (500.0, 500.0, 200.0));
        let id = self.add_spacecraft(craft, name);
        self.events.push(WorldEvent::new(
            self.time(),
            EventKind::Launch,
            format!("{} 入轨 (alt={:.0} km)", name, altitude_m / 1000.0),
        ));
        id
    }

    /// 结局事件发生的时间（若无则返回当前时间）
    pub fn time_at_outcome(&self) -> f64 {
        self.events
            .iter()
            .find(|e| e.kind == EventKind::Outcome)
            .map(|e| e.time)
            .unwrap_or_else(|| self.time())
    }

    /// 添加弹道目标（ICBM）
    pub fn add_ballistic(&mut self, cfg: BallisticConfig) -> u64 {
        let id = self.alloc_id();
        let ent = Entity::new(
            id,
            EntityKind::Icbm,
            &cfg.name,
            cfg.position,
            cfg.velocity,
            cfg.mass,
            self.earth_radius,
        );
        self.events.push(WorldEvent::new(
            self.time(),
            EventKind::Launch,
            format!("{} 发射", cfg.name),
        ));
        self.ballistic.push((ent, cfg));
        id
    }

    /// 由雷达探测到的弹道目标 ID（第一个存活的）
    pub fn detected_target(&self) -> Option<u64> {
        self.ballistic
            .iter()
            .find(|(e, _)| e.alive && e.altitude_m > 10_000.0)
            .map(|(e, _)| e.id)
    }

    /// 发射一枚拦截导弹拦截指定目标
    pub fn fire_interceptor(
        &mut self,
        config: AamConfig,
        pos: Vec3,
        vel: Vec3,
        target_id: u64,
    ) -> Option<u64> {
        let (tpos, tvel, _tacc) = self.entity_state(target_id)?;
        let mut ms = MissileState::new(config, pos, vel, tpos, tvel);
        ms.launch();
        ms.activate_seeker();
        let id = self.alloc_id();
        self.events.push(WorldEvent::new(
            self.time(),
            EventKind::Launch,
            format!("拦截导弹 #{} 发射 → 目标 #{}", id, target_id),
        ));
        self.entities.push(Entity::new(
            id,
            EntityKind::Missile,
            &format!("Interceptor #{}", id),
            pos,
            vel,
            ms.config.mass_kg,
            self.earth_radius,
        ));
        self.missile_entity_ids.push(id);
        self.missiles.push(ms);
        self.launched += 1;
        Some(id)
    }

    fn entity_state(&self, id: u64) -> Option<(Vec3, Vec3, Vec3)> {
        for (e, _) in &self.ballistic {
            if e.id == id && e.alive {
                return Some((e.position, e.velocity, e.acceleration));
            }
        }
        for e in &self.entities {
            if e.id == id {
                return Some((e.position, e.velocity, e.acceleration));
            }
        }
        None
    }

    /// 统一世界时间
    pub fn time(&self) -> f64 {
        self.space.time
    }

    /// 取实体视图（按 id）
    pub fn entity(&self, id: u64) -> Option<&Entity> {
        self.entities.iter().find(|e| e.id == id)
    }

    /// 事件流（最近 N 条，新的在前）
    pub fn recent_events(&self, n: usize) -> Vec<&WorldEvent> {
        self.events.iter().rev().take(n).collect()
    }

    /// 推进一个世界步（N 体 + 航天器 + 弹道 + 拦截 + 探测 + 命中判定）
    pub fn step(&mut self) {
        // 1. 底层世界（N 体 + 6DOF 航天器 + SoI）
        self.space.step();

        // 2. 弹道目标推进（自由函数，避免借用冲突）
        let dt = self.space.dt * self.space.warp_mode.factor();
        let time = self.space.time;
        let atmosphere = self.atmosphere.clone();
        let earth_radius = self.earth_radius;
        for (ent, cfg) in self.ballistic.iter_mut() {
            if ent.alive {
                step_ballistic(ent, cfg, dt, time, &atmosphere, earth_radius);
            }
        }

        // 3. 拦截导弹推进（气动 + ProNav）
        let mut dead = Vec::new();
        // 预取目标状态，避免在可变借用循环内再次借用 self
        let target_state = self.detected_target().and_then(|tid| self.entity_state(tid));
        for (i, ms) in self.missiles.iter_mut().enumerate() {
            if !ms.is_alive() {
                dead.push(i);
                continue;
            }
            // 目标状态更新（含 APN 加速度补偿）
            if let Some((tp, tv, ta)) = target_state {
                ms.target_position = tp;
                ms.target_velocity = tv;
                ms.target_acceleration = ta;
            }
            let alt = ms.position.length() - self.earth_radius;
            let rho = self.atmosphere.get_density(alt.max(0.0));
            // 注入地心引力（与弹道目标同坐标系）
            let r = ms.position.length().max(1.0);
            ms.gravity = -ms.position * (crate::G * 5.9722e24 / (r * r * r));
            ms.step(dt, rho);
            // 回填实体视图（平行数组取实体 id）
            let eid = self.missile_entity_ids.get(i).copied();
            if let Some(eid) = eid {
                if let Some(ent) = self.entities.iter_mut().find(|e| e.id == eid) {
                    ent.sync(ms.position, ms.velocity, self.earth_radius);
                    ent.status = format!("{:?} G={:.1}", ms.phase, ms.current_g);
                }
            }
        }
        for i in dead.into_iter().rev() {
            self.missiles.remove(i);
            self.missile_entity_ids.remove(i);
        }

        // 4. 回填弹道实体视图 + 大气阻尼（已在上一步推进）
        for (ent, _) in self.ballistic.iter_mut() {
            ent.sync(ent.position, ent.velocity, self.earth_radius);
            ent.status = if ent.alive { "FLYING" } else { "DESTROYED" }.to_string();
        }

        // 4.5 回填航天器实体视图（与 space.spacecraft 平行——按实体注册顺序匹配）
        {
            let spacecraft = &self.space.spacecraft;
            let mut craft_idx = 0usize;
            for ent in self.entities.iter_mut() {
                if ent.kind == EntityKind::Spacecraft {
                    if let Some(c) = spacecraft.get(craft_idx) {
                        ent.sync(c.position, c.velocity, self.earth_radius);
                        ent.status = "IN ORBIT".to_string();
                    }
                    craft_idx += 1;
                }
            }
        }

        // 5. 雷达探测
        self.update_radar();

        // 6. 命中判定（拦截导弹 vs 弹道目标）
        self.check_hits();
    }

    fn missile_entity_id(&self, missile_idx: usize) -> u64 {
        self.missile_entity_ids.get(missile_idx).copied().unwrap_or(0)
    }

    /// 雷达探测：拦截雷达扫描弹道目标，更新事件
    fn update_radar(&mut self) {
        for (ent, _) in &self.ballistic {
            if !ent.alive {
                continue;
            }
            let dist = ent.position.length() - self.earth_radius;
            // 探测目标 RCS 按 1.0 m² 估算
            let p = self.radar.detection_probability(dist, 1.0);
            if p > 0.5 {
                let already = self
                    .events
                    .iter()
                    .any(|e| e.kind == EventKind::Detect && e.text.contains(&ent.name));
                if !already {
                    self.events.push(WorldEvent::new(
                        self.time(),
                        EventKind::Detect,
                        format!("雷达锁定 {} (alt={:.0} km)", ent.name, dist / 1000.0),
                    ));
                }
            }
        }
    }

    /// 命中判定：拦截导弹与弹道目标距离 < 杀伤半径 → 目标摧毁
    fn check_hits(&mut self) {
        let mut destroyed: Vec<u64> = Vec::new();
        for ms in &self.missiles {
            for (ent, _) in &self.ballistic {
                if ent.alive && ms.check_hit(&ent.position) {
                    destroyed.push(ent.id);
                }
            }
        }
        if !destroyed.is_empty() {
            let time = self.time();
            for id in destroyed {
                if let Some((ent, _)) = self.ballistic.iter_mut().find(|(e, _)| e.id == id) {
                    if !ent.alive {
                        continue;
                    }
                    ent.alive = false;
                    let name = ent.name.clone();
                    self.events.push(WorldEvent::new(
                        time,
                        EventKind::Hit,
                        format!("{} 被拦截摧毁 (T+{:.1}s)", name, time),
                    ));
                }
            }
        }
        // 结局判定：存在命中事件且所有目标非存活 → 拦截成功
        if !self.ballistic.is_empty()
            && self.ballistic.iter().all(|(e, _)| !e.alive)
            && !self
                .events
                .iter()
                .any(|e| e.kind == EventKind::Outcome)
        {
            let hit_count = self
                .events
                .iter()
                .filter(|e| e.kind == EventKind::Hit)
                .count();
            let text = if hit_count > 0 {
                format!("INTERCEPT SUCCESS — 全部目标已摧毁 (T+{:.1}s)", self.time())
            } else {
                format!("MISSION FAIL — 目标坠地未被拦截 (T+{:.1}s)", self.time())
            };
            self.events.push(WorldEvent::new(
                self.time(),
                EventKind::Outcome,
                text,
            ));
        }
    }

    fn alloc_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
}

/// 弹道目标简化推进：地心引力 + 大气阻力 + 程序推力
fn step_ballistic(
    ent: &mut Entity,
    cfg: &BallisticConfig,
    dt: f64,
    time: f64,
    atmosphere: &Atmosphere,
    earth_radius: f64,
) {
    let r = ent.position;
    let dist = r.length();
    // 坠地判定：低于地表 → 结束多飞行，不再推进
    if dist <= earth_radius {
        ent.alive = false;
        return;
    }
    if dist < 1e-9 {
        return;
    }
    // 地心引力
    let acc = -r * (G * 5.9722e24 / (dist * dist * dist));
    let mut a = acc;

    // 大气阻力（US Std 1976 密度）
    let alt = (dist - earth_radius).max(0.0);
    let rho = atmosphere.get_density(alt);
    let speed = ent.velocity.length();
    if speed > 1.0 && rho > 0.0 {
        let q = 0.5 * rho * speed * speed;
        let drag = q * cfg.ref_area_m2 * cfg.cd;
        a = a - ent.velocity.normalized() * (drag / cfg.mass);
    }

    // 程序推力（前 thrust_duration_s）
    if cfg.thrust_n > 0.0 && time < cfg.thrust_duration_s {
        a = a + ent.velocity.normalized() * (cfg.thrust_n / cfg.mass);
    }

    ent.velocity = ent.velocity + a * dt;
    ent.position = ent.position + ent.velocity * dt;
    ent.acceleration = a;
}

impl Default for World {
    fn default() -> Self {
        let mut sys = GravitationalSystem::new(1e6);
        // 地球固定在原点（演示用简化：不绕日公转）
        sys.add_body(crate::physics::GravBody::new(
            "Earth",
            5.9722e24,
            6_371_000.0,
            Vec3::zero(),
            Vec3::zero(),
        ));
        World::new(sys, 0.1, 6_371_000.0)
    }
}

// =====================================================================
// 测试
// =====================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn test_world() -> World {
        World::default()
    }

    #[test]
    fn world_add_ballistic_registers_entity() {
        let mut w = test_world();
        let cfg = BallisticConfig {
            name: "TGT-1".into(),
            position: Vec3::new(0.0, 6_500_000.0, 0.0),
            velocity: Vec3::new(1500.0, 500.0, 0.0),
            mass: 1000.0,
            ref_area_m2: 0.5,
            cd: 0.2,
            thrust_n: 0.0,
            thrust_duration_s: 0.0,
        };
        let id = w.add_ballistic(cfg);
        assert_eq!(w.entities.iter().filter(|e| e.kind == EntityKind::Icbm).count(), 0);
        assert_eq!(w.ballistic.len(), 1);
        assert_eq!(w.ballistic[0].0.id, id);
        assert!(w.ballistic[0].0.alive);
    }

    #[test]
    fn world_ballistic_moves_under_gravity() {
        let mut w = test_world();
        let cfg = BallisticConfig {
            name: "TGT-2".into(),
            position: Vec3::new(0.0, 6_500_000.0, 0.0),
            velocity: Vec3::new(3000.0, 1000.0, 0.0),
            mass: 1000.0,
            ref_area_m2: 0.5,
            cd: 0.2,
            thrust_n: 0.0,
            thrust_duration_s: 0.0,
        };
        w.add_ballistic(cfg);
        let p0 = w.ballistic[0].0.position;
        for _ in 0..10 {
            w.step();
        }
        let p1 = w.ballistic[0].0.position;
        assert!((p1 - p0).length() > 0.0);
        assert!(w.time() > 0.0);
    }

    #[test]
    fn world_fire_interceptor_destroys_target() {
        let mut w = test_world();
        // 目标：静止在高空（简化让拦截易命中）
        let cfg = BallisticConfig {
            name: "TGT-3".into(),
            position: Vec3::new(0.0, 6_771_000.0, 0.0),
            velocity: Vec3::new(0.0, 0.0, 0.0),
            mass: 1000.0,
            ref_area_m2: 0.5,
            cd: 0.2,
            thrust_n: 0.0,
            thrust_duration_s: 0.0,
        };
        let tid = w.add_ballistic(cfg);
        // 拦截导弹从 45km 高处（目标下方）发射，THAAD 高超声速追击
        let start = Vec3::new(0.0, 6_446_000.0, 0.0);
        let vel = Vec3::new(0.0, 2500.0, 0.0);
        let mid = w.fire_interceptor(AamConfig::interceptor(), start, vel, tid);
        assert!(mid.is_some());

        // 推进直到命中或超时
        let mut hit = false;
        for _ in 0..4000 {
            w.step();
            if !w.ballistic[0].0.alive && w.events.iter().any(|e| e.kind == EventKind::Hit) {
                hit = true;
                break;
            }
            if w.time() > 180.0 {
                break;
            }
        }
        assert!(hit, "目标应被拦截");
        assert!(
            w.events
                .iter()
                .any(|e| e.kind == EventKind::Outcome),
            "应有结局事件"
        );
    }
}
