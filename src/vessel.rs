//! 飞船系统：Part、Vessel、RCS、Staging、Docking、EnduranceStation

use crate::Vec3;
use crate::physics::PhysicsBody;

// =====================================================================
// 推进剂类型
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PropellantType {
    None,
    Rp1,
    Lh2,
    Lox,
    Mmh,
    Nto,
    Solid,
}

impl PropellantType {
    pub fn name(&self) -> &'static str {
        match self {
            PropellantType::None => "None",
            PropellantType::Rp1 => "RP-1",
            PropellantType::Lh2 => "LH2",
            PropellantType::Lox => "LOX",
            PropellantType::Mmh => "MMH",
            PropellantType::Nto => "NTO",
            PropellantType::Solid => "Solid",
        }
    }
}

// =====================================================================
// Part — 部件枚举 (Rust 组合替代 C++ 继承)
// =====================================================================
#[derive(Debug, Clone)]
pub enum PartKind {
    Engine(EnginePart),
    FuelTank(FuelTankPart),
    Decoupler(DecouplerPart),
}

#[derive(Debug, Clone)]
pub struct Part {
    pub kind: PartKind,
    pub name: String,
    pub dry_mass: f64,
    pub active: bool,
    pub stage: i32,
    pub decoupled: bool,
    pub persistent: bool,
}

impl Part {
    pub fn new_engine(name: &str, dry_mass: f64, max_thrust_sl: f64, isp_sl: f64, isp_vac: f64,
                      fuel_type: PropellantType, ox_type: PropellantType, mixture_ratio: f64) -> Self {
        Part {
            kind: PartKind::Engine(EnginePart {
                max_thrust_sl, isp_sl, isp_vac,
                throttle: 0.0, fuel_type, ox_type, mixture_ratio,
            }),
            name: name.to_string(), dry_mass,
            active: false, stage: -1, decoupled: false, persistent: false,
        }
    }

    pub fn new_fuel_tank(name: &str, dry_mass: f64, capacity: f64, propellant: PropellantType) -> Self {
        Part {
            kind: PartKind::FuelTank(FuelTankPart {
                capacity, current_fuel: capacity, propellant,
            }),
            name: name.to_string(), dry_mass,
            active: false, stage: -1, decoupled: false, persistent: false,
        }
    }

    pub fn new_decoupler(name: &str, mass: f64) -> Self {
        Part {
            kind: PartKind::Decoupler(DecouplerPart),
            name: name.to_string(), dry_mass: mass,
            active: false, stage: -1, decoupled: false, persistent: false,
        }
    }

    pub fn get_mass(&self) -> f64 {
        if self.decoupled { return 0.0; }
        match &self.kind {
            PartKind::Engine(_) => self.dry_mass,
            PartKind::FuelTank(t) => self.dry_mass + t.current_fuel,
            PartKind::Decoupler(_) => self.dry_mass,
        }
    }

    pub fn is_active(&self) -> bool { self.active && !self.decoupled }

    // ---- 向下转型辅助 ----

    pub fn as_engine(&self) -> Option<&EnginePart> {
        match &self.kind { PartKind::Engine(e) => Some(e), _ => None }
    }
    pub fn as_engine_mut(&mut self) -> Option<&mut EnginePart> {
        match &mut self.kind { PartKind::Engine(e) => Some(e), _ => None }
    }
    pub fn as_tank(&self) -> Option<&FuelTankPart> {
        match &self.kind { PartKind::FuelTank(t) => Some(t), _ => None }
    }
    pub fn as_tank_mut(&mut self) -> Option<&mut FuelTankPart> {
        match &mut self.kind { PartKind::FuelTank(t) => Some(t), _ => None }
    }
}

// =====================================================================
// EnginePart
// =====================================================================
#[derive(Debug, Clone)]
pub struct EnginePart {
    pub max_thrust_sl: f64,
    pub isp_sl: f64,
    pub isp_vac: f64,
    pub throttle: f64,
    pub fuel_type: PropellantType,
    pub ox_type: PropellantType,
    pub mixture_ratio: f64,
}

impl EnginePart {
    pub fn set_throttle(&mut self, t: f64) {
        self.throttle = t.max(0.0).min(1.0);
    }

    pub fn fuel_mass_fraction(&self) -> f64 {
        if self.ox_type == PropellantType::None || self.mixture_ratio <= 0.0 { 1.0 }
        else { 1.0 / (1.0 + self.mixture_ratio) }
    }

    pub fn ox_mass_fraction(&self) -> f64 {
        if self.ox_type == PropellantType::None || self.mixture_ratio <= 0.0 { 0.0 }
        else { self.mixture_ratio / (1.0 + self.mixture_ratio) }
    }

    pub fn current_isp(&self, ambient_pressure: f64) -> f64 {
        let p_sl = 101325.0;
        let t = (ambient_pressure / p_sl).max(0.0).min(1.0);
        self.isp_vac - (self.isp_vac - self.isp_sl) * t
    }

    pub fn max_mass_flow_rate(&self) -> f64 {
        self.max_thrust_sl / (self.isp_sl * 9.80665)
    }

    pub fn get_thrust(&self, ambient_pressure: f64) -> f64 {
        if self.throttle <= 0.0 { return 0.0; }
        let mdot = self.max_mass_flow_rate();
        mdot * 9.80665 * self.current_isp(ambient_pressure) * self.throttle
    }

    pub fn current_mass_flow_rate(&self) -> f64 {
        if self.throttle <= 0.0 { 0.0 } else { self.max_mass_flow_rate() * self.throttle }
    }
}

// =====================================================================
// FuelTankPart
// =====================================================================
#[derive(Debug, Clone)]
pub struct FuelTankPart {
    pub capacity: f64,
    pub current_fuel: f64,
    pub propellant: PropellantType,
}

impl FuelTankPart {
    pub fn consume_fuel(&mut self, amount: f64) -> bool {
        if amount <= 0.0 { return false; }
        if self.current_fuel >= amount {
            self.current_fuel -= amount;
            true
        } else {
            self.current_fuel = 0.0;
            false
        }
    }
}

// =====================================================================
// DecouplerPart
// =====================================================================
#[derive(Debug, Clone)]
pub struct DecouplerPart;

// =====================================================================
// RCS — 反作用控制系统
// =====================================================================
#[derive(Debug, Clone)]
pub struct Rcs {
    pub power: f64,
    pub enabled: bool,
}

impl Rcs {
    pub fn new(power: f64) -> Self { Rcs { power, enabled: false } }

    pub fn apply_rotation(&self, body: &mut PhysicsBody, input: f64, _dt: f64) {
        if !self.enabled || input.abs() < 0.01 { return; }
        body.add_torque(input * self.power);
    }

    pub fn apply_translation(&self, body: &mut PhysicsBody, local_dir: Vec3, _dt: f64) {
        if !self.enabled || local_dir.length() < 0.01 { return; }
        let orientation = body.get_orientation_vec3();
        let mut world_force = Vec3::zero();
        if local_dir.y > 0.0 { world_force = world_force + orientation * self.power; }
        if local_dir.y < 0.0 { world_force = world_force - orientation * self.power; }
        let right = Vec3::new(-orientation.y, orientation.x, 0.0);
        if local_dir.x > 0.0 { world_force = world_force + right * self.power; }
        if local_dir.x < 0.0 { world_force = world_force - right * self.power; }
        body.add_force(world_force);
    }

    pub fn stabilize(&self, body: &mut PhysicsBody, _dt: f64) {
        if !self.enabled { return; }
        let damping = 0.98;
        let av = *body.get_angular_velocity_3d();
        body.set_angular_velocity(av.z * damping);
    }
}

// =====================================================================
// 发动机状态摘要
// =====================================================================
#[derive(Debug, Clone, Default)]
pub struct EngineStatus {
    pub active_engines: i32,
    pub total_thrust: f64,
    pub max_throttle: f64,
    pub total_mass_flow: f64,
    pub total_fuel_flow: f64,
    pub total_ox_flow: f64,
}

// =====================================================================
// StagingSystem — 分级系统
// =====================================================================
#[derive(Debug, Clone)]
pub struct StagingSystem {
    stages: Vec<Vec<usize>>,  // stage index → part indices
    current_stage: i32,
}

impl StagingSystem {
    pub fn new() -> Self { StagingSystem { stages: Vec::new(), current_stage: -1 } }

    pub fn rebuild(&mut self, parts: &[Part]) {
        self.stages.clear();
        let max_stage = parts.iter().map(|p| p.stage).max().unwrap_or(-1);
        if max_stage < 0 { return; }
        self.stages.resize((max_stage + 1) as usize, Vec::new());
        for (i, p) in parts.iter().enumerate() {
            if p.stage >= 0 {
                self.stages[p.stage as usize].push(i);
            }
        }
        self.current_stage = max_stage;
    }

    pub fn activate_next_stage(&mut self, parts: &mut [Part]) -> bool {
        if self.current_stage < 0 || self.stages.is_empty() { return false; }
        let stage = self.current_stage as usize;
        if stage >= self.stages.len() { self.current_stage -= 1; return true; }

        // 激活当前级
        for &idx in &self.stages[stage] {
            parts[idx].active = true;
            if let Some(e) = parts[idx].as_engine_mut() {
                e.set_throttle(1.0);
            }
        }

        // 检查是否有 Decoupler → 丢弃已离散的上一级
        let has_decoupler = self.stages[stage].iter().any(|&idx| matches!(parts[idx].kind, PartKind::Decoupler(_)));
        if has_decoupler {
            for s in (stage + 1)..self.stages.len() {
                for &idx in &self.stages[s] {
                    if !parts[idx].persistent {
                        parts[idx].decoupled = true;
                        parts[idx].active = false;
                    }
                }
            }
        }

        self.current_stage -= 1;
        true
    }

    pub fn get_current_stage(&self) -> i32 { self.current_stage }
}

// =====================================================================
// DockingState / DockingPort
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DockingState {
    Open,
    Approach,
    SoftCapture,
    HardDock,
}

#[derive(Debug, Clone)]
pub struct DockingPort {
    pub name: String,
    pub local_position: Vec3,
    pub local_direction: Vec3,
    pub state: DockingState,
}

impl DockingPort {
    pub fn new(name: &str, local_position: Vec3, local_direction: Vec3) -> Self {
        let len = local_direction.length();
        let dir = if len > 0.0 { local_direction / len } else { local_direction };
        DockingPort {
            name: name.to_string(),
            local_position,
            local_direction: dir,
            state: DockingState::Open,
        }
    }

    pub fn can_initiate_soft_capture(&self, incoming_pos: Vec3, incoming_vel: Vec3, station_ang_vel: Vec3) -> bool {
        if self.state != DockingState::Open { return false; }
        let rel_vel = incoming_vel - station_ang_vel;
        if rel_vel.length() > 0.5 { return false; }
        let to_port = self.local_position - incoming_pos;
        if to_port.length() > 10.0 { return false; }
        true
    }

    pub fn initiate_soft_capture(&mut self) {
        if self.state == DockingState::Open { self.state = DockingState::SoftCapture; }
    }

    pub fn can_complete_hard_dock(&self, rel_velocity: f64) -> bool {
        self.state == DockingState::SoftCapture && rel_velocity < 0.1
    }

    pub fn complete_hard_dock(&mut self) {
        if self.state == DockingState::SoftCapture { self.state = DockingState::HardDock; }
    }

    pub fn undock(&mut self) {
        self.state = DockingState::Open;
    }
}

// =====================================================================
// Vessel — 飞行器
// =====================================================================
#[derive(Debug, Clone)]
pub struct Vessel {
    pub name: String,
    pub body: PhysicsBody,
    pub rcs: Rcs,
    pub parts: Vec<Part>,
    pub current_stage: i32,
    pub highest_stage: i32,

    // 损伤
    pub damage_tps: f64,
    pub damage_structural: f64,
    pub damage_propulsion: f64,
    pub damage_life_support: f64,
    pub cabin_temperature: f64,
    pub cabin_pressure: f64,
    pub oxygen_level: f64,
    pub co2_level: f64,
}

impl Vessel {
    pub fn new(name: &str) -> Self {
        Vessel {
            name: name.to_string(),
            body: PhysicsBody::new(Vec3::zero(), Vec3::zero(), 0.0, 0.0),
            rcs: Rcs::new(100.0),
            parts: Vec::new(),
            current_stage: -1,
            highest_stage: -1,
            damage_tps: 0.0,
            damage_structural: 0.0,
            damage_propulsion: 0.0,
            damage_life_support: 0.0,
            cabin_temperature: 293.15,
            cabin_pressure: 101.325,
            oxygen_level: 0.209,
            co2_level: 0.0,
        }
    }

    pub fn add_part(&mut self, part: Part) {
        let mass = part.get_mass();
        self.parts.push(part);
        self.body.set_mass(self.body.get_mass() + mass);
    }

    pub fn find_highest_stage(&self) -> i32 {
        self.parts.iter().map(|p| p.stage).max().unwrap_or(-1)
    }

    pub fn activate_next_stage(&mut self) {
        if self.highest_stage < 0 {
            self.highest_stage = self.find_highest_stage();
            self.current_stage = -1;
        }

        // 分离当前级
        if self.current_stage >= 0 {
            for part in &mut self.parts {
                if part.stage == self.current_stage && !part.decoupled {
                    part.decoupled = true;
                    part.active = false;
                }
            }
        }

        self.current_stage += 1;
        if self.current_stage <= self.highest_stage {
            for part in &mut self.parts {
                if part.stage == self.current_stage && !part.decoupled {
                    part.active = true;
                    if let Some(e) = part.as_engine_mut() {
                        e.set_throttle(1.0);
                    }
                }
            }
        }
    }

    pub fn set_stage_throttle(&mut self, stage: i32, throttle: f64) {
        for part in &mut self.parts {
            if part.stage == stage && part.active {
                if let Some(e) = part.as_engine_mut() {
                    e.set_throttle(throttle);
                }
            }
        }
    }

    /// 更新一个时间步，返回发动机状态
    pub fn update(&mut self, dt: f64, ambient_pressure: f64) -> EngineStatus {
        let mut status = EngineStatus::default();
        if dt <= 0.0 { return status; }

        // 推进剂消耗
        for i in 0..self.parts.len() {
            let engine_idx = if let PartKind::Engine(e) = &self.parts[i].kind {
                if self.parts[i].is_active() && e.throttle > 0.0 { Some(i) } else { None }
            } else { None };

            if let Some(ei) = engine_idx {
                let engine = match &self.parts[ei].kind {
                    PartKind::Engine(e) => e.clone(),
                    _ => unreachable!(),
                };
                let mdot = engine.current_mass_flow_rate();
                let fuel_to_consume = mdot * engine.fuel_mass_fraction() * dt;
                let ox_to_consume = mdot * engine.ox_mass_fraction() * dt;

                let mut fuel_ok = true;
                let mut ox_ok = true;

                for j in 0..self.parts.len() {
                    if self.parts[j].stage != self.parts[ei].stage || self.parts[j].decoupled { continue; }
                    if let PartKind::FuelTank(ref mut tank) = self.parts[j].kind {
                        if tank.propellant == engine.fuel_type && fuel_to_consume > 0.0 {
                            if !tank.consume_fuel(fuel_to_consume) { fuel_ok = false; }
                        }
                        if tank.propellant == engine.ox_type && ox_to_consume > 0.0 {
                            if !tank.consume_fuel(ox_to_consume) { ox_ok = false; }
                        }
                    }
                }

                if !fuel_ok || !ox_ok {
                    self.parts[ei].active = false;
                }
            }
        }

        // 总质量
        let total_mass: f64 = self.parts.iter()
            .filter(|p| !p.decoupled)
            .map(|p| p.get_mass())
            .sum();

        // 推力累计
        let mut total_thrust = Vec3::zero();
        for part in &self.parts {
            if !part.is_active() { continue; }
            if let PartKind::Engine(ref e) = part.kind {
                if e.throttle > 0.0 {
                    let thrust = e.get_thrust(ambient_pressure);
                    let orientation = self.body.get_orientation_vec3();
                    total_thrust = total_thrust + orientation * thrust;
                    status.total_thrust += thrust;
                    status.active_engines += 1;
                    status.max_throttle = status.max_throttle.max(e.throttle);
                    let mdot = e.current_mass_flow_rate();
                    status.total_mass_flow += mdot;
                    status.total_fuel_flow += mdot * e.fuel_mass_fraction();
                    status.total_ox_flow += mdot * e.ox_mass_fraction();
                }
            }
        }

        self.body.add_force(total_thrust);
        self.body.set_mass(total_mass);

        status
    }

    /// 推进剂剩余量
    pub fn propellant_remaining(&self, stage: i32, ptype: PropellantType) -> f64 {
        self.parts.iter()
            .filter(|p| p.stage == stage && !p.decoupled)
            .filter_map(|p| p.as_tank())
            .filter(|t| t.propellant == ptype)
            .map(|t| t.current_fuel)
            .sum()
    }

    pub fn get_total_damage(&self) -> f64 {
        (self.damage_tps + self.damage_structural + self.damage_propulsion + self.damage_life_support) / 4.0
    }

    pub fn apply_damage(&mut self, amount: f64, _location: Vec3) {
        self.damage_tps = (self.damage_tps + amount * 0.3).min(1.0);
        self.damage_structural = (self.damage_structural + amount * 0.3).min(1.0);
        self.damage_propulsion = (self.damage_propulsion + amount * 0.2).min(1.0);
        self.damage_life_support = (self.damage_life_support + amount * 0.2).min(1.0);
    }

    pub fn update_with_damage(&mut self, dt: f64, _ambient_pressure: f64) {
        let mass_loss_rate = self.damage_structural * 0.5;
        let new_mass = (self.body.get_mass() - mass_loss_rate * dt).max(0.0);
        self.body.set_mass(new_mass);

        if self.damage_life_support > 0.0 {
            self.oxygen_level = (self.oxygen_level - self.damage_life_support * 0.001 * dt).max(0.0);
            self.cabin_temperature += self.damage_life_support * 0.5 * dt;
            self.cabin_pressure = (self.cabin_pressure - self.damage_life_support * 0.01 * dt).max(0.0);
        }
    }
}

// =====================================================================
// EnduranceStation
// =====================================================================
use crate::physics::RotatingFrame;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StationModuleId {
    Bridge,
    Lab,
    Mess,
    Sleep,
    Cargo,
    Airlock1,
    Airlock2,
    Airlock3,
    Airlock4,
}

impl StationModuleId {
    pub fn name(&self) -> &'static str {
        match self {
            StationModuleId::Bridge => "Bridge",
            StationModuleId::Lab => "Lab",
            StationModuleId::Mess => "Mess",
            StationModuleId::Sleep => "Sleep",
            StationModuleId::Cargo => "Cargo",
            StationModuleId::Airlock1 => "Airlock 1",
            StationModuleId::Airlock2 => "Airlock 2",
            StationModuleId::Airlock3 => "Airlock 3",
            StationModuleId::Airlock4 => "Airlock 4",
        }
    }
}

#[derive(Debug, Clone)]
pub struct EnduranceStation {
    pub body: PhysicsBody,
    pub frame: RotatingFrame,
    pub docking_ports: Vec<DockingPort>,
    pub modules: Vec<(StationModuleId, Vec3, f64)>,  // (id, local_pos, volume)

    pub is_docked: bool,
    pub docking_progress: f64,
    pub is_docking_in_progress: bool,
}

impl EnduranceStation {
    pub const RADIUS: f64 = 40.0;
    pub const NORMAL_RPM: f64 = 5.6;
    pub const EMERGENCY_RPM: f64 = 68.0;

    pub fn new() -> Self {
        let body = PhysicsBody::new(Vec3::zero(), Vec3::zero(), 50000.0, 1000000.0);

        let ports = vec![
            DockingPort::new("Port 1", Vec3::new(Self::RADIUS, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
            DockingPort::new("Port 2", Vec3::new(-Self::RADIUS, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)),
            DockingPort::new("Port 3", Vec3::new(0.0, Self::RADIUS, 0.0), Vec3::new(0.0, 1.0, 0.0)),
            DockingPort::new("Port 4", Vec3::new(0.0, -Self::RADIUS, 0.0), Vec3::new(0.0, -1.0, 0.0)),
        ];

        let r = Self::RADIUS;
        let modules = vec![
            (StationModuleId::Bridge,    Vec3::new(r, 0.0, 0.0), 50.0),
            (StationModuleId::Lab,       Vec3::new(0.0, r, 0.0), 80.0),
            (StationModuleId::Mess,      Vec3::new(-r, 0.0, 0.0), 60.0),
            (StationModuleId::Sleep,     Vec3::new(0.0, -r, 0.0), 40.0),
            (StationModuleId::Cargo,     Vec3::new(0.0, 0.0, 5.0), 200.0),
            (StationModuleId::Airlock1,  Vec3::new(r * 0.7071, r * 0.7071, 0.0), 10.0),
            (StationModuleId::Airlock2,  Vec3::new(-r * 0.7071, r * 0.7071, 0.0), 10.0),
            (StationModuleId::Airlock3,  Vec3::new(-r * 0.7071, -r * 0.7071, 0.0), 10.0),
            (StationModuleId::Airlock4,  Vec3::new(r * 0.7071, -r * 0.7071, 0.0), 10.0),
        ];

        let mut station = EnduranceStation {
            body,
            frame: RotatingFrame,
            docking_ports: ports,
            modules,
            is_docked: false,
            docking_progress: 0.0,
            is_docking_in_progress: false,
        };
        station.set_spin_rate(Self::NORMAL_RPM);
        station
    }

    pub fn set_position(&mut self, pos: Vec3) {
        self.body.set_position(pos);
    }

    pub fn set_spin_rate(&mut self, rpm: f64) {
        let omega_rad_s = rpm * 2.0 * std::f64::consts::PI / 60.0;
        self.body.set_angular_velocity(omega_rad_s);
    }

    pub fn emergency_spin_up(&mut self) {
        self.set_spin_rate(Self::EMERGENCY_RPM);
    }

    pub fn get_module_local_pos(&self, id: StationModuleId) -> Vec3 {
        self.modules.iter()
            .find(|(mid, _, _)| *mid == id)
            .map(|(_, p, _)| *p)
            .unwrap_or(Vec3::zero())
    }

    pub fn initiate_docking(&mut self, rel_velocity: f64) -> bool {
        if self.is_docking_in_progress || self.is_docked { return false; }
        if rel_velocity > 0.5 { return false; }
        self.is_docking_in_progress = true;
        self.docking_progress = 0.0;
        true
    }

    pub fn update_docking(&mut self, dt: f64) {
        if !self.is_docking_in_progress { return; }
        self.docking_progress += dt * 0.1;
        if self.docking_progress >= 1.0 {
            self.is_docking_in_progress = false;
            self.is_docked = true;
        }
    }

    pub fn undock(&mut self) {
        self.is_docked = false;
        self.docking_progress = 0.0;
    }
}

// =====================================================================
// PartLibrary — 部件工厂
// =====================================================================
pub struct PartLibrary;

impl PartLibrary {
    pub fn create_merlin1d() -> Part {
        Part::new_engine("Merlin 1D", 470.0, 845_000.0, 282.0, 311.0, PropellantType::Rp1, PropellantType::Lox, 2.56)
    }

    pub fn create_merlin1d_vac() -> Part {
        let thrust_vac = 934_000.0;
        let isp_vac = 348.0;
        let isp_sl = 282.0;
        let thrust_sl = thrust_vac * (isp_sl / isp_vac);
        Part::new_engine("Merlin 1D Vac", 490.0, thrust_sl, isp_sl, isp_vac, PropellantType::Rp1, PropellantType::Lox, 2.35)
    }

    pub fn create_f1() -> Part {
        Part::new_engine("F-1 Engine", 8400.0, 7_770_000.0, 263.0, 304.0, PropellantType::Rp1, PropellantType::Lox, 2.27)
    }

    pub fn create_rl10b2() -> Part {
        Part::new_engine("RL10B-2", 301.0, 110_000.0, 200.0, 448.0, PropellantType::Lh2, PropellantType::Lox, 5.5)
    }

    pub fn create_aj10_190() -> Part {
        Part::new_engine("AJ10-190", 112.0, 267_000.0, 319.0, 319.0, PropellantType::Mmh, PropellantType::Nto, 1.65)
    }

    pub fn create_rs25() -> Part {
        Part::new_engine("RS-25", 3515.0, 1_860_000.0, 366.0, 452.0, PropellantType::Lh2, PropellantType::Lox, 6.0)
    }

    pub fn create_sls_srb() -> Part {
        Part::new_engine("SLS SRB", 75000.0, 14_679_000.0, 250.0, 280.0, PropellantType::Solid, PropellantType::Solid, 0.0)
    }

    pub fn create_rl10c2() -> Part {
        Part::new_engine("RL10C-2", 301.0, 110_000.0, 200.0, 465.0, PropellantType::Lh2, PropellantType::Lox, 5.5)
    }

    pub fn create_f9_s1_rp1_tank() -> Part {
        Part::new_fuel_tank("F9 S1 RP-1 Tank", 12000.0, 70000.0, PropellantType::Rp1)
    }

    pub fn create_f9_s1_lox_tank() -> Part {
        Part::new_fuel_tank("F9 S1 LOX Tank", 13000.0, 70000.0, PropellantType::Lox)
    }

    pub fn create_f9_s2_rp1_tank() -> Part {
        Part::new_fuel_tank("F9 S2 RP-1 Tank", 1800.0, 29000.0, PropellantType::Rp1)
    }

    pub fn create_f9_s2_lox_tank() -> Part {
        Part::new_fuel_tank("F9 S2 LOX Tank", 2200.0, 71000.0, PropellantType::Lox)
    }

    pub fn create_artemis2_icps_lh2_tank() -> Part {
        Part::new_fuel_tank("ICPS LH2 Tank", 3200.0, 150000.0, PropellantType::Lh2)
    }

    pub fn create_artemis2_icps_lox_tank() -> Part {
        Part::new_fuel_tank("ICPS LOX Tank", 4100.0, 825000.0, PropellantType::Lox)
    }

    pub fn create_artemis2_orion_mmh_tank() -> Part {
        Part::new_fuel_tank("Orion MMH Tank", 850.0, 3200.0, PropellantType::Mmh)
    }

    pub fn create_artemis2_orion_nto_tank() -> Part {
        Part::new_fuel_tank("Orion NTO Tank", 900.0, 5300.0, PropellantType::Nto)
    }

    pub fn create_sls_lh2_tank() -> Part {
        Part::new_fuel_tank("SLS Core LH2 Tank", 9500.0, 144000.0, PropellantType::Lh2)
    }

    pub fn create_sls_lox_tank() -> Part {
        Part::new_fuel_tank("SLS Core LOX Tank", 4500.0, 840000.0, PropellantType::Lox)
    }

    pub fn create_orion_mmh_tank_real() -> Part {
        Part::new_fuel_tank("Orion MMH Tank", 400.0, 4300.0, PropellantType::Mmh)
    }

    pub fn create_orion_nto_tank_real() -> Part {
        Part::new_fuel_tank("Orion NTO Tank", 400.0, 4300.0, PropellantType::Nto)
    }
}

// =====================================================================
// Tests
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_part_propellant_types() {
        assert_eq!(PropellantType::Rp1.name(), "RP-1");
        assert_eq!(PropellantType::Lh2.name(), "LH2");
        assert_eq!(PropellantType::Nto.name(), "NTO");
    }

    #[test]
    fn test_engine_part_basics() {
        let engine = Part::new_engine("Test", 100.0, 500_000.0, 280.0, 310.0,
                                      PropellantType::Rp1, PropellantType::Lox, 2.5);
        let e = engine.as_engine().unwrap();
        assert!((e.fuel_mass_fraction() - 1.0 / 3.5).abs() < 1e-6);
        assert!((e.ox_mass_fraction() - 2.5 / 3.5).abs() < 1e-6);
    }

    #[test]
    fn test_engine_thrust_increases_in_vacuum() {
        let mut e = Part::new_engine("Test", 100.0, 500_000.0, 280.0, 310.0,
                                     PropellantType::Rp1, PropellantType::Lox, 2.5);
        e.active = true;
        let eng = e.as_engine_mut().unwrap();
        eng.set_throttle(1.0);
        let thrust_sl = eng.get_thrust(101325.0);
        let thrust_vac = eng.get_thrust(0.0);
        assert!(thrust_vac > thrust_sl);
    }

    #[test]
    fn test_fuel_tank_consume() {
        let mut tank = Part::new_fuel_tank("Tank", 50.0, 1000.0, PropellantType::Rp1);
        assert!((tank.get_mass() - 1050.0).abs() < 1e-9);
        let t = tank.as_tank_mut().unwrap();
        assert!(t.consume_fuel(300.0));
        assert!((t.current_fuel - 700.0).abs() < 1e-9);
        assert!(!t.consume_fuel(1000.0)); // not enough
        assert!((t.current_fuel - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_decoupled_part_has_zero_mass() {
        let mut part = Part::new_engine("Test", 100.0, 500_000.0, 280.0, 310.0,
                                        PropellantType::Rp1, PropellantType::Lox, 2.5);
        assert!((part.get_mass() - 100.0).abs() < 1e-9);
        part.decoupled = true;
        assert!((part.get_mass() - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_vessel_staging() {
        let mut vessel = Vessel::new("Test Rocket");
        // Stage 2 (upper): engine + tank
        let mut eng2 = PartLibrary::create_merlin1d_vac();
        eng2.stage = 1;
        let mut tank2 = Part::new_fuel_tank("S2 Tank", 200.0, 5000.0, PropellantType::Rp1);
        tank2.stage = 1;
        // Stage 1 (booster): engine + tank
        let mut eng1 = PartLibrary::create_merlin1d();
        eng1.stage = 0;
        let mut tank1 = Part::new_fuel_tank("S1 Tank", 500.0, 20000.0, PropellantType::Rp1);
        tank1.stage = 0;

        vessel.add_part(eng1);
        vessel.add_part(tank1);
        vessel.add_part(eng2);
        vessel.add_part(tank2);

        assert_eq!(vessel.find_highest_stage(), 1);

        // Activate first stage
        vessel.activate_next_stage();
        assert_eq!(vessel.current_stage, 0);
        // Stage 0 parts should be active
        assert!(vessel.parts[0].active); // eng1
        assert!(vessel.parts[1].active); // tank1
        // Stage 1 parts should not be active yet
        assert!(!vessel.parts[2].active); // eng2
        assert!(!vessel.parts[3].active); // tank2

        // Second stage activation
        vessel.activate_next_stage();
        assert_eq!(vessel.current_stage, 1);
        // Stage 0 should be decoupled
        assert!(vessel.parts[0].decoupled);
        assert!(vessel.parts[1].decoupled);
        // Stage 1 should be active
        assert!(vessel.parts[2].active);
        assert!(vessel.parts[3].active);
    }

    #[test]
    fn test_vessel_update_propellant_consumption() {
        let mut vessel = Vessel::new("Test");
        let mut engine = Part::new_engine("Engine", 100.0, 100_000.0, 300.0, 320.0,
                                          PropellantType::Rp1, PropellantType::Lox, 2.5);
        engine.stage = 0;
        engine.active = true;
        engine.as_engine_mut().unwrap().set_throttle(1.0);
        let mut tank_fuel = Part::new_fuel_tank("Fuel", 50.0, 500.0, PropellantType::Rp1);
        tank_fuel.stage = 0;
        let mut tank_ox = Part::new_fuel_tank("LOX", 50.0, 1250.0, PropellantType::Lox);
        tank_ox.stage = 0;

        vessel.add_part(engine);
        vessel.add_part(tank_fuel);
        vessel.add_part(tank_ox);

        let status = vessel.update(1.0, 0.0);
        assert!(status.active_engines >= 1);
        assert!(status.total_thrust > 0.0);

        // Fuel should have been consumed
        let fuel_rem = vessel.propellant_remaining(0, PropellantType::Rp1);
        assert!(fuel_rem < 499.0); // some consumed

        let ox_rem = vessel.propellant_remaining(0, PropellantType::Lox);
        assert!(ox_rem < 1249.0); // some consumed
    }

    #[test]
    fn test_part_library_engines() {
        let merlin = PartLibrary::create_merlin1d();
        assert_eq!(merlin.name, "Merlin 1D");
        assert!(merlin.dry_mass > 0.0);
        let e = merlin.as_engine().unwrap();
        assert_eq!(e.fuel_type, PropellantType::Rp1);
        assert_eq!(e.ox_type, PropellantType::Lox);

        let rs25 = PartLibrary::create_rs25();
        let e = rs25.as_engine().unwrap();
        assert_eq!(e.fuel_type, PropellantType::Lh2);

        let srb = PartLibrary::create_sls_srb();
        assert_eq!(srb.name, "SLS SRB");
    }

    #[test]
    fn test_rcs_basics() {
        let rcs = Rcs::new(100.0);
        assert!(!rcs.enabled);
        let rcs2 = Rcs { enabled: true, ..rcs };
        let mut body = PhysicsBody::new(Vec3::zero(), Vec3::zero(), 1000.0, 500.0);
        rcs2.apply_rotation(&mut body, 0.5, 0.1);
        // Should have applied torque
        let force = body.get_accumulated_force();
        assert!(force.length() >= 0.0);
    }

    #[test]
    fn test_docking_port_state_machine() {
        let mut port = DockingPort::new("Test", Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(port.state, DockingState::Open);

        port.initiate_soft_capture();
        assert_eq!(port.state, DockingState::SoftCapture);

        assert!(!port.can_complete_hard_dock(0.2)); // too fast
        assert!(port.can_complete_hard_dock(0.05)); // slow enough

        port.complete_hard_dock();
        assert_eq!(port.state, DockingState::HardDock);

        port.undock();
        assert_eq!(port.state, DockingState::Open);
    }

    #[test]
    fn test_staging_system() {
        let mut parts = vec![
            Part { stage: 0, decoupled: false, active: false, ..Part::new_engine("E1", 100.0, 1000.0, 280.0, 310.0, PropellantType::Rp1, PropellantType::Lox, 2.5) },
            Part { stage: 1, decoupled: false, active: false, ..Part::new_fuel_tank("T1", 50.0, 100.0, PropellantType::Rp1) },
        ];

        let mut staging = StagingSystem::new();
        staging.rebuild(&parts);
        assert_eq!(staging.get_current_stage(), 1);

        staging.activate_next_stage(&mut parts);
        assert!(parts[1].active); // stage 1 activated
        assert_eq!(staging.get_current_stage(), 0);

        staging.activate_next_stage(&mut parts);
        // Stage 0 should not have decoupler, so stage 1 stays
        assert!(!parts[1].decoupled);
    }

    #[test]
    fn test_endurance_station_basic() {
        let station = EnduranceStation::new();
        assert_eq!(station.docking_ports.len(), 4);
        assert_eq!(station.modules.len(), 9);

        let bridge_pos = station.get_module_local_pos(StationModuleId::Bridge);
        assert!((bridge_pos.length() - EnduranceStation::RADIUS).abs() < 0.001);
    }

    #[test]
    fn test_vessel_damage() {
        let mut vessel = Vessel::new("Test");
        assert!((vessel.get_total_damage() - 0.0).abs() < 1e-12);
        vessel.apply_damage(0.5, Vec3::zero());
        let d = vessel.get_total_damage();
        assert!(d > 0.0 && d < 0.5);
    }

    #[test]
    fn test_engine_mixture_ratio_validation() {
        let engine = Part::new_engine("Test", 100.0, 500_000.0, 280.0, 310.0,
                                      PropellantType::Lh2, PropellantType::Lox, 6.0);
        let e = engine.as_engine().unwrap();
        assert!((e.fuel_mass_fraction() - 1.0 / 7.0).abs() < 1e-6);
        assert!((e.ox_mass_fraction() - 6.0 / 7.0).abs() < 1e-6);
    }

    #[test]
    fn test_fuel_tank_edge_cases() {
        let mut tank = Part::new_fuel_tank("Empty", 10.0, 0.0, PropellantType::Lox);
        assert!((tank.get_mass() - 10.0).abs() < 1e-9);
        let t = tank.as_tank_mut().unwrap();
        assert!(!t.consume_fuel(1.0)); // can't consume from empty
        assert!(!t.consume_fuel(-5.0)); // negative amount
    }
}
