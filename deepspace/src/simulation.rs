//! 任务系统：任务数据、配置、控制、脚本、Artemis2 与深空任务规划

use crate::Vec3;
use crate::vessel::{Vessel, EngineStatus};
use crate::physics::OrbitalMechanics;
use crate::environment::Planet;

// =====================================================================
// 任务相位 (发射阶段)
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MissionPhase {
    PreLaunch,
    Launch,
    Ascent,
    MaxQ,
    Staging,
    Coast,
    Circularization,
    Orbit,
    Tei,
    Translunar,
    MissionEvents,
    Reentry,
    Success,
    Failure,
    Abort,
}

impl MissionPhase {
    pub fn to_str(&self) -> &'static str {
        match self {
            MissionPhase::PreLaunch => "PRE_LAUNCH",
            MissionPhase::Launch => "LAUNCH",
            MissionPhase::Ascent => "ASCENT",
            MissionPhase::MaxQ => "MAX_Q",
            MissionPhase::Staging => "STAGING",
            MissionPhase::Coast => "COAST",
            MissionPhase::Circularization => "CIRCULARIZATION",
            MissionPhase::Orbit => "ORBIT",
            MissionPhase::Tei => "TEI",
            MissionPhase::Translunar => "TRANSLUNAR",
            MissionPhase::MissionEvents => "MISSION_EVENTS",
            MissionPhase::Reentry => "REENTRY",
            MissionPhase::Success => "SUCCESS",
            MissionPhase::Failure => "FAILURE",
            MissionPhase::Abort => "ABORT",
        }
    }
}

// =====================================================================
// 深空任务相位 (DeepSpaceMissionPlanner)
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeepPhase {
    PreLaunch,
    EarthOrbit,
    TransLunarInjection,
    CoastToMoon,
    LunarOrbitInsertion,
    Landing,
    Completed,
    Failed,
}

// =====================================================================
// 任务结果
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MissionOutcome {
    InProgress,
    Success,
    Failure,
    Abort,
    Timeout,
}

impl MissionOutcome {
    pub fn to_str(&self) -> &'static str {
        match self {
            MissionOutcome::InProgress => "IN_PROGRESS",
            MissionOutcome::Success => "SUCCESS",
            MissionOutcome::Failure => "FAILURE",
            MissionOutcome::Abort => "ABORT",
            MissionOutcome::Timeout => "TIMEOUT",
        }
    }
}

// =====================================================================
// 触发器类型
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TriggerType {
    TimeElapsed,
    AltitudeAbove,
    AltitudeBelow,
    VelocityAbove,
    VelocityBelow,
    PropellantDepleted,
    MaxqPassed,
    ApoapsisAbove,
    PeriapsisAbove,
    ApoapsisBelow,
    PeriapsisBelow,
    OrbitCircularized,
    DamageExceeded,
    StageActivated,
    EngineCutoff,
    MachAbove,
    MachBelow,
}

#[derive(Debug, Clone)]
pub struct TriggerCondition {
    pub trigger_type: TriggerType,
    pub value: f64,
    pub stage: i32,
    pub once: bool,
    pub triggered: bool,
}

impl TriggerCondition {
    pub fn new(trigger_type: TriggerType, value: f64) -> Self {
        TriggerCondition { trigger_type, value, stage: -1, once: false, triggered: false }
    }
}

// =====================================================================
// 命令类型
// =====================================================================
#[derive(Debug, Clone)]
pub enum Command {
    StageSeparation { stage: i32 },
    SetThrottle { stage: i32, value: f64 },
    SetOrientation { orientation: String },
    EnableRcs,
    LogMessage { message: String },
    CircularizationBurn,
    AbortMission { message: String },
    TriggerDamage { message: String },
    Wait { duration: f64 },
}

// =====================================================================
// 轨道状态
// =====================================================================
#[derive(Debug, Clone, Default)]
pub struct OrbitalState {
    pub apoapsis_m: f64,
    pub periapsis_m: f64,
    pub inclination_deg: f64,
    pub period_s: f64,
    pub is_bound: bool,
}

// =====================================================================
// 遥测数据
// =====================================================================
#[derive(Debug, Clone)]
pub struct TelemetryData {
    pub mission_time: f64,
    pub timestamp: f64,
    pub phase: MissionPhase,
    pub altitude_m: f64,
    pub velocity_mps: f64,
    pub mach: f64,
    pub dynamic_pressure_pa: f64,
    pub ambient_pressure_pa: f64,
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub total_mass_kg: f64,
    pub thrust_n: f64,
    pub throttle_pct: f64,
    pub mass_flow_kg_s: f64,
    pub fuel_flow_kg_s: f64,
    pub ox_flow_kg_s: f64,
    pub max_q_pa: f64,
    pub max_q_altitude_m: f64,
    pub max_q_time_s: f64,
    pub damage_total: f64,
    pub damage_tps: f64,
    pub damage_structural: f64,
    pub damage_propulsion: f64,
    pub damage_life_support: f64,
    pub survival_probability: f64,
    pub vessel_health: f64,
    pub orbit: OrbitalState,
    pub active_engines: i32,
    pub current_stage: i32,
}

impl Default for TelemetryData {
    fn default() -> Self {
        TelemetryData {
            mission_time: 0.0, timestamp: 0.0,
            phase: MissionPhase::PreLaunch,
            altitude_m: 0.0, velocity_mps: 0.0, mach: 0.0,
            dynamic_pressure_pa: 0.0, ambient_pressure_pa: 101325.0,
            position: Vec3::zero(), velocity: Vec3::zero(), acceleration: Vec3::zero(),
            total_mass_kg: 0.0, thrust_n: 0.0, throttle_pct: 0.0,
            mass_flow_kg_s: 0.0, fuel_flow_kg_s: 0.0, ox_flow_kg_s: 0.0,
            max_q_pa: 0.0, max_q_altitude_m: 0.0, max_q_time_s: 0.0,
            damage_total: 0.0, damage_tps: 0.0, damage_structural: 0.0,
            damage_propulsion: 0.0, damage_life_support: 0.0,
            survival_probability: 1.0, vessel_health: 1.0,
            orbit: OrbitalState::default(),
            active_engines: 0, current_stage: 0,
        }
    }
}

// =====================================================================
// 任务事件
// =====================================================================
#[derive(Debug, Clone)]
pub struct MissionEvent {
    pub time: f64,
    pub name: String,
    pub description: String,
    pub phase: MissionPhase,
    pub triggered: bool,
    pub triggers: Vec<TriggerCondition>,
    pub commands: Vec<Command>,
}

// =====================================================================
// 任务摘要
// =====================================================================
#[derive(Debug, Clone)]
pub struct MissionSummary {
    pub mission_name: String,
    pub start_time: String,
    pub end_time: String,
    pub duration_s: f64,
    pub outcome: MissionOutcome,
    pub max_q_pa: f64,
    pub max_q_altitude_m: f64,
    pub max_q_time_s: f64,
    pub final_orbit: OrbitalState,
    pub target_orbit: OrbitalState,
    pub staging_events: Vec<MissionEvent>,
    pub all_events: Vec<MissionEvent>,
    pub peak_acceleration_g: f64,
    pub peak_heat_flux_w_m2: f64,
    pub total_heat_load_j: f64,
}

impl Default for MissionSummary {
    fn default() -> Self {
        MissionSummary {
            mission_name: String::new(),
            start_time: String::new(),
            end_time: String::new(),
            duration_s: 0.0,
            outcome: MissionOutcome::InProgress,
            max_q_pa: 0.0, max_q_altitude_m: 0.0, max_q_time_s: 0.0,
            final_orbit: OrbitalState::default(),
            target_orbit: OrbitalState::default(),
            staging_events: Vec::new(),
            all_events: Vec::new(),
            peak_acceleration_g: 0.0,
            peak_heat_flux_w_m2: 0.0,
            total_heat_load_j: 0.0,
        }
    }
}

// =====================================================================
// 目标轨道
// =====================================================================
#[derive(Debug, Clone)]
pub struct TargetOrbit {
    pub apoapsis_km: f64,
    pub periapsis_km: f64,
    pub inclination_deg: f64,
}

impl Default for TargetOrbit {
    fn default() -> Self { TargetOrbit { apoapsis_km: 185.0, periapsis_km: 180.0, inclination_deg: 28.5 } }
}

// =====================================================================
// 退出条件
// =====================================================================
#[derive(Debug, Clone)]
pub struct ExitCondition {
    pub name: String,
    pub exit_type: String,
    pub threshold: f64,
    pub stage: i32,
    pub mandatory: bool,
}

// =====================================================================
// 任务脚本
// =====================================================================
#[derive(Debug, Clone)]
pub struct MissionScript {
    pub name: String,
    pub description: String,
    pub target_orbit: TargetOrbit,
    pub events: Vec<MissionEvent>,
    pub success_conditions: Vec<ExitCondition>,
    pub failure_conditions: Vec<ExitCondition>,
    pub abort_conditions: Vec<ExitCondition>,
    pub max_duration_s: f64,
    pub auto_mode: bool,
}

impl Default for MissionScript {
    fn default() -> Self {
        MissionScript {
            name: String::new(),
            description: String::new(),
            target_orbit: TargetOrbit::default(),
            events: Vec::new(),
            success_conditions: Vec::new(),
            failure_conditions: Vec::new(),
            abort_conditions: Vec::new(),
            max_duration_s: 7200.0,
            auto_mode: true,
        }
    }
}

// =====================================================================
// EventTriggerSystem
// =====================================================================
#[derive(Debug, Clone)]
pub struct EventTriggerSystem {
    script: MissionScript,
    events: Vec<MissionEvent>,
    triggered_events: Vec<MissionEvent>,
}

#[allow(dead_code)]
impl EventTriggerSystem {
    pub fn new() -> Self {
        EventTriggerSystem {
            script: MissionScript::default(),
            events: Vec::new(),
            triggered_events: Vec::new(),
        }
    }

    pub fn load_script(&mut self, script: &MissionScript) {
        self.script = script.clone();
        self.events.clear();
        self.triggered_events.clear();
        for evt_def in &self.script.events {
            self.events.push(MissionEvent {
                name: evt_def.name.clone(),
                description: String::new(),
                phase: MissionPhase::PreLaunch,
                time: 0.0,
                triggered: false,
                triggers: evt_def.triggers.clone(),
                commands: evt_def.commands.clone(),
            });
        }
    }

    pub fn reset(&mut self) {
        for evt in &mut self.events {
            evt.triggered = false;
        }
        self.triggered_events.clear();
    }

    pub fn check_triggers(
        &mut self,
        vessel: &Vessel,
        mission_time: f64,
        altitude: f64,
        velocity: f64,
        max_q: f64,
        damage: f64,
    ) -> Vec<Command> {
        let mut commands = Vec::new();
        let mut to_fire: Vec<usize> = Vec::new();
        let mut triggered_names: Vec<String> = Vec::new();

        // Phase 1: check conditions (immutable borrow of self)
        for (i, evt) in self.events.iter().enumerate() {
            if evt.triggered {
                continue;
            }
            let mut all_met = true;
            for cond in &evt.triggers {
                if !self.check_condition(cond, vessel, mission_time, altitude, velocity, max_q, damage) {
                    all_met = false;
                    break;
                }
            }
            if all_met {
                to_fire.push(i);
            }
        }

        // Phase 2: fire events (mutable borrow of self)
        for &i in &to_fire {
            let evt = &mut self.events[i];
            evt.triggered = true;
            evt.time = mission_time;
            triggered_names.push(evt.name.clone());
            commands.extend(evt.commands.iter().cloned());
        }
        for name in triggered_names {
            if let Some(idx) = self.events.iter().position(|e| e.name == name) {
                self.triggered_events.push(self.events[idx].clone());
            }
        }

        commands
    }

    pub fn update(
        &mut self,
        vessel: &Vessel,
        mission_time: f64,
        altitude: f64,
        velocity: f64,
        max_q: f64,
        damage: f64,
    ) -> Vec<Command> {
        self.check_triggers(vessel, mission_time, altitude, velocity, max_q, damage)
    }

    fn check_condition(
        &self,
        cond: &TriggerCondition,
        _vessel: &Vessel,
        mission_time: f64,
        altitude: f64,
        velocity: f64,
        max_q: f64,
        damage: f64,
    ) -> bool {
        if cond.once && cond.triggered { return false; }
        match cond.trigger_type {
            TriggerType::TimeElapsed => mission_time >= cond.value,
            TriggerType::AltitudeAbove => altitude > cond.value,
            TriggerType::AltitudeBelow => altitude < cond.value,
            TriggerType::VelocityAbove => velocity > cond.value,
            TriggerType::VelocityBelow => velocity < cond.value,
            TriggerType::MaxqPassed => max_q > cond.value,
            TriggerType::DamageExceeded => damage >= cond.value,
            _ => false,
        }
    }

    fn get_event_triggers(&self, event_name: &str) -> Vec<TriggerCondition> {
        for evt in &self.script.events {
            if evt.name == event_name {
                return evt.triggers.clone();
            }
        }
        Vec::new()
    }

    fn get_event_commands(&self, event_name: &str) -> Vec<Command> {
        for evt in &self.script.events {
            if evt.name == event_name {
                return evt.commands.clone();
            }
        }
        Vec::new()
    }

    pub fn get_triggered_events(&self) -> &[MissionEvent] { &self.triggered_events }
    pub fn get_pending_events(&self) -> &[MissionEvent] { &self.events }
}

// =====================================================================
// MissionConfig — INI 解析器
// =====================================================================
use std::collections::HashMap;
use std::fs;

#[derive(Debug, Clone)]
pub struct MissionConfig {
    pub mission_name: String,
    pub target_ap_km: f64,
    pub target_pe_km: f64,
    pub max_duration_s: f64,

    // SLS / Orion 引擎
    pub rs25: EngineConfig,
    pub srb: EngineConfig,
    pub rl10: EngineConfig,
    pub aj10: EngineConfig,

    // Falcon 9 引擎
    pub merlin: EngineConfig,
    pub merlin_vacuum: EngineConfig,

    // 油箱
    pub core_lh2: TankConfig,
    pub core_lox: TankConfig,
    pub srb_fuel: TankConfig,
    pub icps_lh2: TankConfig,
    pub icps_lox: TankConfig,
    pub orion_mmh: TankConfig,
    pub orion_nto: TankConfig,
    pub core_rp1: TankConfig,
    pub core_lox_old: TankConfig,
    pub second_stage_rp1: TankConfig,
    pub second_stage_lox: TankConfig,

    // 配置
    pub guidance: GuidanceConfig,
    pub launch: LaunchConfig,
    pub launch_location: LaunchLocationConfig,
    pub weather: WeatherConfig,
}

impl Default for MissionConfig {
    fn default() -> Self {
        MissionConfig {
            mission_name: String::new(),
            target_ap_km: 185.0,
            target_pe_km: 180.0,
            max_duration_s: 7200.0,
            rs25: EngineConfig::default(),
            srb: EngineConfig::default(),
            rl10: EngineConfig::default(),
            aj10: EngineConfig::default(),
            merlin: EngineConfig::default(),
            merlin_vacuum: EngineConfig::default(),
            core_lh2: TankConfig::default(),
            core_lox: TankConfig::default(),
            srb_fuel: TankConfig::default(),
            icps_lh2: TankConfig::default(),
            icps_lox: TankConfig::default(),
            orion_mmh: TankConfig::default(),
            orion_nto: TankConfig::default(),
            core_rp1: TankConfig::default(),
            core_lox_old: TankConfig::default(),
            second_stage_rp1: TankConfig::default(),
            second_stage_lox: TankConfig::default(),
            guidance: GuidanceConfig::default(),
            launch: LaunchConfig::default(),
            launch_location: LaunchLocationConfig::default(),
            weather: WeatherConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct EngineConfig {
    pub thrust_n: f64,
    pub thrust_sea_level_n: f64,
    pub thrust_vacuum_n: f64,
    pub engine_count: i32,
    pub sea_level_isp_s: f64,
    pub vacuum_isp_s: f64,
    pub fuel_ratio: f64,
    pub ox_ratio: f64,
    pub of_ratio: f64,
}

#[derive(Debug, Clone, Default)]
pub struct TankConfig {
    pub name: String,
    pub fuel_mass_kg: f64,
    pub dry_mass_kg: f64,
    pub propellant: String,
}

#[derive(Debug, Clone)]
pub struct GuidanceConfig {
    pub algorithm: String,
    pub pitch_start_alt_m: f64,
    pub pitch_end_alt_m: f64,
    pub pitch_end_angle_deg: f64,
    pub orbit_tolerance_m: f64,
}

impl Default for GuidanceConfig {
    fn default() -> Self {
        GuidanceConfig {
            algorithm: "cosine".into(),
            pitch_start_alt_m: 2000.0,
            pitch_end_alt_m: 20000.0,
            pitch_end_angle_deg: 85.0,
            orbit_tolerance_m: 10000.0,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct LaunchWindowConfig {
    pub start: String,
    pub end: String,
    pub auto_calculate: bool,
}

#[derive(Debug, Clone, Default)]
pub struct LaunchConfig {
    pub datetime: String,
    pub timezone: String,
    pub window: LaunchWindowConfig,
}

#[derive(Debug, Clone, Default)]
pub struct LaunchLocationConfig {
    pub name: String,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f64,
    pub timezone: String,
    pub pad: String,
}

#[derive(Debug, Clone)]
pub struct WeatherConfig {
    pub enabled: bool,
    pub real_time_data: bool,
    pub temperature_c: f64,
    pub humidity_pct: f64,
    pub pressure_hpa: f64,
    pub wind_speed_ms: f64,
    pub wind_direction_deg: f64,
    pub cloud_cover_pct: i32,
    pub variation_enabled: bool,
    pub random_seed: i32,
}

impl Default for WeatherConfig {
    fn default() -> Self {
        WeatherConfig {
            enabled: false, real_time_data: false,
            temperature_c: 15.0, humidity_pct: 50.0, pressure_hpa: 1013.25,
            wind_speed_ms: 0.0, wind_direction_deg: 0.0,
            cloud_cover_pct: 0, variation_enabled: false, random_seed: 0,
        }
    }
}

impl MissionConfig {
    pub fn load(path: &str) -> Result<Self, String> {
        let content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read config: {}", e))?;

        let mut sections: HashMap<String, HashMap<String, String>> = HashMap::new();
        let mut current_section = String::new();

        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') { continue; }

            if line.starts_with('[') && line.ends_with(']') {
                current_section = line[1..line.len()-1].to_string();
                sections.entry(current_section.clone()).or_default();
                continue;
            }

            if let Some(eq) = line.find('=') {
                let key = line[..eq].trim().to_string();
                let val = line[eq+1..].trim().to_string();
                sections.entry(current_section.clone()).or_default().insert(key, val);
            }
        }

        let mut config = MissionConfig::default();

        // [mission]
        if let Some(m) = sections.get("mission") {
            config.mission_name = m.get("name").cloned().unwrap_or_default();
            config.target_ap_km = m.get("targetAp_km").and_then(|v| v.parse().ok()).unwrap_or(185.0);
            config.target_pe_km = m.get("targetPe_km").and_then(|v| v.parse().ok()).unwrap_or(180.0);
            config.max_duration_s = m.get("maxDuration_s").and_then(|v| v.parse().ok()).unwrap_or(7200.0);
        }

        // Engine sections
        if let Some(rs25) = sections.get("rs25") {
            let mut ec = EngineConfig::default();
            ec.thrust_sea_level_n = rs25.get("thrustSeaLevel_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.thrust_vacuum_n = rs25.get("thrustVacuum_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.engine_count = rs25.get("engineCount").and_then(|v| v.parse().ok()).unwrap_or(0);
            ec.sea_level_isp_s = rs25.get("seaLevelIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.vacuum_isp_s = rs25.get("vacuumIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.fuel_ratio = rs25.get("fuelRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.ox_ratio = rs25.get("oxRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.of_ratio = rs25.get("OF_ratio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.thrust_n = ec.thrust_sea_level_n;
            config.rs25 = ec;
        }

        if let Some(srb) = sections.get("srb") {
            let mut ec = EngineConfig::default();
            ec.thrust_sea_level_n = srb.get("thrustSeaLevel_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.thrust_vacuum_n = srb.get("thrustVacuum_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.engine_count = srb.get("engineCount").and_then(|v| v.parse().ok()).unwrap_or(0);
            if let Some(v) = srb.get("ispSeaLevel_s") { ec.sea_level_isp_s = v.parse().unwrap_or(0.0); }
            if let Some(v) = srb.get("ispVacuum_s") { ec.vacuum_isp_s = v.parse().unwrap_or(0.0); }
            config.srb = ec;
        }

        // RL10
        if let Some(rl10) = sections.get("rl10") {
            let mut ec = EngineConfig::default();
            ec.thrust_n = rl10.get("thrust_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.sea_level_isp_s = rl10.get("seaLevelIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.vacuum_isp_s = rl10.get("vacuumIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.fuel_ratio = rl10.get("fuelRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.ox_ratio = rl10.get("oxRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.of_ratio = rl10.get("OF_ratio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.rl10 = ec;
        }

        if let Some(aj10) = sections.get("aj10") {
            let mut ec = EngineConfig::default();
            ec.thrust_n = aj10.get("thrust_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.sea_level_isp_s = aj10.get("seaLevelIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.vacuum_isp_s = aj10.get("vacuumIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.fuel_ratio = aj10.get("fuelRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.ox_ratio = aj10.get("oxRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.of_ratio = aj10.get("OF_ratio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.aj10 = ec;
        }

        if let Some(cs) = sections.get("core_stage").or_else(|| sections.get("core_tanks")) {
            let is_sls = sections.contains_key("core_stage");
            if is_sls {
                config.core_lh2.name = "SLS Core LH2 Tank".into();
                config.core_lh2.fuel_mass_kg = cs.get("lh2Mass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_lh2.dry_mass_kg = cs.get("lh2Dry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_lh2.propellant = "LH2".into();
                config.core_lox.name = "SLS Core LOX Tank".into();
                config.core_lox.fuel_mass_kg = cs.get("loxMass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_lox.dry_mass_kg = cs.get("loxDry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_lox.propellant = "LOX".into();
            } else {
                config.core_rp1.name = "F9 S1 RP-1 Tank".into();
                config.core_rp1.fuel_mass_kg = cs.get("rp1Mass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_rp1.dry_mass_kg = cs.get("rp1Dry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_rp1.propellant = "RP1".into();
                config.core_lox_old.name = "F9 S1 LOX Tank".into();
                config.core_lox_old.fuel_mass_kg = cs.get("loxMass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_lox_old.dry_mass_kg = cs.get("loxDry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                config.core_lox_old.propellant = "LOX".into();
            }
        }

        if let Some(st) = sections.get("second_stage_tanks") {
            config.second_stage_rp1.name = "F9 S2 RP-1 Tank".into();
            config.second_stage_rp1.fuel_mass_kg = st.get("rp1Mass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.second_stage_rp1.dry_mass_kg = st.get("rp1Dry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.second_stage_rp1.propellant = "RP1".into();
            config.second_stage_lox.name = "F9 S2 LOX Tank".into();
            config.second_stage_lox.fuel_mass_kg = st.get("loxMass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.second_stage_lox.dry_mass_kg = st.get("loxDry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.second_stage_lox.propellant = "LOX".into();
        }

        if let Some(it) = sections.get("icps_tanks") {
            config.icps_lh2.name = "ICPS LH2 Tank".into();
            config.icps_lh2.fuel_mass_kg = it.get("lh2Mass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.icps_lh2.dry_mass_kg = it.get("lh2Dry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.icps_lh2.propellant = "LH2".into();
            config.icps_lox.name = "ICPS LOX Tank".into();
            config.icps_lox.fuel_mass_kg = it.get("loxMass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.icps_lox.dry_mass_kg = it.get("loxDry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.icps_lox.propellant = "LOX".into();
        }

        if let Some(sf) = sections.get("srb_fuel") {
            config.srb_fuel.name = "SRB Solid Propellant".into();
            config.srb_fuel.fuel_mass_kg = sf.get("solidMass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.srb_fuel.dry_mass_kg = sf.get("solidDry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.srb_fuel.propellant = "Solid".into();
        }

        if let Some(ot) = sections.get("orion_tanks") {
            config.orion_mmh.fuel_mass_kg = ot.get("mmhMass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.orion_mmh.dry_mass_kg = ot.get("mmhDry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.orion_mmh.propellant = "MMH".into();
            config.orion_nto.name = "Orion NTO Tank".into();
            config.orion_nto.fuel_mass_kg = ot.get("ntoMass_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.orion_nto.dry_mass_kg = ot.get("ntoDry_kg").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            config.orion_nto.propellant = "NTO".into();
        }

        if let Some(merlin) = sections.get("merlin") {
            let mut ec = EngineConfig::default();
            ec.thrust_sea_level_n = merlin.get("thrustSeaLevel_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.thrust_vacuum_n = merlin.get("thrustVacuum_N").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.engine_count = merlin.get("engineCount").and_then(|v| v.parse().ok()).unwrap_or(0);
            ec.sea_level_isp_s = merlin.get("seaLevelIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.vacuum_isp_s = merlin.get("vacuumIsp_s").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.fuel_ratio = merlin.get("fuelRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.ox_ratio = merlin.get("oxRatio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.of_ratio = merlin.get("OF_ratio").and_then(|v| v.parse().ok()).unwrap_or(0.0);
            ec.thrust_n = ec.thrust_sea_level_n;
            config.merlin = ec;
        }

        if let Some(g) = sections.get("guidance") {
            config.guidance.algorithm = g.get("algorithm").cloned().unwrap_or_else(|| "cosine".into());
            config.guidance.pitch_start_alt_m = g.get("pitchStartAlt_m").and_then(|v| v.parse().ok()).unwrap_or(2000.0);
            config.guidance.pitch_end_alt_m = g.get("pitchEndAlt_m").and_then(|v| v.parse().ok()).unwrap_or(20000.0);
            config.guidance.pitch_end_angle_deg = g.get("pitchEndAngle_deg").and_then(|v| v.parse().ok()).unwrap_or(85.0);
            config.guidance.orbit_tolerance_m = g.get("orbitTolerance_m").and_then(|v| v.parse().ok()).unwrap_or(10000.0);
        }

        if let Some(lc) = sections.get("launch") {
            if let Some(v) = lc.get("datetime") { config.launch.datetime = v.clone(); }
            if let Some(v) = lc.get("timezone") { config.launch.timezone = v.clone(); }
            if let Some(v) = lc.get("window_start") { config.launch.window.start = v.clone(); }
            if let Some(v) = lc.get("window_end") { config.launch.window.end = v.clone(); }
            if let Some(v) = lc.get("auto_calculate_window") { config.launch.window.auto_calculate = v == "true"; }
        }

        if let Some(ls) = sections.get("launch_site") {
            if let Some(v) = ls.get("name") { config.launch_location.name = v.clone(); }
            if let Some(v) = ls.get("latitude") { config.launch_location.latitude = v.parse().unwrap_or(0.0); }
            if let Some(v) = ls.get("longitude") { config.launch_location.longitude = v.parse().unwrap_or(0.0); }
        }

        Ok(config)
    }
}

// =====================================================================
// MissionControl
// =====================================================================
#[derive(Debug, Clone)]
pub struct MissionControl {
    pub mission_time: f64,
    pub current_phase: MissionPhase,
    pub outcome: MissionOutcome,
    pub trigger_system: EventTriggerSystem,
    pub telemetry: TelemetryData,
    pub telemetry_log: Vec<TelemetryData>,
    pub triggered_events: Vec<MissionEvent>,
    pub summary: MissionSummary,
    pub script: MissionScript,
    pub max_q: f64,
    pub max_q_altitude: f64,
    pub max_q_time: f64,
    pub max_q_passed: bool,
    pub last_engine_status: EngineStatus,
    pub cutoff_fired: bool,
    /// 滑行阶段：MECO 后等待远地点再烧 ICPS
    pub coasting: bool,
    pub coast_start_time: f64,
    pub icps_ignited: bool,
    pub icps_ignition_time: f64,
}

impl MissionControl {
    pub fn new() -> Self {
        MissionControl {
            mission_time: 0.0,
            current_phase: MissionPhase::PreLaunch,
            outcome: MissionOutcome::InProgress,
            trigger_system: EventTriggerSystem::new(),
            telemetry: TelemetryData::default(),
            telemetry_log: Vec::new(),
            triggered_events: Vec::new(),
            summary: MissionSummary::default(),
            script: MissionScript::default(),
            max_q: 0.0,
            max_q_altitude: 0.0,
            max_q_time: 0.0,
            max_q_passed: false,
            last_engine_status: EngineStatus::default(),
            cutoff_fired: false,
            coasting: false,
            coast_start_time: 0.0,
            icps_ignited: false,
            icps_ignition_time: 0.0,
        }
    }

    pub fn load_mission(&mut self, script: &MissionScript) {
        self.script = script.clone();
        self.trigger_system.load_script(script);
        self.reset();
    }

    pub fn reset(&mut self) {
        self.mission_time = 0.0;
        self.current_phase = MissionPhase::PreLaunch;
        self.outcome = MissionOutcome::InProgress;
        self.trigger_system.reset();
        self.summary = MissionSummary::default();
        self.summary.mission_name = self.script.name.clone();
        self.summary.target_orbit.apoapsis_m = self.script.target_orbit.apoapsis_km * 1000.0;
        self.summary.target_orbit.periapsis_m = self.script.target_orbit.periapsis_km * 1000.0;
        self.triggered_events.clear();
        self.cutoff_fired = false;
        self.coasting = false;
        self.coast_start_time = 0.0;
        self.icps_ignited = false;
        self.icps_ignition_time = 0.0;
    }

    pub fn update(&mut self, dt: f64, engine_status: &EngineStatus, vessel: &mut Vessel, earth: &Planet) {
        if self.outcome != MissionOutcome::InProgress { return; }
        if dt == 0.0 { return; }

        self.mission_time += dt;
        self.last_engine_status = engine_status.clone();

        self.update_phase(vessel, earth);
        self.update_telemetry(vessel, earth);

        let commands = self.trigger_system.check_triggers(
            vessel,
            self.mission_time,
            self.get_current_altitude(vessel, earth),
            self.get_current_velocity(vessel),
            self.max_q,
            vessel.get_total_damage(),
        );

        self.execute_commands(&commands, vessel);

        // ICPS 三阶段：滑行 → 远地点点火 → 圆化断油
        // 配合 PEGGuidance（stage≥1 时沿速度方向推进）
        if !self.cutoff_fired && self.current_phase == MissionPhase::Orbit && self.mission_time > 100.0 && vessel.current_stage >= 1 {
            let pos = *vessel.body.get_position(); // copy to drop borrow
            let vel = *vessel.body.get_velocity(); // copy to drop borrow
            let oe = OrbitalMechanics::calculate_elements(pos, vel, earth);
            if oe.semi_major_axis > 0.0 && oe.eccentricity < 1.0 {
                let periapsis_alt =
                    oe.semi_major_axis * (1.0 - oe.eccentricity) - earth.get_radius();
                let apoapsis_alt =
                    oe.semi_major_axis * (1.0 + oe.eccentricity) - earth.get_radius();

                if !self.coasting {
                    self.coasting = true;
                    self.coast_start_time = self.mission_time;
                    // 自动级分离后 ICPS 默认油门=1.0，立即归零开始滑行
                    let active_stage = vessel.current_stage;
                    vessel.set_stage_throttle(active_stage, 0.0);
                    eprintln!("  T+ {:7.1}s  MECO — Coasting to apogee (pe={:.0}km, ap={:.0}km, e={:.4})",
                        self.mission_time, periapsis_alt/1000.0, apoapsis_alt/1000.0, oe.eccentricity);
                }

                if !self.icps_ignited {
                    // 远地点检测：r·v 从正→负（上升→下降），且至少滑行 200s
                    let r_dot_v = pos.x * vel.x + pos.y * vel.y + pos.z * vel.z;
                    let coast_dur = self.mission_time - self.coast_start_time;
                    let at_apogee = coast_dur > 200.0 && r_dot_v < 0.0;
                    let timeout = coast_dur > 3600.0; // 安全超时 1h

                    if at_apogee || timeout {
                        self.icps_ignited = true;
                        self.icps_ignition_time = self.mission_time;
                        let active_stage = vessel.current_stage;
                        vessel.set_stage_throttle(active_stage, 1.0);
                        eprintln!("  T+ {:7.1}s  ICPS Ignition @ apogee (pe={:.0}km, ap={:.0}km)",
                            self.mission_time, periapsis_alt/1000.0, apoapsis_alt/1000.0);
                    }
                }

                // ── 阶段 2：圆化燃烧 ──
                if self.icps_ignited {
                    let pe_km = periapsis_alt / 1000.0;
                    let ap_km = apoapsis_alt / 1000.0;
                    let pe_diff_km = ap_km - pe_km;
                    let burn_duration = self.mission_time - self.icps_ignition_time;

                    // 在远地点附近单次 prograde 燃烧：近地点持续抬升
                    // 当近地点接近远地点（轨道近乎圆形）时断油
                    // 推进剂耗尽时自然熄火——此处提前断油避免过烧
                    // 最佳断油时间：~120s（~342m/s ΔV 可完成圆化），超过 200s 后
                    // 飞船会过近地点，prograde 燃烧反而拉高远地点
                    if pe_diff_km < 50.0 || burn_duration > 200.0 {
                        let active_stage = vessel.current_stage;
                        vessel.set_stage_throttle(active_stage, 0.0);
                        self.cutoff_fired = true;
                        eprintln!("  T+ {:7.1}s  Orbit circularized (pe={:.0}km, ap={:.0}km, e={:.4})",
                            self.mission_time, pe_km, ap_km, oe.eccentricity);
                    }
                }
            }
        }

        self.check_exit_conditions(vessel, earth);
    }

    fn update_phase(&mut self, vessel: &Vessel, earth: &Planet) {
        let altitude = self.get_current_altitude(vessel, earth);
        let velocity = self.get_current_velocity(vessel);
        let orbital_vel = OrbitalMechanics::circular_orbit_velocity(altitude, earth);

        match self.current_phase {
            MissionPhase::PreLaunch if self.mission_time >= 0.0 => {
                self.set_phase(MissionPhase::Launch);
            }
            MissionPhase::Launch if altitude > 1000.0 => {
                self.set_phase(MissionPhase::Ascent);
            }
            MissionPhase::Ascent if self.max_q_passed => {
                self.set_phase(MissionPhase::MaxQ);
            }
            MissionPhase::MaxQ if velocity > orbital_vel * 0.95 => {
                self.set_phase(MissionPhase::Orbit);
            }
            _ => {}
        }
    }

    fn update_telemetry(&mut self, vessel: &Vessel, earth: &Planet) {
        let pos = vessel.body.get_position();
        let vel = vessel.body.get_velocity();
        let altitude = earth.get_altitude(*pos);
        let velocity = vel.length();
        let density = earth.get_atmosphere().get_density(altitude);
        let speed_of_sound = earth.get_atmosphere().get_speed_of_sound(altitude);

        let mut data = TelemetryData::default();
        data.mission_time = self.mission_time;
        data.phase = self.current_phase;
        data.altitude_m = altitude;
        data.velocity_mps = velocity;
        data.mach = if speed_of_sound > 0.0 { velocity / speed_of_sound } else { 0.0 };
        data.dynamic_pressure_pa = 0.5 * density * velocity * velocity;
        data.total_mass_kg = vessel.body.get_mass();
        data.damage_total = vessel.get_total_damage();
        data.thrust_n = self.last_engine_status.total_thrust;
        data.throttle_pct = self.last_engine_status.max_throttle * 100.0;
        data.mass_flow_kg_s = self.last_engine_status.total_mass_flow;
        data.fuel_flow_kg_s = self.last_engine_status.total_fuel_flow;
        data.ox_flow_kg_s = self.last_engine_status.total_ox_flow;
        data.position = *pos;
        data.velocity = *vel;

        if data.dynamic_pressure_pa > self.max_q {
            self.max_q = data.dynamic_pressure_pa;
            self.max_q_altitude = altitude;
            self.max_q_time = self.mission_time;
            self.summary.max_q_pa = self.max_q;
            self.summary.max_q_altitude_m = self.max_q_altitude;
            self.summary.max_q_time_s = self.max_q_time;
        }
        data.max_q_pa = self.max_q;

        // MaxQ 峰值识别：当动压超过 30kPa 且不再增长时标记（约真实 SLS MaxQ 水平）
        let q_decreasing = self.mission_time > 10.0 && data.dynamic_pressure_pa < self.max_q * 0.95;
        if self.max_q > 30_000.0 && q_decreasing && !self.max_q_passed {
            self.max_q_passed = true;
            self.trigger_event("MaxQ_Pass", "Dynamic pressure peak passed");
        }

        let oe = OrbitalMechanics::calculate_elements(*pos, *vel, earth);
        data.orbit.apoapsis_m = oe.semi_major_axis * (1.0 + oe.eccentricity);
        data.orbit.periapsis_m = oe.semi_major_axis * (1.0 - oe.eccentricity);
        data.orbit.is_bound = oe.eccentricity < 1.0;

        self.telemetry = data.clone();

        static mut LAST_LOG_TIME: f64 = 0.0;
        unsafe {
            if self.mission_time - LAST_LOG_TIME >= 2.0 {
                self.telemetry_log.push(data);
                LAST_LOG_TIME = self.mission_time;
            }
        }
    }

    fn execute_commands(&mut self, commands: &[Command], vessel: &mut Vessel) {
        for cmd in commands {
            match cmd {
                Command::StageSeparation { .. } => {
                    vessel.activate_next_stage();
                    self.trigger_event("Stage_Separation", "Stage separated");
                    self.summary.staging_events.push(
                        self.triggered_events.last().cloned().unwrap()
                    );
                }
                Command::SetThrottle { stage, value } => {
                    vessel.set_stage_throttle(*stage, *value);
                }
                Command::LogMessage { message } => {
                    self.trigger_event("CMD", message);
                }
                Command::AbortMission { message } => {
                    self.abort_mission(message);
                }
                Command::TriggerDamage { message } => {
                    self.trigger_event("Damage_Applied", message);
                }
                _ => {}
            }
        }
    }

    fn check_exit_conditions(&mut self, vessel: &Vessel, earth: &Planet) {
        if self.mission_time > self.script.max_duration_s {
            self.outcome = MissionOutcome::Timeout;
            self.trigger_event("Timeout", "Mission exceeded maximum duration");
            self.finalize_summary();
            return;
        }

        let pos = vessel.body.get_position();
        if earth.get_altitude(*pos) < 0.0 && self.mission_time > 10.0 {
            let speed = vessel.body.get_velocity().length();
            if speed < 10.0 {
                // 软着陆/溅落 → 成功
                self.outcome = MissionOutcome::Success;
                self.trigger_event("Landing", &format!("Soft landing/splashdown at T+{:.1}s, speed={:.1}m/s", self.mission_time, speed));
            } else {
                // 硬撞击 → 坠毁
                self.outcome = MissionOutcome::Failure;
                self.trigger_event("Crash", &format!("Vehicle impacted surface at T+{:.1}s, speed={:.1}m/s", self.mission_time, speed));
            }
            self.finalize_summary();
            return;
        }

        if self.current_phase == MissionPhase::Orbit && self.mission_time > 100.0 {
            let vel = vessel.body.get_velocity();
            let oe = OrbitalMechanics::calculate_elements(*pos, *vel, earth);
            if oe.eccentricity < 1.0 {
                let ap_error =
                    ((oe.semi_major_axis * (1.0 + oe.eccentricity)) - self.script.target_orbit.apoapsis_km * 1000.0).abs();
                let pe_error =
                    ((oe.semi_major_axis * (1.0 - oe.eccentricity)) - self.script.target_orbit.periapsis_km * 1000.0).abs();
                if ap_error < 10000.0 && pe_error < 10000.0 {
                    self.outcome = MissionOutcome::Success;
                    self.summary.final_orbit = self.telemetry.orbit.clone();
                    self.trigger_event("Mission_Complete", "Target orbit achieved");
                    self.finalize_summary();
                }
            }
        }
    }

    fn finalize_summary(&mut self) {
        self.summary.duration_s = self.mission_time;
        self.summary.outcome = self.outcome;
    }

    fn set_phase(&mut self, phase: MissionPhase) {
        if phase != self.current_phase {
            self.current_phase = phase;
        }
    }

    fn trigger_event(&mut self, name: &str, description: &str) {
        let evt = MissionEvent {
            time: self.mission_time,
            name: name.to_string(),
            description: description.to_string(),
            phase: self.current_phase,
            triggered: true,
            triggers: Vec::new(),
            commands: Vec::new(),
        };
        self.triggered_events.push(evt.clone());
        self.summary.all_events.push(evt);
    }

    pub fn abort_mission(&mut self, reason: &str) {
        self.outcome = MissionOutcome::Abort;
        self.trigger_event("ABORT", reason);
        self.finalize_summary();
    }

    fn get_current_altitude(&self, vessel: &Vessel, earth: &Planet) -> f64 {
        earth.get_altitude(*vessel.body.get_position())
    }

    fn get_current_velocity(&self, vessel: &Vessel) -> f64 {
        vessel.body.get_velocity().length()
    }
}

// =====================================================================
// DeepSpaceMissionPlanner — 深空任务规划器
// =====================================================================
const G: f64 = 6.67430e-11;

pub struct DeepSpaceMissionPlanner {
    pub earth: Planet,
    pub moon: Planet,
    pub mu_earth: f64,
}

#[derive(Debug, Clone)]
pub struct MissionStepResult {
    pub success: bool,
    pub delta_v: f64,
    pub next_phase: DeepPhase,
    pub message: String,
}

impl DeepSpaceMissionPlanner {
    pub fn new(earth: Planet, moon: Planet) -> Self {
        let mu_earth = G * earth.get_mass();
        DeepSpaceMissionPlanner { earth, moon, mu_earth }
    }

    /// TLI: 霍曼转移从停泊轨道到月球轨道
    pub fn plan_trans_lunar_injection(&self, r_park: f64, r_moon: f64) -> MissionStepResult {
        if r_park <= 0.0 || r_moon <= 0.0 {
            return MissionStepResult {
                success: false, delta_v: 0.0, next_phase: DeepPhase::Failed,
                message: "TLI: invalid orbit radii (rPark/rMoon <= 0)".into(),
            };
        }

        // Simplified Hohmann: deltaV1 = sqrt(mu/r1) * (sqrt(2*r2/(r1+r2)) - 1)
        let v1 = (self.mu_earth / r_park).sqrt();
        let a_transfer = (r_park + r_moon) / 2.0;
        let v_transfer_at_peri = (self.mu_earth * (2.0 / r_park - 1.0 / a_transfer)).sqrt();
        let dv = v_transfer_at_peri - v1;

        let transfer_time = std::f64::consts::PI * (a_transfer.powi(3) / self.mu_earth).sqrt();

        if !dv.is_finite() {
            return MissionStepResult {
                success: false, delta_v: 0.0, next_phase: DeepPhase::Failed,
                message: "TLI: Hohmann solution invalid (degenerate geometry)".into(),
            };
        }

        MissionStepResult {
            success: true, delta_v: dv, next_phase: DeepPhase::CoastToMoon,
            message: format!("TLI planned: deltaV={:.1} m/s, transferTime={:.0} s", dv, transfer_time),
        }
    }

    /// LOI: 月球轨道插入
    pub fn plan_lunar_orbit_insertion(&self, v_arrival: Vec3, v_target: Vec3) -> MissionStepResult {
        let dv = (v_arrival - v_target).length();
        if !(dv > 0.0) {
            return MissionStepResult {
                success: false, delta_v: 0.0, next_phase: DeepPhase::Failed,
                message: "LOI: zero/undefined relative velocity change".into(),
            };
        }
        MissionStepResult {
            success: true, delta_v: dv, next_phase: DeepPhase::Landing,
            message: format!("LOI planned: deltaV={:.1} m/s", dv),
        }
    }

    /// 相位状态机 (遥测驱动)
    pub fn advance_phase(&self, current: DeepPhase, altitude: f64, speed: f64) -> MissionStepResult {
        const K_PARK_ALT: f64 = 1.60e5;      // LEO 停泊轨道高度 (m)
        const K_TLI_SPEED: f64 = 1.00e4;     // 转移速度阈值 (m/s)
        const K_LUNAR_APPROACH: f64 = 6.00e7; // 月球影响球距离 (m)
        const K_LUNAR_ORBIT_R: f64 = 1.84e6; // 环月轨道半径 (m): moon radius + 100km
        const K_LUNAR_ORBIT_SPEED: f64 = 2.00e3; // 环月捕获后速率 (m/s)
        const K_LANDING_ALT: f64 = 1.0;      // 触月高度 (m)
        const K_TOUCHDOWN_SPEED: f64 = 5.0;  // 安全着陆速度 (m/s)

        match current {
            DeepPhase::Completed => return MissionStepResult {
                success: true, delta_v: 0.0, next_phase: DeepPhase::Completed,
                message: "Mission completed.".into(),
            },
            DeepPhase::Failed => return MissionStepResult {
                success: false, delta_v: 0.0, next_phase: DeepPhase::Failed,
                message: "Mission failed.".into(),
            },
            _ => {},
        }

        if altitude < 0.0 && current != DeepPhase::Landing {
            return MissionStepResult {
                success: false, delta_v: 0.0, next_phase: DeepPhase::Failed,
                message: "Altitude <= 0: reentry/crash detected.".into(),
            };
        }

        match current {
            DeepPhase::PreLaunch => {
                if altitude >= K_PARK_ALT {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::EarthOrbit, message: "Reached parking orbit.".into() }
                } else {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::PreLaunch, message: "On pad / ascending.".into() }
                }
            }
            DeepPhase::EarthOrbit => {
                if speed >= K_TLI_SPEED && altitude >= K_PARK_ALT {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::TransLunarInjection, message: "TLI burn complete, departing Earth.".into() }
                } else {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::EarthOrbit, message: "In Earth parking orbit.".into() }
                }
            }
            DeepPhase::TransLunarInjection => {
                if altitude >= K_LUNAR_APPROACH {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::CoastToMoon, message: "Left Earth vicinity, coasting to Moon.".into() }
                } else {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::TransLunarInjection, message: "Climbing onto trans-lunar trajectory.".into() }
                }
            }
            DeepPhase::CoastToMoon => {
                if altitude <= K_LUNAR_ORBIT_R {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::LunarOrbitInsertion, message: "Entered lunar capture zone.".into() }
                } else {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::CoastToMoon, message: "Coasting to Moon.".into() }
                }
            }
            DeepPhase::LunarOrbitInsertion => {
                if speed <= K_LUNAR_ORBIT_SPEED && altitude <= K_LUNAR_ORBIT_R {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::Landing, message: "Captured into lunar orbit, begin descent.".into() }
                } else {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::LunarOrbitInsertion, message: "Braking into lunar orbit.".into() }
                }
            }
            DeepPhase::Landing => {
                if altitude <= K_LANDING_ALT && speed <= K_TOUCHDOWN_SPEED {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::Completed, message: "Soft landing confirmed.".into() }
                } else if altitude <= 0.0 && speed > K_TOUCHDOWN_SPEED {
                    MissionStepResult { success: false, delta_v: 0.0, next_phase: DeepPhase::Failed, message: "Hard impact: touchdown too fast.".into() }
                } else {
                    MissionStepResult { success: true, delta_v: 0.0, next_phase: DeepPhase::Landing, message: "Descending to surface.".into() }
                }
            }
            _ => MissionStepResult { success: false, delta_v: 0.0, next_phase: DeepPhase::Failed, message: "Unknown phase.".into() },
        }
    }

    pub fn get_initial_phase(&self) -> DeepPhase { DeepPhase::PreLaunch }
}

// Default impl for MissionEvent to support the `..Default::default()` shorthand
impl Default for MissionEvent {
    fn default() -> Self {
        MissionEvent {
            time: 0.0,
            name: String::new(),
            description: String::new(),
            phase: MissionPhase::PreLaunch,
            triggered: false,
            triggers: Vec::new(),
            commands: Vec::new(),
        }
    }
}

// =====================================================================
// Tests
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mission_phase_to_string() {
        assert_eq!(MissionPhase::PreLaunch.to_str(), "PRE_LAUNCH");
        assert_eq!(MissionPhase::Orbit.to_str(), "ORBIT");
        assert_eq!(MissionPhase::Abort.to_str(), "ABORT");
    }

    #[test]
    fn test_mission_outcome_to_string() {
        assert_eq!(MissionOutcome::InProgress.to_str(), "IN_PROGRESS");
        assert_eq!(MissionOutcome::Success.to_str(), "SUCCESS");
        assert_eq!(MissionOutcome::Failure.to_str(), "FAILURE");
    }

    #[test]
    fn test_mission_outcome_values() {
        assert_ne!(MissionOutcome::InProgress, MissionOutcome::Success);
    }

    #[test]
    fn test_trigger_condition_basic() {
        let cond = TriggerCondition::new(TriggerType::TimeElapsed, 10.0);
        assert_eq!(cond.trigger_type, TriggerType::TimeElapsed);
        assert!((cond.value - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_mission_script_default() {
        let script = MissionScript::default();
        assert_eq!(script.name, "");
        assert_eq!(script.target_orbit.apoapsis_km, 185.0);
        assert!(script.auto_mode);
    }

    #[test]
    fn test_default_mission_script() {
        let script = MissionScript::default();
        assert!(script.events.is_empty());
        assert_eq!(script.target_orbit.apoapsis_km, 185.0);
    }

    #[test]
    fn test_default_mission_empty_events() {
        let script = MissionScript::default();
        assert!(script.events.is_empty());
    }

    #[test]
    fn test_deepspace_planner_initial_phase() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);
        assert_eq!(planner.get_initial_phase(), DeepPhase::PreLaunch);
    }

    #[test]
    fn test_deepspace_planner_tli() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        // r_park = 200km + Earth radius, r_moon = Moon orbit ~ 384,400 km
        let r_park = 6_571_000.0;
        let r_moon = 384_400_000.0;
        let result = planner.plan_trans_lunar_injection(r_park, r_moon);
        assert!(result.success);
        assert!(result.delta_v > 0.0);
        assert_eq!(result.next_phase, DeepPhase::CoastToMoon);
    }

    #[test]
    fn test_deepspace_planner_tli_invalid() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        let result = planner.plan_trans_lunar_injection(0.0, -1.0);
        assert!(!result.success);
        assert_eq!(result.next_phase, DeepPhase::Failed);
    }

    #[test]
    fn test_deepspace_planner_loi() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        let v_arrival = Vec3::new(2500.0, 0.0, 0.0);
        let v_target = Vec3::new(1630.0, 0.0, 0.0); // ~circular at 100km
        let result = planner.plan_lunar_orbit_insertion(v_arrival, v_target);
        assert!(result.success);
        assert!((result.delta_v - 870.0).abs() < 1.0);
    }

    #[test]
    fn test_advance_phase_prelaunch_to_orbit() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        // Pre-launch → EarthOrbit (altitude >= 160km)
        let result = planner.advance_phase(DeepPhase::PreLaunch, 200_000.0, 0.0);
        assert_eq!(result.next_phase, DeepPhase::EarthOrbit);
    }

    #[test]
    fn test_advance_phase_still_on_pad() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        let result = planner.advance_phase(DeepPhase::PreLaunch, 0.0, 0.0);
        assert_eq!(result.next_phase, DeepPhase::PreLaunch);
        assert!(result.message.contains("On pad"));
    }

    #[test]
    fn test_advance_phase_crash_detection() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        let result = planner.advance_phase(DeepPhase::EarthOrbit, -100.0, 5000.0);
        assert!(!result.success);
        assert_eq!(result.next_phase, DeepPhase::Failed);
    }

    #[test]
    fn test_advance_phase_soft_landing() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        let result = planner.advance_phase(DeepPhase::Landing, 0.5, 2.0);
        assert!(result.success);
        assert_eq!(result.next_phase, DeepPhase::Completed);
    }

    #[test]
    fn test_advance_phase_hard_impact() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        let result = planner.advance_phase(DeepPhase::Landing, -1.0, 50.0);
        assert!(!result.success);
        assert_eq!(result.next_phase, DeepPhase::Failed);
    }

    #[test]
    fn test_advance_phase_terminal_states() {
        use crate::environment::Atmosphere;
        let earth = Planet::new("Earth", 5.972e24, 6_371_000.0, Atmosphere::new(101325.0, 8500.0));
        let moon = Planet::new("Moon", 7.342e22, 1_737_000.0, Atmosphere::new(0.0, 0.0));
        let planner = DeepSpaceMissionPlanner::new(earth, moon);

        // Terminal states don't change
        let r1 = planner.advance_phase(DeepPhase::Completed, 0.0, 0.0);
        assert_eq!(r1.next_phase, DeepPhase::Completed);

        let r2 = planner.advance_phase(DeepPhase::Failed, 100.0, 100.0);
        assert_eq!(r2.next_phase, DeepPhase::Failed);
    }

    #[test]
    fn test_event_trigger_system_time() {
        let mut ets = EventTriggerSystem::new();
        let mut script = MissionScript::default();
        script.events.push(MissionEvent {
            name: "test_event".into(),
            description: String::new(),
            triggers: vec![TriggerCondition::new(TriggerType::TimeElapsed, 5.0)],
            commands: vec![Command::LogMessage { message: "Fired!".into() }],
            ..Default::default()
        });

        ets.load_script(&script);

        use crate::vessel::Vessel;
        let vessel = Vessel::new("Test");

        // Before time
        let cmds = ets.check_triggers(&vessel, 3.0, 0.0, 0.0, 0.0, 0.0);
        assert!(cmds.is_empty());

        // After time
        let cmds = ets.check_triggers(&vessel, 10.0, 0.0, 0.0, 0.0, 0.0);
        assert!(!cmds.is_empty());
    }

    #[test]
    fn test_event_trigger_system_altitude() {
        let mut ets = EventTriggerSystem::new();
        let mut script = MissionScript::default();
        script.events.push(MissionEvent {
            name: "high_alt".into(),
            description: String::new(),
            triggers: vec![TriggerCondition::new(TriggerType::AltitudeAbove, 10000.0)],
            commands: vec![Command::LogMessage { message: "High altitude!".into() }],
            ..Default::default()
        });

        ets.load_script(&script);

        use crate::vessel::Vessel;
        let vessel = Vessel::new("Test");

        let cmds = ets.check_triggers(&vessel, 0.0, 5000.0, 0.0, 0.0, 0.0);
        assert!(cmds.is_empty());

        let cmds = ets.check_triggers(&vessel, 0.0, 15000.0, 0.0, 0.0, 0.0);
        assert!(!cmds.is_empty());
    }

    #[test]
    fn test_default_script_events_empty() {
        let script = MissionScript::default();
        assert!(script.events.is_empty());
    }

    #[test]
    fn test_mission_control_new() {
        let mc = MissionControl::new();
        assert_eq!(mc.current_phase, MissionPhase::PreLaunch);
        assert_eq!(mc.outcome, MissionOutcome::InProgress);
    }

    #[test]
    fn test_mission_control_load_script() {
        let mut mc = MissionControl::new();
        let script = MissionScript::default();
        mc.load_mission(&script);
        assert!(mc.script.events.is_empty());
    }
}
