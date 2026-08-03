//! DeepSpace CLI Simulation Application
//!
//! Headless mission simulation: builds a vessel from config, runs physics,
//! manages mission events, and outputs telemetry to CSV.
//!
//! Usage:
//!   cargo run -- --headless [--mission missions/artemis2.conf] [--csv telemetry.csv]

use std::fs::File;
use std::io::{BufWriter, Write};

use deepspace::environment::{Atmosphere, Planet, ThermalSimulation};
use deepspace::guidance::{FlightComputer, GuidanceState};
use deepspace::simulation::{
    MissionConfig, MissionControl, MissionOutcome, TelemetryData,
};
use deepspace::vessel::{Part, PropellantType, Vessel};
use deepspace::{Vec3, G};

// =====================================================================
// CLI 参数解析
// =====================================================================
#[derive(Debug)]
pub struct CliArgs {
    pub headless: bool,
    pub mission_path: String,
    pub csv_path: Option<String>,
    pub dt: f64,
    pub duration: Option<f64>,
}

impl CliArgs {
    pub fn parse() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut headless = false;
        let mut mission_path = "missions/artemis2.conf".to_string();
        let mut csv_path: Option<String> = None;
        let mut dt = 0.1;
        let mut duration: Option<f64> = None;

        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--headless" | "-h" => headless = true,
                "--mission" => {
                    headless = true;
                    if i + 1 < args.len() {
                        i += 1;
                        let m = &args[i];
                        if m.contains('/') || m.contains('.') {
                            mission_path = m.clone();
                        } else {
                            mission_path = format!("missions/{}", m);
                        }
                    }
                }
                "--csv" => {
                    if i + 1 < args.len() {
                        i += 1;
                        csv_path = Some(args[i].clone());
                    }
                }
                "--dt" => {
                    if i + 1 < args.len() {
                        i += 1;
                        if let Ok(v) = args[i].parse() {
                            dt = v;
                        }
                    }
                }
                "--duration" => {
                    if i + 1 < args.len() {
                        i += 1;
                        if let Ok(v) = args[i].parse() {
                            duration = Some(v);
                        }
                    }
                }
                "--help" => {
                    println!("Usage: deepspace [options]");
                    println!("  --headless, -h   Run in headless mode (simulation)");
                    println!(
                        "  --mission <file> Mission config file (default: missions/artemis2.conf)"
                    );
                    println!("  --csv <file>     Output telemetry CSV file");
                    println!("  --dt <seconds>   Simulation timestep (default: 0.1)");
                    println!("  --duration <s>   Override max simulation duration (default: from config)");
                    println!("  --help           Show this help");
                    std::process::exit(0);
                }
                _ => {}
            }
            i += 1;
        }

        CliArgs {
            headless,
            mission_path,
            csv_path,
            dt,
            duration,
        }
    }
}

// =====================================================================
// 遥测 CSV 写入
// =====================================================================
fn write_telemetry_csv(path: &str, log: &[TelemetryData]) -> Result<(), String> {
    let file = File::create(path).map_err(|e| format!("Failed to create CSV: {}", e))?;
    let mut w = BufWriter::new(file);

    writeln!(
        w,
        "time_s,phase,altitude_m,velocity_mps,mach,dyn_pressure_pa,total_mass_kg,\
                 thrust_n,throttle_pct,damage_total,apoapsis_m,periapsis_m,is_bound,\
                 position_x,position_y,position_z,velocity_x,velocity_y,velocity_z"
    )
    .map_err(|e| format!("CSV header: {}", e))?;

    for t in log {
        writeln!(
            w,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            t.mission_time,
            t.phase.to_str(),
            t.altitude_m,
            t.velocity_mps,
            t.mach,
            t.dynamic_pressure_pa,
            t.total_mass_kg,
            t.thrust_n,
            t.throttle_pct,
            t.damage_total,
            t.orbit.apoapsis_m,
            t.orbit.periapsis_m,
            t.orbit.is_bound,
            t.position.x,
            t.position.y,
            t.position.z,
            t.velocity.x,
            t.velocity.y,
            t.velocity.z,
        )
        .map_err(|e| format!("CSV write: {}", e))?;
    }

    Ok(())
}

// =====================================================================
// 飞船构建器

fn build_vessel_from_config(
    config: &MissionConfig,
    vessel: &mut Vessel,
    body_radius: f64,
    launch_lat_deg: f64,
    launch_lon_deg: f64,
) {
    // 根据经纬度计算地表发射位置（Y-up：Y=极轴, XZ=赤道面）
    let lat_rad = launch_lat_deg.to_radians();
    let lon_rad = launch_lon_deg.to_radians();
    let launch_pos = deepspace::Vec3::new(
        body_radius * lat_rad.cos() * lon_rad.cos(),
        body_radius * lat_rad.sin(),
        body_radius * lat_rad.cos() * lon_rad.sin(),
    );

    vessel.body = deepspace::physics::PhysicsBody::new(
        launch_pos,
        deepspace::Vec3::zero(),
        0.0,
        12_000_000.0,
    );

    let use_sls = config.rs25.engine_count > 0;

    if use_sls {
        build_sls_stack(config, vessel);
    } else {
        build_falcon9_stack(config, vessel);
    }

    println!(
        "  Vessel built: {} stage(s), ~{:.0} kg",
        vessel.find_highest_stage() + 1,
        vessel.body.get_mass()
    );
}

fn build_sls_stack(config: &MissionConfig, vessel: &mut Vessel) {
    // Stage 2: Orion (persistent)
    let mut p = Part::new_fuel_tank(
        "Orion MMH",
        config.orion_mmh.dry_mass_kg,
        config.orion_mmh.fuel_mass_kg,
        PropellantType::Mmh,
    );
    p.stage = 2;
    p.persistent = true;
    vessel.add_part(p);

    let mut p = Part::new_fuel_tank(
        "Orion NTO",
        config.orion_nto.dry_mass_kg,
        config.orion_nto.fuel_mass_kg,
        PropellantType::Nto,
    );
    p.stage = 2;
    p.persistent = true;
    vessel.add_part(p);

    let mut p = Part::new_engine(
        "AJ10-190",
        200.0,
        config.aj10.thrust_n,
        config.aj10.sea_level_isp_s,
        config.aj10.vacuum_isp_s,
        PropellantType::Mmh,
        PropellantType::Nto,
        config.aj10.of_ratio,
    );
    p.stage = 2;
    p.persistent = true;
    vessel.add_part(p);

    // Stage 1: ICPS
    let mut p = Part::new_fuel_tank(
        "ICPS LH2",
        config.icps_lh2.dry_mass_kg,
        config.icps_lh2.fuel_mass_kg,
        PropellantType::Lh2,
    );
    p.stage = 1;
    vessel.add_part(p);

    let mut p = Part::new_fuel_tank(
        "ICPS LOX",
        config.icps_lox.dry_mass_kg,
        config.icps_lox.fuel_mass_kg,
        PropellantType::Lox,
    );
    p.stage = 1;
    vessel.add_part(p);

    let mut p = Part::new_engine(
        "RL10C-2",
        300.0,
        config.rl10.thrust_n,
        config.rl10.sea_level_isp_s.max(200.0),
        config.rl10.vacuum_isp_s.max(400.0),
        PropellantType::Lh2,
        PropellantType::Lox,
        config.rl10.of_ratio,
    );
    p.stage = 1;
    vessel.add_part(p);

    // Stage 0: SLS Core
    let mut p = Part::new_fuel_tank(
        "SLS Core LH2",
        config.core_lh2.dry_mass_kg,
        config.core_lh2.fuel_mass_kg,
        PropellantType::Lh2,
    );
    p.stage = 0;
    vessel.add_part(p);

    let mut p = Part::new_fuel_tank(
        "SLS Core LOX",
        config.core_lox.dry_mass_kg,
        config.core_lox.fuel_mass_kg,
        PropellantType::Lox,
    );
    p.stage = 0;
    vessel.add_part(p);

    // SRB solid propellant tanks (one per booster)
    let srb_dry = if config.srb_fuel.dry_mass_kg > 0.0 {
        config.srb_fuel.dry_mass_kg
    } else {
        1000.0
    };
    let srb_mass = if config.srb_fuel.fuel_mass_kg > 0.0 {
        config.srb_fuel.fuel_mass_kg
    } else {
        628_000.0
    };
    for i in 0..config.srb.engine_count.max(2) {
        let mut p = Part::new_fuel_tank(
            &format!("SRB-{} Solid", i + 1),
            srb_dry,
            srb_mass,
            PropellantType::Solid,
        );
        p.stage = 0;
        vessel.add_part(p);
    }

    for _ in 0..config.rs25.engine_count.max(4) {
        let mut p = Part::new_engine(
            "RS-25",
            3_500.0,
            config.rs25.thrust_sea_level_n.max(1_800_000.0),
            config.rs25.sea_level_isp_s.max(350.0),
            config.rs25.vacuum_isp_s.max(450.0),
            PropellantType::Lh2,
            PropellantType::Lox,
            config.rs25.of_ratio,
        );
        p.stage = 0;
        vessel.add_part(p);
    }
    for _ in 0..config.srb.engine_count.max(2) {
        let mut p = Part::new_engine(
            "SRB",
            2_000.0,
            config.srb.thrust_sea_level_n.max(14_000_000.0),
            config.srb.sea_level_isp_s.max(250.0),
            config.srb.vacuum_isp_s.max(280.0),
            PropellantType::Solid,
            PropellantType::Solid,
            config.srb.of_ratio.max(1.0),
        );
        p.stage = 0;
        vessel.add_part(p);
    }
}

fn build_falcon9_stack(config: &MissionConfig, vessel: &mut Vessel) {
    // Stage 2: Orion (persistent)
    let mut p = Part::new_fuel_tank(
        "Orion MMH",
        config.orion_mmh.dry_mass_kg,
        config.orion_mmh.fuel_mass_kg,
        PropellantType::Mmh,
    );
    p.stage = 2;
    p.persistent = true;
    vessel.add_part(p);

    let mut p = Part::new_fuel_tank(
        "Orion NTO",
        config.orion_nto.dry_mass_kg,
        config.orion_nto.fuel_mass_kg,
        PropellantType::Nto,
    );
    p.stage = 2;
    p.persistent = true;
    vessel.add_part(p);

    let mut p = Part::new_engine(
        "AJ10-190",
        200.0,
        config.aj10.thrust_n,
        config.aj10.sea_level_isp_s,
        config.aj10.vacuum_isp_s.max(300.0),
        PropellantType::Mmh,
        PropellantType::Nto,
        config.aj10.of_ratio,
    );
    p.stage = 2;
    p.persistent = true;
    vessel.add_part(p);

    // Stage 1: Falcon 9 S2
    let mut p = Part::new_fuel_tank(
        "F9 S2 RP-1",
        config.second_stage_rp1.dry_mass_kg,
        config.second_stage_rp1.fuel_mass_kg,
        PropellantType::Rp1,
    );
    p.stage = 1;
    vessel.add_part(p);

    let mut p = Part::new_fuel_tank(
        "F9 S2 LOX",
        config.second_stage_lox.dry_mass_kg,
        config.second_stage_lox.fuel_mass_kg,
        PropellantType::Lox,
    );
    p.stage = 1;
    vessel.add_part(p);

    let mut p = Part::new_engine(
        "Merlin-1D Vac",
        500.0,
        config.merlin_vacuum.thrust_n.max(900_000.0),
        config.merlin.sea_level_isp_s.max(280.0),
        config.merlin_vacuum.vacuum_isp_s.max(340.0),
        PropellantType::Rp1,
        PropellantType::Lox,
        config.merlin_vacuum.of_ratio,
    );
    p.stage = 1;
    vessel.add_part(p);

    // Stage 0: Falcon 9 S1
    let mut p = Part::new_fuel_tank(
        "F9 S1 RP-1",
        config.core_rp1.dry_mass_kg,
        config.core_rp1.fuel_mass_kg,
        PropellantType::Rp1,
    );
    p.stage = 0;
    vessel.add_part(p);

    let mut p = Part::new_fuel_tank(
        "F9 S1 LOX",
        config.core_lox_old.dry_mass_kg,
        config.core_lox_old.fuel_mass_kg,
        PropellantType::Lox,
    );
    p.stage = 0;
    vessel.add_part(p);

    for _ in 0..config.merlin.engine_count.max(9) {
        let mut p = Part::new_engine(
            "Merlin-1D",
            500.0,
            config.merlin.thrust_sea_level_n.max(800_000.0),
            config.merlin.sea_level_isp_s.max(280.0),
            config.merlin.vacuum_isp_s.max(310.0),
            PropellantType::Rp1,
            PropellantType::Lox,
            config.merlin.of_ratio,
        );
        p.stage = 0;
        vessel.add_part(p);
    }
}

// =====================================================================
// SimulationApp — 主仿真循环
// =====================================================================
pub struct SimulationApp {
    pub vessel: Vessel,
    pub earth: Planet,
    /// 所有天体（用于渲染）
    pub bodies: Vec<Planet>,
    /// 天体当前世界坐标
    pub body_positions: Vec<Vec3>,
    /// 天体速度（用于线性动画）
    pub body_velocities: Vec<Vec3>,
    pub mission_control: MissionControl,
    pub thermal: ThermalSimulation,
    pub config: MissionConfig,
    pub headless: bool,
    pub mission_complete: bool,
    pub simulation_time: f64,
    pub telemetry_log: Vec<TelemetryData>,
    pub csv_path: Option<String>,
    pub dt: f64,
    pub flight_computer: FlightComputer,
}

impl SimulationApp {
    pub fn new(args: &CliArgs) -> Self {
        // 加载配置
        let config = match MissionConfig::load(&args.mission_path) {
            Ok(c) => c,
            Err(e) => {
                eprintln!(
                    "WARNING: Could not load mission config '{}': {}",
                    args.mission_path, e
                );
                eprintln!("  Using default mission parameters");
                MissionConfig::load("").ok().unwrap_or(MissionConfig {
                    mission_name: "Artemis II".into(),
                    target_ap_km: 185.0,
                    target_pe_km: 180.0,
                    max_duration_s: 7200.0,
                    rs25: deepspace::simulation::EngineConfig {
                        engine_count: 4,
                        thrust_sea_level_n: 1_860_000.0,
                        sea_level_isp_s: 366.0,
                        vacuum_isp_s: 452.0,
                        of_ratio: 6.0,
                        ..Default::default()
                    },
                    srb: deepspace::simulation::EngineConfig {
                        engine_count: 2,
                        thrust_sea_level_n: 14_000_000.0,
                        sea_level_isp_s: 242.0,
                        vacuum_isp_s: 269.0,
                        of_ratio: 1.0,
                        ..Default::default()
                    },
                    rl10: deepspace::simulation::EngineConfig {
                        thrust_n: 101_400.0,
                        sea_level_isp_s: 200.0,
                        vacuum_isp_s: 462.0,
                        of_ratio: 5.88,
                        ..Default::default()
                    },
                    aj10: deepspace::simulation::EngineConfig {
                        thrust_n: 27_800.0,
                        sea_level_isp_s: 240.0,
                        vacuum_isp_s: 316.0,
                        ..Default::default()
                    },
                    core_lh2: deepspace::simulation::TankConfig {
                        dry_mass_kg: 25000.0,
                        fuel_mass_kg: 120_000.0,
                        propellant: "LH2".into(),
                        ..Default::default()
                    },
                    core_lox: deepspace::simulation::TankConfig {
                        dry_mass_kg: 15000.0,
                        fuel_mass_kg: 720_000.0,
                        propellant: "LOX".into(),
                        ..Default::default()
                    },
                    srb_fuel: deepspace::simulation::TankConfig {
                        dry_mass_kg: 1000.0,
                        fuel_mass_kg: 628_000.0,
                        propellant: "Solid".into(),
                        ..Default::default()
                    },
                    icps_lh2: deepspace::simulation::TankConfig {
                        dry_mass_kg: 3500.0,
                        fuel_mass_kg: 27_000.0,
                        propellant: "LH2".into(),
                        ..Default::default()
                    },
                    icps_lox: deepspace::simulation::TankConfig {
                        dry_mass_kg: 2000.0,
                        fuel_mass_kg: 8_000.0,
                        propellant: "LOX".into(),
                        ..Default::default()
                    },
                    orion_mmh: deepspace::simulation::TankConfig {
                        dry_mass_kg: 800.0,
                        fuel_mass_kg: 5_000.0,
                        propellant: "MMH".into(),
                        ..Default::default()
                    },
                    orion_nto: deepspace::simulation::TankConfig {
                        dry_mass_kg: 500.0,
                        fuel_mass_kg: 3_000.0,
                        propellant: "NTO".into(),
                        ..Default::default()
                    },
                    ..Default::default()
                })
            }
        };

        // 用 CLI --duration 覆盖配置
        let mut config = config;
        if let Some(d) = args.duration {
            if d > 0.0 {
                config.max_duration_s = d;
            }
        }

        // 创建飞船
        let mut vessel = Vessel::new(&config.mission_name);
        let body_radius = config
            .bodies
            .first()
            .map(|b| b.radius)
            .unwrap_or(6_371_000.0);
        build_vessel_from_config(
            &config,
            &mut vessel,
            body_radius,
            config.launch_location.latitude,
            config.launch_location.longitude,
        );
        // 初始化 MissionControl
        let mut mission_control = MissionControl::new();
        mission_control.load_mission(&config.script);
        // 覆盖脚本中的轨道参数（如果配置文件中有）
        if config.target_ap_km > 0.0 {
            mission_control.script.target_orbit.apoapsis_km = config.target_ap_km;
        }
        if config.target_pe_km > 0.0 {
            mission_control.script.target_orbit.periapsis_km = config.target_pe_km;
        }
        mission_control.script.max_duration_s = config.max_duration_s;
        mission_control.tcm_target_dv = config.tcm.target_dv;

        // 初始化飞控计算机
        let mut gc = deepspace::guidance::GuidanceConfig::default();
        gc.algorithm = config.guidance.algorithm.clone();
        gc.pitch_start_alt_m = config.guidance.pitch_start_alt_m;
        gc.pitch_end_alt_m = config.guidance.pitch_end_alt_m;
        gc.pitch_end_angle_deg = config.guidance.pitch_end_angle_deg;
        let flight_computer = FlightComputer::from_config(&gc);

        // 激活第一级
        vessel.activate_next_stage();

        // 应用初始损伤（来自 [damage] 配置 — 仅用于 TPS 材质缺陷等整体性问题）
        // 结构性撞击损伤由外部事件（ApplyDamage 命令）在任务脚本中触发
        vessel.set_damage_tps(config.damage.initial_tps);
        vessel.set_damage_structural(config.damage.initial_structural);

        // 从配置构建天体
        let mut bodies: Vec<Planet> = Vec::new();
        let mut body_positions: Vec<Vec3> = Vec::new();
        let mut body_velocities: Vec<Vec3> = Vec::new();
        for b in &config.bodies {
            let atm = Atmosphere::new(b.sea_level_pressure, b.scale_height);
            bodies.push(Planet::new(&b.name, b.mass, b.radius, atm));
            body_positions.push(b.pos);
            body_velocities.push(b.vel);
        }

        // 若配置未定义天体，使用默认地球
        if bodies.is_empty() {
            let earth_atm = Atmosphere::new(101_325.0, 8_500.0);
            bodies.push(Planet::new("Earth", 5.9722e24, 6_371_000.0, earth_atm));
            body_positions.push(Vec3::zero());
            body_velocities.push(Vec3::zero());
        }

        let earth = bodies[0].clone();
        // DistanceToMoonBelow 按"距月面"口径：注入第二天体半径（配置驱动）
        if bodies.len() > 1 {
            mission_control.moon_radius = bodies[1].get_radius();
        }

        SimulationApp {
            vessel,
            earth,
            bodies,
            body_positions,
            body_velocities,
            mission_control,
            thermal: ThermalSimulation::new(config.thermal),
            config,
            headless: args.headless,
            mission_complete: false,
            simulation_time: 0.0,
            telemetry_log: Vec::new(),
            csv_path: args.csv_path.clone(),
            dt: args.dt,
            flight_computer,
        }
    }

    /// 返回第二颗天体（月球等）在惯性系中的当前位置
    pub fn moon_position(&self) -> Vec3 {
        if self.body_positions.len() > 1 {
            self.body_positions[1]
        } else {
            Vec3::zero()
        }
    }

    /// 主仿真步进（支持倒放：dt 可为负）
    pub fn step(&mut self, dt: f64) {
        if self.mission_complete {
            return;
        }


        // 天体轨道传播：使用圆周轨道近似替代线性漂移
        // 第 0 个天体（地球）固定在原点，其余天体绕其做圆周运动
        // 位置和速度同步绕轨道法线旋转，保持轨道元素不变
        if !self.body_positions.is_empty() {
            let primary = self.body_positions[0];
            for i in 1..self.body_positions.len() {
                let r = self.body_positions[i] - primary;
                let v = self.body_velocities[i];
                let dist = r.length();
                let speed = v.length();
                if dist > 1.0 && speed > 1e-6 {
                    // 角速度 ω = v_perp / r ≈ |v| / |r|（圆周近似）
                    let omega = speed / dist;
                    let dtheta = omega * dt;
                    // 轨道法线轴（r × v 单位化）
                    let normal = r.cross(&v).normalized();
                    // 罗德里格旋转：v' = v*cos(θ) + (n̂ × v)*sin(θ)
                    let cos_a = dtheta.cos();
                    let sin_a = dtheta.sin();
                    let r_rot = r * cos_a + normal.cross(&r) * sin_a;
                    let v_rot = v * cos_a + normal.cross(&v) * sin_a;
                    self.body_positions[i] = primary + r_rot;
                    self.body_velocities[i] = v_rot;
                }
            }
        }

        // 主天体（第一颗）引力
        let primary_pos = if self.body_positions.is_empty() {
            Vec3::zero()
        } else {
            self.body_positions[0]
        };
        let vessel_pos = *self.vessel.body.get_position();
        let r_to_primary = primary_pos - vessel_pos;
        let dist_to_primary = r_to_primary.length();
        if dist_to_primary > 1.0 {
            let g_acc = G * self.bodies[0].get_mass() / (dist_to_primary * dist_to_primary);
            let gravity_dir = r_to_primary.normalized();
            let vessel_mass = self.vessel.body.get_mass();
            let gravity_force = gravity_dir * (g_acc * vessel_mass);
            self.vessel.body.add_force(gravity_force);
        }

        // 其余天体引力扰动（N体近似）
        let pos = *self.vessel.body.get_position();
        for i in 1..self.body_positions.len() {
            let r = self.body_positions[i] - pos;
            let d = r.length();
            if d > 1.0 {
                let acc = r.normalized() * (G * self.bodies[i].get_mass() / (d * d));
                self.vessel.body.add_force(acc * self.vessel.body.get_mass());
            }
        }

        // 大气参数
        let altitude = self.earth.get_altitude(pos);
        let density = self.earth.get_atmosphere().get_density(altitude);
        let speed = self.vessel.body.get_velocity().length();
        let integrity = 1.0 - self.vessel.get_total_damage();
        let mach = speed / 340.0; // 海平面音速近似

        self.thermal.update(dt, speed, density, integrity);


        // 气动阻力（弹道系数 + 马赫数相关 Cd）
        if density > 0.0 && speed > 1.0 {
            // 马赫数相关阻力系数：跨音速峰 Cd~0.8，超音速~0.4，高超音速~0.3
            let cd_mach = if mach < 0.8 {
                0.25
            } else if mach < 1.2 {
                0.25 + (mach - 0.8) / 0.4 * 0.55 // 跨音速陡升
            } else if mach < 3.0 {
                0.80 - (mach - 1.2) / 1.8 * 0.35 // 超音速下降
            } else if mach < 10.0 {
                0.45 - (mach - 3.0) / 7.0 * 0.15
            } else {
                0.30
            };
            // 参考面积（SLS 芯级 ~8m 直径 ~= 50m²，再入时 ~100m²）
            let mut ref_area = if altitude > 100_000.0 { 50.0 } else { 100.0 };
            // 再入末端：海拔 < 15000m 三级降落伞（Orion 3 × 35m 主伞 ~2886m²）
            // 速度门限：仅亚音速（Mach < 0.8）展开，防止高空高速撕裂
            if altitude < 15000.0 && altitude >= 0.0 && self.simulation_time > 500000.0 && mach < 0.8 {
                ref_area = if altitude < 1000.0 {
                    3000.0
                } else if altitude < 5000.0 {
                    1500.0
                } else {
                    500.0
                };
            }
            let cd = (cd_mach + self.vessel.get_total_damage() * 0.3).min(1.5);
            let drag_mag = 0.5 * density * speed * speed * cd * ref_area;
            let vel_dir = -*self.vessel.body.get_velocity() / speed;
            self.vessel.body.add_force(vel_dir * drag_mag);

            // 再入加热 → TPS 烧蚀模型（使用配置阈值）
            let ablation_rate = self
                .thermal
                .ablate(dt, self.config.damage.tps_ablation_threshold);
            if ablation_rate > 0.0 {
                self.vessel.ablate_tps(dt, ablation_rate);
            }

            // 结构损伤传播：已有损伤在再入气动载荷下加速
            // 通用机制（不特指任何损伤源）— 撞击、疲劳、热应力均可触发
            let sp = &self.config.structural;
            let struct_dmg = self.vessel.get_damage_structural();
            let q = 0.5 * density * speed * speed;
            if struct_dmg > 0.01
                && altitude < sp.break_altitude_threshold
                && struct_dmg < 1.0
                && q > sp.onset_dynamic_pressure
            {
                // 损伤越大、动压越高 → 传播越快
                let q_ratio = (q / sp.reference_dynamic_pressure)
                    .max(sp.q_ratio_min)
                    .min(sp.q_ratio_max);
                let propagation = struct_dmg * q_ratio * sp.rate_coefficient * dt;
                self.vessel
                    .set_damage_structural((struct_dmg + propagation).min(1.0));
            }

            // 结构完整性检查：结构损伤超过阈值 → 解体
            let structural_threshold = self.config.damage.structural_failure_threshold;
            if self.vessel.get_damage_structural() >= structural_threshold
                && self.simulation_time > sp.break_time_threshold
                && altitude < sp.break_altitude_threshold
                && self.mission_control.outcome == MissionOutcome::InProgress
            {
                eprintln!();
                eprintln!(
                    "  ⚠ VEHICLE BREAKUP — T+{:.1}s, alt={:.0}m, v={:.0}m/s, Mach={:.1}",
                    self.simulation_time, altitude, speed, mach
                );
                eprintln!(
                    "  ⚠ TPS damage={:.1}%, Structural damage={:.1}% [threshold={:.0}%]",
                    self.vessel.get_damage_tps() * 100.0,
                    self.vessel.get_damage_structural() * 100.0,
                    structural_threshold * 100.0
                );
                self.vessel.apply_damage(sp.breakup_damage_amount, pos);
                self.mission_control.outcome = MissionOutcome::Failure;
                self.mission_complete = true;
            }
        }

        self.simulation_time += dt;

        // 飞控计算制导指令（在 vessel.update 之前设置朝向和油门）
        let state = GuidanceState {
            altitude,
            velocity_mag: speed,
            position: self.vessel.body.get_position().clone(),
            velocity: self.vessel.body.get_velocity().clone(),
            mission_time: self.simulation_time,
            total_mass_kg: self.vessel.body.get_mass(),
            stage: self.vessel.current_stage,
            throttle: self.vessel.get_stage_throttle(self.vessel.current_stage),
        };
        let cmd = self.flight_computer.update(&state);

        // 将制导指令从局部垂直系变换到世界系
        // 注意：制导输出的坐标系取决于当前级：
        //   - Stage 0（芯级/余弦制导）：输出为 (sin(pitch), cos(pitch), 0) LVLH 局部坐标
        //   - Stage ≥1（ICPS/PEG 制导）：输出已经是世界系速度方向，无需变换
        let pos = *self.vessel.body.get_position();
        let up = pos.normalized();

        // 水平方向：优先用速度的水平分量，回退到东向
        let vel = *self.vessel.body.get_velocity();
        let vel_h = vel - vel.dot(&up) * up;
        let horiz = if vel_h.length_squared() > 1e-6 {
            vel_h.normalized()
        } else {
            // 发射台无速度：用东向
            let east = Vec3::new(-pos.z, 0.0, pos.x);
            let east_h = east - east.dot(&up) * up;
            if east_h.length_squared() > 1e-10 {
                east_h.normalized()
            } else {
                Vec3::new(1.0, 0.0, 0.0)
            }
        };
        let normal = up.cross(&horiz).normalized();

        // 制导输出从局部系变换到世界系
        let local_dir = cmd.thrust_direction;
        let world_dir = if self.vessel.current_stage <= 0 {
            // Stage 0（余弦制导）：LVLH → 世界
            (horiz * local_dir.x + up * local_dir.y + normal * local_dir.z).normalized()
        } else {
            // Stage ≥1（PEG 制导）：已经是世界系速度方向，直接使用
            local_dir.normalized()
        };

        self.vessel.body.set_orientation_from_dir(world_dir);
        // 应用制导油门（在 mission_control 之前，让 mission_control 有最终决定权）
        let active_stage = self.vessel.current_stage;
        let mut throttle = cmd.throttle;
        // 滑行阶段（MECO→ICPS点火前）：覆盖油门为 0
        if self.mission_control.coasting && !self.mission_control.icps_ignited {
            throttle = 0.0;
        }
        // 断油后（圆化完成→TLI前）：覆盖油门为 0
        if self.mission_control.cutoff_fired && !self.mission_control.tli_started {
            throttle = 0.0;
        }
        self.vessel.set_stage_throttle(active_stage, throttle);

        // TCM-1 覆盖：制导始终输出顺向（PEG stage≥1），但 TCM-1 需要逆向减速
        // 在引擎燃烧前强制设为逆向朝向
        if self.mission_control.apogee_tcm_started && !self.mission_control.apogee_tcm_attempted {
            let tcm_vel = *self.vessel.body.get_velocity();
            if tcm_vel.length() > 1.0 {
                self.vessel.body.set_orientation_from_dir(-tcm_vel.normalized());
            }
        }

        // TLI 拦截燃烧覆盖：按 Lambert 目标方向定向（覆盖制导的速度方向）
        if self.mission_control.tli_started
            && !self.mission_control.tli_complete
            && self.mission_control.tli_has_target
        {
            self.vessel
                .body
                .set_orientation_from_dir(self.mission_control.tli_target_dir);
        }

        // 发动机 & 推进剂
        let ambient_pressure = self.earth.get_atmosphere().get_pressure(altitude);
        let engine_status = self.vessel.update(dt, ambient_pressure);
        self.vessel.body.update(dt);

        // MissionControl 更新（阶段转换、遥测、触发器、退出条件检查）
        let moon_pos = if self.body_positions.len() > 1 {
            self.body_positions[1]
        } else {
            Vec3::zero()
        };
        let moon_vel = if self.body_velocities.len() > 1 {
            self.body_velocities[1]
        } else {
            Vec3::zero()
        };
        self.mission_control.update(
            dt,
            &engine_status,
            &mut self.vessel,
            &self.earth,
            moon_pos,
            moon_vel,
        );

        // 自动级分离：当前级燃料耗尽且有后续级时触发
        // 注意：滑行阶段（coasting=true）不算燃料耗尽，跳过自动级分离
        if engine_status.total_thrust < 1.0
            && engine_status.active_engines == 0
            && self.vessel.current_stage < self.vessel.find_highest_stage()
            && !self.mission_control.cutoff_fired
            && !self.mission_control.coasting
        {
            self.vessel.activate_next_stage();
            eprintln!(
                "  T+ {:7.1}s  AUTO_STAGE — Fuel depleted, activating next stage",
                self.simulation_time
            );
        }

        // 检查是否完成
        if self.mission_control.outcome != MissionOutcome::InProgress {
            self.mission_complete = true;
        }

        // 防止地面以下 + 溅落/坠毁检测
        if altitude < 0.0 && self.mission_control.outcome == MissionOutcome::InProgress {
            // 着陆检测（缓速触地 = 成功溅落）
            if speed < 20.0 {
                self.mission_control.outcome = MissionOutcome::Success;
                eprintln!(
                    "  T+ {:7.1}s  SPLASHDOWN — Landing at {:.1} m/s",
                    self.simulation_time, speed
                );
            } else {
                // 高速撞击 = 坠毁
                self.vessel.apply_damage((speed / 100.0).min(1.0), pos);
                self.mission_control.outcome = MissionOutcome::Failure;
                eprintln!(
                    "  T+ {:7.1}s  CRASH — Impact at {:.0} m/s, damage={:.1}",
                    self.simulation_time,
                    speed,
                    self.vessel.get_total_damage()
                );
            }
            let ground_pos = pos.normalized() * self.earth.get_radius();
            self.vessel.body.set_position(ground_pos);
            self.vessel.body.set_velocity(deepspace::Vec3::zero());
            self.mission_complete = true;
        }

        // 收集遥测（每 2 秒约 20 步，每 20 步录一次）
        if (self.simulation_time * 10.0) as i64 % 20 == 0 {
            self.telemetry_log
                .push(self.mission_control.telemetry.clone());
        }
    }

    /// 运行仿真直到任务完成或超过时间
    pub fn run(&mut self) {
        let max_time = self.config.max_duration_s;
        let print_interval = (max_time / 50.0).max(1.0);
        let mut next_print = print_interval;

        println!(
            "  Running simulation: dt={:.3}s, max={:.0}s",
            self.dt, max_time
        );
        println!();

        while !self.mission_complete && self.simulation_time < max_time {
            self.step(self.dt);

            if self.simulation_time >= next_print {
                let t = &self.mission_control.telemetry;
                println!(
                    "  T+{:7.1}s  [{:<11}]  alt={:>9.0}m  vel={:>7.0}m/s  mass={:>8.0}kg  thr={:>5.0}kN",
                    self.simulation_time,
                    self.mission_control.phase_name,
                    t.altitude_m, t.velocity_mps,
                    self.vessel.body.get_mass(),
                    t.thrust_n / 1000.0,
                );
                // 显示下一阶段进度
                if let Some(info) = self.mission_control.compute_next_phase_info(&self.vessel, &self.earth, self.moon_position()) {
                    let mut parts: Vec<String> = Vec::new();
                    for c in &info.conditions {
                        if c.is_boolean {
                            let status = if c.is_met { "done" } else { "pending" };
                            parts.push(format!("{} [{}]", c.label, status));
                        } else {
                            let pct = (c.progress * 100.0).min(99.9);
                            // 对比例类（<10）多显示一位小数
                            if c.target < 10.0 {
                                parts.push(format!("{} {:.2}/{:.2} [{:.0}%]", c.label, c.current, c.target, pct));
                            } else {
                                parts.push(format!("{} {:.0}/{:.0} [{:.0}%]", c.label, c.current, c.target, pct));
                            }
                        }
                    }
                    let sep = if info.require_all { " | " } else { " OR " };
                    println!("  Next → {}: {}", info.next_phase, parts.join(sep));
                }
                next_print += print_interval;
            }
        }

        // 最终状态
        println!();
        println!("  ==============================================");
        println!("  Simulation ended at T+{:.1}s", self.simulation_time);
        println!("  Outcome: {:?}", self.mission_control.outcome);
        println!(
            "  Max Q: {:.0} Pa at {:.0}m",
            self.mission_control.max_q, self.mission_control.max_q_altitude
        );

        // 写入 CSV
        if let Some(ref csv) = self.csv_path {
            match write_telemetry_csv(csv, &self.telemetry_log) {
                Ok(_) => println!("  Telemetry written to: {}", csv),
                Err(e) => eprintln!("  ERROR writing CSV: {}", e),
            }
        }

        // 打印触发的事件
        let events = &self.mission_control.triggered_events;
        if !events.is_empty() {
            println!();
            println!("  === Events ({}) ===", events.len());
            for e in events {
                println!("  T+{:7.1}s  {} — {}", e.time, e.name, e.description);
            }
        }
    }
}
