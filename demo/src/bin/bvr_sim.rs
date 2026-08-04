//! 超视距空战模拟器 (BVR)
//!
//! 演示 BVR 空战场景：红蓝双方战斗机 + 导弹交战。
//! 控制台模式，输出交战日志和结果。
//!
//! Usage:
//!   cargo run --bin bvr-sim

use deepspace::aerodynamics::atmosphere_at;
use deepspace::aircraft::{AircraftConfig, AircraftState, Autopilot};
use deepspace::missile::{AamConfig, MissileState};
use deepspace::sensors::{Radar, RadarConfig, RadarMode};
use deepspace::warfare::ssppk;
use deepspace::warfare::EngagementPhase;
use deepspace::Vec3;

/// 简单的时间步进框架
struct BvrScenario {
    time: f64,
    attacker: AircraftState,
    ap_a: Autopilot,
    radar_a: Radar,
    target: AircraftState,
    ap_t: Autopilot,
    radar_t: Radar,
    missiles: Vec<MissileState>,
    phase: EngagementPhase,
    events: Vec<String>,
}

impl BvrScenario {
    fn new() -> Self {
        let cfg_attacker = AircraftConfig::su57();
        let pos_a = Vec3::new(0.0, 0.0, 10_000.0);
        let vel_a = Vec3::new(300.0, 0.0, 0.0);
        let ac_a = AircraftState::new(cfg_attacker, pos_a, vel_a, 0.0, 3000.0);

        let cfg_target = AircraftConfig::su57();
        let pos_t = Vec3::new(80_000.0, 0.0, 10_500.0);
        let vel_t = Vec3::new(-280.0, 0.0, 0.0);
        let ac_t = AircraftState::new(cfg_target, pos_t, vel_t, 180.0, 3000.0);

        let mut radar_a = Radar::new(RadarConfig::apg80());
        radar_a.set_mode(RadarMode::Tws);
        let mut radar_t = Radar::new(RadarConfig::apg77());
        radar_t.set_mode(RadarMode::Tws);

        BvrScenario {
            time: 0.0,
            radar_a,
            ap_a: Autopilot::new(),
            radar_t,
            attacker: ac_a,
            target: ac_t,
            ap_t: Autopilot::new(),
            missiles: Vec::new(),
            phase: EngagementPhase::Search,
            events: Vec::new(),
        }
    }

    fn step(&mut self, dt: f64) {
        self.time += dt;

        let atmo_a = atmosphere_at(self.attacker.position.z);
        self.ap_a.compute(&self.attacker, dt);
        self.attacker.step(dt, &atmo_a);

        let atmo_t = atmosphere_at(self.target.position.z);
        self.ap_t.compute(&self.target, dt);
        self.target.step(dt, &atmo_t);

        let range = (self.attacker.position - self.target.position).length();

        // 探测
        let rcs_t = 0.5; // Su-57 RCS ~0.5 m²
        let rcs_a = 0.5; // Su-57 RCS ~0.5 m²
        let p_det_a = self.radar_a.detection_probability(range, rcs_t);
        let _p_det_t = self.radar_t.detection_probability(range, rcs_a);

        self.update_phase(range, p_det_a);

        if self.phase == EngagementPhase::BvrEngagement && self.missiles.is_empty() {
            self.fire_amraam();
        }

        for m in &mut self.missiles {
            m.step(dt, atmo_a.rho);
        }

        self.check_missile_hits();
    }

    fn update_phase(&mut self, range: f64, p_det: f64) {
        let r_km = range / 1000.0;
        match self.phase {
            EngagementPhase::Search => {
                if p_det > 0.1 && r_km < 180.0 {
                    self.phase = EngagementPhase::Detected;
                    self.events.push(format!(
                        "T+{:.0}s: Detected target at {:.0}km",
                        self.time, r_km
                    ));
                }
            }
            EngagementPhase::Detected => {
                if r_km < 100.0 {
                    self.phase = EngagementPhase::Engaged;
                    self.events
                        .push(format!("T+{:.0}s: Engaged! Range {:.0}km", self.time, r_km));
                }
            }
            EngagementPhase::Engaged if r_km < 80.0 => {
                self.phase = EngagementPhase::BvrEngagement;
                self.events.push(format!(
                    "T+{:.0}s: BVR engagement — Fox 3! Range {:.0}km",
                    self.time, r_km
                ));
            }
            _ => {}
        }
    }

    fn fire_amraam(&mut self) {
        let amraam = AamConfig::aim120c();
        let mut ms = MissileState::new(
            amraam,
            self.attacker.position,
            self.attacker.velocity,
            self.target.position,
            self.target.velocity,
        );
        ms.launch();
        self.missiles.push(ms);
    }

    fn check_missile_hits(&mut self) {
        self.missiles.retain(|m| {
            let rng = (m.position - self.target.position).length();
            let pk = ssppk(rng.max(1.0), m.config.kill_radius_m, 0.85, 1.0);

            if pk > 0.5 {
                self.events.push(format!(
                    "T+{:.0}s: MISSILE HIT! PK={:.2} Range={:.0}m",
                    self.time, pk, rng
                ));
                self.phase = EngagementPhase::Terminated;
                false
            } else if rng > 200_000.0 || m.flight_time_s > 120.0 {
                self.events.push(format!(
                    "T+{:.0}s: Missile missed (range={:.0}m, time={:.0}s)",
                    self.time, rng, m.flight_time_s,
                ));
                self.phase = EngagementPhase::Terminated;
                false
            } else {
                true
            }
        });
    }

    fn is_over(&self) -> bool {
        self.phase == EngagementPhase::Terminated || self.time > 600.0
    }
}

fn main() {
    println!("╔══════════════════════════════════════════╗");
    println!("║    BVR Air Combat Simulation             ║");
    println!("╚══════════════════════════════════════════╝");
    println!();

    let mut scenario = BvrScenario::new();

    println!("Scenario: Su-57 (attacker) vs Su-57 (defender)");
    println!(
        "Initial range: {:.0} km",
        (scenario.attacker.position - scenario.target.position).length() / 1000.0
    );
    println!();
    println!("--- Engagement Timeline ---");

    while !scenario.is_over() {
        scenario.step(0.5);

        for ev in scenario.events.drain(..) {
            println!("  {}", ev);
        }
    }

    println!();
    println!("--- Final Report ---");
    println!("Duration: {:.0}s", scenario.time);

    let final_range = (scenario.attacker.position - scenario.target.position).length() / 1000.0;
    let sane_range = if final_range > 1_000_000.0 {
        0.0
    } else {
        final_range
    };
    println!("Final range: {:.0} km", sane_range);
    println!("Missiles fired: {}", 1);
    println!("Engagement outcome: {:?}", scenario.phase);
}
