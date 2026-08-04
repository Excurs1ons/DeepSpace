//! 洲际弹道导弹飞行模拟器
//!
//! 演示 ICBM 飞行：三级火箭、中段 MIRV 部署、再入。
//! 控制台模式，输出弹道数据和轨迹。
//!
//! Usage:
//!   cargo run --bin icbm-sim

use deepspace::ballistics::{IcbmConfig, IcbmPhase, IcbmState};

fn main() {
    println!("╔══════════════════════════════════════════╗");
    println!("║    ICBM Flight Simulation                ║");
    println!("╚══════════════════════════════════════════╝");
    println!();

    // LGM-30 Minuteman III
    let mut icbm = IcbmState::new(IcbmConfig::minuteman3());
    icbm.launch();

    let (launch_lat, launch_lon, _) = IcbmState::ecef_to_geo(icbm.position_ecef);
    println!("Launch site:  {:.2}°N, {:.2}°W", launch_lat, -launch_lon);
    println!(
        "Target:       {:.2}°N, {:.2}°E",
        icbm.config.target_lat_deg, icbm.config.target_lon_deg
    );
    println!(
        "Range:        {:.0} km",
        IcbmState::great_circle_distance(
            launch_lat,
            launch_lon,
            icbm.config.target_lat_deg,
            icbm.config.target_lon_deg,
        ) / 1000.0
    );
    println!("Total mass:   {:.0} kg", icbm.config.total_mass());
    println!();

    println!("--- Flight Timeline ---");

    // 跟踪关键节点
    let mut printed_stage = 0usize;
    let mut printed_midcourse = false;
    let mut printed_mirv = false;
    let mut printed_reentry = false;
    let mut max_alt = 0.0;
    let mut last_phase = String::new();

    while !matches!(icbm.phase, IcbmPhase::Impact | IcbmPhase::Failed) {
        let dt = match icbm.phase {
            IcbmPhase::Boost(_) => 0.05,
            IcbmPhase::Midcourse | IcbmPhase::BusDeployment | IcbmPhase::Reentry => 0.1,
            _ => 0.5,
        };

        icbm.step(dt);
        let (lat, lon, alt) = IcbmState::ecef_to_geo(icbm.position_ecef);
        if alt > max_alt {
            max_alt = alt;
        }

        // 阶段变化追踪
        let phase_str = format!("{:?}", icbm.phase);
        if phase_str != last_phase {
            println!(
                "  T+{:7.1}s | Phase: {} | Alt {:.0} km | Vel {:.1} km/s",
                icbm.flight_time,
                phase_str,
                alt / 1000.0,
                icbm.velocity_ecef.length() / 1000.0
            );
            last_phase = phase_str.clone();
        }

        // 级分离报告
        if let IcbmPhase::Boost(stage) = icbm.phase {
            if stage != printed_stage {
                if icbm.stage_time < 0.1 {
                    println!(
                        "  T+{:7.1}s | Phase: {} | ({:.1}°,{:.1}°) Alt {:.0} km | Vel {:.1} km/s",
                        icbm.flight_time,
                        phase_str,
                        lat,
                        lon,
                        alt / 1000.0,
                        icbm.velocity_ecef.length() / 1000.0
                    );
                }
                printed_stage = stage;
            }
        }

        // 中段
        if matches!(icbm.phase, IcbmPhase::Midcourse) && !printed_midcourse {
            let pos = icbm.position_ecef;
            println!(
                "  T+{:7.1}s | Midcourse phase | Alt {:.0} km | Vel {:.1} km/s | Range {:.0} km | Pos=({:.0},{:.0},{:.0})",
                icbm.flight_time,
                alt / 1000.0,
                icbm.velocity_ecef.length() / 1000.0,
                icbm.range_to_target_m / 1000.0,
                pos.x, pos.y, pos.z,
            );
            printed_midcourse = true;
        }

        // MIRV 部署
        if matches!(icbm.phase, IcbmPhase::BusDeployment) && !printed_mirv {
            println!(
                "  T+{:7.1}s | Bus deployment: {} RVs | Alt {:.0} km",
                icbm.flight_time,
                icbm.config.reentry_vehicles.len(),
                alt / 1000.0,
            );
            printed_mirv = true;
        }

        // 再入
        if matches!(icbm.phase, IcbmPhase::Reentry) && !printed_reentry {
            println!(
                "  T+{:7.1}s | Reentry | Alt {:.0} km | Vel {:.1} km/s",
                icbm.flight_time,
                alt / 1000.0,
                icbm.velocity_ecef.length() / 1000.0,
            );
            printed_reentry = true;
        }
    }

    // 最终报告
    let (lat, lon, final_alt) = IcbmState::ecef_to_geo(icbm.position_ecef);
    println!(
        "  T+{:7.1}s | IMPACT | Alt {:.1} m | ({:.1}°,{:.1}°) | Range to target {:.0} km",
        icbm.flight_time,
        final_alt,
        lat,
        lon,
        icbm.range_to_target_m / 1000.0,
    );
    println!();

    println!("--- Final Report ---");
    println!("Duration:      {:.1} s", icbm.flight_time);
    println!("Max altitude:  {:.0} km", max_alt / 1000.0);
    println!(
        "RVs deployed:  {}/{}",
        icbm.rv_deployed.iter().filter(|&&d| d).count(),
        icbm.rv_deployed.len()
    );
}
