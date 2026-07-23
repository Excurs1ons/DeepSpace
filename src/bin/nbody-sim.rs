//! N体场景仿真 — 宇宙沙盘风格模拟器
//!
//! 运行预定义的场景文件，支持运行时热切换场景。
//!
//! Usage:
//!   cargo run --bin nbody-sim -- --scene scenes/solar_system.scene
//!   cargo run --bin nbody-sim -- --scene scenes/three_body.scene --csv output.csv
//!   cargo run --bin nbody-sim -- --scene scenes/figure8.scene --switch-file /path/to/switch

use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut scene_path: Option<String> = None;
    let mut csv_path: Option<String> = None;
    let mut switch_file: Option<String> = None;
    let mut print_help = false;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--scene" | "-s" => {
                i += 1;
                if i < args.len() {
                    scene_path = Some(args[i].clone());
                }
            }
            "--csv" | "-c" => {
                i += 1;
                if i < args.len() {
                    csv_path = Some(args[i].clone());
                }
            }
            "--switch-file" | "-w" => {
                i += 1;
                if i < args.len() {
                    switch_file = Some(args[i].clone());
                }
            }
            "--help" | "-h" => {
                print_help = true;
            }
            _ => {}
        }
        i += 1;
    }

    if print_help || scene_path.is_none() {
        eprintln!("Usage: nbody-sim --scene <file> [options]");
        eprintln!("  --scene <file>, -s    Scene configuration file (required)");
        eprintln!("  --csv <file>, -c      Output CSV telemetry file");
        eprintln!("  --switch-file <path>  Watch file for scene hot-switching");
        eprintln!("                        Write a scene file path into it at runtime");
        eprintln!("  --help, -h            Show this help");
        eprintln!();
        eprintln!("Built-in scenes:");
        eprintln!("  scenes/solar_system.scene  Sun + 4 inner planets");
        eprintln!("  scenes/three_body.scene    Star + 2 planets");
        eprintln!("  scenes/figure8.scene       Chenciner-Montgomery figure-8 orbit");
        std::process::exit(if scene_path.is_none() { 1 } else { 0 });
    }

    let path = scene_path.unwrap();
    eprintln!("[nbody-sim] loading scene: {path}");

    match deepspace::scene::SceneConfig::load(&path) {
        Err(e) => {
            eprintln!("[nbody-sim] error: {e}");
            std::process::exit(1);
        }
        Ok(config) => {
            eprintln!("[nbody-sim] scene '{}' loaded ({} bodies)", config.name, config.bodies.len());
            eprintln!("[nbody-sim] dt={}, integrator={}, duration={}s",
                config.dt,
                if config.integrator == deepspace::scene::IntegratorType::Symplectic4 {
                    "symplectic4"
                } else {
                    "leapfrog"
                },
                config.duration,
            );

            let mut runtime = deepspace::scene::SceneRuntime::new(&config);
            let report_interval = (config.duration / config.dt / 100.0).max(1.0) as usize;

            match runtime.run_loop(config.duration, report_interval, csv_path.as_deref(), switch_file.as_deref()) {
                Ok(lines) => {
                    eprintln!("[nbody-sim] simulation complete — {lines} CSV lines");
                }
                Err(e) => {
                    eprintln!("[nbody-sim] runtime error: {e}");
                    std::process::exit(1);
                }
            }
        }
    }
}
