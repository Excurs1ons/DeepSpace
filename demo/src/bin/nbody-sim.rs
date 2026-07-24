//! N体场景仿真 + 3D 可视化
//!
//! 无 --headless → 3D 可视化窗口
//! --headless    → 控制台仿真，可用 --csv 输出遥测
//!
//! Usage:
//!   cargo run --bin nbody-sim -- --scene scenes/figure8.scene
//!   cargo run --bin nbody-sim -- --scene scenes/three_body.scene --headless --csv out.csv
//!   cargo run --bin nbody-sim                                        # 默认 solar_system（viz）

use std::env;

// =====================================================================
// 轨道尾迹
// =====================================================================
const TRAIL_LENGTH: usize = 200;
struct Trail {
    points: Vec<macroquad::math::Vec3>,
    cursor: usize,
    full: bool,
}
impl Trail {
    fn new() -> Self {
        Self { points: vec![macroquad::math::Vec3::ZERO; TRAIL_LENGTH], cursor: 0, full: false }
    }
    fn push(&mut self, pos: macroquad::math::Vec3) {
        self.points[self.cursor] = pos;
        self.cursor = (self.cursor + 1) % TRAIL_LENGTH;
        if self.cursor == 0 { self.full = true; }
    }
    fn points(&self) -> Vec<macroquad::math::Vec3> {
        if self.full {
            let (a, b) = self.points.split_at(self.cursor);
            [a, b].concat()
        } else {
            self.points[..self.cursor].to_vec()
        }
    }
}

fn body_color(name: &str) -> macroquad::color::Color {
    use demo::render::*;
    match name {
        n if n.contains("Sun") || n.contains("sun") || n.contains("Star") => COLOR_SUN,
        n if n.contains("Mercury") => COLOR_MERCURY,
        n if n.contains("Venus") => COLOR_VENUS,
        n if n.contains("Earth") => COLOR_EARTH,
        n if n.contains("Mars") => COLOR_MARS,
        n if n.contains("Jupiter") => COLOR_JUPITER,
        n if n.contains("Moon") || n.contains("moon") => COLOR_MOON,
        _ => macroquad::color::Color::new(0.6, 0.6, 0.6, 1.0),
    }
}

// =====================================================================
// 3D 可视化模式（2D 正交投影，参考 rocket-sim）
// =====================================================================
async fn viz_main(scene_path: String) {
    use macroquad::prelude::*;
    use macroquad::color::Color;
    use macroquad::math::Vec3;
    use demo::render::*;

    let config = deepspace::scene::SceneConfig::load(&scene_path)
        .expect("Failed to load scene config");
    println!("Scene: {} ({} bodies)", config.name, config.bodies.len());

    let mut runtime = deepspace::scene::SceneRuntime::new(&config);
    let n = runtime.sys.bodies.len();
    let mut trails: Vec<Trail> = (0..n).map(|_| Trail::new()).collect();

    // 根据系统尺度自动设置相机距离
    let max_dist = runtime.sys.bodies.iter()
        .map(|b| (b.position.x * b.position.x + b.position.y * b.position.y + b.position.z * b.position.z).sqrt())
        .fold(1.0e10_f64, f64::max);
    let mut camera = OrbitalCamera::new(Vec3::ZERO, (max_dist * 2.5) as f32);
    camera.max_distance = (max_dist * 50.0) as f32;
    camera.min_distance = (max_dist * 0.01) as f32;

    loop {
        camera.update();
        if is_key_down(KeyCode::Escape) {
            break;
        }

        runtime.step();

        for (i, body) in runtime.sys.bodies.iter().enumerate() {
            if i < trails.len() { trails[i].push(to_mvec3(body.position)); }
        }

        // -----------------------------------------------------------------
        // 2D 正交投影渲染（不使用 3D 管线，避免天文尺度下 f32 精度问题）
        // -----------------------------------------------------------------
        let sw = screen_width();
        let sh = screen_height();

        // 绘制轨道尾迹
        for (i, body) in runtime.sys.bodies.iter().enumerate() {
            if i < trails.len() {
                let pts = trails[i].points();
                if pts.len() > 1 {
                    let c = body_color(&body.name);
                    let trail_color = Color::new(c.r * 0.6, c.g * 0.6, c.b * 0.6, 0.4);
                    draw_path_2d(&camera, &pts, sw, sh, trail_color);
                }
            }
        }

        // 绘制天体（2D 圆 + 标签）
        for body in runtime.sys.bodies.iter() {
            let pos = to_mvec3(body.position);
            let (cx, cy) = camera.project_2d(pos, sw, sh);
            // 天体半径投影到像素，最小 3px 保证可见
            let r_px = camera.len_to_px(body.radius as f32, sw, sh).max(3.0);
            let color = body_color(&body.name);

            // 填充圆（用多层同心圆模拟填充）
            draw_circle_2d(cx, cy, r_px, color);
            if r_px > 4.0 {
                draw_circle_2d(cx, cy, r_px * 0.7, color);
                draw_circle_2d(cx, cy, r_px * 0.4, color);
            }
            // 中心点
            draw_line(cx - 1.0, cy, cx + 1.0, cy, 2.0, color);

            // 名称标签
            draw_text(&body.name, cx + r_px + 4.0, cy + 4.0, 14.0, color);
        }

        // -----------------------------------------------------------------
        // HUD 文字
        // -----------------------------------------------------------------
        draw_text(&format!("Scene: {}", config.name), 10.0, 24.0, 20.0, WHITE);
        draw_text(&format!("Time: {:.2e} s", runtime.sys.time), 10.0, 48.0, 18.0, LIGHTGRAY);
        draw_text(&format!("Bodies: {n}"), 10.0, 70.0, 16.0, GRAY);
        draw_text(&format!("dt: {:.1e} s", config.dt), 10.0, 90.0, 16.0, GRAY);
        draw_text("Left-drag: Rotate | Scroll: Zoom | ESC: Exit", 10.0, sh - 50.0, 14.0, DARKGRAY);

        // 天体列表面板
        let lx = sw - 240.0;
        draw_text("Celestial Bodies", lx, 24.0, 18.0, WHITE);
        for (i, body) in runtime.sys.bodies.iter().enumerate() {
            let y = 48.0 + i as f32 * 20.0;
            let c = body_color(&body.name);
            draw_rectangle(lx, y - 2.0, 12.0, 12.0, c);
            draw_text(&format!("{}  M={:.2e}kg", body.name, body.mass), lx + 16.0, y + 8.0, 14.0, LIGHTGRAY);
        }

        next_frame().await;
    }
}

// =====================================================================
// 控制台模式
// =====================================================================
fn headless_main(scene_path: &str, csv_path: Option<&str>, switch_file: Option<&str>) {
    match deepspace::scene::SceneConfig::load(scene_path) {
        Err(e) => { eprintln!("[nbody-sim] error: {e}"); std::process::exit(1); }
        Ok(config) => {
            eprintln!("[nbody-sim] scene '{}' loaded ({} bodies)", config.name, config.bodies.len());
            let mut runtime = deepspace::scene::SceneRuntime::new(&config);
            let report_interval = (config.duration / config.dt / 100.0).max(1.0) as usize;
            match runtime.run_loop(config.duration, report_interval, csv_path, switch_file) {
                Ok(lines) => eprintln!("[nbody-sim] complete — {lines} CSV lines"),
                Err(e) => { eprintln!("[nbody-sim] error: {e}"); std::process::exit(1); }
            }
        }
    }
}

// =====================================================================
// 入口
// =====================================================================
fn main() {
    let args: Vec<String> = env::args().collect();
    let mut scene_path: Option<String> = None;
    let mut csv_path: Option<String> = None;
    let mut switch_file: Option<String> = None;
    let mut headless = false;
    let mut print_help = false;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--scene" | "-s" => { i += 1; if i < args.len() { scene_path = Some(args[i].clone()); } }
            "--csv" | "-c" => { i += 1; if i < args.len() { csv_path = Some(args[i].clone()); } }
            "--switch-file" | "-w" => { i += 1; if i < args.len() { switch_file = Some(args[i].clone()); } }
            "--headless" | "-h" => { headless = true; }
            "--help" => { print_help = true; }
            _ => {}
        }
        i += 1;
    }

    if print_help || (headless && scene_path.is_none()) {
        eprintln!("Usage: nbody-sim [--scene <file>] [options]");
        eprintln!("  (no --headless)        3D visualization (default scene if omitted)");
        eprintln!("  --headless, -h         Console mode (requires --scene)");
        eprintln!("  --scene <file>, -s     Scene configuration file");
        eprintln!("  --csv <file>, -c       Output CSV telemetry (headless only)");
        eprintln!("  --switch-file <path>   Watch file for scene hot-switching");
        eprintln!("  --help                 Show this help");
        eprintln!();
        eprintln!("Built-in scenes:");
        eprintln!("  scenes/solar_system.scene  Sun + 4 inner planets");
        eprintln!("  scenes/three_body.scene    Star + 2 planets");
        eprintln!("  scenes/figure8.scene       Chenciner-Montgomery figure-8 orbit");
        std::process::exit(if headless && scene_path.is_none() { 1 } else { 0 });
    }

    let path = scene_path.unwrap_or_else(|| "scenes/solar_system.scene".to_string());

    if headless {
        headless_main(&path, csv_path.as_deref(), switch_file.as_deref());
    } else {
        macroquad::Window::new("DeepSpace — N-body Simulation", viz_main(path));
    }
}
