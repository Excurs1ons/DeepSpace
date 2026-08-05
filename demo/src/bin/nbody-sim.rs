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

fn body_color(name: &str) -> macroquad::color::Color {
    use demo::render::*;
    match name {
        n if n.contains("Sun") || n.contains("sun") || n.contains("Star") => COLOR_SUN,
        n if n.contains("Mercury") => COLOR_MERCURY,
        n if n.contains("Venus") => COLOR_VENUS,
        n if n.contains("Earth") => COLOR_EARTH,
        n if n.contains("Mars") => COLOR_MARS,
        n if n.contains("Jupiter") => COLOR_JUPITER,
        n if n.contains("Saturn") => COLOR_SATURN,
        n if n.contains("Uranus") => COLOR_URANUS,
        n if n.contains("Neptune") => COLOR_NEPTUNE,
        n if n.contains("Moon") || n.contains("moon") => COLOR_MOON,
        _ => macroquad::color::Color::new(0.6, 0.6, 0.6, 1.0),
    }
}

// =====================================================================
// 3D 可视化模式（2D 正交投影，参考 rocket-sim）
// =====================================================================
async fn viz_main(scene_path: String) {
    use demo::render::*;
    use macroquad::color::Color;
    use macroquad::math::Vec3;
    use macroquad::prelude::*;

    let config =
        deepspace::scene::SceneConfig::load(&scene_path).expect("Failed to load scene config");
    println!("Scene: {} ({} bodies)", config.name, config.bodies.len());

    let mut runtime = deepspace::scene::SceneRuntime::new(&config);

    // 加载自定义字体（NASA Eyes 风格：Metropolis）
    load_custom_font(FONT_PATH).await;

    let n = runtime.sys.bodies.len();
    let mut trails: Vec<Trail> = (0..n).map(|_| Trail::new(200)).collect();

    // 根据系统尺度自动设置相机距离
    let max_dist = runtime
        .sys
        .bodies
        .iter()
        .map(|b| {
            (b.position.x * b.position.x
                + b.position.y * b.position.y
                + b.position.z * b.position.z)
                .sqrt()
        })
        .fold(1.0e10_f64, f64::max);
    let mut cam = OrbitalCamera::new(Vec3::ZERO, (max_dist * 2.5) as f32);
    cam.max_distance = (max_dist * 50.0) as f32;
    cam.min_distance = (max_dist * 0.01) as f32;

    // NASA Eyes 风格：星空 + 时间控制条 + 点击选中
    let stars = StarField::new(600, 0x5EED);
    let mut time_bar = TimeControlBar::new();
    let mut click = ClickDetector::new();
    let mut selected: Option<usize> = None;

    loop {
        cam.update();
        if is_key_down(KeyCode::Escape) {
            break;
        }
        time_bar.update();
        // 窗口尺寸（点击检测 / 渲染共用）
        let sw = screen_width();
        let sh = screen_height();
        if !time_bar.paused {
            // 倍率驱动的步进：rate 为 1 时等价于原 1 步/帧
            let steps = time_bar.rate.max(0.001) as usize;
            for _ in 0..steps {
                runtime.step();
            }
        }

        for (i, body) in runtime.sys.bodies.iter().enumerate() {
            if i < trails.len() {
                trails[i].push(to_mvec3(body.position));
            }
        }

        // 点击选中（短按，区分拖拽）
        if click.update() {
            let (mx, my) = mouse_position();
            let mut best: Option<(usize, f32)> = None;
            for (i, body) in runtime.sys.bodies.iter().enumerate() {
                let (cx, cy) = cam.project_2d(to_mvec3(body.position), sw, sh);
                let d = (mx - cx) * (mx - cx) + (my - cy) * (my - cy);
                if d < 40.0 * 40.0 && best.is_none_or(|(_, bd)| d < bd) {
                    best = Some((i, d));
                }
            }
            selected = best.map(|(i, _)| i);
        }

        // 跟随选中天体（相机 target 平滑趋近 — NASA Eyes follow 相机）
        if let Some(idx) = selected {
            if let Some(body) = runtime.sys.bodies.get(idx) {
                cam.target = cam.target.lerp(to_mvec3(body.position), 0.1);
            }
        }

        // -----------------------------------------------------------------
        // 真 3D 多行星轨道渲染（NASA Eyes solar-system 风格）
        // -----------------------------------------------------------------
        let s = ui_scale();

        // 深空背景 + 恒星（NASA Eyes）
        clear_background(COLOR_SPACE_BG);
        cam.set(); // 激活 3D 相机
        stars.draw(&cam, sw, sh);

        // 轨道环（每颗天体的轨道平面 = position×velocity 法线，半透明）
        for body in runtime.sys.bodies.iter() {
            if body.position.length() < 1.0e-6 {
                continue; // 中心天体（太阳）画星系中心环
            }
            let c = body_color(&body.name);
            let radius = to_mvec3(body.position).length();
            if radius > 0.0 {
                let axis = to_mvec3(body.position)
                    .cross(to_mvec3(body.velocity))
                    .normalize();
                draw_orbit_ring_3d(
                    Vec3::ZERO,
                    radius,
                    axis,
                    128,
                    Color::new(c.r * 0.6, c.g * 0.6, c.b * 0.6, 0.25),
                );
            }
        }

        // 渐变轨迹（3D：旧→新从蓝到天体本色）
        for (i, body) in runtime.sys.bodies.iter().enumerate() {
            if i >= trails.len() {
                continue;
            }
            let pts = trails[i].points();
            if pts.len() > 1 {
                let c = body_color(&body.name);
                let steps = pts.len() - 1;
                for k in 0..steps {
                    let t = k as f32 / steps as f32;
                    let old = Color::new(0.2, 0.45, 0.9, 0.3);
                    let seg = lerp_color(old, c, t);
                    draw_line_3d(pts[k], pts[k + 1], seg);
                }
            }
        }

        // 发光天体（3D 球体；发光用 2D 光晕在默认相机层叠加）
        for body in runtime.sys.bodies.iter() {
            let pos = to_mvec3(body.position);
            let color = body_color(&body.name);
            let r = body.radius as f32;
            draw_sphere(pos, r, None, color);
        }

        // 切回默认相机叠加 2D 光晕 + 选中环 + 标签
        OrbitalCamera::set_default();

        for body in runtime.sys.bodies.iter() {
            let pos = to_mvec3(body.position);
            let (cx, cy) = cam.project_2d(pos, sw, sh);
            let r_px = cam.len_to_px(body.radius as f32, sw, sh).max(3.0);
            let color = body_color(&body.name);
            let intensity = if body.name.contains("Sun") || body.name.contains("sun") {
                2.5
            } else {
                1.0
            };
            draw_glow_2d(cx, cy, r_px, color, intensity);

            // 名称标签
            text(
                &body.name,
                cx + r_px + 6.0 * s,
                cy + 6.0 * s,
                20.0 * s,
                color,
            );
        }

        // 选中光环 + 附加信息
        if let Some(idx) = selected {
            if let Some(body) = runtime.sys.bodies.get(idx) {
                let (cx, cy) = cam.project_2d(to_mvec3(body.position), sw, sh);
                let r_px = cam.len_to_px(body.radius as f32, sw, sh).max(3.0);
                draw_selection_ring(cx, cy, r_px);
            }
        }

        // -----------------------------------------------------------------
        // HUD 文字（NASA Eyes 面板化）
        // -----------------------------------------------------------------
        // 左上信息面板
        let panel_w = 300.0 * s;
        let panel_h = 130.0 * s;
        draw_panel(10.0 * s, 40.0 * s, panel_w, panel_h);
        text(
            format!("Scene: {}", config.name),
            20.0 * s,
            60.0 * s,
            24.0 * s,
            WHITE,
        );
        text(
            format!("Time: {:.2e} s", runtime.sys.time),
            20.0 * s,
            88.0 * s,
            18.0 * s,
            LIGHTGRAY,
        );
        text(
            format!("Bodies: {n}   dt: {:.1e} s", config.dt),
            20.0 * s,
            112.0 * s,
            18.0 * s,
            LIGHTGRAY,
        );
        if let Some(idx) = selected {
            if let Some(body) = runtime.sys.bodies.get(idx) {
                text(
                    format!("Track: {}", body.name),
                    20.0 * s,
                    142.0 * s,
                    18.0 * s,
                    body_color(&body.name),
                );
            }
        }

        // 天体列表面板（右侧）
        let lx = sw - 260.0 * s;
        let list_h = (runtime.sys.bodies.len() as f32 * 28.0 * s) + 40.0 * s;
        draw_panel(lx, 30.0 * s, 250.0 * s, list_h);
        text("Celestial Bodies", lx + 10.0 * s, 50.0 * s, 20.0 * s, WHITE);
        for (i, body) in runtime.sys.bodies.iter().enumerate() {
            let y = 76.0 * s + i as f32 * 28.0 * s;
            let c = body_color(&body.name);
            draw_rectangle(lx + 10.0 * s, y - 2.0 * s, 16.0 * s, 16.0 * s, c);
            let label = if selected == Some(i) {
                format!("▶ {}  M={:.2e}kg", body.name, body.mass)
            } else {
                format!("{}  M={:.2e}kg", body.name, body.mass)
            };
            text(
                &label,
                lx + 32.0 * s,
                y + 12.0 * s,
                18.0 * s,
                if selected == Some(i) {
                    Color::new(1.0, 0.85, 0.4, 1.0)
                } else {
                    LIGHTGRAY
                },
            );
        }

        // 底部时间控制条（NASA Eyes）
        time_bar.draw(runtime.sys.time);

        // 操作提示（时间条上方一行）
        text(
            "Left-drag: Rotate | Click: Select/Track | Scroll: Zoom | ESC: Exit",
            10.0,
            sh - 46.0 * s,
            16.0 * s,
            Color::new(0.5, 0.6, 0.7, 0.8),
        );

        next_frame().await;
    }
}

// =====================================================================
// 控制台模式
// =====================================================================
fn headless_main(scene_path: &str, csv_path: Option<&str>, switch_file: Option<&str>) {
    match deepspace::scene::SceneConfig::load(scene_path) {
        Err(e) => {
            eprintln!("[nbody-sim] error: {e}");
            std::process::exit(1);
        }
        Ok(config) => {
            eprintln!(
                "[nbody-sim] scene '{}' loaded ({} bodies)",
                config.name,
                config.bodies.len()
            );
            let mut runtime = deepspace::scene::SceneRuntime::new(&config);
            let report_interval = (config.duration / config.dt / 100.0).max(1.0) as usize;
            match runtime.run_loop(config.duration, report_interval, csv_path, switch_file) {
                Ok(lines) => eprintln!("[nbody-sim] complete — {lines} CSV lines"),
                Err(e) => {
                    eprintln!("[nbody-sim] error: {e}");
                    std::process::exit(1);
                }
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
            "--headless" | "-h" => {
                headless = true;
            }
            "--help" => {
                print_help = true;
            }
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
        std::process::exit(if headless && scene_path.is_none() {
            1
        } else {
            0
        });
    }

    let path = scene_path.unwrap_or_else(|| "scenes/solar_system.scene".to_string());

    if headless {
        headless_main(&path, csv_path.as_deref(), switch_file.as_deref());
    } else {
        macroquad::Window::new("DeepSpace — N-body Simulation", viz_main(path));
    }
}
