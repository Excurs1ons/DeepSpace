//! 火箭任务模拟 + 3D 可视化
//!
//! 无 --headless → 3D 可视化窗口
//! --headless    → 控制台仿真，可用 --csv 输出遥测
//!
//! Usage:
//!   cargo run --bin rocket-sim
//!   cargo run --bin rocket-sim -- --headless
//!   cargo run --bin rocket-sim -- --headless --mission missions/artemis2.conf --csv out.csv

use demo::app::CliArgs;

/// 地球标准引力参数 μ = G · M (m³/s²)
const EARTH_MU: f64 = 6.674_30e-11 * 5.972_2e24;

// =====================================================================
// 控制台模式
// =====================================================================
fn headless_main(args: CliArgs) {
    let mut app = demo::app::SimulationApp::new(&args);
    app.run();
}

// =====================================================================
// 3D 可视化模式
// =====================================================================
async fn viz_main(args: CliArgs) {
    use macroquad::prelude::*;
    use macroquad::math::Vec3;
    use demo::render::*;

    // 时间倍率，不受显示器帧率影响
    let mut time_warp: f64 = 1.0;
    let mut app = demo::app::SimulationApp::new(&args);
    let mut flight_path: Vec<Vec3> = Vec::new();
    let mut predicted_path: Vec<Vec3> = Vec::new();

    let earth_radius = app.earth.get_radius() as f32;

    // 轨道相机
    let launch_pad = Vec3::new(0.0, earth_radius, 0.0);
    let mut camera = OrbitalCamera::new(launch_pad, earth_radius * 3.0);
    camera.elevation = 0.2;
    camera.min_distance = (earth_radius * 0.005).max(1000.0); // ≈32km, 不会穿入火箭
    let mut track_rocket = true;

    loop {
        // -----------------------------------------------------------------
        // 1. 输入
        // -----------------------------------------------------------------
        camera.update();
        if is_key_down(KeyCode::Escape) {
            break;
        }
        if is_key_pressed(KeyCode::T) {
            track_rocket = !track_rocket;
        }
        // 时间倍率：右键 ×2，左键 ×0.5，范围 [-200, 200]
        // 负值 = 倒放
        if is_key_pressed(KeyCode::Right) {
            time_warp = (time_warp * 2.0).min(200.0);
        }
        if is_key_pressed(KeyCode::Left) {
            time_warp = (time_warp * 0.5).max(-200.0);
        }

        // -----------------------------------------------------------------
        // 2. 物理步进（基于真实帧间隔 × 时间倍率，帧率无关，支持倒放）
        // -----------------------------------------------------------------
        if !app.mission_complete {
            let real_dt = get_frame_time() as f64;          // 真实秒数
            let sim_dt = real_dt * time_warp;               // 可为负（倒放）
            let n_substeps = ((sim_dt.abs() / 0.016).ceil().max(1.0)) as usize;
            let sub_dt = sim_dt / n_substeps as f64;
            for _ in 0..n_substeps {
                app.step(sub_dt);
            }

            let pos = to_mvec3(*app.vessel.body.get_position());
            if flight_path.len() < 5000 {
                flight_path.push(pos);
            }
            if track_rocket {
                // 2帧 lerp：每帧趋近 50%，2 帧后到达 ~87%
                camera.target = camera.target.lerp(pos, 0.5);
            }
        }

            // 预测轨道（从当前状态前向传播，仅二体引力）
            let current_pos = *app.vessel.body.get_position();
            let current_vel = *app.vessel.body.get_velocity();
            if current_vel.length() > 10.0 {
                let raw = predict_trajectory(current_pos, current_vel, EARTH_MU, 15000.0, 150, app.earth.get_radius());
                predicted_path = raw.iter().map(|&p| to_mvec3(p)).collect();
            } else {
                predicted_path.clear();
            }

        // -----------------------------------------------------------------
        // 3. 2D 正交投影渲染 — 整个场景为 2D HUD
        //   旋转相机 = 切换 2D 投影剖面
        // -----------------------------------------------------------------
        // 注：不使用 set_camera / 3D 管线，全部 2D 绘制
        let sw = screen_width();
        let sh = screen_height();

        // 空间参考网格
        draw_grid_2d(&camera, earth_radius, sw, sh);

        // 地球（2D 圆 + 十字）
        draw_earth_2d(&camera, earth_radius, sw, sh);

        // 飞行路径（历史轨迹）
        if flight_path.len() > 1 {
            draw_path_2d(&camera, &flight_path, sw, sh, COLOR_TRAJECTORY);
        }

        // 预测轨道线（虚线）
        if predicted_path.len() > 1 {
            let raw: Vec<deepspace::Vec3> = predicted_path
                .iter()
                .map(|&v| deepspace::Vec3::new(v.x as f64, v.y as f64, v.z as f64))
                .collect();
            draw_predicted_path_2d(&camera, &raw, sw, sh, COLOR_PREDICTION);
        }

        // 火箭标记 + 速度方向箭头
        let rpos = to_mvec3(*app.vessel.body.get_position());
        let vel = *app.vessel.body.get_velocity();
        draw_rocket_2d(&camera, rpos, to_mvec3(vel), earth_radius, sw, sh);

        // -----------------------------------------------------------------
        // 4. 姿态指示器 + 遥测 HUD
        // -----------------------------------------------------------------
        if vel.length() > 1.0 {
            let vd = to_mvec3(vel.normalized());
            let quat = if vd.dot(Vec3::Y).abs() < 0.999 {
                Quat::from_axis_angle(
                    Vec3::Y.cross(vd).normalize(),
                    Vec3::Y.dot(vd).acos(),
                )
            } else if vd.dot(Vec3::Y) > 0.0 {
                Quat::IDENTITY
            } else {
                Quat::from_axis_angle(Vec3::X, 180.0_f32.to_radians())
            };
            let gx = screen_width() - 70.0;
            let gy = screen_height() - 70.0;
            draw_attitude_indicator_2d(gx, gy, 40.0, &quat, camera.eye_position(), camera.target);
        }

        let tel = &app.mission_control.telemetry;
        let dc = Color::new(0.8, 0.9, 1.0, 1.0);
        let lh = 20.0;
        let y0 = 80.0;

        draw_text(&format!("Mission: {}", app.config.mission_name), 10.0, 24.0, 22.0, WHITE);
        draw_text(&format!("T+ {:.1}s", app.simulation_time), 10.0, 48.0, 20.0, LIGHTGRAY);

        draw_text(
            &format!("Phase: {}", app.mission_control.current_phase.to_str()),
            10.0, y0, 16.0, YELLOW,
        );
        draw_text(&format!("Altitude: {:.0} m", tel.altitude_m), 10.0, y0 + lh, 16.0, dc);
        draw_text(&format!("Velocity: {:.0} m/s", tel.velocity_mps), 10.0, y0 + lh * 2.0, 16.0, dc);
        draw_text(&format!("Mass: {:.0} kg", app.vessel.body.get_mass()), 10.0, y0 + lh * 3.0, 16.0, dc);
        draw_text(&format!("Thrust: {:.0} kN", tel.thrust_n / 1000.0), 10.0, y0 + lh * 4.0, 16.0, dc);
        draw_text(&format!("Throttle: {:.0}%", tel.throttle_pct * 100.0), 10.0, y0 + lh * 5.0, 16.0, dc);
        draw_text(&format!("Mach: {:.2}", tel.mach), 10.0, y0 + lh * 6.0, 16.0, dc);
        draw_text(&format!("Q: {:.0} Pa", tel.dynamic_pressure_pa), 10.0, y0 + lh * 7.0, 16.0, dc);
        draw_text(&format!("Stage: {}", app.vessel.current_stage), 10.0, y0 + lh * 8.0, 16.0, dc);
        draw_text(
            &format!("Apoapsis: {:.0} km", tel.orbit.apoapsis_m / 1000.0),
            10.0, y0 + lh * 9.0, 16.0, dc,
        );
        draw_text(
            &format!("Periapsis: {:.0} km", tel.orbit.periapsis_m / 1000.0),
            10.0, y0 + lh * 10.0, 16.0, dc,
        );
        draw_text(
            &format!(
                "Orbit: {}",
                if tel.orbit.is_bound { "Bound" } else { "Suborbital" }
            ),
            10.0, y0 + lh * 11.0, 16.0,
            if tel.orbit.is_bound { GREEN } else { YELLOW },
        );

        let warp_color = if time_warp < -0.01 {
            Color::new(1.0, 0.2, 0.2, 1.0)       // 红色 = 倒放
        } else if time_warp > 1.0 {
            Color::new(1.0, 0.6, 0.0, 1.0)       // 橙色 = 快进
        } else if time_warp < 0.99 {
            Color::new(0.3, 0.8, 1.0, 1.0)       // 蓝色 = 慢放
        } else {
            LIGHTGRAY                              // 灰色 = 1:1
        };
        draw_text(
            &format!("Time warp: {:.1}x", time_warp),
            screen_width() / 2.0 - 60.0,
            24.0,
            18.0,
            warp_color,
        );

        draw_text(
            "Left-drag: Rotate | Scroll: Zoom | T: Track | ←→: Warp | ESC: Exit",
            10.0, screen_height() - 50.0, 14.0, DARKGRAY,
        );

        next_frame().await;
    }
}

// =====================================================================
// 入口
// =====================================================================
fn main() {
    let args = CliArgs::parse();
    if args.headless {
        headless_main(args);
    } else {
        macroquad::Window::new("DeepSpace — Rocket Launch", viz_main(args));
    }
}
