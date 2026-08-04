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
    use demo::render::*;
    use macroquad::math::Vec3;
    use macroquad::prelude::*;

    // 时间倍率，不受显示器帧率影响
    let mut time_warp: f64 = 1.0;
    let mut app = demo::app::SimulationApp::new(&args);

    // 加载自定义字体（使用项目的 Roboto 字体）
    // 相对于可执行文件运行目录的路径，开发时为项目根目录
    load_custom_font("assets/fonts/Roboto-Regular.ttf").await;

    let mut flight_path = Trail::new(8000);
    let mut predicted_path: Vec<Vec3> = Vec::new();

    let earth_radius = app.earth.get_radius() as f32;

    // 轨道相机（从配置的经纬度计算发射位置, Y-up）
    let lat_rad = app.config.launch_location.latitude.to_radians() as f32;
    let lon_rad = app.config.launch_location.longitude.to_radians() as f32;
    let launch_pad = Vec3::new(
        earth_radius * lat_rad.cos() * lon_rad.cos(),
        earth_radius * lat_rad.sin(),
        earth_radius * lat_rad.cos() * lon_rad.sin(),
    );
    let mut camera = OrbitalCamera::new(launch_pad, earth_radius * 3.0);
    camera.elevation = 0.2;
    camera.min_distance = (earth_radius * 0.005).max(1000.0); // ≈32km, 不会穿入火箭
    let mut track_rocket = true;

    // NASA Eyes 风格：星空 + 时间控制条（显示时间，保留原 ←→ 倍率逻辑）
    let stars = StarField::new(600, 0x870C4);
    let mut time_bar = TimeControlBar::new();
    time_bar.paused = false;

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
        // Space 暂停（覆盖原 time_warp 为 0）
        if is_key_pressed(KeyCode::Space) {
            time_bar.paused = !time_bar.paused;
        }
        // 时间倍率：基数为 2，范围 [-4096, 4096]
        // 正向：0.001 → 0.002 → 0.004 → ... → 1 → 2 → 4 → ... → 4096
        // 反向：-0.001 → -0.002 → -0.004 → ... → -1 → -2 → -4 → ... → -4096
        // 在 0.001/-0.001 处跨越正负（无 0 档）
        if is_key_pressed(KeyCode::Right) {
            if time_warp.abs() < 0.001 {
                time_warp = 0.001;
            } else if time_warp < 0.0 {
                time_warp /= 2.0;
                if time_warp.abs() < 0.001 {
                    time_warp = 0.001;
                }
            } else {
                time_warp = (time_warp * 2.0).min(4096.0);
            }
        }
        if is_key_pressed(KeyCode::Left) {
            if time_warp.abs() < 0.001 {
                time_warp = -0.001;
            } else if time_warp > 0.0 {
                time_warp /= 2.0;
                if time_warp.abs() < 0.001 {
                    time_warp = -0.001;
                }
            } else {
                time_warp = (time_warp * 2.0).max(-4096.0);
            }
        }
        // 同步倍率显示
        if time_bar.paused {
            time_bar.rate = 0.0;
        } else {
            time_bar.rate = time_warp.abs();
        }

        // -----------------------------------------------------------------
        // 2. 物理步进（基于真实帧间隔 × 时间倍率，帧率无关，支持倒放）
        // -----------------------------------------------------------------
        if !app.mission_complete && !time_bar.paused {
            let real_dt = get_frame_time() as f64; // 真实秒数
            let sim_dt = real_dt * time_warp; // 可为负（倒放）
            let n_substeps = ((sim_dt.abs() / 0.016).ceil().max(1.0)) as usize;
            let sub_dt = sim_dt / n_substeps as f64;
            for _ in 0..n_substeps {
                app.step(sub_dt);
            }

            let pos = to_mvec3(*app.vessel.body.get_position());
            flight_path.push(pos);
            if track_rocket {
                // 2帧 lerp：每帧趋近 50%，2 帧后到达 ~87%
                camera.target = camera.target.lerp(pos, 0.5);
            }
        }

        // 预测轨道（前向传播 N 步，步长随工况自适应）
        let current_pos = *app.vessel.body.get_position();
        let current_vel = *app.vessel.body.get_velocity();
        if current_vel.length() > 10.0 {
            // 收集摄动天体（月球等）
            let mut perturbers: Vec<(deepspace::Vec3, f64)> = Vec::new();
            for i in 1..app.body_positions.len() {
                perturbers.push((app.body_positions[i], app.bodies[i].get_mass()));
            }
            // N 步预测：低空小步长显示细节，深空大步长显示轨道弧
            let alt = current_pos.length() - app.earth.get_radius();
            let pred_steps = 300;
            let step_dt = if alt < 100_000.0 {
                1.0 // 大气层内：每步 1s，看清转弯
            } else if alt < 1_000_000.0 {
                5.0 // 上升段：每步 5s
            } else if alt < 100_000_000.0 {
                60.0 // 近地轨道：每步 1min
            } else {
                600.0 // 深空：每步 10min
            };
            let raw = predict_trajectory(
                current_pos,
                current_vel,
                EARTH_MU,
                &perturbers,
                pred_steps as f64 * step_dt,
                pred_steps,
                app.earth.get_radius(),
            );
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
        let s = ui_scale();

        // 深空背景 + 恒星（NASA Eyes）
        clear_background(COLOR_SPACE_BG);
        stars.draw(&camera, sw, sh);

        // 空间参考网格
        draw_grid_2d(&camera, earth_radius, sw, sh);

        // 天体轨道指示 + 天体渲染
        let n = app.body_positions.len().min(app.bodies.len());
        for i in 0..n {
            let bpos = to_mvec3(app.body_positions[i]);
            let rad = app.bodies[i].get_radius() as f32;
            let (px, py) = camera.project_2d(bpos, sw, sh);
            let r_px = camera.len_to_px(rad, sw, sh).max(3.0);

            // 天体标签颜色
            let label_color = match app.bodies[i].get_name() {
                "Earth" => COLOR_EARTH,
                "Moon" | "Luna" => COLOR_MOON,
                "Sun" => COLOR_SUN,
                _ => Color::new(0.7, 0.7, 0.7, 1.0),
            };

            // 轨道环（绕主天体, 以地心为中心）
            if i > 0 && app.body_velocities[i].length() > 0.0 {
                let orbit_r = camera.len_to_px(bpos.length(), sw, sh);
                if orbit_r > 5.0 {
                    let (ecx, ecy) = camera.project_2d(Vec3::ZERO, sw, sh);
                    draw_circle_2d(ecx, ecy, orbit_r, Color::new(0.4, 0.4, 0.4, 0.3));
                }
            }

            if app.bodies[i].get_name() == "Earth" {
                // 地球用专用绘制（含十字标记）
                draw_earth_2d(&camera, earth_radius, sw, sh);
            } else {
                // 其他天体
                draw_circle_2d(px, py, r_px, label_color);
            }

            // 天体名称标签
            text(
                app.bodies[i].get_name(),
                px + r_px + 4.0 * s,
                py + 4.0 * s,
                20.0 * s,
                label_color,
            );
        }

        // 飞行路径（历史轨迹，NASA Eyes 渐变：蓝→橙）
        let flight_pts = flight_path.points();
        if flight_pts.len() > 1 {
            draw_gradient_path_2d(
                &camera,
                &flight_pts,
                sw,
                sh,
                Color::new(0.2, 0.5, 0.9, 0.35),
                Color::new(1.0, 0.75, 0.3, 0.9),
            );
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
        // NASA Eyes 发光火箭（投影光晕 + 原标记）
        let (rx, ry) = camera.project_2d(rpos, sw, sh);
        let marker_r = camera
            .len_to_px(2000.0_f32.max(earth_radius * 0.003), sw, sh)
            .max(2.5);
        draw_glow_2d(rx, ry, marker_r * 0.8, COLOR_SHIP, 1.4);
        draw_rocket_2d(&camera, rpos, to_mvec3(vel), earth_radius, sw, sh);

        // -----------------------------------------------------------------
        // 4. 姿态指示器 + 遥测 HUD
        // -----------------------------------------------------------------
        if vel.length() > 1.0 {
            let vd = to_mvec3(vel.normalized());
            let quat = if vd.dot(Vec3::Y).abs() < 0.999 {
                Quat::from_axis_angle(Vec3::Y.cross(vd).normalize(), Vec3::Y.dot(vd).acos())
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
        let lh = 30.0 * s;
        let y0 = 110.0 * s;

        text(
            format!("Mission: {}", app.config.mission_name),
            10.0,
            30.0 * s,
            32.0 * s,
            WHITE,
        );
        text(
            format!("T+ {:.1}s", app.simulation_time),
            10.0,
            60.0 * s,
            28.0 * s,
            LIGHTGRAY,
        );

        text(
            format!("Phase: {}", app.mission_control.phase_name),
            10.0,
            y0,
            24.0 * s,
            YELLOW,
        );
        text(
            format!("Altitude: {:.0} m", tel.altitude_m),
            10.0,
            y0 + lh,
            24.0 * s,
            dc,
        );
        text(
            format!("Velocity: {:.0} m/s", tel.velocity_mps),
            10.0,
            y0 + lh * 2.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Mass: {:.0} kg", app.vessel.body.get_mass()),
            10.0,
            y0 + lh * 3.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Thrust: {:.0} kN", tel.thrust_n / 1000.0),
            10.0,
            y0 + lh * 4.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Throttle: {:.0}%", tel.throttle_pct * 100.0),
            10.0,
            y0 + lh * 5.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Mach: {:.2}", tel.mach),
            10.0,
            y0 + lh * 6.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Q: {:.0} Pa", tel.dynamic_pressure_pa),
            10.0,
            y0 + lh * 7.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Stage: {}", app.vessel.current_stage),
            10.0,
            y0 + lh * 8.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Apoapsis: {:.0} km", tel.orbit.apoapsis_m / 1000.0),
            10.0,
            y0 + lh * 9.0,
            24.0 * s,
            dc,
        );
        text(
            format!("Periapsis: {:.0} km", tel.orbit.periapsis_m / 1000.0),
            10.0,
            y0 + lh * 10.0,
            24.0 * s,
            dc,
        );
        text(
            format!(
                "Orbit: {}",
                if tel.orbit.is_bound {
                    "Bound"
                } else {
                    "Suborbital"
                }
            ),
            10.0,
            y0 + lh * 11.0,
            24.0 * s,
            if tel.orbit.is_bound { GREEN } else { YELLOW },
        );

        // ---- 任务导航：从配置读取后续阶段转换 ----
        let mc = &app.mission_control;
        let current_phase = mc.phase_name.clone();

        // 收集所有后续阶段转换的进度（从真实配置读取）
        let remaining_phases: Vec<NextPhaseDisplay> = mc
            .compute_all_remaining_phases(&app.vessel, &app.earth, app.moon_position())
            .iter()
            .map(|info| {
                let conditions: Vec<NextPhaseConditionDisplay> = info
                    .conditions
                    .iter()
                    .map(|c| NextPhaseConditionDisplay {
                        label: c.label.clone(),
                        current: c.current,
                        target: c.target,
                        progress: c.progress,
                        is_met: c.is_met,
                        is_boolean: c.is_boolean,
                    })
                    .collect();
                NextPhaseDisplay {
                    next_phase: info.next_phase.clone(),
                    conditions,
                    require_all: info.require_all,
                }
            })
            .collect();

        let mission_state = MissionDisplayState {
            phase_name: current_phase,
            complete: app.mission_complete,
            outcome: mc.outcome.to_str().to_string(),
        };

        let s = ui_scale();

        // 右侧导航面板
        draw_phase_panel(&mission_state, screen_width() - 260.0 * s, 20.0 * s);

        // 后续阶段面板（从配置读取的 phase_transitions）
        draw_remaining_phases_panel(&remaining_phases, screen_width() - 290.0 * s, 360.0 * s);

        let warp_color = if time_warp < -0.01 {
            Color::new(1.0, 0.2, 0.2, 1.0) // 红色 = 倒放
        } else if time_warp > 1.0 {
            Color::new(1.0, 0.6, 0.0, 1.0) // 橙色 = 快进
        } else if time_warp < 0.99 {
            Color::new(0.3, 0.8, 1.0, 1.0) // 蓝色 = 慢放
        } else {
            LIGHTGRAY // 灰色 = 1:1
        };
        let warp_label = if time_warp.abs() < 0.1 {
            format!("Time warp: {:.4}x", time_warp)
        } else if time_warp.abs() < 10.0 {
            format!("Time warp: {:.3}x", time_warp)
        } else {
            format!("Time warp: {:.1}x", time_warp)
        };
        text(
            &warp_label,
            screen_width() / 2.0 - 80.0 * s,
            30.0 * s,
            24.0 * s,
            warp_color,
        );

        text(
            "Left-drag: Rotate | Scroll: Zoom | T: Track | ←→: Warp | ESC: Exit",
            10.0,
            screen_height() - 54.0 * s,
            20.0 * s,
            Color::new(0.7, 0.8, 0.9, 0.9),
        );

        // 底部时间控制条（NASA Eyes）
        time_bar.draw(app.simulation_time);

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
