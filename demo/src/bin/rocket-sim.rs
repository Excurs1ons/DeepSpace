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
        // 时间倍率：基数为 10，范围 [-1000, 1000]
        // 正向：0.001 → 0.01 → 0.1 → 1 → 10 → 100 → 1000
        // 反向：-0.001 → -0.01 → -0.1 → -1 → -10 → -100 → -1000
        // 在 0.001/-0.001 处跨越正负（无 0 档）
        if is_key_pressed(KeyCode::Right) {
            if time_warp.abs() < 0.001 {
                time_warp = 0.001;
            } else if time_warp < 0.0 {
                time_warp /= 10.0;
                if time_warp.abs() < 0.001 { time_warp = 0.001; }
            } else {
                time_warp = (time_warp * 10.0).min(1000.0);
            }
        }
        if is_key_pressed(KeyCode::Left) {
            if time_warp.abs() < 0.001 {
                time_warp = -0.001;
            } else if time_warp > 0.0 {
                time_warp /= 10.0;
                if time_warp.abs() < 0.001 { time_warp = -0.001; }
            } else {
                time_warp = (time_warp * 10.0).max(-1000.0);
            }
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
            if flight_path.len() < 20000 {
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
                let raw = predict_trajectory(current_pos, current_vel, EARTH_MU, 15000.0, 800, app.earth.get_radius());
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

        // ---- 任务导航：计算当前阶段 & 任务进度 ----
        let mc = &app.mission_control;
        let tel = &mc.telemetry;

        // 阶段索引：Artemis II 全任务 10 个阶段
        let phase_idx: Option<usize> = {
            let p = mc.current_phase;
            use deepspace::simulation::MissionPhase::*;
            let early = match p {
                PreLaunch => Some(0),
                Launch => Some(1),
                Ascent | MaxQ => Some(2),
                _ => None,
            };
            early.or_else(|| {
                if app.mission_complete {
                    // 任务结束映射到 SUCCESS（索引 9）
                    return Some(9);
                }
                let apo = tel.orbit.apoapsis_m;
                let alt = tel.altitude_m;
                if apo > 400_000_000.0 {
                    // 跨月轨道
                    if app.simulation_time > 350_000.0 {
                        // 回程
                        if alt < 200_000.0 { Some(8) }    // REENTRY
                        else { Some(7) }                    // RETURN
                    } else if app.simulation_time > 150_000.0 {
                        Some(6)                              // LUNAR_FLYBY
                    } else if mc.coasting && mc.icps_ignited {
                        Some(5)                              // TRANSLUNAR
                    } else {
                        Some(4)                              // TLI
                    }
                } else if tel.orbit.is_bound {
                    let in_tli_burn = mc.icps_ignited && !mc.coasting && app.simulation_time > 6000.0;
                    if in_tli_burn { Some(4) }              // TLI
                    else { Some(3) }                         // ORBIT
                } else {
                    Some(3)                                  // ORBIT (suborbital but above ascent)
                }
            })
        };

        // 任务里程碑进度（12 步）
        let mut tasks_done: usize = 0;
        let task_highlight: Option<usize> = {
            let alt = tel.altitude_m;
            let apo = tel.orbit.apoapsis_m;
            let bound = tel.orbit.is_bound;
            let stage = app.vessel.current_stage;

            // 逐级判断已完成任务
            // 1. Liftoff
            if mc.mission_time > 1.0 { tasks_done = 1; }
            // 2. SRB Separation — stage moved past SRB stage (stage >= 2)
            if stage >= 2 { tasks_done = tasks_done.max(2); }
            // 3. MaxQ
            if mc.max_q_passed || alt > 50_000.0 { tasks_done = tasks_done.max(3); }
            // 4. MECO / Staging — cutoff_fired or stage >= 3
            if mc.cutoff_fired || stage >= 3 { tasks_done = tasks_done.max(4); }
            // 5. ICPS Circularization — icps_ignited && in orbit
            if mc.icps_ignited && bound { tasks_done = tasks_done.max(5); }
            // 6. TLI Burn — apoapsis > 400,000 km
            if apo > 400_000_000.0 { tasks_done = tasks_done.max(6); }
            // 7. Orion Separation — time-based heuristic after TLI
            if apo > 400_000_000.0 && app.simulation_time > 30_000.0 { tasks_done = tasks_done.max(7); }
            // 8. Lunar Flyby — time-based
            if app.simulation_time > 200_000.0 { tasks_done = tasks_done.max(8); }
            // 9. Return Cruise
            if app.simulation_time > 500_000.0 { tasks_done = tasks_done.max(9); }
            // 10. SM Separation — altitude dropping below 200km on return
            if app.simulation_time > 800_000.0 && alt < 200_000.0 { tasks_done = tasks_done.max(10); }
            // 11. Reentry
            if alt < 100_000.0 && app.simulation_time > 850_000.0 { tasks_done = tasks_done.max(11); }
            // 12. Splashdown / Landing
            if app.mission_complete && mc.outcome == deepspace::simulation::MissionOutcome::Success {
                tasks_done = tasks_done.max(12);
            }

            // 当前高亮任务：第一个未完成的
            if tasks_done < 12 { Some(tasks_done) } else { None }
        };

        let mission_state = MissionDisplayState {
            phase_idx,
            tasks_done,
            task_highlight,
            complete: app.mission_complete,
            outcome: mc.outcome.to_str().to_string(),
        };

        // 右侧导航面板
        draw_phase_panel(&mission_state, screen_width() - 140.0, 20.0);
        draw_task_panel(&mission_state, screen_width() - 140.0, 220.0);

        let warp_color = if time_warp < -0.01 {
            Color::new(1.0, 0.2, 0.2, 1.0)       // 红色 = 倒放
        } else if time_warp > 1.0 {
            Color::new(1.0, 0.6, 0.0, 1.0)       // 橙色 = 快进
        } else if time_warp < 0.99 {
            Color::new(0.3, 0.8, 1.0, 1.0)       // 蓝色 = 慢放
        } else {
            LIGHTGRAY                              // 灰色 = 1:1
        };
        let warp_label = if time_warp.abs() < 0.1 {
            format!("Time warp: {:.4}x", time_warp)
        } else if time_warp.abs() < 10.0 {
            format!("Time warp: {:.3}x", time_warp)
        } else {
            format!("Time warp: {:.1}x", time_warp)
        };
        draw_text(
            &warp_label,
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
