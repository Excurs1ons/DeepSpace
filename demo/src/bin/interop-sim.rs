//! 多类型物理模拟互通演示（Interop Sim）
//!
//! 闭环链路：N体世界（地心引力）→ 弹道目标（ICBM 实体）→
//! 雷达探测 → THAAD 拦截导弹（ProNav 制导）→ 命中事件 →
//! 统一 HUD（实体遥测 / 事件日志 / 世界状态条）。
//!
//! 演示 DeepSpace 多类型物理模拟互通能力：
//! - 统一 Entity 抽象：导弹 / 弹道目标 / 天体 / 航天器同表显示
//! - 统一 World 推进：单步推进所有类型
//! - 统一 HUD：一套面板显示所有实体
//! - 自身闭环：事件驱动 → 状态更新 → 显示反馈
//!
//! Usage:
//!   cargo run --bin interop-sim            # 3D 窗口（macroquad）
//!   cargo run --bin interop-sim -- --headless   # 控制台时间线

use deepspace::entity::EntityKind;
use deepspace::missile::AamConfig;
use deepspace::world::{BallisticConfig, World};
use deepspace::Vec3;
use std::time::Instant;

const EARTH_RADIUS: f64 = 6_371_000.0;

/// 构建统一世界：地球 + 弹道目标 + 拦截导弹 + 观测航天器
fn build_world() -> World {
    let mut w = World::default();

    // 弹道目标：再入弹头在 120km 高空水平匀速飞行（拦截窗口充足）
    let tgt = BallisticConfig {
        name: "TGT-1 弹头".into(),
        position: Vec3::new(-150_000.0, EARTH_RADIUS + 120_000.0, 0.0),
        velocity: Vec3::new(1600.0, 0.0, 0.0),
        mass: 800.0,
        ref_area_m2: 0.4,
        cd: 0.12,
        thrust_n: 0.0,
        thrust_duration_s: 0.0,
    };
    let tid = w.add_ballistic(tgt);

    // 观测航天器（统一实体表中同时显示，验证多类型互通）
    w.add_satellite("观测卫星", 400_000.0, 0.0);

    // 拦截导弹从拦截站发射：初速指向目标预测交汇点（迎头拦截，PN 微调）
    let launch = Vec3::new(0.0, EARTH_RADIUS + 5_000.0, 0.0);
    let r = w.ballistic[0].0.position - launch;
    let tgt_v = w.ballistic[0].0.velocity;
    // 交汇时间 ≈ 相对距离 / (导弹速度 + 目标在 LOS 投影) ≈ 150km/4300 ≈ 35s
    let dir = (r + tgt_v * 35.0).normalized();
    let vel = dir * 2700.0;
    w.fire_interceptor(AamConfig::interceptor(), launch, vel, tid);

    w
}

/// 收集统一 HUD 行（从世界实体表）
fn hud_rows(w: &World) -> Vec<demo::render::EntityHudRow> {
    w.entities
        .iter()
        .map(demo::render::EntityHudRow::from_entity)
        .collect()
}

/// 收集事件日志（最近 N 条，纯文本）
fn event_log(w: &World, n: usize) -> Vec<String> {
    w.events
        .iter()
        .rev()
        .take(n)
        .map(|e| e.text.clone())
        .collect()
}

/// 控制台模式：输出完整拦截时间线
fn run_headless() {
    println!("╔══════════════════════════════════════════════╗");
    println!("║  DeepSpace Interop — 多类型物理模拟互通演示  ║");
    println!("╚══════════════════════════════════════════════╝");
    println!();

    let mut w = build_world();
    let start = Instant::now();
    let mut last_print = 0u64;
    let mut outcome = false;

    loop {
        w.step();
        let t = w.time();

        // 每 2 秒打印一次实体遥测
        if t as u64 / 2 > last_print {
            last_print = t as u64 / 2;
            println!("--- T+{:.1}s ---", t);
            for e in w.entities.iter().filter(|e| e.kind != EntityKind::Body) {
                println!(
                    "  [{}] {} alt={:>8.1}km  v={:>7.0}m/s  {}",
                    kind_tag(e.kind),
                    e.name,
                    e.altitude_m / 1000.0,
                    e.speed_mps,
                    e.status
                );
            }
        }

        // 事件输出（新事件立即打印）
        for ev in w.events.iter().skip(last_printed_events()) {
            println!("  >> T+{:.1}s  {}", ev.time, ev.text);
        }
        update_last_printed_events(w.events.len());

        // 命中后继续跑 5 秒展示结局，然后退出
        if w.events
            .iter()
            .any(|e| e.kind == deepspace::entity::EventKind::Outcome)
        {
            outcome = true;
            if t > w.time_at_outcome() + 5.0 {
                break;
            }
        }
        if t > 600.0 {
            break;
        }
        if start.elapsed().as_secs() > 30 {
            println!("(headless 30s 上限)");
            break;
        }
    }

    println!();
    println!("=== 事件时间线 ===");
    for ev in &w.events {
        println!("  T+{:>7.1}s  {:?}  {}", ev.time, ev.kind, ev.text);
    }
    println!();
    if outcome {
        println!("✅ 拦截成功 — 多类型物理模拟闭环完成");
    } else {
        println!("⚠️  未拦截（调参场景）");
    }
}

static mut LAST_PRINTED: usize = 0;
fn last_printed_events() -> usize {
    unsafe { LAST_PRINTED }
}
fn update_last_printed_events(n: usize) {
    unsafe { LAST_PRINTED = n }
}

fn kind_tag(k: EntityKind) -> &'static str {
    match k {
        EntityKind::Rocket => "RKT",
        EntityKind::Spacecraft => "S/C",
        EntityKind::Missile => "MSL",
        EntityKind::Icbm => "ICBM",
        EntityKind::Aircraft => "ACFT",
        EntityKind::Body => "BODY",
    }
}

fn main() {
    let headless = std::env::args().any(|a| a == "--headless");
    if headless {
        run_headless();
    } else {
        macroquad::Window::new("DeepSpace — Interop", run_3d());
    }
}

/// 3D 窗口模式：统一 HUD + 场景
async fn run_3d() {
    use demo::render::*;
    use macroquad::prelude::*;

    let mut w = build_world();
    let mut trails: Vec<Trail> = Vec::new();
    let mut trail_ids: Vec<u64> = Vec::new();

    // 加载字体（NASA Eyes 风格：Metropolis）
    load_custom_font(FONT_PATH).await;

    let mut cam = OrbitalCamera::new(Vec3::new(0.0, 6_500_000.0, 0.0), 200_000.0);

    // NASA Eyes 风格：星空 + 时间控制条 + 点击选中
    let stars = StarField::new(600, 0x1E4E0);
    let mut time_bar = TimeControlBar::new();
    let mut click = ClickDetector::new();
    let mut selected: Option<u64> = None;

    loop {
        let _dt = get_frame_time().min(0.05);
        cam.update();
        time_bar.update();

        // 推进世界（时间条控制，rate 为 1 时等价于原 10 步/帧）
        if !time_bar.paused {
            let steps = (10.0 * time_bar.rate).round().max(0.0) as usize;
            for _ in 0..steps {
                w.step();
            }
        }

        // 轨迹：实体表变化时重建
        for e in w.entities.iter() {
            if e.kind == EntityKind::Body {
                continue;
            }
            if let Some(idx) = trail_ids.iter().position(|&id| id == e.id) {
                trails[idx].push(to_mvec3(e.position));
            } else {
                trail_ids.push(e.id);
                let mut tr = Trail::new(400);
                tr.push(to_mvec3(e.position));
                trails.push(tr);
            }
        }

        // 点击选中（短按，区分拖拽）
        let sw = screen_width();
        let sh = screen_height();
        if click.update() {
            let (mx, my) = mouse_position();
            let mut best: Option<(u64, f32)> = None;
            for e in w.entities.iter() {
                if e.kind == EntityKind::Body {
                    continue;
                }
                let (cx, cy) = cam.project_2d(to_mvec3(e.position), sw, sh);
                let d = (mx - cx) * (mx - cx) + (my - cy) * (my - cy);
                if d < 40.0 * 40.0 && best.is_none_or(|(_, bd)| d < bd) {
                    best = Some((e.id, d));
                }
            }
            selected = best.map(|(id, _)| id);
        }

        // 跟随选中实体（相机 target 平滑趋近）
        if let Some(id) = selected {
            if let Some(e) = w.entities.iter().find(|e| e.id == id) {
                cam.target = cam.target.lerp(to_mvec3(e.position), 0.1);
            }
        }

        // 深空背景 + 恒星
        clear_background(COLOR_SPACE_BG);
        stars.draw(&cam, sw, sh);

        // 3D 相机
        set_camera(&cam.get_camera3d());

        // 地球（发光风格：3D 球体 + 2D 光晕由后处理层处理）
        draw_planet(Vec3::ZERO, 6_371_000.0, COLOR_EARTH);

        // 实体位置（3D 球体）
        for e in w.entities.iter() {
            if e.kind == EntityKind::Body {
                continue;
            }
            let c = entity_kind_color(e.kind);
            let pos = to_mvec3(e.position);
            draw_sphere(pos, 15_000.0, None, c);
        }

        set_default_camera();

        // 渐变轨迹（2D 叠加在 3D 之上）
        for tr in &trails {
            let pts = tr.points();
            if pts.len() > 1 {
                draw_gradient_path_2d(
                    &cam,
                    &pts,
                    sw,
                    sh,
                    Color::new(0.2, 0.5, 0.9, 0.3),
                    Color::new(1.0, 0.75, 0.3, 0.9),
                );
            }
        }

        // 发光实体（2D 光晕叠加在 3D 球体投影上 — NASA Eyes）
        for e in w.entities.iter() {
            if e.kind == EntityKind::Body {
                continue;
            }
            let c = entity_kind_color(e.kind);
            let (cx, cy) = cam.project_2d(to_mvec3(e.position), sw, sh);
            let r_px = cam.len_to_px(15_000.0, sw, sh).max(3.0);
            draw_glow_2d(cx, cy, r_px, c, 1.2);
        }

        // 选中光环
        if let Some(id) = selected {
            if let Some(e) = w.entities.iter().find(|e| e.id == id) {
                let (cx, cy) = cam.project_2d(to_mvec3(e.position), sw, sh);
                let r_px = cam.len_to_px(15_000.0, sw, sh).max(3.0);
                draw_selection_ring(cx, cy, r_px);
            }
        }

        // 统一 HUD（面板化）
        let rows = hud_rows(&w);
        let events = event_log(&w, 8);
        draw_world_status_bar(w.time(), w.entities.len(), w.events.len());
        draw_entity_hud_panel(&rows, 20.0, 60.0);
        draw_event_log_panel(&events, 20.0, screen_height() - 320.0);

        // 底部时间控制条（NASA Eyes）
        time_bar.draw(w.time());

        // 退出
        if is_key_pressed(KeyCode::Escape) {
            break;
        }
        next_frame().await;
    }
}
