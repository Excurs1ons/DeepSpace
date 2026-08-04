// 诊断6：复现 world_fire_interceptor_destroys_target 垂直追击场景
fn main() {
    let mut w = deepspace::world::World::default();
    use deepspace::Vec3;
    let cfg = deepspace::world::BallisticConfig {
        name: "TGT-3".into(),
        position: Vec3::new(0.0, 6_771_000.0, 0.0), // 静止 400km 高空
        velocity: Vec3::zero(),
        mass: 1000.0,
        ref_area_m2: 0.5,
        cd: 0.2,
        thrust_n: 0.0,
        thrust_duration_s: 0.0,
    };
    let tid = w.add_ballistic(cfg);
    let start = Vec3::new(0.0, 6_446_000.0, 0.0); // 75km 高，目标下方
    let vel = Vec3::new(0.0, 2500.0, 0.0);
    w.fire_interceptor(
        deepspace::missile::AamConfig::interceptor(),
        start,
        vel,
        tid,
    );

    let mut min_d = f64::MAX;
    for i in 0..4000 {
        w.step();
        let ms = &w.missiles[0];
        let tgt = w.ballistic[0].0.position;
        let d = (ms.position - tgt).length();
        min_d = min_d.min(d);
        if i % 100 == 0 {
            println!("T+{:.1}s min_d={:.0}m |msl pos y={:.0} tgt y={:.0} v={:.1} cmd_g={:.1} hit={} bal_alive={}",
                w.time(), min_d, ms.position.y, tgt.y, ms.velocity.length(), ms.cmd_accel.length()/9.80665,
                ms.check_hit(&tgt), w.ballistic[0].0.alive);
        }
        if !w.ballistic[0].0.alive {
            println!("TARGET DEAD at t={:.1}s", w.time());
            break;
        }
        if w.time() > 200.0 {
            break;
        }
    }
    println!(
        "FINAL min_d={:.0}m, events={:?}",
        min_d,
        w.events.iter().map(|e| e.text.clone()).collect::<Vec<_>>()
    );
}
