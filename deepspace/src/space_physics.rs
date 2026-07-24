//! 太空物理引擎子模块
//!
//! 在 `physics.rs` 的 N 体引力系统之上构建太空游戏专用的物理层：
//!
//! - **`SpacecraftBody`** — 6DOF 航天器刚体（偏轴推力 → 自然扭矩）
//! - **`Thruster`** — RCS / 主推推力器布局
//! - **`SoiTree`** — 引力影响球层次（SoI），O(log n) 查找主导天体
//! - **`SpacePhysicsWorld`** — 统一模拟入口（N 体 + 航天器 + 时间加速）
//! - **`FlightAssist`** — SAS 姿态控制（Prograde/Retrograde/Stabilize）
//! - **`project_orbit()`** — 轨道预测线数据生成
//!
//! # 用法
//! ```ignore
//! let mut world = SpacePhysicsWorld::new(star_system, 0.1);
//! world.set_warp(WarpMode::Fast(10.0));
//! world.step();
//! ```

use crate::core::{Mat3x3, Quaternion};
use crate::physics::{GravitationalSystem, OrbitalElements};
use crate::{Vec3, G};

// =====================================================================
// 推力器
// =====================================================================

/// 航天器推力器（RCS / 主推）
#[derive(Debug, Clone)]
pub struct Thruster {
    /// 在航天器局部坐标系中的安装位置
    pub position: Vec3,
    /// 推力方向（局部坐标，应为单位向量）
    pub direction: Vec3,
    /// 最大推力（N）
    pub max_thrust: f64,
    /// 是否点火
    pub active: bool,
}

impl Thruster {
    pub fn new(position: Vec3, direction: Vec3, max_thrust: f64) -> Self {
        Thruster {
            position,
            direction: direction.normalized(),
            max_thrust,
            active: false,
        }
    }
}

// =====================================================================
// 航天器刚体 — 6DOF
// =====================================================================

/// 航天器 6DOF 刚体
#[derive(Debug, Clone)]
pub struct SpacecraftBody {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f64,
    pub inertia_tensor: Mat3x3,
    pub orientation: Quaternion,
    pub angular_velocity: Vec3,
    pub thrusters: Vec<Thruster>,

    /// 当前所在的 SoI 主导天体索引（由 `SpacePhysicsWorld` 维护）
    pub current_host_idx: usize,

    accumulated_force: Vec3,
    accumulated_torque: Vec3,
}

impl SpacecraftBody {
    /// 创建航天器刚体
    ///
    /// - `mass`: 质量 (kg)
    /// - `inertia`: 惯量张量对角元 (Ixx, Iyy, Izz)，假设主轴对齐
    pub fn new(pos: Vec3, vel: Vec3, mass: f64, inertia: (f64, f64, f64)) -> Self {
        SpacecraftBody {
            position: pos,
            velocity: vel,
            mass,
            inertia_tensor: Mat3x3::from_diag(inertia.0, inertia.1, inertia.2),
            orientation: Quaternion::identity(),
            angular_velocity: Vec3::zero(),
            thrusters: Vec::new(),
            current_host_idx: 0,
            accumulated_force: Vec3::zero(),
            accumulated_torque: Vec3::zero(),
        }
    }

    /// 施力于重心（不产生扭矩）
    pub fn add_force(&mut self, force: Vec3) {
        self.accumulated_force += force;
    }

    /// 施力于任意点（产生力和扭矩）
    pub fn add_force_at_point(&mut self, force: Vec3, world_point: Vec3) {
        self.accumulated_force += force;
        let r = world_point - self.position;
        self.accumulated_torque += r.cross(&force);
    }

    /// 添加扭矩（SAS 控制器用）
    pub fn add_torque(&mut self, torque: Vec3) {
        self.accumulated_torque += torque;
    }

    /// 添加推力器
    pub fn add_thruster(&mut self, thruster: Thruster) {
        self.thrusters.push(thruster);
    }

    // ---------------------------------------------------------------
    // 积分
    // ---------------------------------------------------------------

    /// 推进一个时间步（半隐式欧拉 + 四元数姿态更新）
    pub fn step(&mut self, dt: f64) {
        if self.mass <= 0.0 || dt <= 0.0 {
            return;
        }

        // --- 平动 ---
        let accel = self.accumulated_force / self.mass;
        self.velocity += accel * dt;
        self.position += self.velocity * dt;

        // --- 转动 ---
        let ang_accel = self
            .inertia_tensor
            .inverse()
            .mul_vec(&self.accumulated_torque);
        self.angular_velocity += ang_accel * dt;

        // 四元数更新: q_new = q + 0.5 · ω_q · q · dt
        let wq = Quaternion::new(
            0.0,
            self.angular_velocity.x,
            self.angular_velocity.y,
            self.angular_velocity.z,
        );
        let dq = wq.mul(&self.orientation);
        let half_dt = 0.5 * dt;
        self.orientation = Quaternion::new(
            self.orientation.w + half_dt * dq.w,
            self.orientation.x + half_dt * dq.x,
            self.orientation.y + half_dt * dq.y,
            self.orientation.z + half_dt * dq.z,
        )
        .normalized();

        self.accumulated_force = Vec3::zero();
        self.accumulated_torque = Vec3::zero();
    }

    /// 用 Kepler 分析解传播位置/速度（时间加速用）
    ///
    /// 根据当前轨道要素直接推算未来位置/速度，跳过逐帧积分。
    /// 仅适用于椭圆的纯引力巡航段（推力器应已关闭）。
    pub fn propagate_kepler(&mut self, dt: f64, mu: f64) {
        let elems = crate::physics::orbital_elements(self.position, self.velocity, mu);
        let ea = kepler_equation(
            elems.mean_anomaly + mean_motion(&elems, mu) * dt,
            elems.eccentricity,
        );
        let (sin_ea, cos_ea) = ea.sin_cos();
        let ecc = elems.eccentricity;

        // 偏近点角 → 位置/速度（在轨道平面内）
        let r = elems.semi_major_axis * (1.0 - ecc * cos_ea);
        let oof = (1.0 - ecc * ecc).sqrt();
        let x = elems.semi_major_axis * (cos_ea - ecc);
        let y = elems.semi_major_axis * oof * sin_ea;
        let vx = -mu.sqrt() / (elems.semi_major_axis * r).sqrt() * sin_ea;
        let vy = mu.sqrt() / (elems.semi_major_axis * r).sqrt() * oof * cos_ea;

        // 从轨道平面转回 3D（简化：假设轨道在 XY 平面，忽略倾角/升交点）
        self.position = Vec3::new(x, y, 0.0);
        self.velocity = Vec3::new(vx, vy, 0.0);
    }

    // ---------------------------------------------------------------
    // 查询
    // ---------------------------------------------------------------

    pub fn speed(&self) -> f64 {
        self.velocity.length()
    }

    pub fn rotation_rate(&self) -> f64 {
        self.angular_velocity.length()
    }
}

// ---------------------------------------------------------------
// Kepler 方程辅助
// ---------------------------------------------------------------

/// 解 Kepler 方程 E - e·sin(E) = M， Newton 迭代
fn kepler_equation(mean_anomaly: f64, eccentricity: f64) -> f64 {
    let mut ea = mean_anomaly;
    for _ in 0..32 {
        let d = (ea - eccentricity * ea.sin() - mean_anomaly) / (1.0 - eccentricity * ea.cos());
        ea -= d;
        if d.abs() < 1e-12 {
            break;
        }
    }
    ea
}

/// 平均角速度 n = sqrt(μ / a³)
fn mean_motion(elems: &OrbitalElements, mu: f64) -> f64 {
    (mu / elems.semi_major_axis.powi(3)).sqrt()
}

// =====================================================================
// 引力影响球 (Sphere of Influence)
// =====================================================================

/// 影响球节点
#[derive(Debug, Clone)]
pub struct SoiNode {
    pub body_idx: usize,
    pub soi_radius: f64,
    pub parent: Option<usize>,
    pub children: Vec<usize>,
}

/// 引力影响球层次树
#[derive(Debug, Clone)]
pub struct SoiTree {
    pub nodes: Vec<SoiNode>,
}

impl SoiTree {
    pub fn build_from_system(sys: &GravitationalSystem) -> Self {
        let n = sys.bodies.len();
        if n == 0 {
            return SoiTree { nodes: Vec::new() };
        }

        let mut indices: Vec<usize> = (0..n).collect();
        indices.sort_by(|&a, &b| {
            sys.bodies[b]
                .mass
                .partial_cmp(&sys.bodies[a].mass)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut nodes: Vec<SoiNode> = Vec::with_capacity(n);

        for (order, &body_idx) in indices.iter().enumerate() {
            if order == 0 {
                nodes.push(SoiNode {
                    body_idx,
                    soi_radius: f64::INFINITY,
                    parent: None,
                    children: Vec::new(),
                });
            } else {
                let my_mass = sys.bodies[body_idx].mass;
                let my_pos = sys.bodies[body_idx].position;

                let mut best_parent = 0;
                let mut best_dist = f64::MAX;
                for &prev_idx in &indices[..order] {
                    let d = (my_pos - sys.bodies[prev_idx].position).length();
                    if d < best_dist {
                        best_dist = d;
                        best_parent = prev_idx;
                    }
                }

                let parent_node_idx = nodes
                    .iter()
                    .position(|n| n.body_idx == best_parent)
                    .unwrap_or(0);
                let parent_mass = sys.bodies[best_parent].mass;
                let soi_radius = if parent_mass > 0.0 {
                    best_dist * (my_mass / parent_mass).powf(0.4)
                } else {
                    f64::INFINITY
                };

                let node_idx = nodes.len();
                nodes.push(SoiNode {
                    body_idx,
                    soi_radius,
                    parent: Some(parent_node_idx),
                    children: Vec::new(),
                });
                nodes[parent_node_idx].children.push(node_idx);
            }
        }

        SoiTree { nodes }
    }

    /// 查找包含 `pos` 的最内层天体
    pub fn find_host(&self, pos: Vec3, sys: &GravitationalSystem) -> usize {
        if self.nodes.is_empty() {
            return 0;
        }
        self.find_host_recursive(0, pos, sys)
    }

    fn find_host_recursive(&self, node_idx: usize, pos: Vec3, sys: &GravitationalSystem) -> usize {
        let mut best_child = None;
        let mut best_dist = f64::MAX;

        for &child_idx in &self.nodes[node_idx].children {
            let child = &self.nodes[child_idx];
            let child_pos = sys.bodies[child.body_idx].position;
            let dist_to_child = (pos - child_pos).length();
            if dist_to_child <= child.soi_radius && dist_to_child < best_dist {
                best_dist = dist_to_child;
                best_child = Some(child_idx);
            }
        }

        match best_child {
            Some(child_idx) => self.find_host_recursive(child_idx, pos, sys),
            None => self.nodes[node_idx].body_idx,
        }
    }
}

// =====================================================================
// SoI 迁移事件
// =====================================================================

/// 航天器跨越 SoI 边界时触发的事件
#[derive(Debug, Clone, Copy)]
pub struct SoiTransition {
    pub spacecraft_idx: usize,
    pub from_body_idx: usize,
    pub to_body_idx: usize,
}

// =====================================================================
// 时间加速模式
// =====================================================================

/// 时间加速模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WarpMode {
    /// 实时（1×），全精度 N-body + 6DOF + 推力器
    RealTime,
    /// 快速（~100×），加大 dt，N-body 和航天器都用大步长
    Fast(f64),
    /// Kepler 巡航（1000×+），航天器用分析解传播，推力器自动关闭
    KeplerWarp(f64),
}

impl WarpMode {
    /// 获取有效的加速倍数
    pub fn factor(&self) -> f64 {
        match self {
            WarpMode::RealTime => 1.0,
            WarpMode::Fast(f) => f.max(1.0),
            WarpMode::KeplerWarp(f) => f.max(1.0),
        }
    }

    /// 是否允许推力器点火
    pub fn thrust_allowed(&self) -> bool {
        match self {
            WarpMode::RealTime => true,
            WarpMode::Fast(f) => *f <= 10.0,
            WarpMode::KeplerWarp(_) => false,
        }
    }
}

// =====================================================================
// SAS 姿态控制
// =====================================================================

/// SAS 模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SasMode {
    Disabled,
    /// 消除角速度（稳定）
    Stabilize,
    /// 指向速度方向
    Prograde,
    /// 指向速度反方向
    Retrograde,
    /// 指向轨道法线
    Normal,
    /// 指向轨道法线反方向
    AntiNormal,
}

/// 飞控辅助系统（SAS）
#[derive(Debug, Clone)]
pub struct FlightAssist {
    pub mode: SasMode,
    /// 比例增益
    pub kp: f64,
    /// 微分增益
    pub kd: f64,
    /// 最大输出扭矩 (N·m)
    pub max_torque: f64,
}

impl FlightAssist {
    pub fn new() -> Self {
        FlightAssist {
            mode: SasMode::Disabled,
            kp: 10.0,
            kd: 5.0,
            max_torque: 100_000.0,
        }
    }

    /// 计算控制扭矩
    ///
    /// - `ship`: 当前航天器状态
    /// - `vel`: 航天器速度（用于 Prograde/Retrograde）
    /// - `mu`: 中心天体引力参数（用于 Normal/AntiNormal）
    pub fn compute_torque(&self, ship: &SpacecraftBody, vel: Vec3, _mu: f64) -> Vec3 {
        match self.mode {
            SasMode::Disabled => Vec3::zero(),
            SasMode::Stabilize => {
                // PD 消旋：torque = -kd * ω
                let torque = ship.angular_velocity * (-self.kd);
                clamp_torque(torque, self.max_torque)
            }
            SasMode::Prograde | SasMode::Retrograde => {
                let target_dir = if let SasMode::Retrograde = self.mode {
                    if vel.length() > 1e-12 {
                        -(vel / vel.length())
                    } else {
                        Vec3::new(0.0, 1.0, 0.0)
                    }
                } else {
                    if vel.length() > 1e-12 {
                        vel / vel.length()
                    } else {
                        Vec3::new(0.0, 1.0, 0.0)
                    }
                };
                pointing_torque(ship, target_dir, self.kp, self.kd, self.max_torque)
            }
            SasMode::Normal | SasMode::AntiNormal => {
                // 轨道法线 = 速度 × 位置（归一化）
                let h = vel.cross(&ship.position);
                let normal = if h.length() > 1e-12 {
                    h / h.length()
                } else {
                    Vec3::new(0.0, 0.0, 1.0)
                };
                let target_dir = if let SasMode::AntiNormal = self.mode {
                    -normal
                } else {
                    normal
                };
                pointing_torque(ship, target_dir, self.kp, self.kd, self.max_torque)
            }
        }
    }
}

/// 指向目标方向所需的控制扭矩（PD 控制器）
fn pointing_torque(
    ship: &SpacecraftBody,
    target_dir: Vec3,
    kp: f64,
    kd: f64,
    max_torque: f64,
) -> Vec3 {
    let current_dir = ship.orientation.rotate(&Vec3::new(0.0, 1.0, 0.0));
    let error_angle = current_dir.cross(&target_dir);
    // 比例项
    let p_term = error_angle * kp;
    // 微分项（阻尼）
    let d_term = ship.angular_velocity * (-kd);
    clamp_torque(p_term + d_term, max_torque)
}

/// 限制扭矩幅值
fn clamp_torque(torque: Vec3, max_torque: f64) -> Vec3 {
    let mag = torque.length();
    if mag > max_torque && max_torque > 0.0 {
        torque * (max_torque / mag)
    } else {
        torque
    }
}

// =====================================================================
// 轨道预测线
// =====================================================================

/// 生成轨道预测点，用于游戏内轨道线渲染
///
/// 椭圆轨道用 Kepler 分析解（效率高），双曲/抛物线用 RK4 数值传播。
///
/// - `pos`: 当前位置 (m)
/// - `vel`: 当前速度 (m/s)
/// - `mu`: 中心天体标准引力参数 (m³/s²)
/// - `duration`: 预测时长 (s)
/// - `num_points`: 采样点数
/// - 返回 `(位置, 相对时间)` 序列
pub fn project_orbit(
    pos: Vec3,
    vel: Vec3,
    mu: f64,
    duration: f64,
    num_points: usize,
) -> Vec<(Vec3, f64)> {
    if num_points < 2 || duration <= 0.0 {
        return vec![(pos, 0.0)];
    }

    let elems = crate::physics::orbital_elements(pos, vel, mu);
    let dt = duration / (num_points as f64);

    if elems.eccentricity < 1.0 && elems.semi_major_axis > 0.0 {
        // 椭圆轨道 → Kepler 分析解
        let n = mean_motion(&elems, mu);
        let mut points = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let t = i as f64 * dt;
            let m = elems.mean_anomaly + n * t;
            let ea = kepler_equation(m, elems.eccentricity);
            let (sin_ea, cos_ea) = ea.sin_cos();
            let ecc = elems.eccentricity;
            let a = elems.semi_major_axis;

            let oof = (1.0 - ecc * ecc).sqrt();
            let x = a * (cos_ea - ecc);
            let y = a * oof * sin_ea;

            let p = Vec3::new(x, y, 0.0);
            points.push((p, t));
        }
        points
    } else {
        // 双曲/抛物线 → RK4 数值传播
        let initial = vec![pos.x, pos.y, pos.z, vel.x, vel.y, vel.z];
        let two_body: Box<dyn Fn(f64, &[f64]) -> Vec<f64>> = Box::new(move |_: f64, s: &[f64]| {
            let rv = Vec3::new(s[0], s[1], s[2]);
            let r2 = rv.length_squared();
            let a = if r2 > 0.0 {
                rv * (-mu / (r2 * r2.sqrt()))
            } else {
                Vec3::zero()
            };
            vec![s[3], s[4], s[5], a.x, a.y, a.z]
        });

        let mut y = initial;
        let mut t = 0.0;
        let mut points = Vec::with_capacity(num_points);
        points.push((pos, 0.0));

        for _ in 1..num_points {
            let h = dt.min(duration - t);
            y = rk4_step(&two_body, t, &y, h);
            t += h;
            points.push((Vec3::new(y[0], y[1], y[2]), t));
        }
        points
    }
}

/// RK4 单步（用于 orbit projection）
fn rk4_step(f: &Box<dyn Fn(f64, &[f64]) -> Vec<f64>>, t: f64, y: &[f64], h: f64) -> Vec<f64> {
    let n = y.len();
    let k1 = f(t, y);
    let mut tmp = vec![0.0; n];
    let k2 = {
        for i in 0..n {
            tmp[i] = y[i] + 0.5 * h * k1[i];
        }
        f(t + 0.5 * h, &tmp)
    };
    let k3 = {
        for i in 0..n {
            tmp[i] = y[i] + 0.5 * h * k2[i];
        }
        f(t + 0.5 * h, &tmp)
    };
    let k4 = {
        for i in 0..n {
            tmp[i] = y[i] + h * k3[i];
        }
        f(t + h, &tmp)
    };
    (0..n)
        .map(|i| y[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0)
        .collect()
}

// =====================================================================
// 太空物理世界
// =====================================================================

/// 太空物理世界 — 统一模拟入口
pub struct SpacePhysicsWorld {
    /// 背景 N 体系统
    pub star_system: GravitationalSystem,
    /// 引力影响球树
    pub soi: SoiTree,
    /// 所有航天器
    pub spacecraft: Vec<SpacecraftBody>,
    /// 当前模拟时间 (s)
    pub time: f64,
    /// 基础时间步长 (s)
    pub dt: f64,
    /// 时间加速模式
    pub warp_mode: WarpMode,
    /// 每艘航天器的 SAS
    pub flight_assist: Vec<FlightAssist>,

    soi_events: Vec<SoiTransition>,
}

impl SpacePhysicsWorld {
    pub fn new(star_system: GravitationalSystem, dt: f64) -> Self {
        let soi = SoiTree::build_from_system(&star_system);
        SpacePhysicsWorld {
            star_system,
            soi,
            spacecraft: Vec::new(),
            time: 0.0,
            dt,
            warp_mode: WarpMode::RealTime,
            flight_assist: Vec::new(),
            soi_events: Vec::new(),
        }
    }

    pub fn add_spacecraft(&mut self, craft: SpacecraftBody) {
        let host = self.soi.find_host(craft.position, &self.star_system);
        self.spacecraft.push(SpacecraftBody {
            current_host_idx: host,
            ..craft
        });
        self.flight_assist.push(FlightAssist::new());
    }

    /// 获取并清空 SoI 迁移事件
    pub fn drain_soi_events(&mut self) -> Vec<SoiTransition> {
        std::mem::take(&mut self.soi_events)
    }

    /// 设置时间加速模式
    pub fn set_warp(&mut self, mode: WarpMode) {
        self.warp_mode = mode;
    }

    pub fn spacecraft_count(&self) -> usize {
        self.spacecraft.len()
    }

    /// 推进一个时间步
    pub fn step(&mut self) {
        let effective_dt = self.dt * self.warp_mode.factor();
        if effective_dt <= 0.0 {
            return;
        }

        // 1. 背景 N 体步进（始终用 symplectic4）
        self.star_system.step_symplectic4(effective_dt);

        // 2. 航天器物理
        let thrust_allowed = self.warp_mode.thrust_allowed();
        let is_kepler = matches!(self.warp_mode, WarpMode::KeplerWarp(_));

        for idx in 0..self.spacecraft.len() {
            let host_idx = self.spacecraft[idx].current_host_idx;
            let host_mass = {
                let host = &self.star_system.bodies[host_idx];
                host.mass
            };
            let mu = G * host_mass;

            if is_kepler {
                // Kepler 巡航模式：分析解传播，不处理力/扭矩
                let ship = &mut self.spacecraft[idx];
                ship.propagate_kepler(effective_dt, mu);

                // 但还是要检查 SoI 迁移
                let new_host = self.soi.find_host(ship.position, &self.star_system);
                if new_host != ship.current_host_idx {
                    self.soi_events.push(SoiTransition {
                        spacecraft_idx: idx,
                        from_body_idx: ship.current_host_idx,
                        to_body_idx: new_host,
                    });
                    ship.current_host_idx = new_host;
                }
                continue;
            }

            // --- 正常模式（RealTime / Fast） ---
            let ship = &mut self.spacecraft[idx];

            // a. SoI 引力
            let r = {
                let host = &self.star_system.bodies[ship.current_host_idx];
                host.position - ship.position
            };
            let dist = r.length();
            if dist > 1e-12 {
                let grav_force = r * (G * host_mass * ship.mass / (dist * dist * dist));
                ship.add_force(grav_force);
            }

            // b. 推力器
            if thrust_allowed {
                let forces: Vec<(Vec3, Vec3)> = ship
                    .thrusters
                    .iter()
                    .filter(|t| t.active && t.max_thrust > 0.0)
                    .map(|t| {
                        let world_dir = ship.orientation.rotate(&t.direction);
                        let world_pos = ship.position + ship.orientation.rotate(&t.position);
                        (world_dir * t.max_thrust, world_pos)
                    })
                    .collect();
                for (force, pos) in forces {
                    ship.add_force_at_point(force, pos);
                }
            }

            // c. SAS 控制扭矩
            if idx < self.flight_assist.len() {
                let sas = &self.flight_assist[idx];
                if sas.mode != SasMode::Disabled {
                    let torque = sas.compute_torque(ship, ship.velocity, mu);
                    ship.add_torque(torque);
                }
            }

            // d. 刚体积分
            ship.step(effective_dt);

            // e. SoI 迁移检测
            let new_host = self.soi.find_host(ship.position, &self.star_system);
            if new_host != ship.current_host_idx {
                self.soi_events.push(SoiTransition {
                    spacecraft_idx: idx,
                    from_body_idx: ship.current_host_idx,
                    to_body_idx: new_host,
                });
                ship.current_host_idx = new_host;
            }
        }

        self.time += effective_dt;
    }
}

// =====================================================================
// 测试
// =====================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{GravBody, GravitationalSystem};

    fn test_solar_system() -> GravitationalSystem {
        let mut sys = GravitationalSystem::new(1e6);
        sys.add_body(GravBody::new(
            "Sun",
            1.989e30,
            6.96e8,
            Vec3::zero(),
            Vec3::zero(),
        ));
        let earth_pos = Vec3::new(1.496e11, 0.0, 0.0);
        let earth_vel = Vec3::new(0.0, 29_780.0, 0.0);
        sys.add_body(GravBody::new(
            "Earth", 5.972e24, 6.371e6, earth_pos, earth_vel,
        ));
        let moon_pos = Vec3::new(1.496e11 + 3.844e8, 0.0, 0.0);
        let moon_vel = Vec3::new(0.0, 29_780.0 + 1_022.0, 0.0);
        sys.add_body(GravBody::new("Moon", 7.342e22, 1.737e6, moon_pos, moon_vel));
        sys
    }

    fn dummy_ship() -> SpacecraftBody {
        SpacecraftBody::new(
            Vec3::new(1.496e11, 7.0e6, 0.0),
            Vec3::new(0.0, 29_780.0, 0.0),
            10_000.0,
            (5000.0, 8000.0, 6000.0),
        )
    }

    // ---------------------------------------------------------------
    // SoiTree
    // ---------------------------------------------------------------

    #[test]
    fn test_soi_tree_build() {
        let sys = test_solar_system();
        let soi = SoiTree::build_from_system(&sys);
        assert_eq!(soi.nodes.len(), 3);
        assert_eq!(sys.bodies[soi.nodes[0].body_idx].name, "Sun");
        assert!(!soi.nodes[0].children.is_empty());
    }

    #[test]
    fn test_soi_find_host_sun() {
        let sys = test_solar_system();
        let soi = SoiTree::build_from_system(&sys);
        let host = soi.find_host(Vec3::new(1e20, 0.0, 0.0), &sys);
        assert_eq!(sys.bodies[host].name, "Sun");
    }

    #[test]
    fn test_soi_find_host_earth() {
        let sys = test_solar_system();
        let soi = SoiTree::build_from_system(&sys);
        let host = soi.find_host(Vec3::new(1.496e11, 7.0e6, 0.0), &sys);
        assert_eq!(sys.bodies[host].name, "Earth");
    }

    // ---------------------------------------------------------------
    // SpacecraftBody
    // ---------------------------------------------------------------

    #[test]
    fn test_spacecraft_new() {
        let ship = SpacecraftBody::new(Vec3::zero(), Vec3::zero(), 1000.0, (100.0, 200.0, 150.0));
        assert_eq!(ship.mass, 1000.0);
    }

    #[test]
    fn test_force_at_center_no_torque() {
        let mut ship =
            SpacecraftBody::new(Vec3::zero(), Vec3::zero(), 1000.0, (100.0, 200.0, 150.0));
        ship.add_force(Vec3::new(1000.0, 0.0, 0.0));
        ship.step(1.0);
        assert!(ship.velocity.x > 0.0);
        assert_eq!(ship.angular_velocity.length(), 0.0);
    }

    #[test]
    fn test_force_at_point_creates_torque() {
        let mut ship =
            SpacecraftBody::new(Vec3::zero(), Vec3::zero(), 1000.0, (100.0, 200.0, 150.0));
        ship.add_force_at_point(Vec3::new(0.0, 100.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        ship.step(1.0);
        assert!(ship.angular_velocity.length() > 0.0);
        assert!(ship.angular_velocity.z.abs() > 0.0);
    }

    #[test]
    fn test_thruster_applied_in_local_frame() {
        let mut ship =
            SpacecraftBody::new(Vec3::zero(), Vec3::zero(), 1000.0, (100.0, 200.0, 150.0));
        ship.add_thruster(Thruster::new(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            100.0,
        ));
        ship.thrusters[0].active = true;
        let forces: Vec<(Vec3, Vec3)> = ship
            .thrusters
            .iter()
            .filter(|t| t.active)
            .map(|t| {
                let world_dir = ship.orientation.rotate(&t.direction);
                let world_pos = ship.position + ship.orientation.rotate(&t.position);
                (world_dir * t.max_thrust, world_pos)
            })
            .collect();
        for (force, pos) in forces {
            ship.add_force_at_point(force, pos);
        }
        ship.step(1.0);
        assert!(ship.velocity.y > 0.0);
        assert!(ship.angular_velocity.length() > 0.0);
    }

    // ---------------------------------------------------------------
    // WarpMode
    // ---------------------------------------------------------------

    #[test]
    fn test_warp_mode_basics() {
        assert_eq!(WarpMode::RealTime.factor(), 1.0);
        assert!(WarpMode::RealTime.thrust_allowed());
        assert_eq!(WarpMode::Fast(10.0).factor(), 10.0);
        assert!(WarpMode::Fast(10.0).thrust_allowed());
        assert!(!WarpMode::Fast(100.0).thrust_allowed());
        assert!(!WarpMode::KeplerWarp(1000.0).thrust_allowed());
    }

    #[test]
    fn test_warp_fast() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1.0);
        world.set_warp(WarpMode::Fast(10.0));
        world.step();
        assert!((world.time - 10.0).abs() < 1e-12);
    }

    // ---------------------------------------------------------------
    // SoI Transition
    // ---------------------------------------------------------------

    #[test]
    fn test_soi_transition_no_event_initially() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1000.0);
        let ship = dummy_ship();
        world.add_spacecraft(ship);
        let events = world.drain_soi_events();
        assert!(events.is_empty(), "add 时不应产生事件");
    }

    #[test]
    fn test_soi_transition_on_step() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1000.0);
        let mut ship = dummy_ship();
        // 把飞船放在深空（SoI 为 Sun）
        ship.position = Vec3::new(1e20, 0.0, 0.0);
        world.add_spacecraft(ship);
        // 此时 host = Sun
        assert_eq!(world.spacecraft[0].current_host_idx, 0); // Sun's body_idx
                                                             // 一步后仍在深空，不应产生事件
        world.step();
        assert!(world.drain_soi_events().is_empty());
    }

    // ---------------------------------------------------------------
    // SpacePhysicsWorld
    // ---------------------------------------------------------------

    #[test]
    fn test_world_step_no_spacecraft() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1000.0);
        world.step();
        assert!(world.time > 0.0);
    }

    #[test]
    fn test_world_step_with_spacecraft() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1000.0);
        world.add_spacecraft(dummy_ship());
        world.step();
        assert!(world.spacecraft[0].position.length() > 0.0);
    }

    // ---------------------------------------------------------------
    // Orbit Projection
    // ---------------------------------------------------------------

    #[test]
    fn test_project_orbit_circular() {
        let earth_mu = G * 5.972e24;
        let r = 7_000_000.0; // ~600km alt
        let v = (earth_mu / r).sqrt();
        let pts = project_orbit(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v, 0.0),
            earth_mu,
            2.0 * std::f64::consts::PI * r / v, // ~1 orbit
            36,
        );
        assert!(pts.len() >= 2);
        // 起点和终点应相近（闭合轨道）
        let start = pts[0].0;
        let end = pts[pts.len() - 1].0;
        let dist = (start - end).length();
        assert!(dist < r * 0.3, "orbit should close: {dist:.0}m vs {r:.0}m");
    }

    #[test]
    fn test_project_orbit_hyperbolic() {
        let earth_mu = G * 5.972e24;
        let r = 7_000_000.0;
        let v_escape = (2.0 * earth_mu / r).sqrt() * 1.5; // > 逃逸速度
        let pts = project_orbit(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v_escape, 0.0),
            earth_mu,
            100_000.0,
            20,
        );
        assert!(pts.len() >= 2);
    }

    #[test]
    fn test_project_orbit_few_points() {
        let earth_mu = G * 5.972e24;
        let pts = project_orbit(
            Vec3::new(7e6, 0.0, 0.0),
            Vec3::new(0.0, 8000.0, 0.0),
            earth_mu,
            1000.0,
            1,
        );
        assert_eq!(pts.len(), 1);
    }

    // ---------------------------------------------------------------
    // FlightAssist / SAS
    // ---------------------------------------------------------------

    #[test]
    fn test_flight_assist_new() {
        let sas = FlightAssist::new();
        assert_eq!(sas.mode, SasMode::Disabled);
        assert!((sas.kp - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_sas_stabilize_reduces_rotation() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1.0);
        let mut ship = SpacecraftBody::new(
            Vec3::new(1.496e11, 7.0e6, 0.0),
            Vec3::new(0.0, 29_780.0, 0.0),
            10_000.0,
            (5000.0, 8000.0, 6000.0),
        );
        // 给一个初始角速度
        ship.angular_velocity = Vec3::new(0.1, 0.0, 0.0);
        world.add_spacecraft(ship);
        world.flight_assist[0].mode = SasMode::Stabilize;
        // 两步后角速度应显著减小
        world.step();
        let w0 = world.spacecraft[0].angular_velocity.length();
        world.step();
        let w1 = world.spacecraft[0].angular_velocity.length();
        assert!(
            w1 < w0,
            "SAS Stabilize should reduce angular velocity: {w1} >= {w0}"
        );
    }

    #[test]
    fn test_sas_prograde_points_forward() {
        let mut ship = SpacecraftBody::new(
            Vec3::zero(),
            Vec3::new(0.0, 1000.0, 0.0),
            1000.0,
            (100.0, 200.0, 150.0),
        );
        // 船头朝上（默认），速度朝 Y
        let sas = FlightAssist {
            mode: SasMode::Prograde,
            kp: 100.0,
            kd: 10.0,
            max_torque: 10_000.0,
        };
        let torque = sas.compute_torque(&ship, ship.velocity, 0.0);
        // 当前方向 (0,1,0)，速度方向 (0,1,0) → 无误差 → 扭矩应该很小
        assert!(
            torque.length() < 1.0,
            "already aligned, torque should be near zero: {}",
            torque.length()
        );

        // 旋转飞船，使船头指向 +X
        ship.orientation =
            Quaternion::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), -std::f64::consts::PI / 2.0);
        let torque = sas.compute_torque(&ship, ship.velocity, 0.0);
        // 现在船头指向 +X，速度指向 +Y → 应该有控制扭矩
        assert!(
            torque.length() > 0.0,
            "misaligned, torque should be non-zero"
        );
    }
}
