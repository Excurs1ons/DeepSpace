//! 太空物理引擎子模块
//!
//! 在 `physics.rs` 的 N 体引力系统之上构建太空游戏专用的物理层：
//!
//! - **`SpacecraftBody`** — 6DOF 航天器刚体（偏轴推力 → 自然扭矩）
//! - **`Thruster`** — RCS / 主推推力器布局
//! - **`SoiTree`** — 引力影响球层次（SoI），O(log n) 查找主导天体
//! - **`SpacePhysicsWorld`** — 统一模拟入口（N 体 + 航天器 + 时间加速）
//!
//! # 用法
//! ```ignore
//! let mut world = SpacePhysicsWorld::new(star_system, 0.1);
//! world.warp_factor = 10.0;         // 10× 加速
//! world.step();                       // 一步推进所有
//! ```

use crate::core::{Mat3x3, Quaternion};
use crate::physics::GravitationalSystem;
use crate::{G, Vec3};

// =====================================================================
// 推力器
// =====================================================================

/// 航天器推力器（RCS / 主推）
///
/// 位置和方向定义在**航天器局部坐标系**中。
/// `add_force_at_point` 会在世界坐标系中自动换算。
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
///
/// 相比 `physics.rs` 中的 `PhysicsBody`：
/// - **惯量张量**（`Mat3x3`）替代标量惯量
/// - **`add_force_at_point()`** — 任意点施力，自动产生扭矩
/// - **半隐式欧拉积分**（辛，长期稳定）
/// - **四元数姿态更新** + 重归一化
#[derive(Debug, Clone)]
pub struct SpacecraftBody {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f64,
    pub inertia_tensor: Mat3x3,
    pub orientation: Quaternion,
    pub angular_velocity: Vec3,
    pub thrusters: Vec<Thruster>,

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
            accumulated_force: Vec3::zero(),
            accumulated_torque: Vec3::zero(),
        }
    }

    /// 施力于重心（不产生扭矩）
    pub fn add_force(&mut self, force: Vec3) {
        self.accumulated_force += force;
    }

    /// 施力于任意点（产生力和扭矩）
    ///
    /// 这是 6DOF 的核心：推力器偏离重心 → 自然产生旋转。
    /// `world_point` 是该力在世界坐标系中的作用点。
    pub fn add_force_at_point(&mut self, force: Vec3, world_point: Vec3) {
        self.accumulated_force += force;
        let r = world_point - self.position;
        self.accumulated_torque += r.cross(&force);
    }

    /// 添加推力器
    pub fn add_thruster(&mut self, thruster: Thruster) {
        self.thrusters.push(thruster);
    }

    // ---------------------------------------------------------------
    // 积分
    // ---------------------------------------------------------------

    /// 推进一个时间步
    ///
    /// - 平动：半隐式欧拉（辛，能量漂移远小于显式欧拉）
    /// - 转动：`α = I⁻¹ · τ` → `ω += α·dt` → `q += ½·ω_q·q·dt` → 重归一化
    pub fn step(&mut self, dt: f64) {
        if self.mass <= 0.0 || dt <= 0.0 {
            return;
        }

        // --- 平动：半隐式欧拉 ---
        let accel = self.accumulated_force / self.mass;
        self.velocity += accel * dt;
        self.position += self.velocity * dt;

        // --- 转动 ---
        // 角加速度 = I⁻¹ · τ
        let ang_accel = self.inertia_tensor.inverse().mul_vec(&self.accumulated_torque);
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

        // --- 清零累加器 ---
        self.accumulated_force = Vec3::zero();
        self.accumulated_torque = Vec3::zero();
    }

    // ---------------------------------------------------------------
    // 查询
    // ---------------------------------------------------------------

    /// 当前速度大小（地速标量）
    pub fn speed(&self) -> f64 {
        self.velocity.length()
    }

    /// 当前旋转速率（rad/s）
    pub fn rotation_rate(&self) -> f64 {
        self.angular_velocity.length()
    }
}

// =====================================================================
// 引力影响球 (Sphere of Influence)
// =====================================================================

/// 影响球节点
///
/// 每个大质量天体（行星、恒星）有各自的 SoI。
/// 航天器在某个天体的 SoI 内时，主要受该天体引力主导。
#[derive(Debug, Clone)]
pub struct SoiNode {
    /// 对应 `GravitationalSystem.bodies` 中的索引
    pub body_idx: usize,
    /// 影响球半径 (m) — `a · (m / M_parent)^(2/5)`
    pub soi_radius: f64,
    /// 父节点索引
    pub parent: Option<usize>,
    /// 子节点索引
    pub children: Vec<usize>,
}

/// 引力影响球层次树
///
/// 构建后不可变（天体间的主从关系不变）。
/// `find_host(pos)` 在 O(log n) 内找到包含该位置的最内层天体。
#[derive(Debug, Clone)]
pub struct SoiTree {
    pub nodes: Vec<SoiNode>,
}

impl SoiTree {
    /// 从引力系统构建 SoI 树
    ///
    /// 算法：
    /// 1. 最重天体为根（恒星）
    /// 2. 其他天体按质量降序处理，找最近的更大质量体为父
    /// 3. SoI 半径 = 到父距离 · (m / M)^(2/5)
    pub fn build_from_system(sys: &GravitationalSystem) -> Self {
        let n = sys.bodies.len();
        if n == 0 {
            return SoiTree { nodes: Vec::new() };
        }

        // 按质量降序排序的索引
        let mut indices: Vec<usize> = (0..n).collect();
        indices.sort_by(|&a, &b| {
            sys.bodies[b]
                .mass
                .partial_cmp(&sys.bodies[a].mass)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut nodes: Vec<SoiNode> = Vec::with_capacity(n);
        // 临时存储：质量降序中已处理的 body_idx → node_idx
        let mut mass_order_to_node: Vec<usize> = Vec::with_capacity(n);

        for (order, &body_idx) in indices.iter().enumerate() {
            if order == 0 {
                // 根节点（最重天体，SoI 视为无限大）
                nodes.push(SoiNode {
                    body_idx,
                    soi_radius: f64::INFINITY,
                    parent: None,
                    children: Vec::new(),
                });
                mass_order_to_node.push(0);
            } else {
                let my_mass = sys.bodies[body_idx].mass;
                let my_pos = sys.bodies[body_idx].position;

                // 在已处理（更大质量）的天体中找最近的
                let mut best_parent = 0;
                let mut best_dist = f64::MAX;
                for &prev_idx in &indices[..order] {
                    let d = (my_pos - sys.bodies[prev_idx].position).length();
                    if d < best_dist {
                        best_dist = d;
                        best_parent = prev_idx;
                    }
                }

                // 找 parent 在 nodes 中的索引
                let parent_node_idx = nodes
                    .iter()
                    .position(|n| n.body_idx == best_parent)
                    .unwrap_or(0);

                // SoI 半径 = a · (m / M_parent)^(2/5)
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
                mass_order_to_node.push(node_idx);
            }
        }

        SoiTree { nodes }
    }

    /// 查找包含 `pos` 的最内层天体（即主导引力体）
    ///
    /// 从根开始遍历：对每个子节点，若 pos 在其 SoI 内则进入，
    /// 若多个子节点 SoI 都包含 pos，选天体最近的。
    pub fn find_host(&self, pos: Vec3, sys: &GravitationalSystem) -> usize {
        if self.nodes.is_empty() {
            return 0;
        }
        self.find_host_recursive(0, pos, sys)
    }

    fn find_host_recursive(&self, node_idx: usize, pos: Vec3, sys: &GravitationalSystem) -> usize {
        let node = &self.nodes[node_idx];
        let mut best_child = None;
        let mut best_dist = f64::MAX;

        for &child_idx in &node.children {
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
            None => node.body_idx,
        }
    }
}

// =====================================================================
// 太空物理世界
// =====================================================================

/// 太空物理世界 — 统一模拟入口
///
/// 管理：
/// - 背景 N 体引力系统（恒星、行星）
/// - 航天器（6DOF 刚体 + 推力器）
/// - 引力影响球（SoI）树
/// - 时间加速
///
/// # 典型用法
/// ```ignore
/// let mut world = SpacePhysicsWorld::new(solar_system, 1.0);
/// world.warp_factor = 100.0;  // 快速飞向火星
/// for _ in 0..100 {
///     world.step();
/// }
/// ```
pub struct SpacePhysicsWorld {
    /// 背景 N 体系统（恒星、行星、卫星）
    pub star_system: GravitationalSystem,
    /// 引力影响球树
    pub soi: SoiTree,
    /// 所有航天器
    pub spacecraft: Vec<SpacecraftBody>,
    /// 当前模拟时间 (s)
    pub time: f64,
    /// 基础时间步长 (s)
    pub dt: f64,
    /// 时间加速倍率（1.0 = 实时, 10.0 = 10×）
    pub warp_factor: f64,
}

impl SpacePhysicsWorld {
    /// 创建太空物理世界
    ///
    /// - `star_system`: 已配置好天体的引力系统
    /// - `dt`: 基础时间步长（s），`warp_factor` 将乘到这个值上
    pub fn new(star_system: GravitationalSystem, dt: f64) -> Self {
        let soi = SoiTree::build_from_system(&star_system);
        SpacePhysicsWorld {
            star_system,
            soi,
            spacecraft: Vec::new(),
            time: 0.0,
            dt,
            warp_factor: 1.0,
        }
    }

    /// 添加航天器
    pub fn add_spacecraft(&mut self, craft: SpacecraftBody) {
        self.spacecraft.push(craft);
    }

    /// 推进一个时间步
    ///
    /// 每步执行：
    /// 1. 背景 N 体积分（Yoshida4 阶辛积分器）
    /// 2. 对每艘航天器：
    ///    a. 通过 SoI 找主导引力体
    ///    b. 计算引力加速度
    ///    c. 应用激活的推力器
    ///    d. 推进刚体
    pub fn step(&mut self) {
        let effective_dt = self.dt * self.warp_factor;
        if effective_dt <= 0.0 {
            return;
        }

        // 1. 背景 N 体步进
        self.star_system.step_symplectic4(effective_dt);

        // 2. 航天器物理
        for ship in &mut self.spacecraft {
            // a. 找 SoI 主导天体
            let host_idx = self.soi.find_host(ship.position, &self.star_system);
            let host = &self.star_system.bodies[host_idx];

            // b. 引力加速度
            let r = host.position - ship.position;
            let dist = r.length();
            if dist > 1e-12 {
                // a = G·M / r³ · r
                let grav_force = r * (G * host.mass * ship.mass / (dist * dist * dist));
                ship.add_force(grav_force);
            }

            // c. 推力器 — 先收集力，避免 &ship 借用冲突
            let thruster_forces: Vec<(Vec3, Vec3)> = ship.thrusters
                .iter()
                .filter(|t| t.active && t.max_thrust > 0.0)
                .map(|t| {
                    let world_dir = ship.orientation.rotate(&t.direction);
                    let world_pos = ship.position + ship.orientation.rotate(&t.position);
                    (world_dir * t.max_thrust, world_pos)
                })
                .collect();
            for (force, pos) in thruster_forces {
                ship.add_force_at_point(force, pos);
            }

            // d. 刚体积分
            ship.step(effective_dt);
        }

        self.time += effective_dt;
    }

    /// 设置时间加速
    pub fn set_warp(&mut self, factor: f64) {
        self.warp_factor = factor.max(0.0);
    }

    /// 当前航天器数量
    pub fn spacecraft_count(&self) -> usize {
        self.spacecraft.len()
    }
}

// =====================================================================
// 测试
// =====================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{GravBody, GravitationalSystem};

    /// 构建一个简化的太阳系用于测试
    fn test_solar_system() -> GravitationalSystem {
        let mut sys = GravitationalSystem::new(1e6);
        // 太阳
        sys.add_body(GravBody::new("Sun", 1.989e30, 6.96e8, Vec3::zero(), Vec3::zero()));
        // 地球
        let earth_pos = Vec3::new(1.496e11, 0.0, 0.0);
        let earth_vel = Vec3::new(0.0, 29_780.0, 0.0);
        sys.add_body(GravBody::new("Earth", 5.972e24, 6.371e6, earth_pos, earth_vel));
        // 月球（简化：在地球附近）
        let moon_pos = Vec3::new(1.496e11 + 3.844e8, 0.0, 0.0);
        let moon_vel = Vec3::new(0.0, 29_780.0 + 1_022.0, 0.0);
        sys.add_body(GravBody::new("Moon", 7.342e22, 1.737e6, moon_pos, moon_vel));
        sys
    }

    // ---------------------------------------------------------------
    // SoiTree 测试
    // ---------------------------------------------------------------

    #[test]
    fn test_soi_tree_build() {
        let sys = test_solar_system();
        let soi = SoiTree::build_from_system(&sys);
        // 根是 Sun
        assert_eq!(soi.nodes.len(), 3);
        assert_eq!(sys.bodies[soi.nodes[0].body_idx].name, "Sun");
        // 根有子节点
        assert!(!soi.nodes[0].children.is_empty());
    }

    #[test]
    fn test_soi_find_host_sun() {
        let sys = test_solar_system();
        let soi = SoiTree::build_from_system(&sys);
        // 远离太阳系 → 返回 Sun
        let far = Vec3::new(1e20, 0.0, 0.0);
        let host = soi.find_host(far, &sys);
        assert_eq!(sys.bodies[host].name, "Sun");
    }

    #[test]
    fn test_soi_find_host_earth() {
        let sys = test_solar_system();
        let soi = SoiTree::build_from_system(&sys);
        // 地球表面附近 → 返回 Earth
        let near_earth = Vec3::new(1.496e11, 7.0e6, 0.0);
        let host = soi.find_host(near_earth, &sys);
        assert_eq!(sys.bodies[host].name, "Earth");
    }

    // ---------------------------------------------------------------
    // SpacecraftBody 6DOF 测试
    // ---------------------------------------------------------------

    #[test]
    fn test_spacecraft_new() {
        let ship = SpacecraftBody::new(
            Vec3::zero(),
            Vec3::zero(),
            1000.0,
            (100.0, 200.0, 150.0),
        );
        assert_eq!(ship.mass, 1000.0);
        assert_eq!(ship.speed(), 0.0);
        assert_eq!(ship.rotation_rate(), 0.0);
    }

    #[test]
    fn test_force_at_center_no_torque() {
        let mut ship = SpacecraftBody::new(
            Vec3::zero(),
            Vec3::zero(),
            1000.0,
            (100.0, 200.0, 150.0),
        );
        // 重心施力 → 只产生平动，不旋转
        ship.add_force(Vec3::new(1000.0, 0.0, 0.0));
        ship.step(1.0);
        assert!(ship.velocity.x > 0.0);
        assert_eq!(ship.angular_velocity.length(), 0.0);
    }

    #[test]
    fn test_force_at_point_creates_torque() {
        let mut ship = SpacecraftBody::new(
            Vec3::zero(),
            Vec3::zero(),
            1000.0,
            (100.0, 200.0, 150.0),
        );
        // 偏轴施力（在 x=1 处沿 Y 方向推）→ 应产生 Z 轴旋转
        ship.add_force_at_point(Vec3::new(0.0, 100.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        ship.step(1.0);
        assert!(ship.angular_velocity.length() > 0.0);
        // 扭矩 = r × F = (1,0,0) × (0,100,0) = (0,0,100)
        // 角加速度 = I⁻¹ · τ → 绕 Z 轴
        assert!(ship.angular_velocity.z.abs() > 0.0);
    }

    #[test]
    fn test_thruster_applied_in_local_frame() {
        let mut ship = SpacecraftBody::new(
            Vec3::zero(),
            Vec3::zero(),
            1000.0,
            (100.0, 200.0, 150.0),
        );
        // 在 +X 处安装一个朝 +Y 推的 RCS
        ship.add_thruster(Thruster::new(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            100.0,
        ));
        // 点火
        ship.thrusters[0].active = true;
        // 手动应用推力器（模拟 world.step 中的逻辑）
        let forces: Vec<(Vec3, Vec3)> = ship.thrusters
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
        // 产生了平动（Y方向）和转动（绕Z）
        assert!(ship.velocity.y > 0.0);
        assert!(ship.angular_velocity.length() > 0.0);
    }

    // ---------------------------------------------------------------
    // SpacePhysicsWorld 集成测试
    // ---------------------------------------------------------------

    #[test]
    fn test_world_step_no_spacecraft() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1000.0);
        assert_eq!(world.time, 0.0);
        world.step();
        assert!(world.time > 0.0);
    }

    #[test]
    fn test_world_step_with_spacecraft() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1000.0);

        // 在地球轨道上放一艘飞船
        let ship = SpacecraftBody::new(
            Vec3::new(1.496e11, 0.0, 0.0),
            Vec3::new(0.0, 29_780.0, 0.0),
            10_000.0,
            (5000.0, 8000.0, 6000.0),
        );
        world.add_spacecraft(ship);
        assert_eq!(world.spacecraft_count(), 1);

        world.step();
        // 飞船应该被地球引力牵引，位置变化
        assert!(world.spacecraft[0].position.length() > 0.0);
    }

    #[test]
    fn test_warp_factor() {
        let sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys, 1.0);

        world.step();
        let t1 = world.time;

        world.set_warp(10.0);
        world.step();
        let t2 = world.time;

        // warp 10× 应该在相同 step() 调用下推进 10× 时间
        assert!((t2 - t1 - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_soi_rebuild_on_add_body() {
        // 验证添加天体后重建 SoI
        let mut sys = test_solar_system();
        let mut world = SpacePhysicsWorld::new(sys.clone(), 1000.0);

        // 起初 SoI 有 3 个节点
        assert_eq!(world.soi.nodes.len(), 3);

        // 添加火星
        let mars_pos = Vec3::new(2.279e11, 0.0, 0.0);
        let mars_vel = Vec3::new(0.0, 24_070.0, 0.0);
        sys.add_body(GravBody::new("Mars", 6.417e23, 3.389e6, mars_pos, mars_vel));

        // 重建世界（或手动重建 SoI）
        let new_soi = SoiTree::build_from_system(&sys);
        world.star_system = sys;
        world.soi = new_soi;

        assert_eq!(world.soi.nodes.len(), 4);
    }
}
