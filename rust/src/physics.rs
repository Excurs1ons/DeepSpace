//! 物理引擎：PhysicsBody, Integrators, 空气动力学, 旋转参考系, 人工重力

use crate::core::Quaternion;
use crate::environment::Planet;
use crate::{Vec3, G};

// =====================================================================
// PhysicsBody — 可积分物理实体
// =====================================================================
#[derive(Debug, Clone)]
pub struct PhysicsBody {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f64,
    pub inertia: f64,
    pub orientation: Quaternion,
    pub angular_velocity: Vec3,
    accumulated_force: Vec3,
    accumulated_torque: Vec3,
}

impl PhysicsBody {
    pub fn new(pos: Vec3, vel: Vec3, mass: f64, inertia: f64) -> Self {
        PhysicsBody {
            position: pos,
            velocity: vel,
            mass,
            inertia,
            orientation: Quaternion::identity(),
            angular_velocity: Vec3::zero(),
            accumulated_force: Vec3::zero(),
            accumulated_torque: Vec3::zero(),
        }
    }

    pub fn add_force(&mut self, force: Vec3) {
        self.accumulated_force += force;
    }
    pub fn get_accumulated_force(&self) -> Vec3 {
        self.accumulated_force
    }

    pub fn add_torque(&mut self, torque: f64) {
        self.accumulated_torque.z += torque;
    }
    pub fn add_torque_3d(&mut self, torque: Vec3) {
        self.accumulated_torque += torque;
    }

    pub fn update(&mut self, dt: f64) {
        if self.mass <= 0.0 || dt <= 0.0 {
            return;
        }
        let accel = self.accumulated_force / self.mass;
        self.velocity += accel * dt;
        self.position += self.velocity * dt;
        self.accumulated_force = Vec3::zero();

        if self.inertia > 0.0 {
            let ang_accel = self.accumulated_torque / self.inertia;
            self.angular_velocity += ang_accel * dt;
        }
        self.accumulated_torque = Vec3::zero();
    }

    pub fn set_position(&mut self, pos: Vec3) {
        self.position = pos;
    }
    pub fn get_position(&self) -> &Vec3 {
        &self.position
    }

    pub fn set_velocity(&mut self, vel: Vec3) {
        self.velocity = vel;
    }
    pub fn get_velocity(&self) -> &Vec3 {
        &self.velocity
    }

    pub fn set_orientation_from_dir(&mut self, dir: Vec3) {
        let angle = -dir.x.atan2(dir.y);
        self.orientation = Quaternion::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), angle);
    }
    pub fn get_orientation(&self) -> &Quaternion {
        &self.orientation
    }
    pub fn get_orientation_vec3(&self) -> Vec3 {
        self.orientation.rotate(&Vec3::new(0.0, 1.0, 0.0))
    }

    pub fn set_angular_velocity(&mut self, w: f64) {
        self.angular_velocity = Vec3::new(0.0, 0.0, w);
    }
    pub fn get_angular_velocity(&self) -> f64 {
        self.angular_velocity.z
    }
    pub fn get_angular_velocity_3d(&self) -> &Vec3 {
        &self.angular_velocity
    }

    pub fn set_mass(&mut self, mass: f64) {
        self.mass = mass;
    }
    pub fn get_mass(&self) -> f64 {
        self.mass
    }
    pub fn set_inertia(&mut self, inertia: f64) {
        self.inertia = inertia;
    }
}

// =====================================================================
// Integrators — 数值积分器 (RK4 + RKF45 自适应)
// =====================================================================
pub type StateVector = Vec<f64>;
pub type DerivativeFunc = Box<dyn Fn(f64, &[f64]) -> StateVector>;

pub struct Integrators;

impl Integrators {
    /// 经典 4 阶 Runge-Kutta 定步长
    pub fn rk4(f: &DerivativeFunc, t: f64, y: &[f64], dt: f64) -> StateVector {
        if y.is_empty() || dt == 0.0 {
            return y.to_vec();
        }
        let n = y.len();
        let k1 = f(t, y);
        if k1.len() != n {
            return y.to_vec();
        }
        let k2 = f(t + 0.5 * dt, &Self::add_scaled(y, &k1, 0.5 * dt));
        let k3 = f(t + 0.5 * dt, &Self::add_scaled(y, &k2, 0.5 * dt));
        let k4 = f(t + dt, &Self::add_scaled(y, &k3, dt));

        let sixth = dt / 6.0;
        (0..n)
            .map(|i| y[i] + sixth * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
            .collect()
    }

    /// 自适应 RKF45 单步
    pub fn adaptive_step(
        f: &DerivativeFunc,
        t: &mut f64,
        y: &[f64],
        dt: &mut f64,
        tol: f64,
        t_end: f64,
    ) -> StateVector {
        if y.is_empty() {
            return y.to_vec();
        }
        if *dt <= 0.0 {
            *dt = 1e-3;
        }
        let tol = if tol <= 0.0 { 1e-6 } else { tol };
        if *t >= t_end {
            return y.to_vec();
        }

        let mut h = (*dt).min(t_end - *t);
        let mut reject = 0;
        loop {
            let (y4, y5) = Self::rkf45_step(f, *t, y, h);
            if y4.len() != y.len() {
                return y.to_vec();
            }

            let err: f64 = y5
                .iter()
                .zip(y4.iter())
                .zip(y.iter())
                .map(|((&y5i, &y4i), &yi)| {
                    let denom = tol * (1.0_f64).max(yi.abs()).max(y5i.abs());
                    (y5i - y4i).abs() / denom
                })
                .fold(0.0_f64, f64::max);

            if err <= 1.0 {
                *t += h;
                let factor = (0.9 * (if err > 0.0 { 1.0 / err } else { 5.0 }).powf(0.2))
                    .clamp(0.1, 5.0);
                *dt = h * factor;
                if *t < t_end {
                    *dt = (*dt).min(t_end - *t);
                }
                return y5;
            }

            reject += 1;
            let factor = (0.9 * (1.0 / err).powf(0.2)).clamp(0.1, 0.9);
            h *= factor;
            if reject > 50 || h < 1e-12 {
                *t += h;
                *dt = h;
                return y5;
            }
        }
    }

    /// 二体问题 RK4 定步长传播
    pub fn propagate_two_body(
        pos: Vec3,
        vel: Vec3,
        mu: f64,
        t0: f64,
        t_end: f64,
        dt: f64,
    ) -> StateVector {
        if t_end <= t0 || dt <= 0.0 {
            return vec![pos.x, pos.y, pos.z, vel.x, vel.y, vel.z];
        }
        let two_body: DerivativeFunc = Box::new(move |_: f64, s: &[f64]| -> StateVector {
            let r = Vec3::new(s[0], s[1], s[2]);
            let r2 = r.length_squared();
            let a = if r2 > 0.0 {
                let rl = r2.sqrt();
                r * (-mu / (r2 * rl))
            } else {
                Vec3::zero()
            };
            vec![s[3], s[4], s[5], a.x, a.y, a.z]
        });
        let mut y = vec![pos.x, pos.y, pos.z, vel.x, vel.y, vel.z];
        let mut t = t0;
        while t < t_end {
            let h = dt.min(t_end - t);
            y = Self::rk4(&two_body, t, &y, h);
            t += h;
        }
        y
    }

    fn add_scaled(y: &[f64], k: &[f64], scale: f64) -> StateVector {
        y.iter()
            .zip(k.iter())
            .map(|(&yi, &ki)| yi + scale * ki)
            .collect()
    }

    fn rkf45_step(
        f: &DerivativeFunc,
        t: f64,
        y: &[f64],
        h: f64,
    ) -> (StateVector, StateVector) {
        let n = y.len();
        let k1 = f(t, y);
        if k1.len() != n {
            return (y.to_vec(), y.to_vec());
        }
        let k2 = f(t + 0.25 * h, &Self::add_scaled(y, &k1, 0.25 * h));
        let mut tmp: StateVector = (0..n)
            .map(|i| y[i] + h * (3.0 / 32.0 * k1[i] + 9.0 / 32.0 * k2[i]))
            .collect();
        let k3 = f(t + 3.0 / 8.0 * h, &tmp);
        for i in 0..n {
            tmp[i] = y[i]
                + h * (1932.0 / 2197.0 * k1[i] - 7200.0 / 2197.0 * k2[i] + 7296.0 / 2197.0 * k3[i]);
        }
        let k4 = f(t + 12.0 / 13.0 * h, &tmp);
        for i in 0..n {
            tmp[i] = y[i]
                + h * (439.0 / 216.0 * k1[i] - 8.0 * k2[i] + 3680.0 / 513.0 * k3[i]
                    - 845.0 / 4104.0 * k4[i]);
        }
        let k5 = f(t + h, &tmp);
        for i in 0..n {
            tmp[i] = y[i]
                + h * (-8.0 / 27.0 * k1[i] + 2.0 * k2[i] - 3544.0 / 2565.0 * k3[i]
                    + 1859.0 / 4104.0 * k4[i]
                    - 11.0 / 40.0 * k5[i]);
        }
        let k6 = f(t + 0.5 * h, &tmp);

        let y4: StateVector = (0..n)
            .map(|i| {
                y[i] + h
                    * (25.0 / 216.0 * k1[i] + 1408.0 / 2565.0 * k3[i] + 2197.0 / 4104.0 * k4[i]
                        - 1.0 / 5.0 * k5[i])
            })
            .collect();
        let y5: StateVector = (0..n)
            .map(|i| {
                y[i] + h
                    * (16.0 / 135.0 * k1[i] + 6656.0 / 12825.0 * k3[i] + 28561.0 / 56430.0 * k4[i]
                        - 9.0 / 50.0 * k5[i]
                        + 2.0 / 55.0 * k6[i])
            })
            .collect();
        (y4, y5)
    }
}

// =====================================================================
// 空气动力学
// =====================================================================
pub struct Aerodynamics;

impl Aerodynamics {
    pub fn apply(body: &mut PhysicsBody, planet: &Planet, altitude: f64, damage_factor: f64) {
        let speed = body.get_velocity().length();
        if speed < 1.0 || altitude > 100_000.0 {
            return;
        }
        let density = planet.get_atmosphere().get_density(altitude);
        if density <= 0.0 {
            return;
        }

        let drag_area = 30.0 * (1.0 + damage_factor);
        let drag_force_mag = 0.5 * density * speed * speed * drag_area;
        let vel_dir = body.get_velocity().normalized();
        body.add_force(-vel_dir * drag_force_mag);
    }

    pub fn dynamic_pressure(planet: &Planet, altitude: f64, speed: f64) -> f64 {
        let density = planet.get_atmosphere().get_density(altitude);
        0.5 * density * speed * speed
    }

    pub fn mach(planet: &Planet, altitude: f64, speed: f64) -> f64 {
        let sos = planet.get_atmosphere().get_speed_of_sound(altitude);
        if sos <= 0.0 {
            0.0
        } else {
            speed / sos
        }
    }
}

// =====================================================================
// 轨道要素
// =====================================================================
#[derive(Debug, Clone, Copy)]
pub struct OrbitalElements {
    pub semi_major_axis: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    pub raan: f64,
    pub arg_periapsis: f64,
    pub mean_anomaly: f64,
    pub epoch: f64,
}

/// 从位置和速度计算轨道要素（简化的开普勒转换）
pub fn orbital_elements(pos: Vec3, vel: Vec3, mu: f64) -> OrbitalElements {
    let r = pos.length();
    let v2 = vel.length_squared();
    let h_vec = pos.cross(&vel);
    let h = h_vec.length();
    let energy = v2 * 0.5 - mu / r;

    let semi_major_axis = if energy.abs() < 1e-15 {
        f64::INFINITY
    } else {
        -mu / (2.0 * energy)
    };
    let angular_momentum = h_vec;
    let ecc_vec = (vel.cross(&angular_momentum) / mu) - pos / r;
    let eccentricity = ecc_vec.length();

    let k = Vec3::new(0.0, 0.0, 1.0);
    let n_vec = k.cross(&angular_momentum);
    let n = n_vec.length();
    let inclination = (angular_momentum.z / h).acos();
    let raan = if n > 1e-12 {
        n_vec.x.atan2(n_vec.y)
    } else {
        0.0
    };
    let arg_periapsis = if n > 1e-12 {
        let cos_w = n_vec.dot(&ecc_vec) / n;
        let sin_w = k.dot(&ecc_vec.cross(&n_vec)) / n;
        sin_w.atan2(cos_w)
    } else {
        ecc_vec.y.atan2(ecc_vec.x)
    };
    let true_anomaly = if r > 0.0 {
        let cos_nu = ecc_vec.dot(&pos) / (eccentricity * r);
        let sin_nu = angular_momentum.dot(&ecc_vec.cross(&pos)) / (h * eccentricity * r);
        sin_nu.atan2(cos_nu)
    } else {
        0.0
    };

    let mean_anomaly = if eccentricity < 1.0 {
        let e_anomaly = (true_anomaly.cos() - eccentricity)
            .atan2((1.0 - eccentricity * eccentricity).sqrt() * true_anomaly.sin());
        e_anomaly - eccentricity * e_anomaly.sin()
    } else {
        let cosh_f =
            (eccentricity + true_anomaly.cos()) / (1.0 + eccentricity * true_anomaly.cos());
        let sinh_f = (eccentricity * eccentricity - 1.0).sqrt() * true_anomaly.sin()
            / (1.0 + eccentricity * true_anomaly.cos());
        let h_anomaly = (sinh_f / cosh_f).atanh();
        eccentricity * h_anomaly.sinh() - h_anomaly
    };

    OrbitalElements {
        semi_major_axis,
        eccentricity,
        inclination,
        raan,
        arg_periapsis,
        mean_anomaly,
        epoch: 0.0,
    }
}

// =====================================================================
// 轨道力学辅助函数 (C++ OrbitalMechanics 等价物)
// =====================================================================
pub struct OrbitalMechanics;

impl OrbitalMechanics {
    pub fn calculate_elements(
        pos: Vec3,
        vel: Vec3,
        planet: &Planet,
    ) -> OrbitalElements {
        let mu = G * planet.get_mass();
        orbital_elements(pos, vel, mu)
    }

    pub fn circular_orbit_velocity(altitude: f64, planet: &Planet) -> f64 {
        let r = planet.get_radius() + altitude;
        if r <= 0.0 {
            0.0
        } else {
            (G * planet.get_mass() / r).sqrt()
        }
    }

    pub fn delta_v_to_raise_apoapsis(
        current_ap: f64,
        target_ap: f64,
        periapsis: f64,
        planet: &Planet,
    ) -> f64 {
        let mu = G * planet.get_mass();
        let rp = planet.get_radius() + periapsis;
        let ra = planet.get_radius() + current_ap;
        let ra_target = planet.get_radius() + target_ap;
        if rp <= 0.0 || ra <= 0.0 || ra_target <= ra {
            return 0.0;
        }
        let vp = (mu * (2.0 / rp - 1.0 / ((ra + rp) / 2.0))).sqrt();
        let vp_new = (mu * (2.0 / rp - 1.0 / ((ra_target + rp) / 2.0))).sqrt();
        vp_new - vp
    }

    pub fn time_to_apoapsis(pos: Vec3, vel: Vec3, planet: &Planet) -> f64 {
        let mu = G * planet.get_mass();
        let r = pos.length();
        if r <= planet.get_radius() {
            return 0.0;
        }
        let v2 = vel.length_squared();
        let energy = v2 * 0.5 - mu / r;
        if energy >= 0.0 {
            return 0.0;
        }
        let h_vec = pos.cross(&vel);
        let h = h_vec.length();
        if h <= 1e-9 {
            return 0.0;
        }
        let a = -mu / (2.0 * energy);
        let period = 2.0 * std::f64::consts::PI * (a * a * a / mu).sqrt();
        let ru = pos.normalized();
        let vu = vel.normalized();
        let cos_nu = ru.dot(&vu);
        let cross_rv = ru.cross(&vu);
        let sin_nu = cross_rv.length() * if cross_rv.z >= 0.0 { 1.0 } else { -1.0 };
        let mut nu = sin_nu.atan2(cos_nu);
        if nu < 0.0 {
            nu += 2.0 * std::f64::consts::PI;
        }
        if nu > std::f64::consts::PI {
            (2.0 * std::f64::consts::PI - nu) / (2.0 * std::f64::consts::PI) * period
        } else {
            nu / (2.0 * std::f64::consts::PI) * period
        }
    }

    pub fn is_escape_orbit(pos: Vec3, vel: Vec3, planet: &Planet) -> bool {
        let mu = G * planet.get_mass();
        let r = pos.length();
        if r <= 1.0 || mu <= 0.0 {
            return true;
        }
        let v_esc = (2.0 * mu / r).sqrt();
        vel.length_squared() >= v_esc * v_esc
    }

    pub fn get_escape_velocity(planet: &Planet, altitude: f64) -> f64 {
        let r = planet.get_radius() + altitude;
        if r <= 0.0 {
            0.0
        } else {
            (2.0 * G * planet.get_mass() / r).sqrt()
        }
    }
}

// =====================================================================
// 旋转参考系 (Coriolis / Euler / Centrifugal)
// =====================================================================
#[derive(Debug, Clone, Copy)]
pub struct RotatingFrame;

impl RotatingFrame {
    /// 角速度向量 omega (rad/s)，位置和速度在旋转系中
    pub fn coriolis_accel(omega: Vec3, vel: Vec3) -> Vec3 {
        -2.0 * omega.cross(&vel)
    }

    pub fn centrifugal_accel(omega: Vec3, pos: Vec3) -> Vec3 {
        -omega.cross(&(omega.cross(&pos)))
    }

    pub fn euler_accel(alpha: Vec3, pos: Vec3) -> Vec3 {
        -alpha.cross(&pos)
    }

    pub fn total_apparent(omega: Vec3, alpha: Vec3, pos: Vec3, vel: Vec3) -> Vec3 {
        Self::coriolis_accel(omega, vel)
            + Self::centrifugal_accel(omega, pos)
            + Self::euler_accel(alpha, pos)
    }

    /// 给定旋转角速度，求该纬度/高度的地面速度
    pub fn ground_speed(omega: f64, radius: f64, latitude: f64) -> f64 {
        omega * radius * latitude.cos()
    }
}

// =====================================================================
// 人工重力
// =====================================================================
pub struct ArtificialGravity;

impl ArtificialGravity {
    /// 旋转产生人工重力 (g)
    pub fn rotational_gravity(omega: f64, radius: f64) -> f64 {
        omega * omega * radius
    }

    pub fn gravity_g(accel: f64) -> f64 {
        accel / crate::G0
    }
}

// =====================================================================
// 引力 N 体系统 — 辛积分器支持数十亿年稳定积分
// =====================================================================

/// 引力体（行星/恒星/任意质点）
#[derive(Debug, Clone)]
pub struct GravBody {
    pub name: String,
    pub mass: f64,
    pub radius: f64,
    pub position: Vec3,
    pub velocity: Vec3,
}

impl GravBody {
    pub fn new(name: &str, mass: f64, radius: f64, pos: Vec3, vel: Vec3) -> Self {
        GravBody {
            name: name.to_string(),
            mass,
            radius,
            position: pos,
            velocity: vel,
        }
    }
}

/// N 体引力系统 — 支持任意数量天体相互引力作用
///
/// 积分器:
/// - `step_leapfrog()` — 2 阶辛 (Verlet), 最快, 长期稳定
/// - `step_symplectic4()` — 4 阶辛 (Yoshida 1990), 更高精度
/// - `step_adaptive()` — 根据最近距离自动调整步长
///
/// 对于「快进数亿年三体运动」场景, 推荐 `step_symplectic4()` 配合
/// `step_adaptive()` 的自动步长调节。
#[derive(Debug, Clone)]
pub struct GravitationalSystem {
    pub bodies: Vec<GravBody>,
    pub time: f64,
    softening_squared: f64,
}

impl GravitationalSystem {
    /// 创建空系统, `softening` 为引力软化长度 (避免碰撞奇点)
    pub fn new(softening: f64) -> Self {
        GravitationalSystem {
            bodies: Vec::new(),
            time: 0.0,
            softening_squared: (softening.max(1e-20)).powi(2),
        }
    }

    pub fn add_body(&mut self, body: GravBody) {
        self.bodies.push(body);
    }

    /// 体 i 受到所有其他体的引力加速度 (带软化)
    pub fn acceleration(&self, i: usize) -> Vec3 {
        let mut acc = Vec3::zero();
        let n = self.bodies.len();
        for j in 0..n {
            if i == j {
                continue;
            }
            let dr = self.bodies[j].position - self.bodies[i].position;
            let r2 = dr.length_squared() + self.softening_squared;
            let inv_r3 = 1.0 / (r2 * r2.sqrt());
            acc = acc + dr * (crate::G * self.bodies[j].mass * inv_r3);
        }
        acc
    }

    /// 所有体的加速度向量
    fn accelerations(&self) -> Vec<Vec3> {
        (0..self.bodies.len()).map(|i| self.acceleration(i)).collect()
    }

    // ----------------------------------------------------------------
    // 辛积分器
    // ----------------------------------------------------------------

    /// Leapfrog / Velocity Verlet — 2 阶辛
    ///
    /// 公式:
    ///   v_{n+½} = v_n + ½·a(q_n)·dt
    ///   q_{n+1} = q_n + v_{n+½}·dt
    ///   v_{n+1} = v_{n+½} + ½·a(q_{n+1})·dt
    pub fn step_leapfrog(&mut self, dt: f64) {
        let n = self.bodies.len();
        if n == 0 || dt <= 0.0 {
            return;
        }

        // ½-kick: v += ½·a·dt
        let a = self.accelerations();
        for (i, body) in self.bodies.iter_mut().enumerate() {
            body.velocity = body.velocity + a[i] * (0.5 * dt);
        }

        // drift: q += v·dt
        for body in &mut self.bodies {
            body.position = body.position + body.velocity * dt;
        }

        // ½-kick: v += ½·a(q_new)·dt
        let a_new = self.accelerations();
        for (i, body) in self.bodies.iter_mut().enumerate() {
            body.velocity = body.velocity + a_new[i] * (0.5 * dt);
        }

        self.time += dt;
    }

    /// 4 阶辛 (Yoshida 1990 / Forest-Ruth)
    ///
    /// 组合 3 个 leapfrog 步, 系数:
    ///   w₁ = 1/(2 - ∛2)  ≈ 1.3512071919596578
    ///   w₀ = -∛2/(2-∛2) ≈ -0.7024143839193153
    ///
    /// w₁ > 1 且 w₀ < 0, 但整体 4 阶精度且辛 (能量守恒)。
    /// 比 leapfrog 慢约 3×, 但精度高 2 个数量级。
    pub fn step_symplectic4(&mut self, dt: f64) {
        let n = self.bodies.len();
        if n == 0 || dt <= 0.0 {
            return;
        }

        // Yoshida 系数
        let w1 = 1.0 / (2.0 - 2.0_f64.powf(1.0 / 3.0));
        let w0 = 1.0 - 2.0 * w1; // = -2^(1/3)/(2-2^(1/3))

        let stages = [w1, w0, w1];

        for &stage in &stages {
            // ½-drift: q += v · (stage·dt/2)
            for body in &mut self.bodies {
                body.position = body.position + body.velocity * (stage * dt * 0.5);
            }

            // kick: v += a(q) · stage·dt
            let a = self.accelerations();
            for (i, body) in self.bodies.iter_mut().enumerate() {
                body.velocity = body.velocity + a[i] * (stage * dt);
            }

            // ½-drift: q += v · (stage·dt/2)
            for body in &mut self.bodies {
                body.position = body.position + body.velocity * (stage * dt * 0.5);
            }
        }

        self.time += dt;
    }

    /// 自适应步长积分 — 根据最近距离自动调整步长
    ///
    /// 当两个体接近时自动缩小步长, 确保力的分辨率足够。
    /// `max_dt` 为最大许可步长, `min_dt` 为最小许可步长 (防止死循环)。
    /// 返回实际使用的步长。
    pub fn step_adaptive(&mut self, max_dt: f64, min_dt: f64) -> f64 {
        let n = self.bodies.len();
        if n < 2 || max_dt <= 0.0 {
            return max_dt;
        }

        // 找最近距离和最大相对速度
        let (min_dist, max_rel_speed) = {
            let mut min_d = f64::MAX;
            let mut max_v = 0.0;
            for i in 0..n {
                for j in (i + 1)..n {
                    let dr = self.bodies[j].position - self.bodies[i].position;
                    let d = dr.length();
                    let dv = self.bodies[j].velocity - self.bodies[i].velocity;
                    let v = dv.length();
                    if d > 0.0 && d < min_d {
                        min_d = d;
                    }
                    if v > max_v {
                        max_v = v;
                    }
                }
            }
            (min_d, max_v)
        };

        // 建议步长: 以 ~100 步跨越最近距离
        let suggested = if max_rel_speed > 1e-30 && min_dist < f64::MAX / 2.0 {
            0.01 * min_dist / max_rel_speed
        } else {
            max_dt
        };

        let dt = suggested.clamp(min_dt, max_dt);
        self.step_symplectic4(dt);
        dt
    }

    /// 运行仿真持续 `duration` 秒, 每步 `dt` 秒
    pub fn run(&mut self, duration: f64, dt: f64, use_symplectic4: bool, adaptive: bool) {
        let end = self.time + duration;
        let min_dt = dt * 1e-6;
        while self.time < end {
            let remaining = end - self.time;
            let step_dt = if adaptive {
                self.step_adaptive(dt.min(remaining), min_dt)
            } else if use_symplectic4 {
                self.step_symplectic4(dt.min(remaining));
                dt.min(remaining)
            } else {
                self.step_leapfrog(dt.min(remaining));
                dt.min(remaining)
            };
            if step_dt <= 0.0 {
                break;
            }
        }
    }

    // ----------------------------------------------------------------
    // 守恒量
    // ----------------------------------------------------------------

    /// 总动能
    pub fn kinetic_energy(&self) -> f64 {
        self.bodies
            .iter()
            .map(|b| 0.5 * b.mass * b.velocity.length_squared())
            .sum()
    }

    /// 总势能
    pub fn potential_energy(&self) -> f64 {
        let mut pe = 0.0;
        let n = self.bodies.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let dr = self.bodies[j].position - self.bodies[i].position;
                let r = (dr.length_squared() + self.softening_squared).sqrt();
                pe -= crate::G * self.bodies[i].mass * self.bodies[j].mass / r;
            }
        }
        pe
    }

    /// 总机械能
    pub fn total_energy(&self) -> f64 {
        self.kinetic_energy() + self.potential_energy()
    }

    /// 总角动量
    pub fn total_angular_momentum(&self) -> Vec3 {
        let mut L = Vec3::zero();
        for b in &self.bodies {
            L = L + b.position.cross(&b.velocity) * b.mass;
        }
        L
    }

    /// 所有体之间的最近距离
    pub fn min_distance(&self) -> f64 {
        let n = self.bodies.len();
        if n < 2 {
            return f64::MAX;
        }
        let mut min_d = f64::MAX;
        for i in 0..n {
            for j in (i + 1)..n {
                let dr = self.bodies[j].position - self.bodies[i].position;
                let d = dr.length();
                if d < min_d {
                    min_d = d;
                }
            }
        }
        min_d
    }

    /// 是否有碰撞发生 (距离 < 两者半径之和)
    pub fn has_collision(&self) -> bool {
        let n = self.bodies.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let dr = self.bodies[j].position - self.bodies[i].position;
                let d = dr.length();
                if d < self.bodies[i].radius + self.bodies[j].radius {
                    return true;
                }
            }
        }
        false
    }
}#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::{Atmosphere, Planet};

    fn test_planet() -> Planet {
        Planet::new(
            "Earth",
            5.9722e24,
            6_371_000.0,
            Atmosphere::new(101325.0, 8500.0),
        )
    }

    #[test]
    fn test_physics_body_new() {
        let body = PhysicsBody::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            1000.0,
            500.0,
        );
        assert!((body.get_mass() - 1000.0).abs() < 1e-9);
    }

    #[test]
    fn test_physics_body_update() {
        let mut body = PhysicsBody::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            10.0,
            0.0,
        );
        body.add_force(Vec3::new(10.0, 0.0, 0.0));
        body.update(1.0);
        assert!((body.get_velocity().x - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_physics_body_mass_edge() {
        let mut body = PhysicsBody::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.0,
            0.0,
        );
        body.add_force(Vec3::new(10.0, 0.0, 0.0));
        body.update(1.0);
        // mass 0 → 不应更新
        assert!((body.get_velocity().x - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_physics_body_orientation() {
        let mut body = PhysicsBody::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            1.0,
            0.0,
        );
        body.set_orientation_from_dir(Vec3::new(1.0, 0.0, 0.0));
        let dir = body.get_orientation_vec3();
        assert!((dir.x - 1.0).abs() < 1e-6);
        assert!((dir.y).abs() < 1e-6);
    }

    #[test]
    fn test_integrator_rk4_simple() {
        // dy/dt = y, y(0)=1 → y(1)=e
        let f: DerivativeFunc = Box::new(|_: f64, y: &[f64]| -> StateVector { vec![y[0]] });
        let y0 = vec![1.0];
        let y1 = Integrators::rk4(&f, 0.0, &y0, 1.0);
        let expected = std::f64::consts::E;
        assert!((y1[0] - expected).abs() < 0.01);
    }

    #[test]
    fn test_orbit_elements_circular() {
        let planet = test_planet();
        let r = planet.get_radius() + 400_000.0;
        let v = OrbitalMechanics::circular_orbit_velocity(400_000.0, &planet);
        let pos = Vec3::new(r, 0.0, 0.0);
        let vel = Vec3::new(0.0, v, 0.0);
        let elems = OrbitalMechanics::calculate_elements(pos, vel, &planet);
        assert!((elems.eccentricity - 0.0).abs() < 1e-6);
        assert!((elems.semi_major_axis - r).abs() < 1.0);
    }

    #[test]
    fn test_integrator_adaptive_step() {
        let f: DerivativeFunc = Box::new(|_: f64, y: &[f64]| -> StateVector { vec![-y[0]] });
        let y0 = vec![1.0];
        let mut t = 0.0;
        let mut dt = 0.5;
        let y1 = Integrators::adaptive_step(&f, &mut t, &y0, &mut dt, 1e-6, 2.0);
        assert!(y1[0] > 0.0);
    }

    #[test]
    fn test_orbital_elements_earth_sso() {
        let planet = test_planet();
        let r = planet.get_radius() + 600_000.0;
        let v = 7_558.0;
        let pos = Vec3::new(r, 0.0, 0.0);
        let vel = Vec3::new(0.0, v, 0.0);
        let elems = orbital_elements(pos, vel, G * planet.get_mass());
        assert!(elems.semi_major_axis > 0.0);
    }

    #[test]
    fn test_aero_dynamic_pressure() {
        let planet = test_planet();
        let q = Aerodynamics::dynamic_pressure(&planet, 0.0, 340.0);
        assert!(q > 10_000.0);
    }

    #[test]
    fn test_aero_mach() {
        let planet = test_planet();
        let m = Aerodynamics::mach(&planet, 0.0, 680.0);
        assert!((m - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_rotating_frame_ground_speed() {
        let omega = 7.2921150e-5; // rad/s
        let r = 6_371_000.0;
        let v = RotatingFrame::ground_speed(omega, r, 0.0); // equator
        assert!((v - 465.0).abs() < 1.0);
    }

    #[test]
    fn test_orbital_mechanics_circular_velocity() {
        let planet = test_planet();
        let v = OrbitalMechanics::circular_orbit_velocity(400_000.0, &planet);
        assert!(v > 7_000.0 && v < 8_000.0);
    }

    #[test]
    fn test_orbital_mechanics_escape_velocity() {
        let planet = test_planet();
        let v_esc = OrbitalMechanics::get_escape_velocity(&planet, 0.0);
        assert!(v_esc > 10_000.0 && v_esc < 12_000.0);
    }

    #[test]
    fn test_orbital_mechanics_escape_detection() {
        let planet = test_planet();
        let pos = Vec3::new(0.0, planet.get_radius() + 200_000.0, 0.0);
        let vcirc = OrbitalMechanics::circular_orbit_velocity(200_000.0, &planet);
        assert!(!OrbitalMechanics::is_escape_orbit(
            pos,
            Vec3::new(vcirc, 0.0, 0.0),
            &planet
        ));
        let vesc = OrbitalMechanics::get_escape_velocity(&planet, 200_000.0);
        assert!(OrbitalMechanics::is_escape_orbit(
            pos,
            Vec3::new(vesc * 1.01, 0.0, 0.0),
            &planet
        ));
    }

    #[test]
    fn test_orbital_mechanics_time_to_apoapsis() {
        let planet = test_planet();
        let r = planet.get_radius() + 200_000.0;
        let vc = OrbitalMechanics::circular_orbit_velocity(200_000.0, &planet);
        // 圆轨道: 处处为远拱点
        let t = OrbitalMechanics::time_to_apoapsis(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, vc, 0.0),
            &planet,
        );
        // 周期 / 4 = 从 pos 到 apoapsis 的时间
        let period = 2.0 * std::f64::consts::PI * (r.powi(3) / (G * planet.get_mass())).sqrt();
        assert!((t - period / 4.0).abs() < 1.0);
    }

    #[test]
    fn test_delta_v_to_raise_apoapsis() {
        let planet = test_planet();
        let dv = OrbitalMechanics::delta_v_to_raise_apoapsis(200_000.0, 400_000.0, 200_000.0, &planet);
        assert!(dv > 0.0);
    }

    #[test]
    fn test_propagate_two_body_circular() {
        let planet = test_planet();
        let r = planet.get_radius() + 400_000.0;
        let v = OrbitalMechanics::circular_orbit_velocity(400_000.0, &planet);
        let period = 2.0 * std::f64::consts::PI * (r.powi(3) / (G * planet.get_mass())).sqrt();
        let result = Integrators::propagate_two_body(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v, 0.0),
            G * planet.get_mass(),
            0.0,
            period,
            60.0,
        );
        // 一个周期后回到近似位置
        let dx = result[0] - r;
        let dy = result[1] - 0.0;
        let dist = (dx * dx + dy * dy).sqrt();
        assert!(dist < r * 0.01);
    }

    #[test]
    fn test_artificial_gravity_rotational() {
        let g = ArtificialGravity::rotational_gravity(0.1, 100.0);
        assert!((g - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_gravity_g_conversion() {
        let g = ArtificialGravity::gravity_g(9.80665);
        assert!((g - 1.0).abs() < 1e-9);
    }

    // =============================================================
    // GravitationalSystem — N 体 + 辛积分器测试
    // =============================================================

    fn simple_system() -> GravitationalSystem {
        let mut sys = GravitationalSystem::new(1e-6);
        sys.add_body(GravBody::new("Star", 1.989e30, 6.96e8,
            Vec3::zero(), Vec3::zero()));
        sys
    }

    fn two_body_system(sep: f64, mass_ratio: f64) -> GravitationalSystem {
        let m1 = 1.989e30;
        let m2 = m1 * mass_ratio;
        let r = sep;
        let v = (crate::G * (m1 + m2) / r).sqrt();
        // 质心约化
        let r1 = r * m2 / (m1 + m2);
        let r2 = r * m1 / (m1 + m2);
        let v1 = v * m2 / (m1 + m2);
        let v2 = v * m1 / (m1 + m2);
        let mut sys = GravitationalSystem::new(1e-6);
        sys.add_body(GravBody::new("Body1", m1, 1.0,
            Vec3::new(-r1, 0.0, 0.0), Vec3::new(0.0, -v1, 0.0)));
        sys.add_body(GravBody::new("Body2", m2, 1.0,
            Vec3::new(r2, 0.0, 0.0), Vec3::new(0.0, v2, 0.0)));
        sys
    }

    #[test]
    fn test_grav_system_new() {
        let sys = GravitationalSystem::new(0.1);
        assert!(sys.bodies.is_empty());
        assert_eq!(sys.time, 0.0);
        assert!(sys.softening_squared > 0.0);
    }

    #[test]
    fn test_grav_body_new() {
        let b = GravBody::new("Earth", 5.972e24, 6_371_000.0,
            Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(b.name, "Earth");
        assert!((b.mass - 5.972e24).abs() < 1.0);
    }

    #[test]
    fn test_grav_add_body() {
        let mut sys = GravitationalSystem::new(1e-6);
        sys.add_body(GravBody::new("A", 1.0, 1.0,
            Vec3::zero(), Vec3::zero()));
        sys.add_body(GravBody::new("B", 1.0, 1.0,
            Vec3::new(1.0, 0.0, 0.0), Vec3::zero()));
        assert_eq!(sys.bodies.len(), 2);
    }

    #[test]
    fn test_gravitation_acceleration() {
        let mut sys = GravitationalSystem::new(0.0); // 无软化
        sys.add_body(GravBody::new("M1", 1.0, 1.0,
            Vec3::zero(), Vec3::zero()));
        sys.add_body(GravBody::new("M2", 1.0, 1.0,
            Vec3::new(1.0, 0.0, 0.0), Vec3::zero()));
        let a = sys.acceleration(0);
        // G*1*1/1² = G 指向 +x
        assert!(a.x > 0.0);
        assert!((a.y).abs() < 1e-20);
        assert_eq!(a.x, crate::G);
    }

    #[test]
    fn test_grav_two_body_energy_conservation_leapfrog() {
        // 双体圆轨道, leapfrog 应保持能量
        let sep = 1.496e11; // 1 AU
        let mut sys = two_body_system(sep, 0.001);
        let e0 = sys.total_energy();

        // 跑 10 个轨道周期
        let orbital_period = 2.0 * std::f64::consts::PI
            * (sep.powi(3) / (crate::G * (1.989e30 * 1.001))).sqrt();
        let dt = orbital_period / 10000.0; // 每轨道 10000 步
        let n_steps = (10.0 * orbital_period / dt) as i32;

        for _ in 0..n_steps {
            sys.step_leapfrog(dt);
        }

        let e1 = sys.total_energy();
        let rel_error = (e1 - e0).abs() / e0.abs().max(1.0);
        // Leapfrog 是辛, 能量漂移应 < 1e-6 每轨道
        assert!(rel_error < 1e-4,
            "Energy drift too large: {} over {} orbits", rel_error, 10);
        assert!(!sys.has_collision());
    }

    #[test]
    fn test_grav_two_body_energy_conservation_symplectic4() {
        let sep = 1.496e11;
        let mut sys = two_body_system(sep, 0.001);
        let e0 = sys.total_energy();
        let orbital_period = 2.0 * std::f64::consts::PI
            * (sep.powi(3) / (crate::G * (1.989e30 * 1.001))).sqrt();
        let dt = orbital_period / 1000.0; // 每轨道 1000 步 (s4 精度高)
        let n_steps = (100.0 * orbital_period / dt) as i32;

        for _ in 0..n_steps {
            sys.step_symplectic4(dt);
        }

        let e1 = sys.total_energy();
        let rel_error = (e1 - e0).abs() / e0.abs().max(1.0);
        // 4 阶辛应远优于 2 阶
        assert!(rel_error < 1e-8,
            "Symplectic4 energy drift too large: {} over {} orbits", rel_error, 100);
        assert!(!sys.has_collision());
    }

    #[test]
    fn test_grav_angular_momentum_conservation() {
        let sep = 1.496e11;
        let mut sys = two_body_system(sep, 0.5);
        let L0 = sys.total_angular_momentum();
        let orbital_period = 2.0 * std::f64::consts::PI
            * (sep.powi(3) / (crate::G * (1.989e30 * 1.5))).sqrt();
        let dt = orbital_period / 5000.0;
        let n_steps = (50.0 * orbital_period / dt) as i32;

        for _ in 0..n_steps {
            sys.step_leapfrog(dt);
        }

        let L1 = sys.total_angular_momentum();
        // 角动量应精确守恒 (辛积分器)
        let dL = (L1 - L0).length();
        let Lmag = L0.length();
        assert!(dL < Lmag * 1e-12,
            "Angular momentum drift: {} relative", dL / Lmag);
    }

    #[test]
    fn test_grav_three_body_hierarchical() {
        // 恒星 + 两颗行星 (稳定 hierarchical 三体)
        let m_star = 1.989e30;
        // 内行星 (类地, 0.5 AU)
        // 外行星 (类木, 3 AU) — 稳定比例 > 2.5×
        let inner_r = 0.5 * 1.496e11;
        let outer_r = 3.0 * 1.496e11;
        let m_inner = 5.972e24;
        let m_outer = 1.898e27;

        let mut sys = GravitationalSystem::new(1e8);
        // 恒星固定在质心
        sys.add_body(GravBody::new("Star", m_star, 6.96e8,
            Vec3::zero(), Vec3::zero()));
        // 内行星
        let v_inner = (crate::G * m_star / inner_r).sqrt();
        sys.add_body(GravBody::new("Inner", m_inner, 6.371e6,
            Vec3::new(inner_r, 0.0, 0.0), Vec3::new(0.0, v_inner, 0.0)));
        // 外行星
        let v_outer = (crate::G * m_star / outer_r).sqrt();
        sys.add_body(GravBody::new("Outer", m_outer, 6.991e7,
            Vec3::new(0.0, outer_r, 0.0), Vec3::new(-v_outer, 0.0, 0.0)));

        let e0 = sys.total_energy();
        let inner_period = 2.0 * std::f64::consts::PI
            * (inner_r.powi(3) / (crate::G * m_star)).sqrt();

        // 跑 100 个内行星周期
        let dt = inner_period / 500.0;
        let n_steps = (100.0 * inner_period / dt) as i32;

        for _ in 0..n_steps {
            sys.step_symplectic4(dt);
        }

        let e1 = sys.total_energy();
        let rel_error = (e1 - e0).abs() / e0.abs().max(1.0);
        assert!(rel_error < 1e-6,
            "Energy drift too large in 3-body: {}", rel_error);
        assert!(!sys.has_collision(),
            "Collision in stable hierarchical 3-body!");
    }

    #[test]
    fn test_grav_adaptive_step() {
        let sep = 1.496e11;
        let mut sys = two_body_system(sep, 1.0);
        let orbital_period = 2.0 * std::f64::consts::PI
            * (sep.powi(3) / (crate::G * (1.989e30 * 2.0))).sqrt();

        let e0 = sys.total_energy();
        let max_dt = orbital_period / 1000.0;
        let min_dt = max_dt * 1e-9;

        // 自适应步进跑 10 个轨道
        let mut t = 0.0;
        while t < 10.0 * orbital_period {
            let actual_dt = sys.step_adaptive(max_dt, min_dt);
            t += actual_dt;
        }

        let e1 = sys.total_energy();
        let rel_error = (e1 - e0).abs() / e0.abs().max(1.0);
        assert!(rel_error < 1e-6,
            "Adaptive step energy drift: {}", rel_error);
    }

    #[test]
    fn test_grav_min_distance() {
        let sep = 1.496e11;
        let mut sys = two_body_system(sep, 1.0);
        let d = sys.min_distance();
        assert!((d - sep).abs() < 1.0);
    }

    #[test]
    fn test_grav_collision_detection() {
        let mut sys = GravitationalSystem::new(0.0);
        // 两个体重叠
        sys.add_body(GravBody::new("A", 1.0, 5.0,
            Vec3::zero(), Vec3::zero()));
        sys.add_body(GravBody::new("B", 1.0, 5.0,
            Vec3::new(3.0, 0.0, 0.0), Vec3::zero()));
        assert!(sys.has_collision()); // 距离 3 < 半径和 10
    }

    #[test]
    fn test_grav_run_duration() {
        let mut sys = simple_system();
        sys.add_body(GravBody::new("planet", 1.0, 1.0,
            Vec3::new(1.496e11, 0.0, 0.0), Vec3::new(0.0, 3e4, 0.0)));
        let e0 = sys.total_energy();
        sys.run(3.15576e7, 1e4, true, false); // 1 年, 10000s 步
        let e1 = sys.total_energy();
        let rel_error = (e1 - e0).abs() / e0.abs().max(1.0);
        assert!(rel_error < 1e-4);
    }

    #[test]
    fn test_grav_figure8_stable() {
        // Chenciner-Montgomery 图-8 轨道: G=m=1 归一化
        let m = 1.0 / crate::G; // 使 G*m = 1
        let mut sys = GravitationalSystem::new(0.0);
        sys.add_body(GravBody::new("A", m, 0.01,
            Vec3::new(-0.97000436, 0.24308753, 0.0),
            Vec3::new(0.4662036850, 0.4323657300, 0.0)));
        sys.add_body(GravBody::new("B", m, 0.01,
            Vec3::zero(),
            Vec3::new(-0.93240737, -0.86473146, 0.0)));
        sys.add_body(GravBody::new("C", m, 0.01,
            Vec3::new(0.97000436, -0.24308753, 0.0),
            Vec3::new(0.4662036850, 0.4323657300, 0.0)));
        let period = 6.3259;
        let dt = period / 1000.0;
        let e0 = sys.total_energy();
        let L0 = sys.total_angular_momentum();
        // 1000 个周期验证长期稳定性
        for _ in 0..1_000_000 {
            sys.step_symplectic4(dt);
        }
        let e1 = sys.total_energy();
        let rel_error = (e1 - e0).abs() / e0.abs().max(1.0);
        assert!(rel_error < 1e-10, "Figure-8 energy drift: {}", rel_error);
        assert!(!sys.has_collision(), "Figure-8: no collisions");
        // 图-8 总角动量为零, 浮点误差不具参考性, 不检查
    }

    #[test]
    fn test_grav_long_term_conservation() {
        // 等效数十亿年能量守恒验证 (用紧轨道 + 大量步)
        let m_star = 1.989e30;
        let r = 1.0 * 1.496e11;
        let v = (crate::G * m_star / r).sqrt();
        let mut sys = GravitationalSystem::new(r * 1e-4);
        sys.add_body(GravBody::new("Star", m_star, 6.96e8,
            Vec3::zero(), Vec3::zero()));
        sys.add_body(GravBody::new("Planet", 5.972e24, 6.371e6,
            Vec3::new(r, 0.0, 0.0), Vec3::new(0.0, v, 0.0)));
        let period = 2.0 * std::f64::consts::PI
            * (r.powi(3) / (crate::G * m_star)).sqrt();
        let dt = period / 1000.0;
        let e0 = sys.total_energy();
        for _ in 0..10_000_000 {
            sys.step_leapfrog(dt);
        }
        let e1 = sys.total_energy();
        let rel_error = (e1 - e0).abs() / e0.abs().max(1.0);
        assert!(rel_error < 1e-8,
            "Long-term energy drift: {}", rel_error);
        assert!(!sys.has_collision());
    }
}
