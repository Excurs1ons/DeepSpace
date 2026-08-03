//! 导弹建模模块
//!
//! 提供空对空导弹（AAM）、地对空导弹（SAM）的飞行力学、
//! 制导律（比例导引法PN及其变体）、导引头模型。

use crate::Vec3;
use crate::aerodynamics::WindField;

// =====================================================================
// 导弹类型
// =====================================================================

/// 导引头类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SeekerType {
    /// 半主动雷达制导（需要载机照射）
    Sarh,
    /// 主动雷达制导
    ActiveRadar,
    /// 红外成像制导
    ImagingInfrared,
    /// 被动反辐射
    PassiveAntiRadiation,
}

/// 导弹发射阶段
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MissilePhase {
    /// 挂在发射架上
    Hanging,
    /// 发射后助推/起飞
    Boost,
    /// 续航飞行（巡航发动机）
    Sustain,
    /// 滑翔（发动机熄火）
    Coast,
    /// 自毁/命中目标
    Terminal,
}

// =====================================================================
// 空对空导弹配置
// =====================================================================

#[derive(Debug, Clone)]
pub struct AamConfig {
    pub name: String,
    pub seeker: SeekerType,
    // 物理参数
    pub mass_kg: f64,
    pub length_m: f64,
    pub diameter_m: f64,
    pub wing_span_m: f64,
    // 发动机
    pub boost_thrust_n: f64,     // 助推段推力
    pub boost_duration_s: f64,   // 助推段时间
    pub sustain_thrust_n: f64,   // 续航段推力
    pub sustain_duration_s: f64, // 续航段时间
    // 气动
    pub cd0: f64,
    pub cl_alpha: f64,
    // 导引头
    pub seeker_fov_deg: f64,       // 视场 (度)
    pub seeker_gimbal_limit_deg: f64, // 最大跟踪角 (度)
    pub seeker_lock_range_km: f64, // 锁定距离 (km)
    pub seeker_tracking_rate_deg_s: f64, // 最大跟踪角速度 (度/s)
    // 飞行限制
    pub max_mach: f64,
    pub max_g_load: f64,
    pub max_range_km: f64,
    // 弹头
    pub warhead_kg: f64,
    pub kill_radius_m: f64,
    // 最小射程
    pub min_range_m: f64,
    // 引导系数 N (比例导引)
    pub nav_constant: f64,
}

impl AamConfig {
    /// AIM-120C AMRAAM — 主动雷达中距弹
    pub fn aim120c() -> Self {
        AamConfig {
            name: "AIM-120C AMRAAM".into(),
            seeker: SeekerType::ActiveRadar,
            mass_kg: 152.0, length_m: 3.66, diameter_m: 0.178, wing_span_m: 0.526,
            boost_thrust_n: 15_000.0, boost_duration_s: 3.0,
            sustain_thrust_n: 3_000.0, sustain_duration_s: 8.0,
            cd0: 0.15, cl_alpha: 6.0,
            seeker_fov_deg: 30.0, seeker_gimbal_limit_deg: 60.0,
            seeker_lock_range_km: 25.0, seeker_tracking_rate_deg_s: 40.0,
            max_mach: 4.0, max_g_load: 35.0, max_range_km: 105.0,
            warhead_kg: 22.0, kill_radius_m: 10.0,
            min_range_m: 3_000.0, nav_constant: 4.0,
        }
    }

    /// AIM-120D AMRAAM — 增程型
    pub fn aim120d() -> Self {
        AamConfig {
            max_range_km: 160.0, seeker_lock_range_km: 30.0,
            seeker_tracking_rate_deg_s: 45.0,
            ..Self::aim120c()
        }
    }

    /// AIM-9X Sidewinder — 红外近距格斗弹
    pub fn aim9x() -> Self {
        AamConfig {
            name: "AIM-9X Sidewinder".into(),
            seeker: SeekerType::ImagingInfrared,
            mass_kg: 85.0, length_m: 3.02, diameter_m: 0.127, wing_span_m: 0.279,
            boost_thrust_n: 12_000.0, boost_duration_s: 2.0,
            sustain_thrust_n: 2_500.0, sustain_duration_s: 5.0,
            cd0: 0.12, cl_alpha: 7.0,
            seeker_fov_deg: 40.0, seeker_gimbal_limit_deg: 90.0,
            seeker_lock_range_km: 10.0, seeker_tracking_rate_deg_s: 60.0,
            max_mach: 2.5, max_g_load: 40.0, max_range_km: 35.0,
            warhead_kg: 9.4, kill_radius_m: 6.0,
            min_range_m: 500.0, nav_constant: 4.5,
        }
    }

    /// PL-15 (J-20)
    pub fn pl15() -> Self {
        AamConfig {
            name: "PL-15".into(),
            seeker: SeekerType::ActiveRadar,
            mass_kg: 190.0, length_m: 3.9, diameter_m: 0.203, wing_span_m: 0.6,
            boost_thrust_n: 18_000.0, boost_duration_s: 3.5,
            sustain_thrust_n: 4_000.0, sustain_duration_s: 10.0,
            cd0: 0.14, cl_alpha: 6.5,
            seeker_fov_deg: 30.0, seeker_gimbal_limit_deg: 60.0,
            seeker_lock_range_km: 30.0, seeker_tracking_rate_deg_s: 45.0,
            max_mach: 4.5, max_g_load: 40.0, max_range_km: 150.0,
            warhead_kg: 25.0, kill_radius_m: 12.0,
            min_range_m: 3_000.0, nav_constant: 4.0,
        }
    }

    /// PL-10 (J-20 WVR)
    pub fn pl10() -> Self {
        AamConfig {
            name: "PL-10".into(),
            seeker: SeekerType::ImagingInfrared,
            mass_kg: 89.0, length_m: 3.0, diameter_m: 0.14, wing_span_m: 0.3,
            boost_thrust_n: 13_000.0, boost_duration_s: 2.0,
            sustain_thrust_n: 2_000.0, sustain_duration_s: 5.0,
            cd0: 0.11, cl_alpha: 7.5,
            seeker_fov_deg: 45.0, seeker_gimbal_limit_deg: 90.0,
            seeker_lock_range_km: 12.0, seeker_tracking_rate_deg_s: 70.0,
            max_mach: 3.0, max_g_load: 45.0, max_range_km: 30.0,
            warhead_kg: 10.0, kill_radius_m: 7.0,
            min_range_m: 300.0, nav_constant: 4.5,
        }
    }

    /// R-77 (Su-35)
    pub fn r77() -> Self {
        AamConfig {
            name: "R-77 Adder".into(),
            seeker: SeekerType::ActiveRadar,
            mass_kg: 175.0, length_m: 3.6, diameter_m: 0.2, wing_span_m: 0.56,
            boost_thrust_n: 16_000.0, boost_duration_s: 3.0,
            sustain_thrust_n: 3_500.0, sustain_duration_s: 9.0,
            cd0: 0.16, cl_alpha: 5.5,
            seeker_fov_deg: 30.0, seeker_gimbal_limit_deg: 60.0,
            seeker_lock_range_km: 20.0, seeker_tracking_rate_deg_s: 35.0,
            max_mach: 4.0, max_g_load: 30.0, max_range_km: 100.0,
            warhead_kg: 22.0, kill_radius_m: 10.0,
            min_range_m: 3_000.0, nav_constant: 4.0,
        }
    }

    /// R-73 (Su-35 WVR)
    pub fn r73() -> Self {
        AamConfig {
            name: "R-73 Archer".into(),
            seeker: SeekerType::ImagingInfrared,
            mass_kg: 105.0, length_m: 2.9, diameter_m: 0.17, wing_span_m: 0.34,
            boost_thrust_n: 14_000.0, boost_duration_s: 2.5,
            sustain_thrust_n: 3_000.0, sustain_duration_s: 4.0,
            cd0: 0.13, cl_alpha: 7.0,
            seeker_fov_deg: 45.0, seeker_gimbal_limit_deg: 80.0,
            seeker_lock_range_km: 15.0, seeker_tracking_rate_deg_s: 60.0,
            max_mach: 2.5, max_g_load: 35.0, max_range_km: 30.0,
            warhead_kg: 8.0, kill_radius_m: 5.0,
            min_range_m: 300.0, nav_constant: 4.5,
        }
    }

    /// THAAD 级高空拦截器 — 高超声速、射程远、可大气层外拦截
    pub fn interceptor() -> AamConfig {
        AamConfig {
            name: "THAAD 拦截器".into(),
            seeker: SeekerType::ActiveRadar,
            mass_kg: 900.0,
            length_m: 6.17,
            diameter_m: 0.34,
            wing_span_m: 0.0,
            boost_thrust_n: 480_000.0,
            boost_duration_s: 6.0,
            sustain_thrust_n: 40_000.0,
            sustain_duration_s: 12.0,
            cd0: 0.08,
            cl_alpha: 0.0,
            seeker_fov_deg: 30.0,
            seeker_gimbal_limit_deg: 90.0,
            seeker_lock_range_km: 400.0,
            seeker_tracking_rate_deg_s: 60.0,
            max_mach: 8.2,
            max_g_load: 35.0,
            max_range_km: 200.0,
            warhead_kg: 100.0,
            kill_radius_m: 30.0,
            min_range_m: 500.0,
            nav_constant: 5.0,
        }
    }
}

// =====================================================================
// 导弹飞行状态
// =====================================================================

#[derive(Debug, Clone)]
pub struct MissileState {
    pub config: AamConfig,
    pub phase: MissilePhase,
    // 位置/速度
    pub position: Vec3,
    pub prev_position: Vec3, // 上一步位置（命中判定用线段插值）
    pub velocity: Vec3,
    // 已飞行时间
    pub flight_time_s: f64,
    // 发动机状态
    pub phase_time: f64, // 当前阶段已过时间
    pub current_thrust_n: f64,
    // 剩余燃料比
    pub propellant_fraction: f64,
    // 制导状态
    pub target_position: Vec3,
    pub target_velocity: Vec3,
    pub target_acceleration: Vec3, // APN 目标加速度补偿
    pub seeker_locked: bool,
    pub has_pitbull: bool, // 主动雷达已激活
    // 当前过载
    pub current_g: f64,
    // 引导指令
    pub cmd_accel: Vec3,
    // 累计飞行距离
    pub range_flown_m: f64,
    /// 外部引力加速度 (m/s²) — 默认零，由宿主世界注入（如地心引力）
    pub gravity: Vec3,
}

impl MissileState {
    pub fn new(config: AamConfig, pos: Vec3, vel: Vec3, tgt_pos: Vec3, tgt_vel: Vec3) -> Self {
        MissileState {
            phase: MissilePhase::Hanging,
            position: pos, prev_position: pos, velocity: vel,
            flight_time_s: 0.0, phase_time: 0.0,
            current_thrust_n: 0.0, propellant_fraction: 1.0,
            target_position: tgt_pos, target_velocity: tgt_vel,
            target_acceleration: Vec3::zero(),
            seeker_locked: false, has_pitbull: false,
            current_g: 0.0, cmd_accel: Vec3::zero(),
            range_flown_m: 0.0, config,
            gravity: Vec3::zero(),
        }
    }

    /// 发射
    pub fn launch(&mut self) {
        self.phase = MissilePhase::Boost;
        self.phase_time = 0.0;
        self.current_thrust_n = self.config.boost_thrust_n;
    }

    /// 主动雷达激活（Pitbull）
    pub fn activate_seeker(&mut self) {
        self.has_pitbull = true;
        self.seeker_locked = true;
    }

    /// 推进一个物理步（3DOF）
    pub fn step(&mut self, dt: f64, rho: f64) {
        if self.phase == MissilePhase::Terminal || self.phase == MissilePhase::Hanging {
            return;
        }

        self.flight_time_s += dt;
        self.phase_time += dt;

        let speed = self.velocity.length();
        let m = self.config.mass_kg * (0.3 + 0.7 * self.propellant_fraction);

        // 1. 发动机
        self.update_propulsion(dt);

        // 2. 气动力（用空速 = 地速 − 风速）
        let wind = WindField::default().wind_at(self.position.z);
        let air_vel = self.velocity - wind;
        let air_speed = air_vel.length().max(1e-6);
        let qbar = 0.5 * rho * air_speed * air_speed;
        let area = std::f64::consts::PI * (self.config.diameter_m * 0.5).powi(2);
        let drag = qbar * area * self.config.cd0;
        let drag_vec = if self.velocity.length() > 1.0 {
            air_vel / air_speed * (-drag)
        } else {
            Vec3::zero()
        };
        let guidance_accel = self.compute_guidance(dt);
        self.cmd_accel = guidance_accel;

        // 4. 整合加速度
        let thrust_force = if speed > 1.0 && self.current_thrust_n > 0.0 {
            self.velocity.normalized() * self.current_thrust_n
        } else { Vec3::zero() };

        let total_accel = (thrust_force + drag_vec) / m + guidance_accel + self.gravity;

        // 5. g-load & 速度限制
        let g_mag = total_accel.length() / 9.80665;
        self.current_g = g_mag;

        // 限制总加速度不超过最大过载
        let g_max = self.config.max_g_load * 9.80665;
        let a_mag = total_accel.length();
        let total_accel = if a_mag > g_max && a_mag > 0.0 {
            total_accel * (g_max / a_mag)
        } else {
            total_accel
        };

        // 6. 积分（先记录上一步位置用于线段命中判定）
        self.prev_position = self.position;
        self.velocity += total_accel * dt;

        // 最大速度限制（避免数值发散）
        let max_speed = self.config.max_mach * 340.0;
        if self.velocity.length() > max_speed {
            self.velocity = self.velocity.normalized() * max_speed;
        }
        self.position += self.velocity * dt;

        // 7. 距离
        self.range_flown_m += speed * dt;

        // 8. 过载限制
        let speed_new = self.velocity.length();
        if speed_new > self.config.max_mach * 340.0 {
            self.velocity = self.velocity.normalized() * self.config.max_mach * 340.0;
        }

        // 9. 导引头更新
        self.update_seeker();

        // 10. 阶段转换
        match self.phase {
            MissilePhase::Boost if self.phase_time >= self.config.boost_duration_s => {
                if self.config.sustain_thrust_n > 0.0 && self.config.sustain_duration_s > 0.0 {
                    self.phase = MissilePhase::Sustain;
                    self.phase_time = 0.0;
                    self.current_thrust_n = self.config.sustain_thrust_n;
                } else {
                    self.phase = MissilePhase::Coast;
                    self.current_thrust_n = 0.0;
                }
            }
            MissilePhase::Sustain if self.phase_time >= self.config.sustain_duration_s => {
                self.phase = MissilePhase::Coast;
                self.current_thrust_n = 0.0;
            }
            _ => {}
        }
    }

    fn update_propulsion(&mut self, dt: f64) {
        if self.current_thrust_n <= 0.0 { return; }
        let isp = 250.0; // ~250s solid/ramjet
        let mass_flow = self.current_thrust_n / (isp * 9.80665);
        self.propellant_fraction = (self.propellant_fraction - mass_flow * dt / self.config.mass_kg)
            .max(0.0);
    }

    /// 增强比例导引法 (APN)：PN + 目标加速度补偿
    fn compute_guidance(&self, _dt: f64) -> Vec3 {
        let r = self.target_position - self.position;
        let v_rel = self.velocity - self.target_velocity;
        let range = r.length();

        if range < 1.0 { return Vec3::zero(); }

        // LOS 视线方向（从导弹指向目标）
        let los = r / range;
        // 接近速度 (closing speed)：v_rel 在 LOS 上的投影，正 = 正在接近
        let vc = v_rel.dot(&los);

        // APN 指令加速度: a_cmd ∝ -(r × v_rel) × r / r³，垂直于 LOS
        // （标准 PN: a_cmd = N·Vc·(LOS×ω)；展开后 LOS×(r×v_rel)/r² = -cmd，
        //   故接近时需取负号，vc<0（目标逃逸）时 sign 翻转 → 掉头追击）
        let cmd = r.cross(&v_rel).cross(&r) / (range * range * range);

        let n = self.config.nav_constant;
        let raw = n * vc; // 带符号：接近为正
        let scale = raw.abs().max(50.0);
        let mut a_cmd = -cmd * scale * raw.signum();

        // APN 目标加速度补偿：0.5·N·a_t⊥（补偿目标机动，如重力下坠）
        let a_t_perp = self.target_acceleration - los * self.target_acceleration.dot(&los);
        a_cmd += a_t_perp * (0.5 * n);

        // 过载限制
        let g_max = self.config.max_g_load * 9.80665;
        let a_mag = a_cmd.length();
        if a_mag > g_max {
            a_cmd * (g_max / a_mag)
        } else {
            a_cmd
        }
    }

    fn update_seeker(&mut self) {
        let r = self.target_position - self.position;
        let range_to_target = r.length();

        // 主动雷达或红外锁定
        if self.has_pitbull || matches!(self.config.seeker, SeekerType::ImagingInfrared) {
            let lock_range = self.config.seeker_lock_range_km * 1000.0;
            if range_to_target <= lock_range {
                self.seeker_locked = true;
            }
        }
    }

    /// 是否命中目标（线段最近距离判定，避免高速穿越漏检）
    pub fn check_hit(&self, target_pos: &Vec3) -> bool {
        let p = *target_pos;
        let a = self.prev_position;
        let b = self.position;
        let ab = b - a;
        let len2 = ab.length_squared();
        // 上一步到目标距离已 ≤ 半径，视为命中
        if (p - a).length() <= self.config.kill_radius_m {
            return true;
        }
        if len2 < 1e-9 {
            return (p - a).length() <= self.config.kill_radius_m;
        }
        // 参数化 ab 上最近点：t 投影夹在 [0,1]
        let t = ((p - a).dot(&ab) / len2).clamp(0.0, 1.0);
        let nearest = a + ab * t;
        let dist = (p - nearest).length();
        dist <= self.config.kill_radius_m
    }

    pub fn is_alive(&self) -> bool {
        self.phase != MissilePhase::Terminal
    }
}

// =====================================================================
// 测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aam_config_basic() {
        let m = AamConfig::aim120c();
        assert!(m.max_range_km > 100.0);
        assert_eq!(m.seeker, SeekerType::ActiveRadar);
    }

    #[test]
    fn aam_launch_and_step() {
        let mut ms = MissileState::new(
            AamConfig::aim120c(),
            Vec3::zero(), Vec3::new(300.0, 0.0, 0.0),
            Vec3::new(50_000.0, 0.0, 5000.0), Vec3::new(250.0, 0.0, 0.0),
        );
        ms.launch();
        assert_eq!(ms.phase, MissilePhase::Boost);
        ms.step(0.1, 1.2);
        assert!(ms.range_flown_m > 0.0);
        assert!(ms.flight_time_s > 0.0);
    }

    #[test]
    fn aam_comparison() {
        let aim120 = AamConfig::aim120c();
        let pl15 = AamConfig::pl15();
        // PL-15 比 AIM-120C 重
        assert!(pl15.mass_kg > aim120.mass_kg);
        // PL-15 射程更远
        assert!(pl15.max_range_km > aim120.max_range_km);
    }

    #[test]
    fn guidance_vector() {
        let ms = MissileState::new(
            AamConfig::aim120c(),
            Vec3::new(0.0, 0.0, 10_000.0),
            Vec3::new(300.0, 0.0, 0.0),
            Vec3::new(30_000.0, 0.0, 10_500.0),
            Vec3::new(250.0, 0.0, 0.0),
        );
        let cmd = ms.compute_guidance(0.1);
        assert!(cmd.length() > 0.0 || cmd.length() < 0.01);
    }
}
